/*
 * BNO085 UART-RVC IMU Driver
 *
 * Wiring: PS0->3V3, PS1->GND, BNO SDA/TX -> ESP GPIO3 (UART1 RX), 115200
 * Protocol: 0xAA 0xAA header, 19-byte packets at 100 Hz
 *
 * Critical: uart_driver_install() binds the UART ISR to the calling core
 * (esp_intr_alloc intr_alloc.c: cpu = esp_cpu_get_core_id()).  If called
 * from CPU0, the ISR competes with ESP-Hosted WiFi/SDIO and JPEG send.
 * We run the full UART init sequence inside a CPU1-pinned task so the ISR
 * and imu_uart_task share CPU1, away from network/encode work on CPU0.
 */

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "driver/uart.h"
#include "imu.h"

static const char *TAG = "IMU";

typedef enum {
    IMU_IDLE        = 0,
    IMU_IN_PROGRESS = 1,
    IMU_READY       = 2,
    IMU_FAILED      = 3,
} imu_init_state_t;

// Guarded by s_init_cs — cross-core visibility between app task (CPU0) and
// imu_uart_init_task (CPU1).
static imu_init_state_t s_init_state = IMU_IDLE;
static TaskHandle_t     s_init_task  = NULL;
static portMUX_TYPE     s_init_cs    = portMUX_INITIALIZER_UNLOCKED;

#define RVC_UART       UART_NUM_1
#define RVC_RX_PIN     3
#define RVC_BAUD       115200
#define RVC_PACKET_LEN 19
#define RVC_RX_BUFFER_SIZE (16 * 1024)

static imu_data_t s_latest;
static SemaphoreHandle_t s_mutex = NULL;
static QueueHandle_t s_sample_queue = NULL;

typedef struct {
    uint32_t valid_packet_count;
    uint32_t checksum_error_count;
    uint32_t uart_timeout_count;
    int64_t last_valid_timestamp_us;
    uint32_t rx_overflow_count;
    uint32_t short_packet_count;
    uint32_t bad_header_count;
    uint32_t sync_fail_count;
    uint32_t rvc_header_count;
    uint32_t rvc_candidate_count;
    uint32_t rvc_resync_replay_count;
} imu_health_state_t;

// Protected by s_mutex together with s_latest. ESP32-P4 is 32-bit, so a
// cross-task int64_t read must not occur outside this mutex.
static imu_health_state_t s_health;

static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool rvc_checksum_valid(const uint8_t packet[RVC_PACKET_LEN])
{
    uint8_t sum = 0;
    for (int i = 2; i < RVC_PACKET_LEN - 1; ++i) {
        sum += packet[i];
    }
    return sum == packet[RVC_PACKET_LEN - 1];
}

/*
 * UART-RVC parser shared with the locally hardware-proven imu_test project.
 * It consumes the 100 Hz / 19-byte stream as AA AA then 17 payload bytes.
 * This low-rate bytewise parser is intentionally simpler than a bulk replay
 * FSM: reliable continued publication matters more than micro-optimizing
 * 1.9 kB/s of UART traffic alongside the camera pipeline.
 */

static void imu_uart_task(void *arg)
{
    /*
     * This parser deliberately follows the locally hardware-proven imu_test
     * sequence: one-byte AA AA synchronization, then exactly 17 payload bytes.
     * The former bulk/FSM/replay parser could leave replay bytes unconsumed at a
     * chunk boundary and eventually stop publishing new samples in the full
     * camera application.  Keep parsing simple at 100 Hz / 1.9 kB/s.
     */
    uint8_t packet[RVC_PACKET_LEN];
    uint32_t timeout_streak = 0;

    while (true) {
        uint8_t byte = 0;
        if (uart_read_bytes(RVC_UART, &byte, 1, pdMS_TO_TICKS(1000)) != 1) {
            timeout_streak++;
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                s_health.uart_timeout_count++;
                xSemaphoreGive(s_mutex);
            }
            if (timeout_streak <= 3 || timeout_streak % 10 == 0) {
                ESP_LOGW(TAG, "no UART data for %lu second(s)", (unsigned long)timeout_streak);
            }
            continue;
        }
        timeout_streak = 0;

        if (byte != 0xAA) {
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                s_health.sync_fail_count++;
                xSemaphoreGive(s_mutex);
            }
            continue;
        }
        if (uart_read_bytes(RVC_UART, &byte, 1, pdMS_TO_TICKS(20)) != 1 || byte != 0xAA) {
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                s_health.sync_fail_count++;
                xSemaphoreGive(s_mutex);
            }
            continue;
        }

        packet[0] = 0xAA;
        packet[1] = 0xAA;
        int received = uart_read_bytes(RVC_UART, packet + 2, RVC_PACKET_LEN - 2,
                                       pdMS_TO_TICKS(30));
        if (received != RVC_PACKET_LEN - 2) {
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                s_health.short_packet_count++;
                xSemaphoreGive(s_mutex);
            }
            continue;
        }

        if (!rvc_checksum_valid(packet)) {
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                s_health.checksum_error_count++;
                xSemaphoreGive(s_mutex);
            }
            continue;
        }

        int64_t ts = esp_timer_get_time();
        imu_data_t sample = {
            .timestamp_us = ts,
            .index = packet[2],
            .yaw = le_i16(packet + 3) * 0.01f,
            .pitch = le_i16(packet + 5) * 0.01f,
            .roll = le_i16(packet + 7) * 0.01f,
            .raw_ax_mg = le_i16(packet + 9),
            .raw_ay_mg = le_i16(packet + 11),
            .raw_az_mg = le_i16(packet + 13),
            .valid = true,
        };
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s_latest = sample;
            s_health.last_valid_timestamp_us = ts;
            s_health.valid_packet_count++;
            s_health.rvc_header_count++;
            s_health.rvc_candidate_count++;
            xSemaphoreGive(s_mutex);
        }
        if (s_sample_queue) xQueueOverwrite(s_sample_queue, &sample);
    }
}

// No UART event queue in the production RX path.  The tested camera-only
// C1-A2 image is stable with the ISR doing only ring-buffer work; posting
// an event per receive condition adds an avoidable CPU1/cache interaction.
// Overflow health remains exposed as zero unless a future dedicated diagnostic
// image deliberately enables an event queue.

/* Event queue intentionally disabled in production; see note above. */

// Runs on CPU1. Performs full UART driver init so the ISR is bound to CPU1.
// Signals the caller via xTaskNotifyGive (task notification) — no semaphore
// to delete, so a late signal after caller timeout is always safe (no UAF).
// State transitions (s_init_cs protected) are visible to any caller core.
static esp_err_t s_init_result = ESP_OK;

static void imu_uart_init_task(void *arg)
{
    TaskHandle_t caller = (TaskHandle_t)arg;
    s_init_result = ESP_OK;

    uart_config_t cfg = {
        .baud_rate  = RVC_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_param_config(RVC_UART, &cfg);
    if (ret != ESP_OK) {
        s_init_result = ret;
        portENTER_CRITICAL(&s_init_cs);
        s_init_state = IMU_FAILED;
        portEXIT_CRITICAL(&s_init_cs);
        xTaskNotifyGive(caller);
        vTaskDelete(NULL);
        return;
    }

    ret = uart_set_pin(RVC_UART, UART_PIN_NO_CHANGE, RVC_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        s_init_result = ret;
        portENTER_CRITICAL(&s_init_cs);
        s_init_state = IMU_FAILED;
        portEXIT_CRITICAL(&s_init_cs);
        xTaskNotifyGive(caller);
        vTaskDelete(NULL);
        return;
    }

    // CSI/ISP/JPEG may suspend flash cache during DMA.  With the matching
    // Kconfig option this forces the UART driver ISR into IRAM, so the BNO085
    // 100 Hz RX stream remains serviceable through camera work.
    ret = uart_driver_install(RVC_UART, RVC_RX_BUFFER_SIZE, 0, 0,
                              NULL, ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK) {
        s_init_result = ret;
        portENTER_CRITICAL(&s_init_cs);
        s_init_state = IMU_FAILED;
        portEXIT_CRITICAL(&s_init_cs);
        xTaskNotifyGive(caller);
        vTaskDelete(NULL);
        return;
    }

    uart_flush_input(RVC_UART);

    s_init_result = ESP_OK;
    portENTER_CRITICAL(&s_init_cs);
    s_init_state = IMU_IN_PROGRESS;  // parent creates reader tasks, then sets READY
    portEXIT_CRITICAL(&s_init_cs);
    xTaskNotifyGive(caller);
    vTaskDelete(NULL);
}

esp_err_t imu_init(void)
{
    portENTER_CRITICAL(&s_init_cs);
    imu_init_state_t state = s_init_state;
    portEXIT_CRITICAL(&s_init_cs);

    if (state == IMU_READY) return ESP_OK;

    // --- Resource ownership model ---
    // s_mutex and s_sample_queue are created once and persist for the
    // lifetime of the driver.  On timeout the init task may still be
    // running, so we intentionally LEAK them here — they will be reused
    // on the next call.  Cleanup only occurs when the previous init task
    // is guaranteed finished (FAILED state).

    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
        if (!s_mutex) return ESP_ERR_NO_MEM;
    }

    memset(&s_latest, 0, sizeof(s_latest));
    memset(&s_health, 0, sizeof(s_health));

    if (!s_sample_queue) {
        s_sample_queue = xQueueCreate(1, sizeof(imu_data_t));
        if (!s_sample_queue) {
            ESP_LOGE(TAG, "sample queue create failed");
            vSemaphoreDelete(s_mutex); s_mutex = NULL;
            return ESP_ERR_NO_MEM;
        }
    }

    if (state == IMU_IN_PROGRESS) {
        ESP_LOGW(TAG, "init already in progress (concurrent call?)");
        return ESP_ERR_INVALID_STATE;
    }

    // If the previous attempt failed (or we just waited for a timed-out
    // attempt that also failed), clean up the UART driver that the old
    // task may have partially installed.  The old task is guaranteed
    // finished at this point — either it completed normally (FAILED) or
    // we waited for it above.
    if (state == IMU_FAILED) {
        uart_driver_delete(RVC_UART);
        // Reset to IDLE so we can retry.
        portENTER_CRITICAL(&s_init_cs);
        s_init_state = IMU_IDLE;
        portEXIT_CRITICAL(&s_init_cs);
        state = IMU_IDLE;
    }

    // Drain any residual task notification.  After the serialisation
    // above, any stale notification from a completed old task has been
    // consumed.  This catch-all ensures a clean slate for our own wait.
    ulTaskNotifyTake(pdTRUE, 0);

    // Run uart_driver_install on CPU1 so the UART ISR is bound to CPU1.
    // esp_intr_alloc intr_alloc.c:580 — cpu = esp_cpu_get_core_id().
    // No ESP_INTR_FLAG exists to override core selection; the ISR always
    // goes to the calling core.
    //
    // We use a task notification instead of a binary semaphore to signal
    // completion.  The init task receives our TaskHandle as its argument and
    // calls xTaskNotifyGive(caller) on every exit path.  This eliminates the
    // use-after-free: there is no semaphore object to delete, so a late
    // notification after our timeout is always safe.
    portENTER_CRITICAL(&s_init_cs);
    s_init_state = IMU_IN_PROGRESS;
    portEXIT_CRITICAL(&s_init_cs);

    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    BaseType_t ok = xTaskCreatePinnedToCore(imu_uart_init_task, "imu_uart_init",
                                             4096, (void *)self, 12, &s_init_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create UART init task");
        portENTER_CRITICAL(&s_init_cs);
        s_init_state = IMU_IDLE;
        portEXIT_CRITICAL(&s_init_cs);
        vQueueDelete(s_sample_queue); s_sample_queue = NULL;
        vSemaphoreDelete(s_mutex); s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Wait for init to complete — should take < 50 ms.
    // On timeout we return ESP_ERR_TIMEOUT and leave state as IN_PROGRESS.
    // The init task continues running on CPU1.  Subsequent calls to imu_init
    // will wait for this task to finish before attempting any UART operation.
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3000)) == 0) {
        ESP_LOGE(TAG, "UART init timed out — init task still running on CPU1");
        return ESP_ERR_TIMEOUT;
    }

    // Task completed — read the state it set before notifying us.
    portENTER_CRITICAL(&s_init_cs);
    state = s_init_state;
    portEXIT_CRITICAL(&s_init_cs);

    if (state == IMU_FAILED || (state == IMU_IN_PROGRESS && s_init_result != ESP_OK)) {
        ESP_LOGE(TAG, "UART init failed: 0x%x", s_init_result);
        uart_driver_delete(RVC_UART);
        portENTER_CRITICAL(&s_init_cs);
        s_init_state = IMU_IDLE;
        portEXIT_CRITICAL(&s_init_cs);
        vQueueDelete(s_sample_queue); s_sample_queue = NULL;
        vSemaphoreDelete(s_mutex); s_mutex = NULL;
        return s_init_result;
    }

    ESP_LOGI(TAG, "UART driver installed on CPU1, creating tasks...");

    // Only the high-priority CPU1 parser is needed.  The UART event queue is
    // intentionally disabled to match the stable C1-A2 camera diagnostic.
    TaskHandle_t uart_task_h = NULL;

    // ESP-Hosted runs priority 23.  Keep the short RX parser one level above
    // it on CPU1, but it blocks in uart_read_bytes() whenever no byte arrives,
    // so C6 work still runs between 100 Hz BNO085 packets.
    if (xTaskCreatePinnedToCore(imu_uart_task, "imu_uart", 4096, NULL,
                                24, &uart_task_h, 1) != pdPASS) {
        ESP_LOGE(TAG, "failed to create UART task");
        uart_driver_delete(RVC_UART);
        portENTER_CRITICAL(&s_init_cs);
        s_init_state = IMU_IDLE;
        portEXIT_CRITICAL(&s_init_cs);
        vQueueDelete(s_sample_queue); s_sample_queue = NULL;
        vSemaphoreDelete(s_mutex); s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_init_cs);
    s_init_state = IMU_READY;
    portEXIT_CRITICAL(&s_init_cs);

    ESP_LOGI(TAG, "UART-RVC ready (UART%d RX=GPIO%d %d baud, RX buf=%u, "
             "IRAM ISR + parser CPU1 prio=24, event queue disabled)",
             RVC_UART, RVC_RX_PIN, RVC_BAUD, (unsigned)RVC_RX_BUFFER_SIZE);

    return ESP_OK;
}

esp_err_t imu_read(imu_data_t *data)
{
    portENTER_CRITICAL(&s_init_cs);
    imu_init_state_t state = s_init_state;
    portEXIT_CRITICAL(&s_init_cs);

    if (!data || state != IMU_READY) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *data = s_latest;
        xSemaphoreGive(s_mutex);
        return data->valid ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    return ESP_ERR_TIMEOUT;
}

bool imu_is_ready(void)
{
    portENTER_CRITICAL(&s_init_cs);
    imu_init_state_t state = s_init_state;
    portEXIT_CRITICAL(&s_init_cs);
    return state == IMU_READY;
}

void imu_get_health(imu_health_t *health)
{
    if (!health) return;
    memset(health, 0, sizeof(*health));
    if (!s_mutex) return;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        health->valid_packet_count = s_health.valid_packet_count;
        health->checksum_error_count = s_health.checksum_error_count;
        health->uart_timeout_count = s_health.uart_timeout_count;
        health->last_valid_timestamp_us = s_health.last_valid_timestamp_us;
        health->rx_overflow_count = s_health.rx_overflow_count;
        health->short_packet_count = s_health.short_packet_count;
        health->bad_header_count = s_health.bad_header_count;
        health->sync_fail_count = s_health.sync_fail_count;
        health->rvc_header_count = s_health.rvc_header_count;
        health->rvc_candidate_count = s_health.rvc_candidate_count;
        health->rvc_resync_replay_count = s_health.rvc_resync_replay_count;
        xSemaphoreGive(s_mutex);
    }
}

QueueHandle_t imu_get_sample_queue(void)
{
    return s_sample_queue;
}
