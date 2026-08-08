/*
 * Mars Lander ESP32-P4 Firmware — Reading/Archive Snapshot
 *
 * Purpose: one-file, human-readable archival view of the source files compiled
 * into the saved ESP32-P4 production application.
 *
 * IMPORTANT: This is NOT a build input and must NOT replace wifi_test/main/.
 * The active firmware is intentionally split into modules. Concatenating these
 * modules preserves their text for reading, but makes duplicate module-private
 * static symbols share one C translation unit and therefore is not directly
 * compilable. Build the original modular source tree instead.
 *
 * Active ESP-IDF source units: main.c, camera.c, imu.c, mega_sr04.c, actuator.c.
 * Excluded: ultrasonic.c/.h because it is not listed in the active
 * idf_component_register(SRCS ...) entry; HC-SR04 is currently owned by Mega.
 * The project-level CMakeLists.txt was excluded from this source-only finalware
 * snapshot because it is build configuration, not a C/C++ source unit.
 */


/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/idf_component.yml
 * SHA256 (UTF-8 source): bb60363a5a36b0ed45f8c4fb9246ed57018bb95764b80f13787754c3b3ca91ca
 * ========================================================================== */

dependencies:
  idf: ">=5.4"
  # P4 + C6 SDIO stack pinned to the previously stable local combination.
  # New esp_hosted 2.12.11 asserted during early Hosted task creation on this board.
  espressif/esp_hosted: "1.4.7"
  espressif/esp_wifi_remote: "0.14.5"
  espressif/usb_host_ch34x_vcp: "2.2.1"

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/idf_component.yml
 * ========================================================================== */

/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/camera.h
 * SHA256 (UTF-8 source): 22bbe631136f94cab9509c7643939aefe0597880d9ce4c5a62235848d3dea15c
 * ========================================================================== */

/*
 * OV5647 MIPI CSI Camera Driver - ESP-IDF
 */

#ifndef CAMERA_H
#define CAMERA_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

esp_err_t camera_init(void);
esp_err_t camera_start(void);
esp_err_t camera_stop(void);
uint8_t *camera_get_frame(size_t *out_len, uint32_t timeout_ms);
bool camera_is_ready(void);
uint32_t camera_get_width(void);
uint32_t camera_get_height(void);

#endif

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/camera.h
 * ========================================================================== */

/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/camera.c
 * SHA256 (UTF-8 source): 00ee4db63c704c7c1b6def478ba0e9ac34da3bcc335262a2f37774271065f392
 * ========================================================================== */

/*
 * OV5647 MIPI CSI Camera Driver - ESP-IDF
 * XCLK: GPIO40 (P4 clock router)  I2C: GPIO7(SDA), GPIO8(SCL)
 * 1920x1080 or 1280x720, PSRAM对齐缓冲
 *
 * 流程: LDO → XCLK → I2C → SCCB → 检测传感器 → 设置格式 → 启动流 → CSI → ISP
 */

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_cam_sensor_xclk.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cam_sensor_types.h"
#include "esp_sccb_i2c.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_ldo_regulator.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "esp_heap_caps.h"
#include "camera.h"

static const char *TAG = "CAM";

// 引脚
#define CAM_I2C_PORT        0
#define CAM_I2C_SDA_PIN     7
#define CAM_I2C_SCL_PIN     8
#define CAM_I2C_FREQ        100000
#define CAM_XCLK_PIN        40
#define CAM_XCLK_FREQ       24000000
#define CAM_CSI_LDO_CHAN    3
#define CAM_CSI_LDO_MV      2500
#define CAM_CSI_DATA_LANES  2

// 状态
static esp_cam_ctlr_handle_t s_cam_handle = NULL;
static esp_cam_sensor_xclk_handle_t s_xclk_handle = NULL;
static esp_ldo_channel_handle_t s_ldo_handle = NULL;
static isp_proc_handle_t s_isp_handle = NULL;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static esp_sccb_io_handle_t s_sccb_handle = NULL;
static esp_cam_sensor_device_t *s_sensor_dev = NULL;
static uint8_t *s_frame_buffer = NULL;
static size_t s_frame_size = 0;
static SemaphoreHandle_t s_frame_ready = NULL;
static uint8_t *s_latest_frame = NULL;
static size_t s_latest_frame_len = 0;
static bool s_initialized = false;
static uint32_t s_width = 0;
static uint32_t s_height = 0;

// CSI回调
static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle,
                                        esp_cam_ctlr_trans_t *trans, void *user_data)
{
    trans->buffer = s_frame_buffer;
    trans->buflen = s_frame_size;
    return false;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle,
                                         esp_cam_ctlr_trans_t *trans, void *user_data)
{
    s_latest_frame = (uint8_t *)trans->buffer;
    s_latest_frame_len = trans->received_size;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_frame_ready, &xHigherPriorityTaskWoken);
    // 注意: IRAM_ATTR回调里不能用ESP_LOG, 但可以设置标记
    return xHigherPriorityTaskWoken;
}

// 检测传感器
static esp_cam_sensor_device_t *detect_sensor(esp_cam_sensor_config_t *cfg)
{
    esp_cam_sensor_detect_fn_t *start = NULL;
    esp_cam_sensor_detect_fn_t *end = NULL;
    esp_cam_sensor_detect_get_array(&start, &end);

    for (esp_cam_sensor_detect_fn_t *fn = start; fn < end; fn++) {
        if (fn->detect) {
            esp_cam_sensor_device_t *dev = fn->detect(cfg);
            if (dev) {
                ESP_LOGI(TAG, "Sensor detected via: port=%d, addr=0x%02X", fn->port, fn->sccb_addr);
                return dev;
            }
        }
    }
    return NULL;
}

esp_err_t camera_init(void)
{
    if (s_initialized) return ESP_OK;
    esp_err_t ret;

    s_frame_ready = xSemaphoreCreateBinary();
    ESP_LOGI(TAG, "Initializing camera...");

    // Step 1: LDO 2.5V for MIPI PHY
    esp_ldo_channel_config_t ldo_cfg = { .chan_id = CAM_CSI_LDO_CHAN, .voltage_mv = CAM_CSI_LDO_MV };
    ret = esp_ldo_acquire_channel(&ldo_cfg, &s_ldo_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "LDO failed: 0x%x", ret); return ret; }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 2: P4 dedicated clock router produces 24 MHz XCLK. This avoids
    // consuming LEDC resources needed by future actuator PWM.
    esp_cam_sensor_xclk_config_t xclk_cfg = {
        .esp_clock_router_cfg = {
            .xclk_freq_hz = CAM_XCLK_FREQ,
            .xclk_pin = CAM_XCLK_PIN,
        },
    };
    ret = esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &s_xclk_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "XCLK router alloc failed: 0x%x", ret); return ret; }
    ret = esp_cam_sensor_xclk_start(s_xclk_handle, &xclk_cfg);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "XCLK router start failed: 0x%x", ret); return ret; }
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "XCLK router 24MHz OK");

    // Step 3: I2C bus
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = CAM_I2C_PORT, .sda_io_num = CAM_I2C_SDA_PIN, .scl_io_num = CAM_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .intr_priority = 0, .trans_queue_depth = 0, .flags = { .enable_internal_pullup = 1 },
    };
    ret = i2c_new_master_bus(&i2c_cfg, &s_i2c_bus);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "I2C failed: 0x%x", ret); return ret; }

    // Step 4: SCCB I2C IO (sensor I2C handle)
    sccb_i2c_config_t sccb_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,  // OV5647 default
        .scl_speed_hz = CAM_I2C_FREQ,
        .addr_bits_width = 16,   // OV5647 uses 16-bit register addresses
        .val_bits_width = 8,     // 8-bit register values
    };
    ret = sccb_new_i2c_io(s_i2c_bus, &sccb_cfg, &s_sccb_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "SCCB init failed: 0x%x", ret); return ret; }

    // Step 5: 检测传感器
    esp_cam_sensor_config_t sensor_cfg = {
        .sccb_handle = s_sccb_handle,
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,       // XCLK is configured separately through P4 clock router
        .xclk_freq_hz = 0,
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };
    s_sensor_dev = detect_sensor(&sensor_cfg);
    if (!s_sensor_dev) {
        ESP_LOGE(TAG, "No camera sensor detected!");
        ESP_LOGI(TAG, "Check: CSI cable, XCLK(GPIO40), I2C(GPIO7/8)");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Sensor: %s", esp_cam_sensor_get_name(s_sensor_dev));

    // Step 6: 查询格式，优先选1080p
    esp_cam_sensor_format_array_t fmt_array = {0};
    ret = esp_cam_sensor_query_format(s_sensor_dev, &fmt_array);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Query format failed: 0x%x", ret);
        return ret;
    }

    const esp_cam_sensor_format_t *selected_fmt = NULL;
    ESP_LOGI(TAG, "Available formats (%" PRIu32 "):", fmt_array.count);
    for (int i = 0; i < fmt_array.count; i++) {
        const esp_cam_sensor_format_t *f = &fmt_array.format_array[i];
        ESP_LOGI(TAG, "  [%d] %s %dx%d", i, f->name ? f->name : "?", f->width, f->height);
        // 优先1080p
        if (f->height == 1080 && !selected_fmt) selected_fmt = f;
    }
    // fallback to 720p
    if (!selected_fmt) {
        for (int i = 0; i < fmt_array.count; i++) {
            if (fmt_array.format_array[i].height == 720) {
                selected_fmt = &fmt_array.format_array[i];
                break;
            }
        }
    }
    // fallback to first
    if (!selected_fmt && fmt_array.count > 0) {
        selected_fmt = &fmt_array.format_array[0];
    }

    if (!selected_fmt) {
        ESP_LOGE(TAG, "No format available!");
        return ESP_ERR_NOT_FOUND;
    }

    s_width = selected_fmt->width;
    s_height = selected_fmt->height;
    ESP_LOGI(TAG, "Selected: %s %" PRIu32 "x%" PRIu32, selected_fmt->name ? selected_fmt->name : "?", s_width, s_height);

    ret = esp_cam_sensor_set_format(s_sensor_dev, selected_fmt);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Set format failed: 0x%x", ret); return ret; }

    // Step 7: 启动传感器数据流
    bool stream_on = true;
    ret = esp_cam_sensor_ioctl(s_sensor_dev, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_on);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Start stream failed: 0x%x", ret); return ret; }
    ESP_LOGI(TAG, "Sensor streaming ON");

    // Step 8: CSI控制器
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id = 0, .h_res = s_width, .v_res = s_height,
        .data_lane_num = CAM_CSI_DATA_LANES, .lane_bit_rate_mbps = 200,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8, .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .queue_items = 1, .bk_buffer_dis = true, .byte_swap_en = false,
    };
    ret = esp_cam_new_csi_ctlr(&csi_cfg, &s_cam_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "CSI failed: 0x%x", ret); return ret; }

    esp_cam_ctlr_evt_cbs_t cbs = { .on_get_new_trans = on_get_new_trans, .on_trans_finished = on_trans_finished };
    esp_cam_ctlr_register_event_callbacks(s_cam_handle, &cbs, NULL);

    // Step 9: 帧缓冲 (PSRAM, 64字节对齐)
    s_frame_size = s_width * s_height * 2;
    s_frame_buffer = heap_caps_aligned_alloc(64, s_frame_size, MALLOC_CAP_SPIRAM);
    if (!s_frame_buffer) s_frame_buffer = heap_caps_malloc(s_frame_size, MALLOC_CAP_SPIRAM);
    if (!s_frame_buffer) { ESP_LOGE(TAG, "Buffer alloc failed! (%zuKB)", s_frame_size/1024); return ESP_ERR_NO_MEM; }
    memset(s_frame_buffer, 0, s_frame_size);
    ESP_LOGI(TAG, "Buffer: %zuKB @ %p (%" PRIu32 "x%" PRIu32 " RGB565)", s_frame_size/1024, s_frame_buffer, s_width, s_height);

    // Step 10: ISP (RAW8 → RGB565)
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80000000, .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8, .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false, .has_line_end_packet = false,
        .h_res = s_width, .v_res = s_height,
    };
    ret = esp_isp_new_processor(&isp_cfg, &s_isp_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "ISP failed: 0x%x", ret); return ret; }
    esp_isp_enable(s_isp_handle);

    // Step 11: 使能CSI
    esp_cam_ctlr_enable(s_cam_handle);

    s_initialized = true;
    ESP_LOGI(TAG, "Camera ready! %" PRIu32 "x%" PRIu32, s_width, s_height);
    return ESP_OK;
}

esp_err_t camera_start(void) { return s_initialized ? esp_cam_ctlr_start(s_cam_handle) : ESP_ERR_INVALID_STATE; }
esp_err_t camera_stop(void) { return s_initialized ? esp_cam_ctlr_stop(s_cam_handle) : ESP_ERR_INVALID_STATE; }

uint8_t *camera_get_frame(size_t *out_len, uint32_t timeout_ms)
{
    if (!s_initialized) return NULL;
    if (timeout_ms > 0 && xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return NULL;
    if (out_len) *out_len = s_latest_frame_len;
    return s_latest_frame;
}

bool camera_is_ready(void) { return s_initialized; }
uint32_t camera_get_width(void) { return s_width; }
uint32_t camera_get_height(void) { return s_height; }

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/camera.c
 * ========================================================================== */

/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/imu.h
 * SHA256 (UTF-8 source): 070cd5a5840006e2e48d942d435d14280d150b47c2c57f3dbdff2c36e3358574
 * ========================================================================== */

#ifndef IMU_H
#define IMU_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    int64_t timestamp_us;
    uint8_t index;
    float yaw;
    float pitch;
    float roll;
    int16_t raw_ax_mg;
    int16_t raw_ay_mg;
    int16_t raw_az_mg;
    bool valid;
} imu_data_t;

typedef struct {
    uint32_t valid_packet_count;
    uint32_t checksum_error_count;
    uint32_t uart_timeout_count;
    int64_t  last_valid_timestamp_us;
    uint32_t rx_overflow_count;
    uint32_t short_packet_count;
    uint32_t bad_header_count;
    uint32_t sync_fail_count;
    uint32_t rvc_header_count;
    uint32_t rvc_candidate_count;
    uint32_t rvc_resync_replay_count;
} imu_health_t;

esp_err_t imu_init(void);
esp_err_t imu_read(imu_data_t *data);
bool imu_is_ready(void);
void imu_get_health(imu_health_t *health);

QueueHandle_t imu_get_sample_queue(void);

#endif

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/imu.h
 * ========================================================================== */

/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/imu.c
 * SHA256 (UTF-8 source): c383df64ce7a5e7bf883343d89711a672aacf668d2b23ddc80bf2d4e0fbdfdb1
 * ========================================================================== */

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

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/imu.c
 * ========================================================================== */

/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/mega_sr04.h
 * SHA256 (UTF-8 source): 10cbdeef5fb9494db4b7c7285e6738a221b64095164bcbdd6654d5aa897d9077
 * ========================================================================== */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Single authoritative compile-time gate for motion-capable IMU commands. */
#ifndef MEGA_IMU_CONTROL_ENABLED
#define MEGA_IMU_CONTROL_ENABLED 1
#endif

typedef struct {
    bool mega_connected;
    bool valid;
    int32_t distance_mm;
    uint32_t pulse_us;
    uint32_t sequence;
    int64_t timestamp_us;
    int64_t sample_age_us;
    uint32_t request_count;
    uint32_t valid_count;
    uint32_t parse_error_count;
    uint32_t rx_byte_count;
    uint32_t rx_line_count;
    int32_t last_error;
    bool actuator_sensor_valid[4];
    int32_t actuator_sensor_mm[4];
    uint32_t actuator_sensor_version[4];
    char status[24];
    char last_rx_line[96];
} mega_sr04_reading_t;

/** IMU controller state parsed from Mega status lines. */
typedef struct {
    bool connected;
    bool armed;
    bool active;
    bool estop;
    bool motion_enabled;
    uint32_t heartbeat_seq;
    uint32_t fault_count;
    int16_t target_delta_mm[4];   /* solver order FL,FR,RL,RR */
    int16_t applied_delta_mm[4];  /* solver order FL,FR,RL,RR */
    bool remote_active;
    bool ps2_connected;
    int8_t remote_preset;
    uint16_t remote_target_mm[4]; /* physical order A1,A2,A3,A4 */
    char remote_source[8];
    char remote_submode[8];
    char mega_mode[16];           /* LOCKED, REMOTE_*, ARMED, ACTIVE, FAULT, ESTOP */
    char fault_reason[32];
    int64_t timestamp_us;
} mega_imu_state_t;

/** Start P4 USB Host -> CH340 -> Mega SR04 polling. Never controls actuators. */
esp_err_t mega_sr04_init(void);

/** Copies the newest parsed SR04 record; safe before the first valid sample. */
void mega_sr04_get_reading(mega_sr04_reading_t *out);

/**
 * Send four solver axial lengths through P4 USB Host -> CH340 -> Mega.
 * Inputs must be FL/FR/RL/RR in [250, 390] mm. Mega maps each to its verified
 * VL53L1X coordinate by `sensor_target_mm = solver_length_mm - 190`.
 * This writes only the line `SET_TARGET FL FR RL RR\\n`; it never operates P4 PWM.
 */
esp_err_t mega_solver_send_targets(uint16_t fl_mm, uint16_t fr_mm,
                                   uint16_t rl_mm, uint16_t rr_mm);

/**
 * IMU controller command API. All commands serialize to the same CH340 TX mutex.
 * These are gated by mega_imu_control_enabled (compile-time lock, default off).
 */

/** Check whether motion-capable IMU commands are compile-time enabled. */
bool mega_imu_control_is_enabled(void);

/** Send ARM REMOTE command; enables the faithful PS2/manual/preset owner. */
esp_err_t mega_remote_arm(void);

/** Send SET_REMOTE <seq> <FL> <FR> <RL> <RR>, absolute VL53 mm in [0,400]. */
esp_err_t mega_remote_set_absolute(uint32_t seq,
                                   uint16_t fl, uint16_t fr,
                                   uint16_t rl, uint16_t rr);

/** Send ARM IMU command. Returns ESP_OK on success. */
esp_err_t mega_imu_arm(void);

/** Send STOP command. Returns ESP_OK on success. */
esp_err_t mega_imu_stop(void);

/** Send ESTOP command. Returns ESP_OK on success. */
esp_err_t mega_imu_estop(void);

/** Send HEARTBEAT <seq> command. Returns ESP_OK on success. */
esp_err_t mega_imu_heartbeat(uint32_t seq);

/**
 * Send SET_DELTA <seq> <FL> <FR> <RL> <RR> command.
 * Deltas are signed mm corrections in solver order.
 * Protocol deltas must be in [-350, +350]. Mega applies the authoritative
 * per-leg ARM-baseline + delta check against its live 50..400 mm VL53 range.
 */
esp_err_t mega_imu_set_delta(uint32_t seq,
                             int16_t fl, int16_t fr,
                             int16_t rl, int16_t rr);

/** Get the latest parsed IMU controller state. */
void mega_imu_get_state(mega_imu_state_t *out);

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/mega_sr04.h
 * ========================================================================== */

/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/mega_sr04.c
 * SHA256 (UTF-8 source): 7314b9aa3ba7b862ed87617b2b7be7f41a09098fdbabdd9ed23d7508c73b2099
 * ========================================================================== */

/*
 * P4 USB Host -> CH340 -> Mega serial bridge.
 *
 * Parses structured status lines from Mega (SR04, LANDER_STATUS, IMU state).
 * Provides command forwarding for both legacy SET_TARGET and new IMU controller
 * protocol (ARM IMU, HEARTBEAT, SET_DELTA, STOP, ESTOP).
 * All IMU control commands are gated by a compile-time lock (default off).
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.h"
#include "mega_sr04.h"

#define MEGA_SR04_POLL_MS       200
#define MEGA_SR04_STALE_US      1000000LL
#define MEGA_SR04_RX_LINE_MAX   512
#define MEGA_IMU_CMD_TIMEOUT_MS 500

static mega_imu_state_t s_imu_state = {0};

#ifndef MEGA_IMU_CONTROL_ENABLED
#define MEGA_IMU_CONTROL_ENABLED 0
#endif

bool mega_imu_control_is_enabled(void)
{
    return MEGA_IMU_CONTROL_ENABLED != 0;
}

static const char *TAG = "MEGA_SR04";
static cdc_acm_dev_hdl_t s_mega = NULL;
static volatile bool s_mega_connected = false;
static volatile bool s_solver_target_active = false;
static volatile esp_err_t s_last_error = ESP_ERR_NOT_FOUND;
static mega_sr04_reading_t s_reading = {
    .distance_mm = -1,
    .last_error = ESP_ERR_NOT_FOUND,
    .status = "not_connected",
};
static SemaphoreHandle_t s_mutex = NULL;
static SemaphoreHandle_t s_tx_mutex = NULL;
static char s_rx_line[MEGA_SR04_RX_LINE_MAX];
static size_t s_rx_len = 0;

static void reading_set_status(const char *status, esp_err_t error)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        snprintf(s_reading.status, sizeof(s_reading.status), "%s", status);
        s_reading.last_error = error;
        s_reading.mega_connected = s_mega_connected;
        xSemaphoreGive(s_mutex);
    }
}

/** Serialize all P4-to-Mega lines: SR04 polling and solver commands share CH340. */
static esp_err_t mega_write_line(const char *line, uint32_t timeout_ms)
{
    if (!line || !s_tx_mutex || !s_mega_connected || !s_mega) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t ret = cdc_acm_host_data_tx_blocking(s_mega, (const uint8_t *)line,
                                                   strlen(line), timeout_ms);
    xSemaphoreGive(s_tx_mutex);
    return ret;
}

static void parse_sr04_line(const char *line)
{
    unsigned long seq = 0, timestamp_ms = 0, pulse = 0;
    long distance = -1;
    char status[24] = {0};
    int fields = sscanf(line, "SR04 seq=%lu t_ms=%lu distance_mm=%ld pulse_us=%lu status=%23s",
                        &seq, &timestamp_ms, &distance, &pulse, status);
    if (fields != 5) {
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_reading.parse_error_count++;
            snprintf(s_reading.status, sizeof(s_reading.status), "parse_error");
            s_reading.last_error = ESP_FAIL;
            xSemaphoreGive(s_mutex);
        }
        ESP_LOGW(TAG, "unparseable Mega line: %s", line);
        return;
    }

    bool valid = (strcmp(status, "ok") == 0 && distance >= 20 && distance <= 4000);
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_reading.sequence = (uint32_t)seq;
        s_reading.distance_mm = (int32_t)distance;
        s_reading.pulse_us = (uint32_t)pulse;
        s_reading.valid = valid;
        s_reading.timestamp_us = esp_timer_get_time();
        snprintf(s_reading.status, sizeof(s_reading.status), "%s", status);
        s_reading.last_error = ESP_OK;
        s_reading.mega_connected = s_mega_connected;
        if (valid) s_reading.valid_count++;
        xSemaphoreGive(s_mutex);
    }
}

static void parse_lander_status_line(const char *line)
{
    long mm[4] = {-1, -1, -1, -1};
    unsigned valid[4] = {0, 0, 0, 0};
    unsigned long version[4] = {0, 0, 0, 0};
    int fields = sscanf(line,
        "LANDER_STATUS mode=DIAGNOSTIC_MOTION_LOCKED "
        "A1_mm=%ld A1_valid=%u A1_ver=%lu "
        "A2_mm=%ld A2_valid=%u A2_ver=%lu "
        "A3_mm=%ld A3_valid=%u A3_ver=%lu "
        "A4_mm=%ld A4_valid=%u A4_ver=%lu",
        &mm[0], &valid[0], &version[0], &mm[1], &valid[1], &version[1],
        &mm[2], &valid[2], &version[2], &mm[3], &valid[3], &version[3]);
    if (fields != 12) {
        reading_set_status("lander_parse_error", ESP_FAIL);
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        for (size_t i = 0; i < 4; ++i) {
            s_reading.actuator_sensor_mm[i] = (int32_t)mm[i];
            s_reading.actuator_sensor_valid[i] = valid[i] != 0;
            s_reading.actuator_sensor_version[i] = (uint32_t)version[i];
        }
        s_reading.mega_connected = s_mega_connected;
        xSemaphoreGive(s_mutex);
    }
}

/** Parse IMU controller LANDER_STATUS lines.
 *  Format: LANDER_STATUS mode=DISARMED|ARMED|ACTIVE|STOPPED|ESTOP
 *          estop=0 faults=N hb_seq=N
 *          A1_mm=N A1_valid=N A1_ver=N ... A4_mm/N/ver
 *          target_FL=N target_FR=N target_RL=N target_RR=N
 *          applied_FL=N applied_FR=N applied_RL=N applied_RR=N
 *          out_A1=IDLE out_A2=IDLE out_A3=IDLE out_A4=IDLE
 */
static void parse_imu_status_line(const char *line)
{
    char mode[16] = {0}, fault[32] = {0};
    unsigned estop = 0, faults = 0;
    unsigned long seq = 0;
    long mm[4] = {-1, -1, -1, -1};
    unsigned valid[4] = {0, 0, 0, 0};
    unsigned long version[4] = {0, 0, 0, 0};
    char direction[4][9] = {{0}};
    unsigned verified[4] = {0, 0, 0, 0};
    int target[4] = {0, 0, 0, 0}, applied[4] = {0, 0, 0, 0};
    unsigned remote_active = 0, ps2_connected = 0;
    int remote_preset = -1;
    unsigned remote_target[4] = {0, 0, 0, 0};
    char remote_source[8] = "none", remote_submode[8] = "manual";

    int fields = sscanf(line,
        "IMU_STATUS mode=%15s estop=%u faults=%u fault=%31s seq=%lu "
        "A1=%ld,%u,%lu,%8[^,],%u A2=%ld,%u,%lu,%8[^,],%u "
        "A3=%ld,%u,%lu,%8[^,],%u A4=%ld,%u,%lu,%8[^,],%u "
        "target=%d,%d,%d,%d applied=%d,%d,%d,%d",
        mode, &estop, &faults, fault, &seq,
        &mm[0], &valid[0], &version[0], direction[0], &verified[0],
        &mm[1], &valid[1], &version[1], direction[1], &verified[1],
        &mm[2], &valid[2], &version[2], direction[2], &verified[2],
        &mm[3], &valid[3], &version[3], direction[3], &verified[3],
        &target[0], &target[1], &target[2], &target[3],
        &applied[0], &applied[1], &applied[2], &applied[3]);
    if (fields != 33) {
        reading_set_status("imu_parse_error", ESP_FAIL);
        return;
    }

    const char *remote = strstr(line, " remote_active=");
    if (remote) {
        (void)sscanf(remote,
            " remote_active=%u remote_source=%7s remote_preset=%d "
            "remote_target=%u,%u,%u,%u ps2_connected=%u remote_submode=%7s",
            &remote_active, remote_source, &remote_preset,
            &remote_target[0], &remote_target[1], &remote_target[2], &remote_target[3],
            &ps2_connected, remote_submode);
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        for (size_t i = 0; i < 4; ++i) {
            s_reading.actuator_sensor_mm[i] = (int32_t)mm[i];
            s_reading.actuator_sensor_valid[i] = valid[i] != 0;
            s_reading.actuator_sensor_version[i] = (uint32_t)version[i];
            s_imu_state.target_delta_mm[i] = (int16_t)target[i];
            s_imu_state.applied_delta_mm[i] = (int16_t)applied[i];
        }
        int64_t now = esp_timer_get_time();
        s_reading.mega_connected = s_mega_connected;
        s_imu_state.connected = s_mega_connected;
        bool remote_owner = strcmp(mode, "REMOTE_MANUAL") == 0 || strcmp(mode, "REMOTE_PRESET") == 0;
        s_imu_state.armed = remote_owner || strcmp(mode, "ARMED") == 0 || strcmp(mode, "ACTIVE") == 0;
        s_imu_state.active = remote_active != 0 || strcmp(mode, "ACTIVE") == 0;
        s_imu_state.estop = estop != 0;
        s_imu_state.motion_enabled = s_imu_state.active;
        s_imu_state.remote_active = remote_active != 0;
        s_imu_state.ps2_connected = ps2_connected != 0;
        s_imu_state.remote_preset = (int8_t)remote_preset;
        for (size_t i = 0; i < 4; ++i) s_imu_state.remote_target_mm[i] = (uint16_t)remote_target[i];
        snprintf(s_imu_state.remote_source, sizeof(s_imu_state.remote_source), "%s", remote_source);
        snprintf(s_imu_state.remote_submode, sizeof(s_imu_state.remote_submode), "%s", remote_submode);
        s_imu_state.heartbeat_seq = (uint32_t)seq;
        s_imu_state.fault_count = (uint32_t)faults;
        snprintf(s_imu_state.mega_mode, sizeof(s_imu_state.mega_mode), "%s", mode);
        snprintf(s_imu_state.fault_reason, sizeof(s_imu_state.fault_reason), "%s", fault);
        s_imu_state.timestamp_us = now;
        xSemaphoreGive(s_mutex);
    }
}

static bool mega_rx_callback(const uint8_t *data, size_t data_len, void *user_arg)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_reading.rx_byte_count += (uint32_t)data_len;
        xSemaphoreGive(s_mutex);
    }
    for (size_t i = 0; i < data_len; ++i) {
        char ch = (char)data[i];
        if (ch == '\r') continue;
        if (ch == '\n') {
            s_rx_line[s_rx_len] = '\0';
            if (s_rx_len > 0 && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                s_reading.rx_line_count++;
                snprintf(s_reading.last_rx_line, sizeof(s_reading.last_rx_line), "%.95s", s_rx_line);
                xSemaphoreGive(s_mutex);
            }
            if (s_rx_len > 0 && strncmp(s_rx_line, "SR04 ", 5) == 0) {
                parse_sr04_line(s_rx_line);
            } else if (s_rx_len > 0 && strncmp(s_rx_line, "IMU_STATUS ", 11) == 0) {
                parse_imu_status_line(s_rx_line);
            } else if (s_rx_len > 0 && strncmp(s_rx_line, "LANDER_STATUS ", 14) == 0) {
                parse_lander_status_line(s_rx_line);
            }
            s_rx_len = 0;
        } else if (s_rx_len < sizeof(s_rx_line) - 1) {
            s_rx_line[s_rx_len++] = ch;
        } else {
            s_rx_len = 0;
            reading_set_status("line_too_long", ESP_ERR_INVALID_SIZE);
        }
    }
    return true;
}

static void mega_event_callback(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        s_mega_connected = false;
        s_mega = NULL;
        s_last_error = ESP_ERR_NOT_FOUND;
        reading_set_status("disconnected", s_last_error);
    } else if (event->type == CDC_ACM_HOST_ERROR) {
        s_last_error = event->data.error;
        reading_set_status("usb_error", s_last_error);
    }
}

static void usb_lib_task(void *arg)
{
    while (true) {
        uint32_t flags = 0;
        esp_err_t ret = usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (ret != ESP_OK) {
            s_last_error = ret;
            reading_set_status("host_error", ret);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void mega_sr04_task(void *arg)
{
    const cdc_acm_host_device_config_t config = {
        .connection_timeout_ms = 3000,
        .out_buffer_size = 128,
        .in_buffer_size = 256,
        .event_cb = mega_event_callback,
        .data_cb = mega_rx_callback,
        .user_arg = NULL,
    };
    const cdc_acm_line_coding_t line = {
        .dwDTERate = 115200,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 8,
    };
    static const uint8_t request[] = "STATUS\nGET_DISTANCE\n";

    while (true) {
        if (!s_mega_connected) {
            cdc_acm_dev_hdl_t handle = NULL;
            esp_err_t ret = ch34x_vcp_open(CH34X_PID_AUTO, 0, &config, &handle);
            if (ret == ESP_OK) {
                s_mega = handle;
                s_mega_connected = true;
                s_last_error = cdc_acm_host_line_coding_set(s_mega, &line);
                if (s_last_error == ESP_OK) {
                    s_last_error = cdc_acm_host_set_control_line_state(s_mega, true, true);
                }
                if (s_last_error == ESP_OK) {
                    // Same downstream-Mega reset pulse proven by the temporary bridge:
                    // assert reset, release it, then allow boot + TCA/VL53 validation.
                    s_last_error = cdc_acm_host_set_control_line_state(s_mega, false, true);
                    if (s_last_error == ESP_OK) vTaskDelay(pdMS_TO_TICKS(100));
                    if (s_last_error == ESP_OK) {
                        s_last_error = cdc_acm_host_set_control_line_state(s_mega, true, true);
                    }
                    if (s_last_error == ESP_OK) vTaskDelay(pdMS_TO_TICKS(6500));
                }
                reading_set_status(s_last_error == ESP_OK ? "waiting_sample" : "line_config_error", s_last_error);
                ESP_LOGI(TAG, "Mega CH340 connected; STATUS polling active, IMU control=%s",
                         mega_imu_control_is_enabled() ? "ENABLED" : "LOCKED");
            } else if (ret != ESP_ERR_TIMEOUT && ret != ESP_ERR_NOT_FOUND) {
                s_last_error = ret;
                reading_set_status("open_error", ret);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (s_solver_target_active) {
            vTaskDelay(pdMS_TO_TICKS(MEGA_SR04_POLL_MS));
            continue;
        }

        esp_err_t ret = mega_write_line((const char *)request, 500);
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_reading.request_count++;
            s_reading.mega_connected = s_mega_connected;
            xSemaphoreGive(s_mutex);
        }
        if (ret != ESP_OK) {
            s_last_error = ret;
            reading_set_status("request_error", ret);
        }
        vTaskDelay(pdMS_TO_TICKS(MEGA_SR04_POLL_MS));
    }
}

esp_err_t mega_sr04_init(void)
{
    if (s_mutex) return ESP_OK;
    s_mutex = xSemaphoreCreateMutex();
    s_tx_mutex = xSemaphoreCreateMutex();
    if (!s_mutex || !s_tx_mutex) return ESP_ERR_NO_MEM;

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_RETURN_ON_ERROR(usb_host_install(&host_config), TAG, "USB host install failed");
    xTaskCreate(usb_lib_task, "usb_host", 4096, NULL, 5, NULL);

    const cdc_acm_host_driver_config_t cdc_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 4,
        .xCoreID = tskNO_AFFINITY,
        .new_dev_cb = NULL,
    };
    ESP_RETURN_ON_ERROR(cdc_acm_host_install(&cdc_config), TAG, "CDC host install failed");
    xTaskCreate(mega_sr04_task, "mega_sr04", 4096, NULL, 4, NULL);
    return ESP_OK;
}

void mega_sr04_get_reading(mega_sr04_reading_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->distance_mm = -1;
    snprintf(out->status, sizeof(out->status), "not_initialized");
    if (!s_mutex) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        *out = s_reading;
        out->mega_connected = s_mega_connected;
        int64_t now = esp_timer_get_time();
        out->sample_age_us = out->timestamp_us > 0 ? now - out->timestamp_us : -1;
        if (out->valid && out->sample_age_us > MEGA_SR04_STALE_US) {
            out->valid = false;
            snprintf(out->status, sizeof(out->status), "stale");
        }
        xSemaphoreGive(s_mutex);
    }
}

esp_err_t mega_solver_send_targets(uint16_t fl_mm, uint16_t fr_mm,
                                   uint16_t rl_mm, uint16_t rr_mm)
{
    const uint16_t lengths[] = {fl_mm, fr_mm, rl_mm, rr_mm};
    for (size_t i = 0; i < sizeof(lengths) / sizeof(lengths[0]); ++i) {
        if (lengths[i] < 250 || lengths[i] > 390) return ESP_ERR_INVALID_ARG;
    }

    char command[64];
    int length = snprintf(command, sizeof(command), "SET_TARGET %u %u %u %u\n",
                          (unsigned)fl_mm, (unsigned)fr_mm,
                          (unsigned)rl_mm, (unsigned)rr_mm);
    if (length <= 0 || length >= (int)sizeof(command)) return ESP_ERR_INVALID_SIZE;

    esp_err_t ret = mega_write_line(command, 1000);
    if (ret == ESP_OK) {
        s_solver_target_active = true;
        ESP_LOGI(TAG, "solver targets sent FL=%u FR=%u RL=%u RR=%u",
                 (unsigned)fl_mm, (unsigned)fr_mm, (unsigned)rl_mm, (unsigned)rr_mm);
    } else {
        ESP_LOGW(TAG, "solver target send failed: 0x%x", ret);
    }
    return ret;
}

/* ================================================================
 * IMU controller command API
 * ================================================================ */

/* Compile-time IMU command gate is defined in main.c and defaults to 0. */
static esp_err_t imu_send_command(const char *cmd, bool emergency_stop_path)
{
    if (!emergency_stop_path && !mega_imu_control_is_enabled()) return ESP_ERR_NOT_SUPPORTED;
    return mega_write_line(cmd, MEGA_IMU_CMD_TIMEOUT_MS);
}

esp_err_t mega_remote_arm(void)
{
    return imu_send_command("ARM REMOTE\n", false);
}

esp_err_t mega_remote_set_absolute(uint32_t seq,
                                   uint16_t fl, uint16_t fr,
                                   uint16_t rl, uint16_t rr)
{
    const uint16_t values[] = {fl, fr, rl, rr};
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        if (values[i] > 400) return ESP_ERR_INVALID_ARG;
    }
    char cmd[96];
    int len = snprintf(cmd, sizeof(cmd), "SET_REMOTE %lu %u %u %u %u\n",
                       (unsigned long)seq, (unsigned)fl, (unsigned)fr,
                       (unsigned)rl, (unsigned)rr);
    if (seq == 0 || len <= 0 || len >= (int)sizeof(cmd)) return ESP_ERR_INVALID_ARG;
    return imu_send_command(cmd, false);
}

esp_err_t mega_imu_arm(void)
{
    return imu_send_command("ARM IMU\n", false);
}

esp_err_t mega_imu_stop(void)
{
    // STOP/ESTOP must remain available even if the motion compile gate is closed
    // after a fault or during rollback.
    return imu_send_command("STOP\n", true);
}

esp_err_t mega_imu_estop(void)
{
    return imu_send_command("ESTOP\n", true);
}

esp_err_t mega_imu_heartbeat(uint32_t seq)
{
    char cmd[32];
    int len = snprintf(cmd, sizeof(cmd), "HEARTBEAT %lu\n", (unsigned long)seq);
    if (len <= 0 || len >= (int)sizeof(cmd)) return ESP_ERR_INVALID_SIZE;
    return imu_send_command(cmd, false);
}

esp_err_t mega_imu_set_delta(uint32_t seq,
                             int16_t fl, int16_t fr,
                             int16_t rl, int16_t rr)
{
    if (fl < -350 || fl > 350 || fr < -350 || fr > 350 ||
        rl < -350 || rl > 350 || rr < -350 || rr > 350) {
        return ESP_ERR_INVALID_ARG;
    }
    char cmd[96];
    int len = snprintf(cmd, sizeof(cmd), "SET_DELTA %lu %d %d %d %d\n",
                       (unsigned long)seq, (int)fl, (int)fr, (int)rl, (int)rr);
    if (len <= 0 || len >= (int)sizeof(cmd)) return ESP_ERR_INVALID_SIZE;
    return imu_send_command(cmd, false);
}

void mega_imu_get_state(mega_imu_state_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!s_mutex) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        *out = s_imu_state;
        out->connected = s_mega_connected;
        int64_t now = esp_timer_get_time();
        if (out->timestamp_us > 0 && (now - out->timestamp_us) > MEGA_SR04_STALE_US) {
            out->connected = false;
        }
        xSemaphoreGive(s_mutex);
    }
}

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/mega_sr04.c
 * ========================================================================== */

/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/actuator.h
 * SHA256 (UTF-8 source): 11edf38e0c4cda988242c9438db55c99923f1b242ce25dacfc6e899101487477
 * ========================================================================== */

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "esp_err.h"
#include <stdint.h>

typedef enum {
    ACT_FL = 0,
    ACT_FR,
    ACT_RL,
    ACT_RR,
    ACT_COUNT
} actuator_id_t;

typedef enum {
    ACT_STOP = 0,
    ACT_EXTEND,
    ACT_RETRACT
} actuator_dir_t;

esp_err_t actuator_init(void);
esp_err_t actuator_set(actuator_id_t id, actuator_dir_t dir, uint8_t duty);
esp_err_t actuator_set_length(actuator_id_t id, uint32_t length_mm);
esp_err_t actuator_stop_all(void);

#endif

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/actuator.h
 * ========================================================================== */

/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/actuator.c
 * SHA256 (UTF-8 source): 1c51c3d6330539275147d206388d86ae6e8a01fbfdfe54e980567948d7eba95f
 * ========================================================================== */

/*
 * 4-Channel Linear Actuator PWM Control
 * FL: GPIO5/6, FR: GPIO9/10, RL: GPIO11/12, RR: GPIO13/14
 * 1000Hz, 8-bit duty
 */

#include "esp_log.h"
#include "driver/ledc.h"
#include "actuator.h"

static const char *TAG = "ACT";

typedef struct {
    int pin_in1;
    int pin_in2;
    ledc_channel_t ch_in1;
    ledc_channel_t ch_in2;
} act_ch_t;

static const act_ch_t s_acts[ACT_COUNT] = {
    [ACT_FL] = { 5,  6,  LEDC_CHANNEL_0, LEDC_CHANNEL_1 },
    [ACT_FR] = { 9,  10, LEDC_CHANNEL_2, LEDC_CHANNEL_3 },
    [ACT_RL] = { 11, 12, LEDC_CHANNEL_4, LEDC_CHANNEL_5 },
    [ACT_RR] = { 13, 14, LEDC_CHANNEL_6, LEDC_CHANNEL_7 },
};

static bool s_initialized = false;

esp_err_t actuator_init(void)
{
    if (s_initialized) return ESP_OK;

    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Timer failed: 0x%x", ret); return ret; }

    for (int i = 0; i < ACT_COUNT; i++) {
        ledc_channel_config_t ch1 = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = s_acts[i].ch_in1,
            .timer_sel = LEDC_TIMER_1,
            .gpio_num = s_acts[i].pin_in1,
            .duty = 0,
            .intr_type = LEDC_INTR_DISABLE,
        };
        ledc_channel_config(&ch1);

        ledc_channel_config_t ch2 = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = s_acts[i].ch_in2,
            .timer_sel = LEDC_TIMER_1,
            .gpio_num = s_acts[i].pin_in2,
            .duty = 0,
            .intr_type = LEDC_INTR_DISABLE,
        };
        ledc_channel_config(&ch2);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Actuators ready (FL=5/6 FR=9/10 RL=11/12 RR=13/14)");
    return ESP_OK;
}

esp_err_t actuator_set(actuator_id_t id, actuator_dir_t dir, uint8_t duty)
{
    if (!s_initialized || id >= ACT_COUNT) return ESP_ERR_INVALID_STATE;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_acts[id].ch_in1, dir == ACT_EXTEND ? duty : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_acts[id].ch_in1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_acts[id].ch_in2, dir == ACT_RETRACT ? duty : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_acts[id].ch_in2);

    return ESP_OK;
}

esp_err_t actuator_set_length(actuator_id_t id, uint32_t length_mm)
{
    if (!s_initialized || id >= ACT_COUNT) return ESP_ERR_INVALID_STATE;

    if (length_mm < 250) length_mm = 250;
    if (length_mm > 450) length_mm = 450;

    uint8_t duty = (uint8_t)((length_mm - 250) * 255 / 200);

    actuator_set(id, ACT_EXTEND, duty);
    return ESP_OK;
}

esp_err_t actuator_stop_all(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    for (int i = 0; i < ACT_COUNT; i++) {
        actuator_set(i, ACT_STOP, 0);
    }
    return ESP_OK;
}

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/actuator.c
 * ========================================================================== */

/* ============================================================================
 * BEGIN ORIGINAL FILE: wifi_test/main/main.c
 * SHA256 (UTF-8 source): 0a82fe11e0b7133767a5050a3995909f7391bcbd6e3cc23fea219e9577c76ce0
 * ========================================================================== */

/*
 * Mars Lander - Camera + WiFi TCP + HTTP + IMU + Ultrasonic + Actuators
 * 800x800 RGB565 → 灰度 → JPEG → WiFi → PC
 * 三缓冲 + TCP + HTTP server
 */

#include <string.h>
#include <math.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "esp_http_server.h"
#include "camera.h"
#include "imu.h"
#include "mega_sr04.h"
#include "actuator.h"
#include "driver/jpeg_encode.h"

static const char *TAG = "STREAM";

#define WIFI_SSID      "MarsLander"
#define WIFI_PASS      "REPLACE_WITH_YOUR_WIFI_PASSWORD"
// Deployment behavior: keep retrying forever. Windows Mobile Hotspot may start
// after the P4, and DHCP/Wi-Fi can disappear temporarily during operation.
#define WIFI_RETRY_BACKOFF_MIN_MS 1000
#define WIFI_RETRY_BACKOFF_MAX_MS 30000

// The current actuator wiring has no position feedback or verified travel
// limits. Keep it electrically initialized but reject motion commands by
// default; only explicit, future hardware validation may enable this.
#define ACTUATOR_MOTION_ENABLED 0

#define HTTP_PORT      80
#define JPEG_QUALITY   50
#define JPEG_BUF_SIZE  (300 * 1024)
#define SW 800
#define SH 800

// Stage A Mega bring-up: enable USB Host/CH340 read-only diagnostics.
// The Mega diagnostic sketch rejects every motion command and holds EN/IN LOW.
#define MEGA_SR04_ENABLED 1
#define MEGA_ACTUATOR_COMMANDS_ENABLED 0

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static volatile bool s_wifi_connected = false;
static volatile bool s_wifi_started = false;
static uint32_t s_wifi_connect_attempts = 0;
static uint32_t s_wifi_disconnect_count = 0;
static int64_t s_wifi_last_change_us = 0;
static httpd_handle_t s_http_server = NULL;
static bool s_camera_started = false;
static bool s_actuator_ready = false;
static uint32_t s_camera_restart_count = 0;


static uint8_t *s_local_buf = NULL;
static uint8_t *s_gray_a = NULL;
static uint8_t *s_gray_b = NULL;
static uint8_t *s_write_gray = NULL;
static uint8_t *s_send_gray = NULL;
static SemaphoreHandle_t s_frame_ready = NULL;
static SemaphoreHandle_t s_buf_free = NULL;

static jpeg_encoder_handle_t s_jpeg_enc = NULL;
static uint8_t *s_jpeg_buf = NULL;
static size_t s_jpeg_buf_sz = 0;

static imu_data_t s_imu_data;
static SemaphoreHandle_t s_imu_mutex = NULL;
static int32_t s_distance_mm = -1;
static SemaphoreHandle_t s_dist_mutex = NULL;
static char s_ip_str[16] = "0.0.0.0";
static int64_t s_boot_time = 0;
#define VIDEO_META_RING_SIZE 128

typedef struct {
    uint32_t frame_id;
    int64_t  timestamp_us;
    size_t   jpeg_size;
} video_meta_entry_t;

static SemaphoreHandle_t s_video_meta_mutex = NULL;
static video_meta_entry_t s_meta_ring[VIDEO_META_RING_SIZE];
static uint32_t s_ring_head = 0;
static uint32_t s_ring_count = 0;
static int64_t s_jpeg_timestamp_us = 0;
static uint32_t s_video_frame_id = 0;
static size_t s_video_jpeg_size = 0;
static bool s_video_meta_valid = false;

typedef struct {
    int64_t capture_us;
    int64_t copy_us;
    int64_t gray_us;
} cap_timing_t;

static cap_timing_t s_cap_timing;
static SemaphoreHandle_t s_cap_timing_mutex = NULL;

// RGB565 -> BT.601 grayscale lookup tables.  The former inner loop expanded
// each channel to 8 bits with three integer divisions per pixel; at 800x800
// that cost about 87 ms/frame.  These tables preserve the exact original
// arithmetic while replacing divisions with three tiny cache-resident lookups.
// Entries remain in the original pre-shift scale, so their summed result then
// shifted by 8 is bit-identical to the previous expression.
static uint16_t s_gray_r_lut[32];
static uint16_t s_gray_g_lut[64];
static uint16_t s_gray_b_lut[32];

static void init_gray_lut(void)
{
    for (uint32_t r = 0; r < 32; ++r) {
        uint8_t r8 = (uint8_t)(r * 255 / 31);
        s_gray_r_lut[r] = (uint16_t)(77 * r8);
    }
    for (uint32_t g = 0; g < 64; ++g) {
        uint8_t g8 = (uint8_t)(g * 255 / 63);
        s_gray_g_lut[g] = (uint16_t)(150 * g8);
    }
    for (uint32_t b = 0; b < 32; ++b) {
        uint8_t b8 = (uint8_t)(b * 255 / 31);
        s_gray_b_lut[b] = (uint16_t)(29 * b8);
    }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        s_wifi_started = true;
        s_wifi_connected = false;
        s_wifi_last_change_us = esp_timer_get_time();
        ESP_LOGI(TAG, "WiFi station started; network supervisor will connect");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        s_wifi_disconnect_count++;
        s_wifi_last_change_us = esp_timer_get_time();
        snprintf(s_ip_str, sizeof(s_ip_str), "0.0.0.0");
        if (s_wifi_event_group) xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "WiFi disconnected (count=%" PRIu32 "); retrying in background", s_wifi_disconnect_count);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        s_wifi_last_change_us = esp_timer_get_time();
        if (s_wifi_event_group) xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi ready, DHCP IP: %s", s_ip_str);
    }
}

static esp_err_t wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop init failed");
    if (!esp_netif_create_default_wifi_sta()) return ESP_FAIL;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                        &event_handler, NULL, NULL), TAG, "WiFi handler register failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                        &event_handler, NULL, NULL), TAG, "IP handler register failed");

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "WiFi STA mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "WiFi config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "WiFi start failed");
    s_wifi_started = true;
    s_wifi_last_change_us = esp_timer_get_time();
    ESP_LOGI(TAG, "WiFi configured for %s; continuing boot without waiting for hotspot", WIFI_SSID);
    return ESP_OK;
}

#define PC_PORT        5000
// Independent BNO085 JSONL stream. It is intentionally not associated with
// video frames; UART sampling and camera transport must remain independent.
#define IMU_TCP_PORT   5001

static int s_client_sock = -1;

static void network_supervisor_task(void *arg)
{
    uint32_t backoff_ms = WIFI_RETRY_BACKOFF_MIN_MS;
    while (1) {
        if (s_wifi_started && !s_wifi_connected) {
            s_wifi_connect_attempts++;
            esp_err_t ret = esp_wifi_connect();
            if (ret == ESP_OK || ret == ESP_ERR_WIFI_STATE) {
                ESP_LOGI("NET", "connect attempt=%" PRIu32 " backoff=%" PRIu32 "ms",
                         s_wifi_connect_attempts, backoff_ms);
            } else {
                ESP_LOGW("NET", "connect attempt=%" PRIu32 " failed: 0x%x",
                         s_wifi_connect_attempts, ret);
            }
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = backoff_ms < WIFI_RETRY_BACKOFF_MAX_MS / 2
                         ? backoff_ms * 2 : WIFI_RETRY_BACKOFF_MAX_MS;
        } else {
            backoff_ms = WIFI_RETRY_BACKOFF_MIN_MS;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static bool send_all(int sock, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(sock, p + sent, len - sent, 0);
        if (n <= 0) {
            ESP_LOGW(TAG, "send_all failed at %u/%u (n=%d)", (unsigned)sent, (unsigned)len, (int)n);
            return false;
        }
        sent += n;
    }
    return true;
}

static void tcp_server_task(void *arg)
{
    while (1) {
        struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(PC_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
        int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_sock < 0) {
            ESP_LOGE("TCP", "socket failed errno=%d; retrying", errno);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(listen_sock, 1) < 0) {
            ESP_LOGE("TCP", "bind/listen failed errno=%d; retrying", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "TCP video listening on port %d", PC_PORT);

        while (1) {
            struct sockaddr_in ca;
            socklen_t cl = sizeof(ca);
            int sock = accept(listen_sock, (struct sockaddr *)&ca, &cl);
            if (sock < 0) {
                ESP_LOGW("TCP", "accept failed errno=%d; recreating listener", errno);
                break;
            }
            ESP_LOGI(TAG, "Video client connected");
            s_client_sock = sock;
            int flag = 1;
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
            char buf[16];
            while (recv(sock, buf, sizeof(buf), 0) > 0) {}
            close(sock);
            if (s_client_sock == sock) s_client_sock = -1;
            ESP_LOGI("TCP", "Video client disconnected");
        }
        close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void imu_tcp_server_task(void *arg)
{
    while (1) {
        struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(IMU_TCP_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
        int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_sock < 0) {
            ESP_LOGE("IMU_TCP", "socket create failed errno=%d; retrying", errno);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(listen_sock, 1) < 0) {
            ESP_LOGE("IMU_TCP", "bind/listen failed errno=%d; retrying", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "IMU TCP listening on port %d", IMU_TCP_PORT);

        while (1) {
            struct sockaddr_in ca;
            socklen_t cl = sizeof(ca);
            int client = accept(listen_sock, (struct sockaddr *)&ca, &cl);
            if (client < 0) {
                ESP_LOGW("IMU_TCP", "accept failed errno=%d; recreating listener", errno);
                break;
            }

            ESP_LOGI("IMU_TCP", "client connected");
        int flag = 1;
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        int64_t last_sent_timestamp_us = 0;
        while (1) {
            // Latest-sample stream: publish every new valid sample without
            // an artificial rate cap.  TCP backpressure remains handled by
            // send_all() and a failed send disconnects the client.
            imu_data_t sample;
            esp_err_t sample_ret = imu_read(&sample);
            if (sample_ret != ESP_OK || !sample.valid ||
                sample.timestamp_us == last_sent_timestamp_us) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            last_sent_timestamp_us = sample.timestamp_us;

            imu_health_t health;
            imu_get_health(&health);

            char line[560];
            int n = snprintf(line, sizeof(line),
                "{\"timestamp_us\":%lld,\"index\":%u,"
                "\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,"
                "\"raw_accel_mg\":{\"x\":%d,\"y\":%d,\"z\":%d},"
                "\"valid_packet_count\":%lu,\"checksum_error_count\":%lu,"
                "\"uart_timeout_count\":%lu,"
                "\"rx_overflow_count\":%lu,\"short_packet_count\":%lu,"
                "\"bad_header_count\":%lu,\"sync_fail_count\":%lu,"
                "\"rvc_header_count\":%lu,\"rvc_candidate_count\":%lu,"
                "\"rvc_resync_replay_count\":%lu}\n",
                (long long)sample.timestamp_us, (unsigned)sample.index,
                sample.yaw, sample.pitch, sample.roll,
                sample.raw_ax_mg, sample.raw_ay_mg, sample.raw_az_mg,
                (unsigned long)health.valid_packet_count,
                (unsigned long)health.checksum_error_count,
                (unsigned long)health.uart_timeout_count,
                (unsigned long)health.rx_overflow_count,
                (unsigned long)health.short_packet_count,
                (unsigned long)health.bad_header_count,
                (unsigned long)health.sync_fail_count,
                (unsigned long)health.rvc_header_count,
                (unsigned long)health.rvc_candidate_count,
                (unsigned long)health.rvc_resync_replay_count);

            if (!send_all(client, line, n)) {
                ESP_LOGW("IMU_TCP", "send failed, client disconnected");
                break;
            }
        }

        close(client);
            ESP_LOGI("IMU_TCP", "client disconnected, waiting for new connection");
        }
        close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void capture_task(void *arg)
{
    uint32_t fc = 0;
    uint32_t consecutive_camera_timeouts = 0;
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI("CAP", "capture_task started on CPU%d", xPortGetCoreID());
    while (1) {
        size_t len = 0;
        int64_t t_cap0 = esp_timer_get_time();
        uint8_t *frame = camera_get_frame(&len, 5000);
        int64_t t_cap1 = esp_timer_get_time();

        if (frame && len > 0) {
            consecutive_camera_timeouts = 0;
            if (xSemaphoreTake(s_buf_free, pdMS_TO_TICKS(1000)) == pdTRUE) {
                int64_t t_cp0 = esp_timer_get_time();
                memcpy(s_local_buf, frame, SW * SH * 2);
                int64_t t_cp1 = esp_timer_get_time();

                uint16_t *src = (uint16_t *)s_local_buf;
                for (int i = 0; i < SW * SH; i++) {
                    uint16_t px = src[i];
                    s_write_gray[i] = (uint8_t)((
                        s_gray_r_lut[(px >> 11) & 0x1F] +
                        s_gray_g_lut[(px >> 5) & 0x3F] +
                        s_gray_b_lut[px & 0x1F]) >> 8);
                }
                int64_t t_gray1 = esp_timer_get_time();

                if (xSemaphoreTake(s_cap_timing_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    s_cap_timing.capture_us = t_cap1 - t_cap0;
                    s_cap_timing.copy_us = t_cp1 - t_cp0;
                    s_cap_timing.gray_us = t_gray1 - t_cp1;
                    xSemaphoreGive(s_cap_timing_mutex);
                }

                uint8_t *tmp = s_write_gray;
                s_write_gray = s_send_gray;
                s_send_gray = tmp;

                xSemaphoreGive(s_frame_ready);
                fc++;
                if (fc <= 3 || fc % 20 == 0)
                    ESP_LOGI("CAP", "frame %" PRIu32 " len=%u ready", fc, (unsigned)len);
            } else {
                ESP_LOGW("CAP", "buf_free timeout");
            }
        } else {
            consecutive_camera_timeouts++;
            ESP_LOGW("CAP", "camera_get_frame timeout/null #%" PRIu32 " (len=%u)",
                     consecutive_camera_timeouts, (unsigned)len);
            // CSI/ISP recovery has resource-order dependencies. A bounded full
            // reboot is safer than attempting partial teardown while DMA may run.
            if (consecutive_camera_timeouts >= 3) {
                s_camera_restart_count++;
                ESP_LOGE("CAP", "camera stalled for ~15s; rebooting for clean recovery");
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

static void mega_distance_mirror_task(void *arg)
{
    // Mega owns HC-SR04 electrical timing. This task only mirrors the newest
    // USB-delivered sample into the existing HTTP-compatible distance field.
    while (1) {
        mega_sr04_reading_t reading;
        mega_sr04_get_reading(&reading);
        if (xSemaphoreTake(s_dist_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_distance_mm = reading.valid ? reading.distance_mm : -1;
            xSemaphoreGive(s_dist_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void imu_task(void *arg)
{
    int cnt = 0;
    while (1) {
        imu_data_t data;
        esp_err_t ret = imu_read(&data);
        if (ret == ESP_OK && data.valid) {
            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_imu_data = data;
                xSemaphoreGive(s_imu_mutex);
            }
            if (cnt < 20) {
                ESP_LOGI("IMU", "[%3u] YPR=[%+.2f %+.2f %+.2f] acc=[%d %d %d] mg ts=%lld",
                         data.index, data.yaw, data.pitch, data.roll,
                         data.raw_ax_mg, data.raw_ay_mg, data.raw_az_mg,
                         (long long)data.timestamp_us);
                cnt++;
            }
        } else if (cnt < 5) {
            ESP_LOGW("IMU", "read ret=0x%x valid=%d", ret, data.valid);
            cnt++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t json_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    imu_data_t imu;
    int32_t dist;
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        imu = s_imu_data;
        xSemaphoreGive(s_imu_mutex);
    } else {
        memset(&imu, 0, sizeof(imu));
    }
    if (xSemaphoreTake(s_dist_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        dist = s_distance_mm;
        xSemaphoreGive(s_dist_mutex);
    } else {
        dist = -1;
    }

    mega_sr04_reading_t sr04;
    mega_sr04_get_reading(&sr04);
    char buf[512];
    int64_t vid_ts;
    if (xSemaphoreTake(s_video_meta_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        vid_ts = s_jpeg_timestamp_us;
        xSemaphoreGive(s_video_meta_mutex);
    } else {
        vid_ts = 0;
    }
    snprintf(buf, sizeof(buf),
        "{\"attitude\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
        "\"imu\":{\"valid\":%s,\"index\":%d,"
        "\"raw_accel_mg\":{\"x\":%d,\"y\":%d,\"z\":%d},"
        "\"timestamp_us\":%lld},"
        "\"video\":{\"timestamp_us\":%lld},"
        "\"ultrasonic\":{\"distance_mm\":%ld,\"valid\":%s,\"status\":\"%s\","
        "\"sample_age_us\":%lld,\"pulse_us\":%lu,\"mega_connected\":%s}}",
        imu.roll, imu.pitch, imu.yaw,
        imu.valid ? "true" : "false", imu.index,
        imu.raw_ax_mg, imu.raw_ay_mg, imu.raw_az_mg,
        (long long)imu.timestamp_us,
        (long long)vid_ts,
        (long)dist, sr04.valid ? "true" : "false", sr04.status,
        (long long)sr04.sample_age_us, (unsigned long)sr04.pulse_us,
        sr04.mega_connected ? "true" : "false");

    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t attitude_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    imu_data_t imu;
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        imu = s_imu_data;
        xSemaphoreGive(s_imu_mutex);
    } else {
        memset(&imu, 0, sizeof(imu));
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"valid\":%s}",
        imu.roll, imu.pitch, imu.yaw, imu.valid ? "true" : "false");

    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t quaternion_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    imu_data_t imu;
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        imu = s_imu_data;
        xSemaphoreGive(s_imu_mutex);
    } else {
        memset(&imu, 0, sizeof(imu));
    }

    char buf[192];
    snprintf(buf, sizeof(buf),
        "{\"valid\":%s,\"index\":%d,\"timestamp_us\":%lld}",
        imu.valid ? "true" : "false", imu.index,
        (long long)imu.timestamp_us);

    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t calibration_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    imu_data_t imu;
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        imu = s_imu_data;
        xSemaphoreGive(s_imu_mutex);
    } else {
        memset(&imu, 0, sizeof(imu));
    }

    char buf[96];
    snprintf(buf, sizeof(buf),
        "{\"valid\":%s,\"index\":%d}",
        imu.valid ? "true" : "false", imu.index);

    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t temp_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"temp\":0}");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t health_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    imu_health_t imu_health;
    imu_get_health(&imu_health);
    int64_t now_us = esp_timer_get_time();
    int64_t imu_age_us = imu_health.last_valid_timestamp_us > 0
                         ? now_us - imu_health.last_valid_timestamp_us : -1;
    mega_sr04_reading_t sr04;
    mega_sr04_get_reading(&sr04);
    mega_imu_state_t imu_st = {0};
    mega_imu_get_state(&imu_st);
    char buf[2304];
    snprintf(buf, sizeof(buf),
        "{\"boot_uptime_s\":%lld,\"network\":{\"connected\":%s,\"ip\":\"%s\","
        "\"connect_attempts\":%lu,\"disconnect_count\":%lu,\"last_change_us\":%lld},"
        "\"camera\":{\"ready\":%s,\"started\":%s,\"restart_count\":%lu},"
        "\"imu\":{\"ready\":%s,\"sample_age_us\":%lld},"
        "\"ultrasonic\":{\"source\":\"mega_usb\",\"mega_connected\":%s,\"valid\":%s,"
        "\"distance_mm\":%ld,\"pulse_us\":%lu,\"status\":\"%s\",\"sample_age_us\":%lld,"
        "\"request_count\":%lu,\"valid_count\":%lu,\"parse_error_count\":%lu,"
        "\"rx_byte_count\":%lu,\"rx_line_count\":%lu,\"last_error\":%ld},"
        "\"mega_lander\":{\"stage\":\"A_DIAGNOSTIC\",\"motion_enabled\":false,"
        "\"a1\":{\"mm\":%ld,\"valid\":%s,\"version\":%lu},"
        "\"a2\":{\"mm\":%ld,\"valid\":%s,\"version\":%lu},"
        "\"a3\":{\"mm\":%ld,\"valid\":%s,\"version\":%lu},"
        "\"a4\":{\"mm\":%ld,\"valid\":%s,\"version\":%lu}},"
        "\"mega_imu\":{\"connected\":%s,\"armed\":%s,\"active\":%s,\"estop\":%s,"
        "\"mode\":\"%s\",\"fault_reason\":\"%s\",\"heartbeat_seq\":%lu,\"fault_count\":%lu,"
        "\"target_fl\":%d,\"target_fr\":%d,\"target_rl\":%d,\"target_rr\":%d,"
        "\"applied_fl\":%d,\"applied_fr\":%d,\"applied_rl\":%d,\"applied_rr\":%d,"
        "\"remote_active\":%s,\"remote_source\":\"%s\",\"remote_preset\":%d,"
        "\"remote_target_fl\":%u,\"remote_target_fr\":%u,\"remote_target_rl\":%u,\"remote_target_rr\":%u,"
        "\"ps2_connected\":%s,\"remote_submode\":\"%s\","
        "\"control_enabled\":%s},"
        "\"actuator\":{\"initialized\":%s,\"motion_enabled\":%s},"
        "\"services\":{\"http\":%s,\"video_tcp\":true,\"imu_tcp\":true}}",
        (long long)((now_us - s_boot_time) / 1000000),
        s_wifi_connected ? "true" : "false", s_ip_str,
        (unsigned long)s_wifi_connect_attempts, (unsigned long)s_wifi_disconnect_count,
        (long long)s_wifi_last_change_us,
        camera_is_ready() ? "true" : "false", s_camera_started ? "true" : "false",
        (unsigned long)s_camera_restart_count,
        imu_is_ready() ? "true" : "false", (long long)imu_age_us,
        sr04.mega_connected ? "true" : "false", sr04.valid ? "true" : "false",
        (long)sr04.distance_mm, (unsigned long)sr04.pulse_us, sr04.status,
        (long long)sr04.sample_age_us, (unsigned long)sr04.request_count,
        (unsigned long)sr04.valid_count, (unsigned long)sr04.parse_error_count,
        (unsigned long)sr04.rx_byte_count, (unsigned long)sr04.rx_line_count,
        (long)sr04.last_error,
        (long)sr04.actuator_sensor_mm[0], sr04.actuator_sensor_valid[0] ? "true" : "false",
        (unsigned long)sr04.actuator_sensor_version[0],
        (long)sr04.actuator_sensor_mm[1], sr04.actuator_sensor_valid[1] ? "true" : "false",
        (unsigned long)sr04.actuator_sensor_version[1],
        (long)sr04.actuator_sensor_mm[2], sr04.actuator_sensor_valid[2] ? "true" : "false",
        (unsigned long)sr04.actuator_sensor_version[2],
        (long)sr04.actuator_sensor_mm[3], sr04.actuator_sensor_valid[3] ? "true" : "false",
        (unsigned long)sr04.actuator_sensor_version[3],
        imu_st.connected ? "true" : "false",
        imu_st.armed ? "true" : "false",
        imu_st.active ? "true" : "false",
        imu_st.estop ? "true" : "false",
        imu_st.mega_mode,
        imu_st.fault_reason,
        (unsigned long)imu_st.heartbeat_seq,
        (unsigned long)imu_st.fault_count,
        (int)imu_st.target_delta_mm[0], (int)imu_st.target_delta_mm[1],
        (int)imu_st.target_delta_mm[2], (int)imu_st.target_delta_mm[3],
        (int)imu_st.applied_delta_mm[0], (int)imu_st.applied_delta_mm[1],
        (int)imu_st.applied_delta_mm[2], (int)imu_st.applied_delta_mm[3],
        imu_st.remote_active ? "true" : "false", imu_st.remote_source,
        (int)imu_st.remote_preset,
        (unsigned)imu_st.remote_target_mm[3], (unsigned)imu_st.remote_target_mm[2],
        (unsigned)imu_st.remote_target_mm[0], (unsigned)imu_st.remote_target_mm[1],
        imu_st.ps2_connected ? "true" : "false", imu_st.remote_submode,
        mega_imu_control_is_enabled() ? "true" : "false",
        s_actuator_ready ? "true" : "false", ACTUATOR_MOTION_ENABLED ? "true" : "false",
        s_http_server ? "true" : "false");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t board_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    int64_t uptime_s = (esp_timer_get_time() - s_boot_time) / 1000000;
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"board\":\"ESP32-P4\",\"ip\":\"%s\",\"uptime\":%lld}",
        s_ip_str, (long long)uptime_s);
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t imu_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    imu_data_t imu;
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        imu = s_imu_data;
        xSemaphoreGive(s_imu_mutex);
    } else {
        memset(&imu, 0, sizeof(imu));
    }

    imu_health_t health;
    imu_get_health(&health);

    int64_t now_us = esp_timer_get_time();
    int64_t sample_age_us = (health.last_valid_timestamp_us > 0)
                            ? (now_us - health.last_valid_timestamp_us) : -1;

    char buf[720];
    snprintf(buf, sizeof(buf),
        "{\"timestamp_us\":%lld,\"index\":%d,"
        "\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,"
        "\"raw_accel_mg\":{\"x\":%d,\"y\":%d,\"z\":%d},"
        "\"valid\":%s,"
        "\"valid_packet_count\":%lu,\"checksum_error_count\":%lu,"
        "\"uart_timeout_count\":%lu,\"last_valid_timestamp_us\":%lld,"
        "\"sample_age_us\":%lld,"
        "\"rx_overflow_count\":%lu,\"short_packet_count\":%lu,"
        "\"bad_header_count\":%lu,\"sync_fail_count\":%lu,"
        "\"rvc_header_count\":%lu,\"rvc_candidate_count\":%lu,"
        "\"rvc_resync_replay_count\":%lu}",
        (long long)imu.timestamp_us, imu.index,
        imu.yaw, imu.pitch, imu.roll,
        imu.raw_ax_mg, imu.raw_ay_mg, imu.raw_az_mg,
        imu.valid ? "true" : "false",
        (unsigned long)health.valid_packet_count,
        (unsigned long)health.checksum_error_count,
        (unsigned long)health.uart_timeout_count,
        (long long)health.last_valid_timestamp_us,
        (long long)sample_age_us,
        (unsigned long)health.rx_overflow_count,
        (unsigned long)health.short_packet_count,
        (unsigned long)health.bad_header_count,
        (unsigned long)health.sync_fail_count,
        (unsigned long)health.rvc_header_count,
        (unsigned long)health.rvc_candidate_count,
        (unsigned long)health.rvc_resync_replay_count);

    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t video_meta_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char qbuf[32] = {0};
    char fval[16] = {0};
    bool has_fid = false;
    uint32_t req_fid = 0;

    size_t qlen = httpd_req_get_url_query_len(req);
    if (qlen > 0 && qlen < sizeof(qbuf)) {
        if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
            if (httpd_query_key_value(qbuf, "frame_id", fval, sizeof(fval)) == ESP_OK) {
                has_fid = true;
                req_fid = (uint32_t)strtoul(fval, NULL, 10);
            }
        }
    }

    if (has_fid) {
        bool found = false;
        uint32_t fid = 0;
        int64_t ts = 0;
        size_t sz = 0;
        if (xSemaphoreTake(s_video_meta_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (uint32_t i = 0; i < s_ring_count; i++) {
                uint32_t idx = (s_ring_head + VIDEO_META_RING_SIZE - s_ring_count + i) % VIDEO_META_RING_SIZE;
                if (s_meta_ring[idx].frame_id == req_fid) {
                    fid = s_meta_ring[idx].frame_id;
                    ts = s_meta_ring[idx].timestamp_us;
                    sz = s_meta_ring[idx].jpeg_size;
                    found = true;
                    break;
                }
            }
            xSemaphoreGive(s_video_meta_mutex);
        }
        char buf[128];
        if (found) {
            snprintf(buf, sizeof(buf),
                "{\"frame_id\":%lu,\"timestamp_us\":%lld,\"jpeg_size\":%u,\"valid\":true}",
                (unsigned long)fid, (long long)ts, (unsigned)sz);
        } else {
            snprintf(buf, sizeof(buf),
                "{\"frame_id\":%lu,\"timestamp_us\":0,\"jpeg_size\":0,\"valid\":false}",
                (unsigned long)req_fid);
        }
        return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    }

    uint32_t fid;
    int64_t ts;
    size_t sz;
    bool valid;
    if (xSemaphoreTake(s_video_meta_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        fid = s_video_frame_id;
        ts = s_jpeg_timestamp_us;
        sz = s_video_jpeg_size;
        valid = s_video_meta_valid;
        xSemaphoreGive(s_video_meta_mutex);
    } else {
        fid = 0; ts = 0; sz = 0; valid = false;
    }

    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"frame_id\":%lu,\"timestamp_us\":%lld,\"jpeg_size\":%u,\"valid\":%s}",
        (unsigned long)fid, (long long)ts, (unsigned)sz,
        valid ? "true" : "false");

    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t camera_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char buf[96];
    snprintf(buf, sizeof(buf),
        "{\"width\":%d,\"height\":%d,\"fps\":0}", SW, SH);
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ip_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"ip\":\"%s\"}", s_ip_str);
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t uptime_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    int64_t uptime_s = (esp_timer_get_time() - s_boot_time) / 1000000;
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"uptime\":%lld}", (long long)uptime_s);
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[512];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
        return ESP_FAIL;
    }
    buf[len] = '\0';

    char resp[128];

    // The four resolver targets go only to the Mega over the already verified
    // USB Host -> CH340 link. Stage A explicitly compiles this motion path out.
    if (strstr(buf, "\"mega_targets\"")) {
#if !MEGA_ACTUATOR_COMMANDS_ENABLED
        httpd_resp_set_status(req, "423 Locked");
        return httpd_resp_send(req,
            "{\"ok\":false,\"target\":\"mega\",\"error\":\"stage_a_motion_disabled\"}",
            HTTPD_RESP_USE_STRLEN);
#else
        int fl = 0, fr = 0, rl = 0, rr = 0;
        char *p = NULL;
        if ((p = strstr(buf, "\"fl\""))) sscanf(p, "\"fl\":%d", &fl);
        else goto mega_targets_invalid;
        if ((p = strstr(buf, "\"fr\""))) sscanf(p, "\"fr\":%d", &fr);
        else goto mega_targets_invalid;
        if ((p = strstr(buf, "\"rl\""))) sscanf(p, "\"rl\":%d", &rl);
        else goto mega_targets_invalid;
        if ((p = strstr(buf, "\"rr\""))) sscanf(p, "\"rr\":%d", &rr);
        else goto mega_targets_invalid;

        esp_err_t mega_ret = mega_solver_send_targets((uint16_t)fl, (uint16_t)fr,
                                                       (uint16_t)rl, (uint16_t)rr);
        if (mega_ret != ESP_OK) {
            httpd_resp_set_status(req, mega_ret == ESP_ERR_INVALID_ARG ? "400 Bad Request" : "503 Service Unavailable");
            snprintf(resp, sizeof(resp),
                     "{\"ok\":false,\"target\":\"mega\",\"error\":\"0x%x\"}", mega_ret);
        } else {
            snprintf(resp, sizeof(resp),
                     "{\"ok\":true,\"target\":\"mega\",\"fl\":%d,\"fr\":%d,\"rl\":%d,\"rr\":%d}",
                     fl, fr, rl, rr);
        }
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
#endif
    }

#if MEGA_ACTUATOR_COMMANDS_ENABLED
mega_targets_invalid:
    if (strstr(buf, "\"mega_targets\"")) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req,
            "{\"ok\":false,\"target\":\"mega\",\"error\":\"require fl/fr/rl/rr in 250..390\"}",
            HTTPD_RESP_USE_STRLEN);
    }
#endif

    // First-stage faithful Remote owner: PS2 manual/preset plus Console absolute VL53 input.
    if (strstr(buf, "\"remote_arm\"")) {
#if !MEGA_IMU_CONTROL_ENABLED
        httpd_resp_set_status(req, "423 Locked");
        return httpd_resp_send(req,
            "{\"ok\":false,\"error\":\"remote_control_compile_locked\"}",
            HTTPD_RESP_USE_STRLEN);
#else
        esp_err_t ret = mega_remote_arm();
        snprintf(resp, sizeof(resp), "{\"ok\":%s,\"cmd\":\"arm_remote\"}",
                 ret == ESP_OK ? "true" : "false");
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
#endif
    }
    if (strstr(buf, "\"remote_set_absolute\"")) {
#if !MEGA_IMU_CONTROL_ENABLED
        httpd_resp_set_status(req, "423 Locked");
        return httpd_resp_send(req,
            "{\"ok\":false,\"error\":\"remote_control_compile_locked\"}",
            HTTPD_RESP_USE_STRLEN);
#else
        uint32_t seq = 0;
        int fl = 0, fr = 0, rl = 0, rr = 0;
        char *p_seq = strstr(buf, "\"seq\"");
        char *p_fl = strstr(buf, "\"fl\"");
        char *p_fr = strstr(buf, "\"fr\"");
        char *p_rl = strstr(buf, "\"rl\"");
        char *p_rr = strstr(buf, "\"rr\"");
        if (!p_seq || !p_fl || !p_fr || !p_rl || !p_rr ||
            sscanf(p_seq, "\"seq\":%lu", &seq) != 1 ||
            sscanf(p_fl, "\"fl\":%d", &fl) != 1 ||
            sscanf(p_fr, "\"fr\":%d", &fr) != 1 ||
            sscanf(p_rl, "\"rl\":%d", &rl) != 1 ||
            sscanf(p_rr, "\"rr\":%d", &rr) != 1) {
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_send(req,
                "{\"ok\":false,\"error\":\"require_seq_and_fl_fr_rl_rr\"}",
                HTTPD_RESP_USE_STRLEN);
        }
        esp_err_t ret = mega_remote_set_absolute(seq, (uint16_t)fl, (uint16_t)fr,
                                                 (uint16_t)rl, (uint16_t)rr);
        if (ret == ESP_ERR_INVALID_ARG) httpd_resp_set_status(req, "400 Bad Request");
        snprintf(resp, sizeof(resp),
                 "{\"ok\":%s,\"cmd\":\"set_remote\",\"seq\":%lu,\"fl\":%d,\"fr\":%d,\"rl\":%d,\"rr\":%d}",
                 ret == ESP_OK ? "true" : "false", (unsigned long)seq, fl, fr, rl, rr);
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
#endif
    }

    // IMU control commands: arm/stop/estop/heartbeat/set_delta forwarded to Mega.
    if (strstr(buf, "\"imu_arm\"")) {
#if !MEGA_IMU_CONTROL_ENABLED
        httpd_resp_set_status(req, "423 Locked");
        return httpd_resp_send(req,
            "{\"ok\":false,\"error\":\"imu_control_compile_locked\"}",
            HTTPD_RESP_USE_STRLEN);
#else
        esp_err_t ret = mega_imu_arm();
        snprintf(resp, sizeof(resp), "{\"ok\":%s,\"cmd\":\"arm_imu\"}",
                 ret == ESP_OK ? "true" : "false");
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
#endif
    }
    if (strstr(buf, "\"imu_stop\"")) {
        esp_err_t ret = mega_imu_stop();
        snprintf(resp, sizeof(resp), "{\"ok\":%s,\"cmd\":\"stop\"}",
                 ret == ESP_OK ? "true" : "false");
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }
    if (strstr(buf, "\"imu_estop\"")) {
        esp_err_t ret = mega_imu_estop();
        snprintf(resp, sizeof(resp), "{\"ok\":%s,\"cmd\":\"estop\"}",
                 ret == ESP_OK ? "true" : "false");
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }
    if (strstr(buf, "\"imu_heartbeat\"")) {
#if !MEGA_IMU_CONTROL_ENABLED
        httpd_resp_set_status(req, "423 Locked");
        return httpd_resp_send(req,
            "{\"ok\":false,\"error\":\"imu_control_compile_locked\"}",
            HTTPD_RESP_USE_STRLEN);
#else
        uint32_t seq = 0;
        char *p = strstr(buf, "\"seq\"");
        if (p) sscanf(p, "\"seq\":%lu", &seq);
        esp_err_t ret = mega_imu_heartbeat(seq);
        snprintf(resp, sizeof(resp), "{\"ok\":%s,\"cmd\":\"heartbeat\",\"seq\":%lu}",
                 ret == ESP_OK ? "true" : "false", (unsigned long)seq);
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
#endif
    }
    if (strstr(buf, "\"imu_set_delta\"")) {
#if !MEGA_IMU_CONTROL_ENABLED
        httpd_resp_set_status(req, "423 Locked");
        return httpd_resp_send(req,
            "{\"ok\":false,\"error\":\"imu_control_compile_locked\"}",
            HTTPD_RESP_USE_STRLEN);
#else
        uint32_t seq = 0;
        int fl = 0, fr = 0, rl = 0, rr = 0;
        char *p = NULL;
        if ((p = strstr(buf, "\"seq\""))) sscanf(p, "\"seq\":%lu", &seq);
        if ((p = strstr(buf, "\"fl\""))) sscanf(p, "\"fl\":%d", &fl);
        if ((p = strstr(buf, "\"fr\""))) sscanf(p, "\"fr\":%d", &fr);
        if ((p = strstr(buf, "\"rl\""))) sscanf(p, "\"rl\":%d", &rl);
        if ((p = strstr(buf, "\"rr\""))) sscanf(p, "\"rr\":%d", &rr);
        esp_err_t ret = mega_imu_set_delta(seq, (int16_t)fl, (int16_t)fr,
                                           (int16_t)rl, (int16_t)rr);
        if (ret == ESP_ERR_INVALID_ARG) {
            httpd_resp_set_status(req, "400 Bad Request");
            snprintf(resp, sizeof(resp),
                     "{\"ok\":false,\"error\":\"deltas_must_be_-350_to_+350\"}");
        } else {
            snprintf(resp, sizeof(resp),
                     "{\"ok\":%s,\"cmd\":\"set_delta\",\"seq\":%lu,\"fl\":%d,\"fr\":%d,\"rl\":%d,\"rr\":%d}",
                     ret == ESP_OK ? "true" : "false", (unsigned long)seq, fl, fr, rl, rr);
        }
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
#endif
    }

#if !ACTUATOR_MOTION_ENABLED
    if (!strstr(buf, "\"stop\"")) {
        httpd_resp_set_status(req, "423 Locked");
        snprintf(resp, sizeof(resp),
                 "{\"ok\":false,\"error\":\"actuator motion safety-locked\"}");
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }
#endif

    if (strstr(buf, "\"all\"")) {
        int fl = 300, fr = 300, rl = 300, rr = 300;
        sscanf(buf, "{\"all\":%d}", &fl);
        fr = fl; rl = fl; rr = fl;

        char *p;
        if ((p = strstr(buf, "\"fl\""))) sscanf(p, "\"fl\":%d", &fl);
        if ((p = strstr(buf, "\"fr\""))) sscanf(p, "\"fr\":%d", &fr);
        if ((p = strstr(buf, "\"rl\""))) sscanf(p, "\"rl\":%d", &rl);
        if ((p = strstr(buf, "\"rr\""))) sscanf(p, "\"rr\":%d", &rr);

        actuator_set_length(ACT_FL, fl);
        actuator_set_length(ACT_FR, fr);
        actuator_set_length(ACT_RL, rl);
        actuator_set_length(ACT_RR, rr);

        snprintf(resp, sizeof(resp),
            "{\"ok\":true,\"fl\":%d,\"fr\":%d,\"rl\":%d,\"rr\":%d}", fl, fr, rl, rr);
    } else if (strstr(buf, "\"stop\"")) {
        actuator_stop_all();
        snprintf(resp, sizeof(resp), "{\"ok\":true,\"stop\":true}");
    } else {
        int id = -1, len_mm = 300;
        if (strstr(buf, "\"id\"") && strstr(buf, "\"length\"")) {
            sscanf(buf, "{\"id\":%d,\"length\":%d", &id, &len_mm);
            if (id >= 0 && id < ACT_COUNT) {
                esp_err_t ret = actuator_set_length((actuator_id_t)id, len_mm);
                snprintf(resp, sizeof(resp), "{\"ok\":%s,\"id\":%d,\"length\":%d}",
                         ret == ESP_OK ? "true" : "false", id, len_mm);
            } else {
                snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"invalid id\"}");
            }
        } else {
            snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"unknown cmd\"}");
        }
    }

    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_http_server(void)
{
    if (s_http_server) return s_http_server;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_PORT;
    config.stack_size = 8192;
    config.max_uri_handlers = 20;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE("HTTP", "server start failed: 0x%x", ret);
        return NULL;
    }

    httpd_uri_t uri_json =       { .uri = "/json",       .method = HTTP_GET,  .handler = json_handler };
    httpd_uri_t uri_attitude =   { .uri = "/attitude",   .method = HTTP_GET,  .handler = attitude_handler };
    httpd_uri_t uri_quaternion = { .uri = "/quaternion",  .method = HTTP_GET,  .handler = quaternion_handler };
    httpd_uri_t uri_calibration ={ .uri = "/calibration", .method = HTTP_GET,  .handler = calibration_handler };
    httpd_uri_t uri_temp =       { .uri = "/temp",       .method = HTTP_GET,  .handler = temp_handler };
    httpd_uri_t uri_valid =      { .uri = "/valid",      .method = HTTP_GET,  .handler = calibration_handler };
    httpd_uri_t uri_ts =         { .uri = "/ts",         .method = HTTP_GET,  .handler = quaternion_handler };
    httpd_uri_t uri_board =      { .uri = "/board",      .method = HTTP_GET,  .handler = board_handler };
    httpd_uri_t uri_health =     { .uri = "/health",     .method = HTTP_GET,  .handler = health_handler };
    httpd_uri_t uri_imu_ep =     { .uri = "/imu",        .method = HTTP_GET,  .handler = imu_handler };
    httpd_uri_t uri_camera =     { .uri = "/camera",     .method = HTTP_GET,  .handler = camera_handler };
    httpd_uri_t uri_ip =         { .uri = "/ip",         .method = HTTP_GET,  .handler = ip_handler };
    httpd_uri_t uri_uptime =     { .uri = "/uptime",     .method = HTTP_GET,  .handler = uptime_handler };
    httpd_uri_t uri_cmd =        { .uri = "/cmd",        .method = HTTP_POST, .handler = cmd_handler };
    httpd_uri_t uri_video_meta = { .uri = "/video_meta", .method = HTTP_GET,  .handler = video_meta_handler };
    httpd_uri_t *uris[] = {
        &uri_json, &uri_attitude, &uri_quaternion, &uri_calibration, &uri_temp,
        &uri_valid, &uri_ts, &uri_board, &uri_health, &uri_imu_ep, &uri_camera,
        &uri_ip, &uri_uptime, &uri_cmd, &uri_video_meta,
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); ++i) {
        ret = httpd_register_uri_handler(server, uris[i]);
        if (ret != ESP_OK) {
            ESP_LOGE("HTTP", "URI registration failed for %s: 0x%x", uris[i]->uri, ret);
            httpd_stop(server);
            return NULL;
        }
    }

    s_http_server = server;
    ESP_LOGI(TAG, "HTTP listening on port %d", HTTP_PORT);
    return s_http_server;
}

static void http_supervisor_task(void *arg)
{
    while (1) {
        if (!s_http_server && s_wifi_connected) {
            ESP_LOGI("HTTP", "network ready; starting HTTP service");
            start_http_server();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    s_boot_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Mars Lander - Camera + WiFi + IMU UART-RVC + Act");
    ESP_LOGI(TAG, "  Video TCP: port %d (4B LE len + JPEG)", PC_PORT);
    ESP_LOGI(TAG, "  IMU TCP: port %d (independent JSONL sample stream)", IMU_TCP_PORT);
    ESP_LOGI(TAG, "  UART-RVC:  UART%d RX=GPIO%d, %d baud, 19-byte frames",
             1, 3, 115200);
    ESP_LOGI(TAG, "  Timestamps: esp_timer_get_time() us since boot");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // UART-RVC must begin consuming its continuous 100 Hz stream before slow
    // WiFi/camera bring-up. This mirrors the proven standalone imu_test path.
    s_imu_mutex = xSemaphoreCreateMutex();
    if (!s_imu_mutex) {
        ESP_LOGE(TAG, "IMU mutex allocation failed");
        return;
    }
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed: 0x%x (continuing without IMU)", ret);
    }

    ret = wifi_init_sta();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: 0x%x; restarting for bounded recovery", ret);
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    xTaskCreate(network_supervisor_task, "net_super", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "WiFi supervisor active; boot continues while hotspot is offline");

    ret = camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x; restarting in 5 seconds", ret);
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    ret = camera_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera start failed: 0x%x; restarting in 5 seconds", ret);
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    s_camera_started = true;
    ESP_LOGI(TAG, "Camera %dx%d started", SW, SH);

    s_dist_mutex = xSemaphoreCreateMutex();
    s_video_meta_mutex = xSemaphoreCreateMutex();
    s_cap_timing_mutex = xSemaphoreCreateMutex();
    if (!s_dist_mutex || !s_video_meta_mutex || !s_cap_timing_mutex) {
        ESP_LOGE(TAG, "Shared mutex allocation failed");
        return;
    }

    bool mega_sr04_ready = false;
#if MEGA_SR04_ENABLED
    mega_sr04_ready = (mega_sr04_init() == ESP_OK);
    if (!mega_sr04_ready) {
        ESP_LOGW(TAG, "Mega USB HC-SR04 init failed (continuing without height reference)");
    } else {
        ESP_LOGI(TAG, "Mega USB HC-SR04 enabled: Mega D53 TRIG / D52 ECHO");
    }
#else
    ESP_LOGI(TAG, "Mega USB HC-SR04 disabled");
#endif

#if ACTUATOR_MOTION_ENABLED
    ret = actuator_init();
    s_actuator_ready = (ret == ESP_OK);
    if (!s_actuator_ready) {
        ESP_LOGW(TAG, "Actuator init failed: 0x%x (motion remains locked)", ret);
    }
#else
    // GPIO14 is currently C6 SDIO D0, while the legacy RR actuator plan also
    // used GPIO14. Do not initialize any actuator PWM until that physical pin
    // conflict and travel/position safeguards are resolved.
    ESP_LOGW(TAG, "Actuators safety-locked: legacy RR GPIO14 conflicts with C6 SDIO D0");
#endif

    init_gray_lut();
    ESP_LOGI(TAG, "RGB565->gray LUT ready (256 bytes, exact BT.601 arithmetic)");

    s_local_buf = heap_caps_aligned_alloc(64, SW * SH * 2, MALLOC_CAP_SPIRAM);
    s_gray_a = heap_caps_aligned_alloc(64, SW * SH, MALLOC_CAP_SPIRAM);
    s_gray_b = heap_caps_aligned_alloc(64, SW * SH, MALLOC_CAP_SPIRAM);
    s_write_gray = s_gray_a;
    s_send_gray = s_gray_b;
    s_frame_ready = xSemaphoreCreateBinary();
    s_buf_free = xSemaphoreCreateBinary();
    xSemaphoreGive(s_buf_free);

    jpeg_encode_engine_cfg_t jeng = {.intr_priority = 0, .timeout_ms = 1000};
    jpeg_new_encoder_engine(&jeng, &s_jpeg_enc);
    jpeg_encode_memory_alloc_cfg_t jmem = {.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER};
    s_jpeg_buf = (uint8_t *)jpeg_alloc_encoder_mem(JPEG_BUF_SIZE, &jmem, &s_jpeg_buf_sz);

    ESP_LOGI(TAG, "Buffers ready");

    // Keep the 87 ms RGB565→gray workload off CPU1: CPU1 is reserved for
    // the BNO085 UART ISR and high-priority parser.  The main JPEG/TCP loop
    // already runs on CPU0, so pin capture conversion to CPU0 as well rather
    // than starving UART reception during active video streaming.
    // Low-rate height reference task stays on CPU0; CPU1 remains reserved for
    // the UART-RVC ISR/parser.
    if (mega_sr04_ready) xTaskCreatePinnedToCore(mega_distance_mirror_task, "mega_dist", 3072, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(capture_task, "cap", 8192, NULL, 5, NULL, 0);
    xTaskCreate(tcp_server_task, "tcp", 4096, NULL, 3, NULL);
    if (imu_is_ready())       xTaskCreate(imu_task, "imu", 8192, NULL, 4, NULL);
    // Priority 9 remains well below the UART parser (24). This independent
    // JSONL publisher must never participate in JPEG/frame timing or block
    // the RVC parser; a socket failure simply disconnects its client.
    if (imu_is_ready())       xTaskCreatePinnedToCore(imu_tcp_server_task, "imu_tcp", 4096, NULL, 9, NULL, 1);
    xTaskCreate(http_supervisor_task, "http_super", 4096, NULL, 3, NULL);

    vTaskDelay(pdMS_TO_TICKS(2000));
    // esp_log_level_set("*", ESP_LOG_NONE);  // 调试: 保持日志开启

    jpeg_encode_cfg_t enc_cfg = {
        .width = SW, .height = SH,
        .src_type = JPEG_ENCODE_IN_FORMAT_GRAY,
        .sub_sample = JPEG_DOWN_SAMPLING_GRAY,
        .image_quality = JPEG_QUALITY,
    };

    uint32_t fc = 0;
    uint32_t perf_count = 0;
    uint32_t perf_timing_samples = 0;
    uint32_t perf_timing_missed = 0;
    int64_t perf_t0 = esp_timer_get_time();
    int64_t acc_cap = 0, acc_cp = 0, acc_gray = 0, acc_enc = 0, acc_send = 0;

    while (1) {
        if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(5000)) == pdTRUE) {
            int64_t t_enc0 = esp_timer_get_time();
            uint32_t jpeg_size = 0;
            esp_err_t enc_ret = jpeg_encoder_process(s_jpeg_enc, &enc_cfg,
                                    s_send_gray, SW * SH,
                                    s_jpeg_buf, s_jpeg_buf_sz, &jpeg_size);
            int64_t t_enc1 = esp_timer_get_time();

            xSemaphoreGive(s_buf_free);

            bool sent = false;
            int64_t send_us = 0;

            if (enc_ret == ESP_OK && jpeg_size > 0 && s_client_sock >= 0) {
                int64_t t_send0 = esp_timer_get_time();
                uint32_t len_le = jpeg_size;
                bool ok = send_all(s_client_sock, &len_le, 4);
                if (ok) ok = send_all(s_client_sock, s_jpeg_buf, jpeg_size);
                int64_t t_send1 = esp_timer_get_time();

                if (!ok) {
                    close(s_client_sock);
                    s_client_sock = -1;
                } else {
                    sent = true;
                    send_us = t_send1 - t_send0;
                    int64_t ts = esp_timer_get_time();
                    uint32_t frame_id = 0;
                    if (xSemaphoreTake(s_video_meta_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        frame_id = ++s_video_frame_id;
                        s_jpeg_timestamp_us = ts;
                        s_video_jpeg_size = jpeg_size;
                        s_video_meta_valid = true;
                        uint32_t idx = s_ring_head;
                        s_meta_ring[idx].frame_id = frame_id;
                        s_meta_ring[idx].timestamp_us = ts;
                        s_meta_ring[idx].jpeg_size = jpeg_size;
                        s_ring_head = (s_ring_head + 1) % VIDEO_META_RING_SIZE;
                        if (s_ring_count < VIDEO_META_RING_SIZE) s_ring_count++;
                        xSemaphoreGive(s_video_meta_mutex);
                    } else {
                        // Frame ID must remain monotonic even if the HTTP
                        // video-meta endpoint briefly owns the mutex.
                        frame_id = ++s_video_frame_id;
                    }
                }
            }

            fc++;
            if (enc_ret == ESP_OK && jpeg_size > 0) {
                cap_timing_t timing_snap;
                if (xSemaphoreTake(s_cap_timing_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    timing_snap = s_cap_timing;
                    xSemaphoreGive(s_cap_timing_mutex);
                    acc_cap += timing_snap.capture_us;
                    acc_cp += timing_snap.copy_us;
                    acc_gray += timing_snap.gray_us;
                    perf_timing_samples++;
                } else {
                    perf_timing_missed++;
                }
                acc_enc += (t_enc1 - t_enc0);
                acc_send += send_us;
                perf_count++;

                if (perf_count >= 20) {
                    int64_t now = esp_timer_get_time();
                    float fps = perf_count * 1000000.0f / (now - perf_t0);
                    int64_t avg_cap = perf_timing_samples ? acc_cap / perf_timing_samples / 1000 : 0;
                    int64_t avg_cp = perf_timing_samples ? acc_cp / perf_timing_samples / 1000 : 0;
                    int64_t avg_gray = perf_timing_samples ? acc_gray / perf_timing_samples / 1000 : 0;
                    int64_t avg_pipe = perf_timing_samples ? (acc_cap + acc_cp + acc_gray) / perf_timing_samples / 1000 : 0;
                    ESP_LOGI(TAG, "PERF[%" PRIu32 "] fps=%.1f timing_samples=%" PRIu32 " timing_missed=%" PRIu32 " capture=%lld copy=%lld gray=%lld pipeline_total=%lld encode=%lld send=%lld sent=%s",
                        fc, fps, perf_timing_samples, perf_timing_missed,
                        (long long)avg_cap,
                        (long long)avg_cp,
                        (long long)avg_gray,
                        (long long)avg_pipe,
                        (long long)(acc_enc / perf_count / 1000),
                        (long long)(acc_send / perf_count / 1000),
                        sent ? "true" : "false");
                    perf_count = 0;
                    perf_timing_samples = 0;
                    perf_timing_missed = 0;
                    acc_cap = 0; acc_cp = 0; acc_gray = 0; acc_enc = 0; acc_send = 0;
                    perf_t0 = now;
                }
            }
        }
    }
}

/* ============================================================================
 * END ORIGINAL FILE: wifi_test/main/main.c
 * ========================================================================== */
