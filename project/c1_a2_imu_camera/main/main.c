/* C1-A2: BNO085 + OV5647/CSI/ISP/JPEG. No C6 Hosted/Wi-Fi/TCP/HTTP. */
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "esp_intr_alloc.h"
#include "driver/uart.h"
#include "driver/jpeg_encode.h"
#include "camera.h"

#define RVC_UART UART_NUM_1
#define RVC_RX_PIN 3
#define RVC_BAUD 115200
#define RVC_PACKET_LEN 19
#define RVC_RX_BUFFER_SIZE (16 * 1024)
#define JPEG_BUF_SIZE (300 * 1024)

static const char *TAG = "C1_A2";
static SemaphoreHandle_t s_mutex;
static SemaphoreHandle_t s_frame_ready;
static SemaphoreHandle_t s_buf_free;
static uint8_t *s_local_buf;
static uint8_t *s_gray_a;
static uint8_t *s_gray_b;
static uint8_t *s_write_gray;
static uint8_t *s_send_gray;
static jpeg_encoder_handle_t s_jpeg;
static uint8_t *s_jpeg_buf;
static size_t s_jpeg_buf_sz;
static uint32_t s_w, s_h;
static esp_err_t s_uart_init_result;

typedef struct {
    uint32_t valid, timeout, checksum, sync, frames, jpeg_ok;
    int64_t last_ts_us;
    uint8_t index;
} stats_t;
static stats_t s_stats;

static bool rvc_checksum_valid(const uint8_t p[RVC_PACKET_LEN])
{
    uint8_t sum = 0;
    for (int i = 2; i < RVC_PACKET_LEN - 1; ++i) sum += p[i];
    return sum == p[RVC_PACKET_LEN - 1];
}

static void imu_task(void *arg)
{
    uint8_t p[RVC_PACKET_LEN];
    uint32_t streak = 0;
    while (true) {
        uint8_t b;
        if (uart_read_bytes(RVC_UART, &b, 1, pdMS_TO_TICKS(1000)) != 1) {
            streak++;
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) { s_stats.timeout++; xSemaphoreGive(s_mutex); }
            ESP_LOGW(TAG, "C1_A2_NO_UART seconds=%" PRIu32, streak);
            continue;
        }
        streak = 0;
        if (b != 0xAA) { if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) { s_stats.sync++; xSemaphoreGive(s_mutex); } continue; }
        if (uart_read_bytes(RVC_UART, &b, 1, pdMS_TO_TICKS(20)) != 1 || b != 0xAA) continue;
        p[0] = p[1] = 0xAA;
        if (uart_read_bytes(RVC_UART, p + 2, RVC_PACKET_LEN - 2, pdMS_TO_TICKS(30)) != RVC_PACKET_LEN - 2) continue;
        if (!rvc_checksum_valid(p)) { if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) { s_stats.checksum++; xSemaphoreGive(s_mutex); } continue; }
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s_stats.valid++; s_stats.last_ts_us = esp_timer_get_time(); s_stats.index = p[2];
            xSemaphoreGive(s_mutex);
        }
    }
}

static void uart_init_cpu1_task(void *arg)
{
    TaskHandle_t caller = (TaskHandle_t)arg;
    uart_config_t uc = {
        .baud_rate = RVC_BAUD, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    s_uart_init_result = uart_param_config(RVC_UART, &uc);
    if (s_uart_init_result == ESP_OK) {
        s_uart_init_result = uart_set_pin(RVC_UART, UART_PIN_NO_CHANGE, RVC_RX_PIN,
                                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (s_uart_init_result == ESP_OK) {
        // Must be IRAM-resident: CSI/ISP/JPEG may suspend flash cache while the
        // BNO085 continues its 100 Hz UART stream.
        s_uart_init_result = uart_driver_install(RVC_UART, RVC_RX_BUFFER_SIZE, 0, 0, NULL,
                                                 ESP_INTR_FLAG_IRAM);
    }
    if (s_uart_init_result == ESP_OK) uart_flush_input(RVC_UART);
    xTaskNotifyGive(caller);
    vTaskDelete(NULL);
}

static void capture_task(void *arg)
{
    while (true) {
        size_t len = 0;
        uint8_t *frame = camera_get_frame(&len, 5000);
        if (!frame || len == 0) { ESP_LOGW(TAG, "C1_A2_CAMERA_TIMEOUT"); continue; }
        if (xSemaphoreTake(s_buf_free, pdMS_TO_TICKS(1000)) != pdTRUE) continue;
        memcpy(s_local_buf, frame, s_w * s_h * 2);
        uint16_t *src = (uint16_t *)s_local_buf;
        for (uint32_t i = 0; i < s_w * s_h; ++i) {
            uint16_t px = src[i];
            uint8_t r = (uint8_t)((px >> 11) << 3), g = (uint8_t)(((px >> 5) & 0x3f) << 2), b = (uint8_t)((px & 0x1f) << 3);
            s_write_gray[i] = (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
        }
        uint8_t *tmp = s_write_gray; s_write_gray = s_send_gray; s_send_gray = tmp;
        xSemaphoreGive(s_frame_ready);
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) { s_stats.frames++; xSemaphoreGive(s_mutex); }
    }
}

static void jpeg_task(void *arg)
{
    // Match the verified production stream: grayscale input requires GRAY sampling.
    jpeg_encode_cfg_t cfg = {.width = 800, .height = 800, .src_type = JPEG_ENCODE_IN_FORMAT_GRAY, .sub_sample = JPEG_DOWN_SAMPLING_GRAY, .image_quality = 50};
    while (true) {
        if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(5000)) != pdTRUE) continue;
        cfg.width = s_w; cfg.height = s_h;
        uint32_t out = 0;
        esp_err_t r = jpeg_encoder_process(s_jpeg, &cfg, s_send_gray, s_w * s_h, s_jpeg_buf, s_jpeg_buf_sz, &out);
        xSemaphoreGive(s_buf_free);
        if (r == ESP_OK && out > 0) { if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) { s_stats.jpeg_ok++; xSemaphoreGive(s_mutex); } }
        else ESP_LOGW(TAG, "C1_A2_JPEG_FAIL err=0x%x", r);
    }
}

static void stats_task(void *arg)
{
    while (true) {
        stats_t x = {0};
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) { x = s_stats; xSemaphoreGive(s_mutex); }
        ESP_LOGI(TAG, "C1_A2_STATS valid=%" PRIu32 " idx=%u age_ms=%" PRId64 " timeout=%" PRIu32 " checksum=%" PRIu32 " sync=%" PRIu32 " frames=%" PRIu32 " jpeg=%" PRIu32 " heap=%u largest=%u",
                 x.valid, x.index, x.last_ts_us ? (esp_timer_get_time()-x.last_ts_us)/1000 : -1, x.timeout, x.checksum, x.sync, x.frames, x.jpeg_ok,
                 (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "C1-A2 BEGIN: IMU + OV5647 CSI/ISP/JPEG ONLY; C6/Wi-Fi is absent");
    ESP_ERROR_CHECK(nvs_flash_init());
    s_mutex = xSemaphoreCreateMutex(); s_frame_ready = xSemaphoreCreateBinary(); s_buf_free = xSemaphoreCreateBinary();
    if (!s_mutex || !s_frame_ready || !s_buf_free) abort();
    xSemaphoreGive(s_buf_free);
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    s_uart_init_result = ESP_FAIL;
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore(uart_init_cpu1_task, "c1a2_uart_init", 4096,
                                             self, 12, NULL, 1) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3000)) ? ESP_OK : ESP_ERR_TIMEOUT);
    ESP_ERROR_CHECK(s_uart_init_result);
    ESP_LOGI(TAG, "C1-A2 UART driver installed on CPU1; parser will run on CPU1");
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore(imu_task, "c1a2_imu", 4096, NULL, 10, NULL, 1) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(camera_init()); ESP_ERROR_CHECK(camera_start());
    s_w = camera_get_width(); s_h = camera_get_height();
    size_t rgb = s_w * s_h * 2, gray = s_w * s_h;
    s_local_buf = heap_caps_aligned_alloc(64, rgb, MALLOC_CAP_SPIRAM);
    s_gray_a = heap_caps_aligned_alloc(64, gray, MALLOC_CAP_SPIRAM); s_gray_b = heap_caps_aligned_alloc(64, gray, MALLOC_CAP_SPIRAM);
    if (!s_local_buf || !s_gray_a || !s_gray_b) abort();
    s_write_gray=s_gray_a; s_send_gray=s_gray_b;
    jpeg_encode_engine_cfg_t ec = {.intr_priority=0,.timeout_ms=1000}; ESP_ERROR_CHECK(jpeg_new_encoder_engine(&ec, &s_jpeg));
    jpeg_encode_memory_alloc_cfg_t mc = {.buffer_direction=JPEG_ENC_ALLOC_OUTPUT_BUFFER}; s_jpeg_buf=(uint8_t *)jpeg_alloc_encoder_mem(JPEG_BUF_SIZE,&mc,&s_jpeg_buf_sz);
    if (!s_jpeg_buf) abort();
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore(capture_task,"c1a2_cap",8192,NULL,5,NULL,0)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore(jpeg_task,"c1a2_jpeg",8192,NULL,4,NULL,0)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(stats_task,"c1a2_stats",4096,NULL,3,NULL)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
}
