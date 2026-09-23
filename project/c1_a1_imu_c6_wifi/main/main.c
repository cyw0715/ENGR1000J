/* C1-A1: BNO085 UART-RVC + real C6 SDIO/Wi-Fi; no camera/CSI/ISP/JPEG/TCP. */
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"

#define WIFI_SSID "MarsLander"
#define WIFI_PASS "mars2026"
#define RVC_UART UART_NUM_1
#define RVC_RX_PIN 3
#define RVC_BAUD 115200
#define RVC_PACKET_LEN 19
#define RVC_RX_BUFFER_SIZE (16 * 1024)

static const char *TAG = "C1_A1";
static SemaphoreHandle_t s_imu_mutex;
typedef struct {
    uint32_t valid;
    uint32_t timeouts;
    uint32_t checksum;
    uint32_t sync;
    int64_t last_ts_us;
    uint8_t last_index;
} imu_stats_t;
static imu_stats_t s_stats;
static volatile bool s_wifi_connected;

static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool rvc_checksum_valid(const uint8_t packet[RVC_PACKET_LEN])
{
    uint8_t sum = 0;
    for (int i = 2; i < RVC_PACKET_LEN - 1; ++i) sum += packet[i];
    return sum == packet[RVC_PACKET_LEN - 1];
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "C1-A1 Wi-Fi station started; C6 SDIO path is active");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        ESP_LOGW(TAG, "C1-A1 Wi-Fi disconnected; reconnecting");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_wifi_connected = true;
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "C1-A1 Wi-Fi IP=" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void imu_uart_task(void *arg)
{
    uint8_t packet[RVC_PACKET_LEN];
    uint32_t timeout_streak = 0;
    while (true) {
        uint8_t byte;
        if (uart_read_bytes(RVC_UART, &byte, 1, pdMS_TO_TICKS(1000)) != 1) {
            timeout_streak++;
            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                s_stats.timeouts++;
                xSemaphoreGive(s_imu_mutex);
            }
            ESP_LOGW(TAG, "C1_A1_NO_UART seconds=%" PRIu32, timeout_streak);
            continue;
        }
        timeout_streak = 0;
        if (byte != 0xAA) {
            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                s_stats.sync++;
                xSemaphoreGive(s_imu_mutex);
            }
            continue;
        }
        if (uart_read_bytes(RVC_UART, &byte, 1, pdMS_TO_TICKS(20)) != 1 || byte != 0xAA) continue;
        packet[0] = packet[1] = 0xAA;
        if (uart_read_bytes(RVC_UART, packet + 2, RVC_PACKET_LEN - 2, pdMS_TO_TICKS(30)) != RVC_PACKET_LEN - 2) continue;
        if (!rvc_checksum_valid(packet)) {
            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                s_stats.checksum++;
                xSemaphoreGive(s_imu_mutex);
            }
            continue;
        }
        if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s_stats.valid++;
            s_stats.last_ts_us = esp_timer_get_time();
            s_stats.last_index = packet[2];
            xSemaphoreGive(s_imu_mutex);
        }
    }
}

static void stats_task(void *arg)
{
    while (true) {
        imu_stats_t stats = {0};
        if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            stats = s_stats;
            xSemaphoreGive(s_imu_mutex);
        }
        ESP_LOGI(TAG,
                 "C1_A1_STATS valid=%" PRIu32 " idx=%u age_ms=%" PRId64
                 " timeout=%" PRIu32 " checksum=%" PRIu32 " sync=%" PRIu32
                 " wifi=%s heap=%u largest=%u",
                 stats.valid, stats.last_index,
                 stats.last_ts_us ? (esp_timer_get_time() - stats.last_ts_us) / 1000 : -1,
                 stats.timeouts, stats.checksum, stats.sync,
                 s_wifi_connected ? "up" : "down",
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void init_bno_uart(void)
{
    uart_config_t cfg = {
        .baud_rate = RVC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(RVC_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RVC_UART, UART_PIN_NO_CHANGE, RVC_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(RVC_UART, RVC_RX_BUFFER_SIZE, 0, 0, NULL, 0));
    uart_flush_input(RVC_UART);
    ESP_ERROR_CHECK(xTaskCreatePinnedToCore(imu_uart_task, "c1_imu_uart", 4096, NULL, 10, NULL, 1) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

static void init_c6_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event, NULL, NULL));
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    wifi_config_t config = {.sta = {.ssid = WIFI_SSID, .password = WIFI_PASS, .threshold.authmode = WIFI_AUTH_WPA2_PSK}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

void app_main(void)
{
    ESP_LOGI(TAG, "C1-A1 BEGIN: IMU + C6 SDIO/Wi-Fi ONLY; camera is not linked or initialized");
    ESP_ERROR_CHECK(nvs_flash_init());
    s_imu_mutex = xSemaphoreCreateMutex();
    if (!s_imu_mutex) abort();
    init_bno_uart();
    init_c6_wifi();
    xTaskCreate(stats_task, "c1_stats", 4096, NULL, 4, NULL);
}
