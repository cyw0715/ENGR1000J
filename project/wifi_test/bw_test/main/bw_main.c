/*
 * WiFi Bandwidth Test - UDP, 1400字节包 (小于MTU, 不分片)
 */
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

static const char *TAG = "BW";

#define WIFI_SSID   "MarsLander"
#define WIFI_PASS   "mars2026"
#define MAX_RETRY   10
#define UDP_PORT    5001
#define PKT_SIZE    1400   // < MTU 1500, 不分片

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
static volatile bool s_client_ready = false;
static struct sockaddr_in s_client_addr;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) { esp_wifi_connect(); s_retry_num++; }
        else xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t inst_any, inst_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &inst_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &inst_ip));
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS, .threshold.authmode = WIFI_AUTH_WPA2_PSK } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Connecting to %s...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

static void udp_discover_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(UDP_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    ESP_LOGI(TAG, "UDP port %d, waiting for client...", UDP_PORT);

    char buf[64];
    while (1) {
        socklen_t len = sizeof(s_client_addr);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&s_client_addr, &len);
        if (n > 0) {
            s_client_ready = true;
            ESP_LOGI(TAG, "Client: %s:%d", inet_ntoa(s_client_addr.sin_addr), ntohs(s_client_addr.sin_port));
        }
    }
}

static void sender_task(void *arg)
{
    uint8_t *pkt = heap_caps_aligned_alloc(64, PKT_SIZE, MALLOC_CAP_SPIRAM);
    if (!pkt) { ESP_LOGE(TAG, "alloc fail"); vTaskDelete(NULL); return; }
    for (int i = 0; i < PKT_SIZE; i++) pkt[i] = (uint8_t)(i & 0xFF);

    uint32_t seq = 0;
    while (!s_client_ready) vTaskDelay(pdMS_TO_TICKS(100));

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    ESP_LOGI(TAG, "Sending %d-byte pkts to %s:%d", PKT_SIZE, inet_ntoa(s_client_addr.sin_addr), ntohs(s_client_addr.sin_port));

    int64_t t0 = esp_timer_get_time();
    int64_t t_log = t0;
    uint32_t log_bytes = 0;

    // 目标带宽: 500 KB/s
    int64_t pkt_interval_us = (int64_t)PKT_SIZE * 1000000 / (500 * 1024);  // 微秒/包
    int64_t next_send = esp_timer_get_time();

    while (1) {
        // 限速: 等到该发下一个包的时间
        int64_t now = esp_timer_get_time();
        if (now < next_send) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        next_send += pkt_interval_us;
        if (next_send < now) next_send = now;  // 落后太多就追赶

        memcpy(pkt, &seq, 4);
        sendto(sock, pkt, PKT_SIZE, 0, (struct sockaddr *)&s_client_addr, sizeof(s_client_addr));
        seq++;
        log_bytes += PKT_SIZE;

        now = esp_timer_get_time();
        if (now - t_log >= 1000000) {
            float dt = (now - t_log) / 1000000.0f;
            float bw = log_bytes / dt / 1024.0f;
            float elapsed = (now - t0) / 1000000.0f;
            esp_log_level_set("*", ESP_LOG_INFO);
            ESP_LOGI(TAG, "seq=%u %.0fKB/s %.0fFPS", seq, bw, seq / elapsed);
            esp_log_level_set("*", ESP_LOG_NONE);
            t_log = now;
            log_bytes = 0;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "WiFi UDP BW Test (%d-byte pkts)", PKT_SIZE);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init_sta();
    ESP_LOGI(TAG, "WiFi OK");
    xTaskCreate(udp_discover_task, "discover", 4096, NULL, 3, NULL);
    xTaskCreate(sender_task, "sender", 4096, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_log_level_set("*", ESP_LOG_NONE);
}
