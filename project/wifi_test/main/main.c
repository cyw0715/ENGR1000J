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
#define WIFI_PASS      "mars2026"
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
