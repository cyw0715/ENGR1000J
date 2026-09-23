/*
 * Stage-1 USB CDC-ACM Host diagnostic for ESP32-P4-NANO USB-A.
 *
 * Safety scope:
 * - USB enumeration + PING/PONG only
 * - no camera, Wi-Fi, IMU, GPIO, actuator, or motor-driver control
 * - opens one CDC or CDC-like device; use only with the intended Mega attached
 */
#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

static const char *TAG = "USB_MEGA";
static cdc_acm_dev_hdl_t s_cdc = NULL;
static volatile bool s_cdc_connected = false;
static uint32_t s_ping_count = 0;

static bool mega_rx_callback(const uint8_t *data, size_t data_len, void *user_arg)
{
    char text[129];
    size_t count = data_len < sizeof(text) - 1 ? data_len : sizeof(text) - 1;
    memcpy(text, data, count);
    text[count] = '\0';
    ESP_LOGI(TAG, "Mega RX (%u B): %s", (unsigned)data_len, text);
    return true;
}

static void mega_event_callback(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGW(TAG, "Mega USB CDC disconnected");
        s_cdc_connected = false;
        s_cdc = NULL;
    } else if (event->type == CDC_ACM_HOST_ERROR) {
        ESP_LOGW(TAG, "Mega USB CDC error: %d", event->data.error);
    }
}

static void usb_lib_task(void *arg)
{
    while (true) {
        uint32_t flags = 0;
        esp_err_t ret = usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "USB host event error: 0x%x", ret);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "USB host has no active client");
        }
        if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB host device resources freed");
        }
    }
}

static void mega_open_task(void *arg)
{
    const cdc_acm_host_open_config_t open_config = {
        .vid = CDC_HOST_ANY_VID,
        .pid = CDC_HOST_ANY_PID,
        .interface_idx = 0,
        .dev_addr = CDC_HOST_ANY_DEV_ADDR,
        .connection_timeout_ms = 5000,
        .out_buffer_size = 64,
        .in_buffer_size = 64,
        .event_cb = mega_event_callback,
        .data_cb = mega_rx_callback,
        .user_arg = NULL,
    };

    while (true) {
        if (!s_cdc_connected) {
            cdc_acm_dev_hdl_t handle = NULL;
            ESP_LOGI(TAG, "Waiting up to 5 s for one USB CDC device...");
            esp_err_t ret = cdc_acm_host_open(&open_config, &handle);
            if (ret != ESP_OK) {
                if (ret != ESP_ERR_TIMEOUT && ret != ESP_ERR_NOT_FOUND) {
                    ESP_LOGW(TAG, "CDC open failed: 0x%x", ret);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            s_cdc = handle;
            s_cdc_connected = true;
            ESP_LOGI(TAG, "USB CDC device opened; printing descriptors");
            cdc_acm_host_desc_print(s_cdc);
            ret = cdc_acm_host_set_control_line_state(s_cdc, true, true);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "DTR/RTS setup failed: 0x%x (continuing)", ret);
            }
        }

        const uint8_t ping[] = "PING\n";
        esp_err_t ret = cdc_acm_host_data_tx_blocking(s_cdc, ping, sizeof(ping) - 1, 500);
        if (ret == ESP_OK) {
            s_ping_count++;
            ESP_LOGI(TAG, "PING sent #%" PRIu32, s_ping_count);
        } else {
            ESP_LOGW(TAG, "PING TX failed: 0x%x; waiting for reconnect", ret);
            s_cdc_connected = false;
            s_cdc = NULL;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Stage-1 USB CDC test: P4-NANO USB-A host -> Arduino Mega");
    ESP_LOGI(TAG, "Safety: this image never configures GPIOs or controls actuators");

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 5, NULL);

    const cdc_acm_host_driver_config_t cdc_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 4,
        .xCoreID = tskNO_AFFINITY,
        .new_dev_cb = NULL,
    };
    ESP_ERROR_CHECK(cdc_acm_host_install(&cdc_config));
    xTaskCreate(mega_open_task, "mega_cdc", 4096, NULL, 3, NULL);
}
