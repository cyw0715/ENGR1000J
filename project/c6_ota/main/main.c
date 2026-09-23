/*
 * ESP32-C6 read-only ESP-Hosted version probe.
 *
 * SAFETY: This diagnostic intentionally does not call any slave OTA API and
 * therefore cannot begin, write, finish, activate, erase, or reboot C6 flash.
 */

#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_hosted.h"

static const char *TAG = "C6_PROBE";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C6 read-only ESP-Hosted probe");
    ESP_LOGI(TAG, "No C6 flash/OTA operation is compiled into this application");

    int ret = esp_hosted_connect_to_slave();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "connect_to_slave failed: %s (0x%x)",
                 esp_err_to_name(ret), (unsigned int)ret);
        return;
    }

    uint32_t chip_id = 0;
    char target[24] = {0};
    ret = esp_hosted_get_cp_info(&chip_id, target, sizeof(target));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "C6 CP info: chip_id=%" PRIu32 " target=%s", chip_id, target);
    } else {
        ESP_LOGW(TAG, "get_cp_info failed: %s (0x%x)",
                 esp_err_to_name(ret), (unsigned int)ret);
    }

    esp_hosted_coprocessor_fwver_t fwver = {0};
    ret = esp_hosted_get_coprocessor_fwversion(&fwver);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "C6 ESP-Hosted firmware: %" PRIu32 ".%" PRIu32 ".%" PRIu32,
                 fwver.major1, fwver.minor1, fwver.patch1);
    } else {
        ESP_LOGE(TAG, "get_coprocessor_fwversion failed: %s (0x%x)",
                 esp_err_to_name(ret), (unsigned int)ret);
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
