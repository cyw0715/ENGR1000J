/*
 * Standalone HC-SR04 wiring diagnostic for ESP32-P4 Function EV Board.
 *
 * Wiring under test:
 *   HC-SR04 VCC  -> 3V3 (experimental; standard HC-SR04 is specified for 5V)
 *   HC-SR04 GND  -> GND
 *   HC-SR04 Trig -> GPIO25
 *   HC-SR04 Echo -> GPIO26 (must not exceed 3.3V)
 *
 * No Wi-Fi, camera, IMU, ESP-Hosted, or actuator code is started.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#define TRIG_PIN        GPIO_NUM_25
#define ECHO_PIN        GPIO_NUM_26
#define MIN_DISTANCE_MM 20
#define MAX_DISTANCE_MM 4000
#define ECHO_TIMEOUT_US ((int64_t)(MAX_DISTANCE_MM * 5.8f))

static const char *TAG = "SR04_TEST";

typedef enum {
    SR04_OK,
    SR04_NO_ECHO_RISE,
    SR04_ECHO_STUCK_HIGH,
    SR04_RANGE_INVALID,
} sr04_status_t;

static const char *status_name(sr04_status_t status)
{
    switch (status) {
    case SR04_OK:              return "ok";
    case SR04_NO_ECHO_RISE:    return "no_echo_rise";
    case SR04_ECHO_STUCK_HIGH: return "echo_stuck_high";
    case SR04_RANGE_INVALID:   return "range_invalid";
    default:                   return "unknown";
    }
}

static int32_t measure_mm(sr04_status_t *status, int64_t *pulse_us)
{
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - wait_start > 1000) {
            *status = SR04_NO_ECHO_RISE;
            *pulse_us = 0;
            return -1;
        }
    }

    int64_t high_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - high_start > ECHO_TIMEOUT_US) {
            *status = SR04_ECHO_STUCK_HIGH;
            *pulse_us = esp_timer_get_time() - high_start;
            return -1;
        }
    }

    *pulse_us = esp_timer_get_time() - high_start;
    int32_t distance_mm = (int32_t)(*pulse_us * 340 / 2 / 1000);
    if (distance_mm < MIN_DISTANCE_MM || distance_mm > MAX_DISTANCE_MM) {
        *status = SR04_RANGE_INVALID;
        return -1;
    }

    *status = SR04_OK;
    return distance_mm;
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Standalone HC-SR04 test");
    ESP_LOGI(TAG, "Trig=GPIO%d Echo=GPIO%d", TRIG_PIN, ECHO_PIN);
    ESP_LOGI(TAG, "No Wi-Fi/camera/IMU/actuator tasks are running");
    ESP_LOGI(TAG, "========================================");

    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << TRIG_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << ECHO_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trig));
    ESP_ERROR_CHECK(gpio_config(&echo));
    gpio_set_level(TRIG_PIN, 0);

    for (uint32_t sample = 1;; ++sample) {
        sr04_status_t status;
        int64_t pulse_us;
        int32_t distance_mm = measure_mm(&status, &pulse_us);
        ESP_LOGI(TAG,
                 "sample=%" PRIu32 " trig=10us echo_initial=%d pulse_us=%" PRId64
                 " distance_mm=%ld valid=%s status=%s",
                 sample, gpio_get_level(ECHO_PIN), pulse_us, (long)distance_mm,
                 distance_mm >= 0 ? "true" : "false", status_name(status));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
