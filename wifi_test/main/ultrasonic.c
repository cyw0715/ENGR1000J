/*
 * HC-SR04 Ultrasonic Distance Sensor
 * Trig=GPIO25, Echo=GPIO26
 * Range: 20mm ~ 4000mm
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "ultrasonic.h"

static const char *TAG = "USONIC";

#define TRIG_PIN        25
#define ECHO_PIN        26
#define MAX_DISTANCE_MM 4000
#define MIN_DISTANCE_MM 20
#define TIMEOUT_US      (MAX_DISTANCE_MM * 5.8f)

static bool s_initialized = false;
static const char *s_last_status = "not_initialized";

const char *ultrasonic_last_status(void)
{
    return s_last_status;
}

esp_err_t ultrasonic_init(void)
{
    if (s_initialized) return ESP_OK;

    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&trig_cfg);
    if (ret != ESP_OK) return ret;

    gpio_config_t echo_cfg = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&echo_cfg);
    if (ret != ESP_OK) return ret;

    gpio_set_level(TRIG_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    s_initialized = true;
    ESP_LOGI(TAG, "HC-SR04 ready (Trig=%d Echo=%d)", TRIG_PIN, ECHO_PIN);
    return ESP_OK;
}

int32_t ultrasonic_read_mm(void)
{
    if (!s_initialized) {
        s_last_status = "not_initialized";
        return -1;
    }

    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - start_wait) > 1000) {
            s_last_status = "no_echo_rise";
            return -1;
        }
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - echo_start) > (int64_t)TIMEOUT_US) {
            s_last_status = "echo_stuck_high";
            return -1;
        }
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t duration_us = echo_end - echo_start;
    int32_t distance_mm = (int32_t)(duration_us * 340 / 2 / 1000);

    if (distance_mm < MIN_DISTANCE_MM || distance_mm > MAX_DISTANCE_MM) {
        s_last_status = "range_invalid";
        return -1;
    }
    s_last_status = "ok";
    return distance_mm;
}
