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
