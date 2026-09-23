/*
 * BNO085 UART-RVC Test - ESP32-P4
 *
 * UART-RVC wiring:
 *   VCC -> 3V3, GND -> GND
 *   PS0 -> 3V3, PS1 -> GND
 *   SDA (BNO TX) -> ESP32 GPIO3 (UART1 RX)
 *   SCL -> unconnected for this RX-only test
 *   CS -> GND, RST -> 3V3
 *
 * UART-RVC: fixed 115200 baud, 8-N-1, 19-byte packets at 100 Hz.
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"

static const char *TAG = "BNO_RVC";

#define RVC_UART       UART_NUM_1
#define RVC_RX_PIN     3
#define RVC_BAUD       115200
#define RVC_PACKET_LEN 19
#define GRAVITY_MPS2   9.80665f
#define BIAS_CAL_SAMPLES 200       // 2 seconds of valid packets at 100 Hz
#define CALIB_MAX_STD_MPS2 0.20f   // reject calibration if platform vibrates/moves
#define ZUPT_ACC_MPS2 0.15f        // stationary linear-acceleration threshold
#define ZUPT_SAMPLES 30            // 0.3 s at 100 Hz before velocity reset
#define DELTA_WINDOW_US 500000LL   // requested short-term relative displacement: 0.5 s
#define NAV_HISTORY_LEN 64          // 0.64 s at 100 Hz, enough for 0.5 s query

typedef struct {
    int64_t timestamp_us;
    float position[3];
} nav_history_sample_t;

typedef struct {
    float velocity[3];       // m/s, world frame
    float position[3];       // m, relative to calibration location
    float acc_reference[3];  // gravity + accelerometer bias, world frame

    // Welford running statistics for static initialization.
    float calib_mean[3];
    float calib_m2[3];
    int calibration_count;
    bool calibrated;

    float prev_linear_acc[3];
    bool has_prev_linear_acc;
    int stationary_count;
    int64_t last_time_us;

    nav_history_sample_t history[NAV_HISTORY_LEN];
    int history_head;
    int history_count;
} position_integrator_t;

static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool rvc_checksum_valid(const uint8_t packet[RVC_PACKET_LEN])
{
    // Datasheet: uint8 sum of Index through Reserved bytes.
    uint8_t sum = 0;
    for (int i = 2; i < RVC_PACKET_LEN - 1; ++i) {
        sum += packet[i];
    }
    return sum == packet[RVC_PACKET_LEN - 1];
}

// Rotate BNO body-frame acceleration to world frame with datasheet order:
// yaw -> pitch -> roll, equivalent to Rz(yaw) * Ry(pitch) * Rx(roll).
static void body_to_world(float yaw_deg, float pitch_deg, float roll_deg,
                          const float body[3], float world[3])
{
    const float d2r = (float)M_PI / 180.0f;
    float cy = cosf(yaw_deg * d2r), sy = sinf(yaw_deg * d2r);
    float cp = cosf(pitch_deg * d2r), sp = sinf(pitch_deg * d2r);
    float cr = cosf(roll_deg * d2r), sr = sinf(roll_deg * d2r);

    world[0] = (cy * cp) * body[0] + (cy * sp * sr - sy * cr) * body[1]
             + (cy * sp * cr + sy * sr) * body[2];
    world[1] = (sy * cp) * body[0] + (sy * sp * sr + cy * cr) * body[1]
             + (sy * sp * cr - cy * sr) * body[2];
    world[2] = (-sp) * body[0] + (cp * sr) * body[1] + (cp * cr) * body[2];
}

static float norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void reset_static_calibration(position_integrator_t *state)
{
    state->calibration_count = 0;
    state->calibrated = false;
    for (int i = 0; i < 3; ++i) {
        state->calib_mean[i] = 0.0f;
        state->calib_m2[i] = 0.0f;
    }
}

// Store a predicted position for short-window relative displacement output.
static void push_history(position_integrator_t *state, int64_t timestamp_us)
{
    nav_history_sample_t *slot = &state->history[state->history_head];
    slot->timestamp_us = timestamp_us;
    for (int i = 0; i < 3; ++i) slot->position[i] = state->position[i];

    state->history_head = (state->history_head + 1) % NAV_HISTORY_LEN;
    if (state->history_count < NAV_HISTORY_LEN) state->history_count++;
}

// Find closest retained sample at or before (now - 0.5 s), then compute delta.
static bool relative_displacement_500ms(const position_integrator_t *state,
                                        int64_t now_us, float delta[3])
{
    if (state->history_count < 2) return false;
    const int64_t target_us = now_us - DELTA_WINDOW_US;
    const nav_history_sample_t *best = NULL;
    int64_t best_error = 0;

    for (int i = 0; i < state->history_count; ++i) {
        int index = state->history_head - 1 - i;
        if (index < 0) index += NAV_HISTORY_LEN;
        const nav_history_sample_t *sample = &state->history[index];
        int64_t error = sample->timestamp_us - target_us;
        if (error < 0) error = -error;
        if (best == NULL || error < best_error) {
            best_error = error;
            best = sample;
        }
    }

    // Require an actual history point reasonably close to the 0.5 s target.
    if (best == NULL || best_error > 30000) return false;
    for (int i = 0; i < 3; ++i) delta[i] = state->position[i] - best->position[i];
    return true;
}

// Inspired by SAD ch3 StaticIMUInit + IMUIntegration:
// - only accept a low-variance stationary initialization window;
// - use world-frame gravity/bias reference;
// - use midpoint (adjacent-sample average) acceleration integration;
// - apply a zero-velocity pseudo-observation (ZUPT) when stationary.
static bool integrate_position(position_integrator_t *state,
                               float yaw, float pitch, float roll,
                               int16_t ax_mg, int16_t ay_mg, int16_t az_mg,
                               int64_t now_us, float linear_world[3])
{
    const float mg_to_mps2 = GRAVITY_MPS2 / 1000.0f;
    float body[3] = { ax_mg * mg_to_mps2, ay_mg * mg_to_mps2, az_mg * mg_to_mps2 };
    float world_acc[3];
    body_to_world(yaw, pitch, roll, body, world_acc);

    if (!state->calibrated) {
        // Welford online mean/variance. This mirrors SAD's static-init
        // requirement that a calibration window must be truly stationary.
        state->calibration_count++;
        const float n = (float)state->calibration_count;
        for (int i = 0; i < 3; ++i) {
            float delta = world_acc[i] - state->calib_mean[i];
            state->calib_mean[i] += delta / n;
            state->calib_m2[i] += delta * (world_acc[i] - state->calib_mean[i]);
        }

        if (state->calibration_count >= BIAS_CAL_SAMPLES) {
            float variance_sum = 0.0f;
            for (int i = 0; i < 3; ++i) {
                float variance = state->calib_m2[i] / (BIAS_CAL_SAMPLES - 1);
                variance_sum += variance;
                state->acc_reference[i] = state->calib_mean[i];
            }
            float std_norm = sqrtf(variance_sum);
            if (std_norm > CALIB_MAX_STD_MPS2) {
                ESP_LOGW(TAG, "static init rejected: accel std=%.3f m/s2 > %.3f; restarting",
                         std_norm, CALIB_MAX_STD_MPS2);
                reset_static_calibration(state);
            } else {
                state->calibrated = true;
                state->last_time_us = now_us;
                state->has_prev_linear_acc = false;
                ESP_LOGI(TAG, "static init OK: gravity+bias=[%+.4f %+.4f %+.4f] m/s2, std=%.4f",
                         state->acc_reference[0], state->acc_reference[1],
                         state->acc_reference[2], std_norm);
            }
        }
        linear_world[0] = linear_world[1] = linear_world[2] = 0.0f;
        return false;
    }

    for (int i = 0; i < 3; ++i) linear_world[i] = world_acc[i] - state->acc_reference[i];

    float dt = (now_us - state->last_time_us) * 1e-6f;
    state->last_time_us = now_us;
    if (dt <= 0.0f || dt > 0.05f) {
        state->has_prev_linear_acc = false;
        return false;
    }

    float acceleration_norm = norm3(linear_world);
    if (acceleration_norm < ZUPT_ACC_MPS2) {
        state->stationary_count++;
    } else {
        state->stationary_count = 0;
    }

    if (state->stationary_count >= ZUPT_SAMPLES) {
        // A zero-velocity update is the same kind of external constraint used
        // by SAD's ESKF; here it is deliberately lightweight.
        state->velocity[0] = state->velocity[1] = state->velocity[2] = 0.0f;
        for (int i = 0; i < 3; ++i) state->prev_linear_acc[i] = linear_world[i];
        state->has_prev_linear_acc = true;
        push_history(state, now_us);
        return true;
    }

    const float *a0 = state->has_prev_linear_acc ? state->prev_linear_acc : linear_world;
    float a_mid[3];
    for (int i = 0; i < 3; ++i) a_mid[i] = 0.5f * (a0[i] + linear_world[i]);

    for (int i = 0; i < 3; ++i) {
        state->position[i] += state->velocity[i] * dt + 0.5f * a_mid[i] * dt * dt;
        state->velocity[i] += a_mid[i] * dt;
        state->prev_linear_acc[i] = linear_world[i];
    }
    state->has_prev_linear_acc = true;
    push_history(state, now_us);
    return true;
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "BNO085 UART-RVC Test");
    ESP_LOGI(TAG, "RX GPIO=%d, UART=%d, %d baud", RVC_RX_PIN, RVC_UART, RVC_BAUD);
    ESP_LOGI(TAG, "========================================");

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
    ESP_ERROR_CHECK(uart_driver_install(RVC_UART, 2048, 0, 0, NULL, 0));
    uart_flush_input(RVC_UART);

    ESP_LOGI(TAG, "Waiting for AA AA RVC packets...");

    uint8_t packet[RVC_PACKET_LEN];
    position_integrator_t integrator = {0};
    int packet_count = 0;
    int timeout_count = 0;

    while (true) {
        uint8_t byte;
        if (uart_read_bytes(RVC_UART, &byte, 1, pdMS_TO_TICKS(1000)) != 1) {
            timeout_count++;
            ESP_LOGW(TAG, "no UART data for %d second(s)", timeout_count);
            continue;
        }
        timeout_count = 0;

        // Synchronize on the RVC packet marker: 0xAA 0xAA.
        if (byte != 0xAA) continue;
        if (uart_read_bytes(RVC_UART, &byte, 1, pdMS_TO_TICKS(20)) != 1 || byte != 0xAA) {
            continue;
        }

        packet[0] = 0xAA;
        packet[1] = 0xAA;
        int received = uart_read_bytes(RVC_UART, packet + 2, RVC_PACKET_LEN - 2,
                                       pdMS_TO_TICKS(30));
        if (received != RVC_PACKET_LEN - 2) {
            ESP_LOGW(TAG, "short RVC packet: %d/%d payload bytes", received,
                     RVC_PACKET_LEN - 2);
            continue;
        }

        bool checksum_ok = rvc_checksum_valid(packet);
        uint8_t index = packet[2];
        float yaw = le_i16(packet + 3) * 0.01f;
        float pitch = le_i16(packet + 5) * 0.01f;
        float roll = le_i16(packet + 7) * 0.01f;
        int16_t accel_x_mg = le_i16(packet + 9);
        int16_t accel_y_mg = le_i16(packet + 11);
        int16_t accel_z_mg = le_i16(packet + 13);

        float linear_world[3] = {0};
        if (checksum_ok) {
            integrate_position(&integrator, yaw, pitch, roll,
                                                   accel_x_mg, accel_y_mg, accel_z_mg,
                                                   esp_timer_get_time(), linear_world);
        }

        // BNO085 sends 100 Hz. Print every tenth packet, approximately 10 Hz.
        if ((packet_count++ % 10) == 0) {
            if (!integrator.calibrated) {
                ESP_LOGI(TAG, "[%3u] bias calibration: %d/%d valid packets — keep IMU still",
                         index, integrator.calibration_count, BIAS_CAL_SAMPLES);
            } else {
                float delta_500ms[3] = {0};
                bool delta_valid = relative_displacement_500ms(&integrator,
                                                               esp_timer_get_time(), delta_500ms);
                ESP_LOGI(TAG,
                         "[%3u] YPR=[%+6.2f %+6.2f %+6.2f] | "
                         "LIN_W=[%+6.3f %+6.3f %+6.3f] | "
                         "VEL=[%+6.3f %+6.3f %+6.3f] | "
                         "DP_0.5=[%+6.3f %+6.3f %+6.3f] m | "
                         "POS=[%+6.3f %+6.3f %+6.3f] m%s%s",
                         index, yaw, pitch, roll,
                         linear_world[0], linear_world[1], linear_world[2],
                         integrator.velocity[0], integrator.velocity[1], integrator.velocity[2],
                         delta_500ms[0], delta_500ms[1], delta_500ms[2],
                         integrator.position[0], integrator.position[1], integrator.position[2],
                         delta_valid ? "" : " (DP warming)",
                         integrator.stationary_count >= ZUPT_SAMPLES ? " ZUPT" : "");
            }
        }
    }
}
