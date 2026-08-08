#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Single authoritative compile-time gate for motion-capable IMU commands. */
#ifndef MEGA_IMU_CONTROL_ENABLED
#define MEGA_IMU_CONTROL_ENABLED 1
#endif

typedef struct {
    bool mega_connected;
    bool valid;
    int32_t distance_mm;
    uint32_t pulse_us;
    uint32_t sequence;
    int64_t timestamp_us;
    int64_t sample_age_us;
    uint32_t request_count;
    uint32_t valid_count;
    uint32_t parse_error_count;
    uint32_t rx_byte_count;
    uint32_t rx_line_count;
    int32_t last_error;
    bool actuator_sensor_valid[4];
    int32_t actuator_sensor_mm[4];
    uint32_t actuator_sensor_version[4];
    char status[24];
    char last_rx_line[96];
} mega_sr04_reading_t;

/** IMU controller state parsed from Mega status lines. */
typedef struct {
    bool connected;
    bool armed;
    bool active;
    bool estop;
    bool motion_enabled;
    uint32_t heartbeat_seq;
    uint32_t fault_count;
    int16_t target_delta_mm[4];   /* solver order FL,FR,RL,RR */
    int16_t applied_delta_mm[4];  /* solver order FL,FR,RL,RR */
    bool remote_active;
    bool ps2_connected;
    int8_t remote_preset;
    uint16_t remote_target_mm[4]; /* physical order A1,A2,A3,A4 */
    char remote_source[8];
    char remote_submode[8];
    char mega_mode[16];           /* LOCKED, REMOTE_*, ARMED, ACTIVE, FAULT, ESTOP */
    char fault_reason[32];
    int64_t timestamp_us;
} mega_imu_state_t;

/** Start P4 USB Host -> CH340 -> Mega SR04 polling. Never controls actuators. */
esp_err_t mega_sr04_init(void);

/** Copies the newest parsed SR04 record; safe before the first valid sample. */
void mega_sr04_get_reading(mega_sr04_reading_t *out);

/**
 * Send four solver axial lengths through P4 USB Host -> CH340 -> Mega.
 * Inputs must be FL/FR/RL/RR in [250, 390] mm. Mega maps each to its verified
 * VL53L1X coordinate by `sensor_target_mm = solver_length_mm - 190`.
 * This writes only the line `SET_TARGET FL FR RL RR\\n`; it never operates P4 PWM.
 */
esp_err_t mega_solver_send_targets(uint16_t fl_mm, uint16_t fr_mm,
                                   uint16_t rl_mm, uint16_t rr_mm);

/**
 * IMU controller command API. All commands serialize to the same CH340 TX mutex.
 * These are gated by mega_imu_control_enabled (compile-time lock, default off).
 */

/** Check whether motion-capable IMU commands are compile-time enabled. */
bool mega_imu_control_is_enabled(void);

/** Send ARM REMOTE command; enables the faithful PS2/manual/preset owner. */
esp_err_t mega_remote_arm(void);

/** Send SET_REMOTE <seq> <FL> <FR> <RL> <RR>, absolute VL53 mm in [0,400]. */
esp_err_t mega_remote_set_absolute(uint32_t seq,
                                   uint16_t fl, uint16_t fr,
                                   uint16_t rl, uint16_t rr);

/** Send ARM IMU command. Returns ESP_OK on success. */
esp_err_t mega_imu_arm(void);

/** Send STOP command. Returns ESP_OK on success. */
esp_err_t mega_imu_stop(void);

/** Send ESTOP command. Returns ESP_OK on success. */
esp_err_t mega_imu_estop(void);

/** Send HEARTBEAT <seq> command. Returns ESP_OK on success. */
esp_err_t mega_imu_heartbeat(uint32_t seq);

/**
 * Send SET_DELTA <seq> <FL> <FR> <RL> <RR> command.
 * Deltas are signed mm corrections in solver order.
 * Protocol deltas must be in [-350, +350]. Mega applies the authoritative
 * per-leg ARM-baseline + delta check against its live 50..400 mm VL53 range.
 */
esp_err_t mega_imu_set_delta(uint32_t seq,
                             int16_t fl, int16_t fr,
                             int16_t rl, int16_t rr);

/** Get the latest parsed IMU controller state. */
void mega_imu_get_state(mega_imu_state_t *out);
