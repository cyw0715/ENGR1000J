#ifndef IMU_H
#define IMU_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    int64_t timestamp_us;
    uint8_t index;
    float yaw;
    float pitch;
    float roll;
    int16_t raw_ax_mg;
    int16_t raw_ay_mg;
    int16_t raw_az_mg;
    bool valid;
} imu_data_t;

typedef struct {
    uint32_t valid_packet_count;
    uint32_t checksum_error_count;
    uint32_t uart_timeout_count;
    int64_t  last_valid_timestamp_us;
    uint32_t rx_overflow_count;
    uint32_t short_packet_count;
    uint32_t bad_header_count;
    uint32_t sync_fail_count;
    uint32_t rvc_header_count;
    uint32_t rvc_candidate_count;
    uint32_t rvc_resync_replay_count;
} imu_health_t;

esp_err_t imu_init(void);
esp_err_t imu_read(imu_data_t *data);
bool imu_is_ready(void);
void imu_get_health(imu_health_t *health);

QueueHandle_t imu_get_sample_queue(void);

#endif
