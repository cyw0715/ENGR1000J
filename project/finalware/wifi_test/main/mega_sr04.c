/*
 * P4 USB Host -> CH340 -> Mega serial bridge.
 *
 * Parses structured status lines from Mega (SR04, LANDER_STATUS, IMU state).
 * Provides command forwarding for both legacy SET_TARGET and new IMU controller
 * protocol (ARM IMU, HEARTBEAT, SET_DELTA, STOP, ESTOP).
 * All IMU control commands are gated by a compile-time lock (default off).
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.h"
#include "mega_sr04.h"

#define MEGA_SR04_POLL_MS       200
#define MEGA_SR04_STALE_US      1000000LL
#define MEGA_SR04_RX_LINE_MAX   512
#define MEGA_IMU_CMD_TIMEOUT_MS 500

static mega_imu_state_t s_imu_state = {0};

#ifndef MEGA_IMU_CONTROL_ENABLED
#define MEGA_IMU_CONTROL_ENABLED 0
#endif

bool mega_imu_control_is_enabled(void)
{
    return MEGA_IMU_CONTROL_ENABLED != 0;
}

static const char *TAG = "MEGA_SR04";
static cdc_acm_dev_hdl_t s_mega = NULL;
static volatile bool s_mega_connected = false;
static volatile bool s_solver_target_active = false;
static volatile esp_err_t s_last_error = ESP_ERR_NOT_FOUND;
static mega_sr04_reading_t s_reading = {
    .distance_mm = -1,
    .last_error = ESP_ERR_NOT_FOUND,
    .status = "not_connected",
};
static SemaphoreHandle_t s_mutex = NULL;
static SemaphoreHandle_t s_tx_mutex = NULL;
static char s_rx_line[MEGA_SR04_RX_LINE_MAX];
static size_t s_rx_len = 0;

static void reading_set_status(const char *status, esp_err_t error)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        snprintf(s_reading.status, sizeof(s_reading.status), "%s", status);
        s_reading.last_error = error;
        s_reading.mega_connected = s_mega_connected;
        xSemaphoreGive(s_mutex);
    }
}

/** Serialize all P4-to-Mega lines: SR04 polling and solver commands share CH340. */
static esp_err_t mega_write_line(const char *line, uint32_t timeout_ms)
{
    if (!line || !s_tx_mutex || !s_mega_connected || !s_mega) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t ret = cdc_acm_host_data_tx_blocking(s_mega, (const uint8_t *)line,
                                                   strlen(line), timeout_ms);
    xSemaphoreGive(s_tx_mutex);
    return ret;
}

static void parse_sr04_line(const char *line)
{
    unsigned long seq = 0, timestamp_ms = 0, pulse = 0;
    long distance = -1;
    char status[24] = {0};
    int fields = sscanf(line, "SR04 seq=%lu t_ms=%lu distance_mm=%ld pulse_us=%lu status=%23s",
                        &seq, &timestamp_ms, &distance, &pulse, status);
    if (fields != 5) {
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_reading.parse_error_count++;
            snprintf(s_reading.status, sizeof(s_reading.status), "parse_error");
            s_reading.last_error = ESP_FAIL;
            xSemaphoreGive(s_mutex);
        }
        ESP_LOGW(TAG, "unparseable Mega line: %s", line);
        return;
    }

    bool valid = (strcmp(status, "ok") == 0 && distance >= 20 && distance <= 4000);
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_reading.sequence = (uint32_t)seq;
        s_reading.distance_mm = (int32_t)distance;
        s_reading.pulse_us = (uint32_t)pulse;
        s_reading.valid = valid;
        s_reading.timestamp_us = esp_timer_get_time();
        snprintf(s_reading.status, sizeof(s_reading.status), "%s", status);
        s_reading.last_error = ESP_OK;
        s_reading.mega_connected = s_mega_connected;
        if (valid) s_reading.valid_count++;
        xSemaphoreGive(s_mutex);
    }
}

static void parse_lander_status_line(const char *line)
{
    long mm[4] = {-1, -1, -1, -1};
    unsigned valid[4] = {0, 0, 0, 0};
    unsigned long version[4] = {0, 0, 0, 0};
    int fields = sscanf(line,
        "LANDER_STATUS mode=DIAGNOSTIC_MOTION_LOCKED "
        "A1_mm=%ld A1_valid=%u A1_ver=%lu "
        "A2_mm=%ld A2_valid=%u A2_ver=%lu "
        "A3_mm=%ld A3_valid=%u A3_ver=%lu "
        "A4_mm=%ld A4_valid=%u A4_ver=%lu",
        &mm[0], &valid[0], &version[0], &mm[1], &valid[1], &version[1],
        &mm[2], &valid[2], &version[2], &mm[3], &valid[3], &version[3]);
    if (fields != 12) {
        reading_set_status("lander_parse_error", ESP_FAIL);
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        for (size_t i = 0; i < 4; ++i) {
            s_reading.actuator_sensor_mm[i] = (int32_t)mm[i];
            s_reading.actuator_sensor_valid[i] = valid[i] != 0;
            s_reading.actuator_sensor_version[i] = (uint32_t)version[i];
        }
        s_reading.mega_connected = s_mega_connected;
        xSemaphoreGive(s_mutex);
    }
}

/** Parse IMU controller LANDER_STATUS lines.
 *  Format: LANDER_STATUS mode=DISARMED|ARMED|ACTIVE|STOPPED|ESTOP
 *          estop=0 faults=N hb_seq=N
 *          A1_mm=N A1_valid=N A1_ver=N ... A4_mm/N/ver
 *          target_FL=N target_FR=N target_RL=N target_RR=N
 *          applied_FL=N applied_FR=N applied_RL=N applied_RR=N
 *          out_A1=IDLE out_A2=IDLE out_A3=IDLE out_A4=IDLE
 */
static void parse_imu_status_line(const char *line)
{
    char mode[16] = {0}, fault[32] = {0};
    unsigned estop = 0, faults = 0;
    unsigned long seq = 0;
    long mm[4] = {-1, -1, -1, -1};
    unsigned valid[4] = {0, 0, 0, 0};
    unsigned long version[4] = {0, 0, 0, 0};
    char direction[4][9] = {{0}};
    unsigned verified[4] = {0, 0, 0, 0};
    int target[4] = {0, 0, 0, 0}, applied[4] = {0, 0, 0, 0};
    unsigned remote_active = 0, ps2_connected = 0;
    int remote_preset = -1;
    unsigned remote_target[4] = {0, 0, 0, 0};
    char remote_source[8] = "none", remote_submode[8] = "manual";

    int fields = sscanf(line,
        "IMU_STATUS mode=%15s estop=%u faults=%u fault=%31s seq=%lu "
        "A1=%ld,%u,%lu,%8[^,],%u A2=%ld,%u,%lu,%8[^,],%u "
        "A3=%ld,%u,%lu,%8[^,],%u A4=%ld,%u,%lu,%8[^,],%u "
        "target=%d,%d,%d,%d applied=%d,%d,%d,%d",
        mode, &estop, &faults, fault, &seq,
        &mm[0], &valid[0], &version[0], direction[0], &verified[0],
        &mm[1], &valid[1], &version[1], direction[1], &verified[1],
        &mm[2], &valid[2], &version[2], direction[2], &verified[2],
        &mm[3], &valid[3], &version[3], direction[3], &verified[3],
        &target[0], &target[1], &target[2], &target[3],
        &applied[0], &applied[1], &applied[2], &applied[3]);
    if (fields != 33) {
        reading_set_status("imu_parse_error", ESP_FAIL);
        return;
    }

    const char *remote = strstr(line, " remote_active=");
    if (remote) {
        (void)sscanf(remote,
            " remote_active=%u remote_source=%7s remote_preset=%d "
            "remote_target=%u,%u,%u,%u ps2_connected=%u remote_submode=%7s",
            &remote_active, remote_source, &remote_preset,
            &remote_target[0], &remote_target[1], &remote_target[2], &remote_target[3],
            &ps2_connected, remote_submode);
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        for (size_t i = 0; i < 4; ++i) {
            s_reading.actuator_sensor_mm[i] = (int32_t)mm[i];
            s_reading.actuator_sensor_valid[i] = valid[i] != 0;
            s_reading.actuator_sensor_version[i] = (uint32_t)version[i];
            s_imu_state.target_delta_mm[i] = (int16_t)target[i];
            s_imu_state.applied_delta_mm[i] = (int16_t)applied[i];
        }
        int64_t now = esp_timer_get_time();
        s_reading.mega_connected = s_mega_connected;
        s_imu_state.connected = s_mega_connected;
        bool remote_owner = strcmp(mode, "REMOTE_MANUAL") == 0 || strcmp(mode, "REMOTE_PRESET") == 0;
        s_imu_state.armed = remote_owner || strcmp(mode, "ARMED") == 0 || strcmp(mode, "ACTIVE") == 0;
        s_imu_state.active = remote_active != 0 || strcmp(mode, "ACTIVE") == 0;
        s_imu_state.estop = estop != 0;
        s_imu_state.motion_enabled = s_imu_state.active;
        s_imu_state.remote_active = remote_active != 0;
        s_imu_state.ps2_connected = ps2_connected != 0;
        s_imu_state.remote_preset = (int8_t)remote_preset;
        for (size_t i = 0; i < 4; ++i) s_imu_state.remote_target_mm[i] = (uint16_t)remote_target[i];
        snprintf(s_imu_state.remote_source, sizeof(s_imu_state.remote_source), "%s", remote_source);
        snprintf(s_imu_state.remote_submode, sizeof(s_imu_state.remote_submode), "%s", remote_submode);
        s_imu_state.heartbeat_seq = (uint32_t)seq;
        s_imu_state.fault_count = (uint32_t)faults;
        snprintf(s_imu_state.mega_mode, sizeof(s_imu_state.mega_mode), "%s", mode);
        snprintf(s_imu_state.fault_reason, sizeof(s_imu_state.fault_reason), "%s", fault);
        s_imu_state.timestamp_us = now;
        xSemaphoreGive(s_mutex);
    }
}

static bool mega_rx_callback(const uint8_t *data, size_t data_len, void *user_arg)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_reading.rx_byte_count += (uint32_t)data_len;
        xSemaphoreGive(s_mutex);
    }
    for (size_t i = 0; i < data_len; ++i) {
        char ch = (char)data[i];
        if (ch == '\r') continue;
        if (ch == '\n') {
            s_rx_line[s_rx_len] = '\0';
            if (s_rx_len > 0 && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                s_reading.rx_line_count++;
                snprintf(s_reading.last_rx_line, sizeof(s_reading.last_rx_line), "%.95s", s_rx_line);
                xSemaphoreGive(s_mutex);
            }
            if (s_rx_len > 0 && strncmp(s_rx_line, "SR04 ", 5) == 0) {
                parse_sr04_line(s_rx_line);
            } else if (s_rx_len > 0 && strncmp(s_rx_line, "IMU_STATUS ", 11) == 0) {
                parse_imu_status_line(s_rx_line);
            } else if (s_rx_len > 0 && strncmp(s_rx_line, "LANDER_STATUS ", 14) == 0) {
                parse_lander_status_line(s_rx_line);
            }
            s_rx_len = 0;
        } else if (s_rx_len < sizeof(s_rx_line) - 1) {
            s_rx_line[s_rx_len++] = ch;
        } else {
            s_rx_len = 0;
            reading_set_status("line_too_long", ESP_ERR_INVALID_SIZE);
        }
    }
    return true;
}

static void mega_event_callback(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        s_mega_connected = false;
        s_mega = NULL;
        s_last_error = ESP_ERR_NOT_FOUND;
        reading_set_status("disconnected", s_last_error);
    } else if (event->type == CDC_ACM_HOST_ERROR) {
        s_last_error = event->data.error;
        reading_set_status("usb_error", s_last_error);
    }
}

static void usb_lib_task(void *arg)
{
    while (true) {
        uint32_t flags = 0;
        esp_err_t ret = usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (ret != ESP_OK) {
            s_last_error = ret;
            reading_set_status("host_error", ret);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void mega_sr04_task(void *arg)
{
    const cdc_acm_host_device_config_t config = {
        .connection_timeout_ms = 3000,
        .out_buffer_size = 128,
        .in_buffer_size = 256,
        .event_cb = mega_event_callback,
        .data_cb = mega_rx_callback,
        .user_arg = NULL,
    };
    const cdc_acm_line_coding_t line = {
        .dwDTERate = 115200,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 8,
    };
    static const uint8_t request[] = "STATUS\nGET_DISTANCE\n";

    while (true) {
        if (!s_mega_connected) {
            cdc_acm_dev_hdl_t handle = NULL;
            esp_err_t ret = ch34x_vcp_open(CH34X_PID_AUTO, 0, &config, &handle);
            if (ret == ESP_OK) {
                s_mega = handle;
                s_mega_connected = true;
                s_last_error = cdc_acm_host_line_coding_set(s_mega, &line);
                if (s_last_error == ESP_OK) {
                    s_last_error = cdc_acm_host_set_control_line_state(s_mega, true, true);
                }
                if (s_last_error == ESP_OK) {
                    // Same downstream-Mega reset pulse proven by the temporary bridge:
                    // assert reset, release it, then allow boot + TCA/VL53 validation.
                    s_last_error = cdc_acm_host_set_control_line_state(s_mega, false, true);
                    if (s_last_error == ESP_OK) vTaskDelay(pdMS_TO_TICKS(100));
                    if (s_last_error == ESP_OK) {
                        s_last_error = cdc_acm_host_set_control_line_state(s_mega, true, true);
                    }
                    if (s_last_error == ESP_OK) vTaskDelay(pdMS_TO_TICKS(6500));
                }
                reading_set_status(s_last_error == ESP_OK ? "waiting_sample" : "line_config_error", s_last_error);
                ESP_LOGI(TAG, "Mega CH340 connected; STATUS polling active, IMU control=%s",
                         mega_imu_control_is_enabled() ? "ENABLED" : "LOCKED");
            } else if (ret != ESP_ERR_TIMEOUT && ret != ESP_ERR_NOT_FOUND) {
                s_last_error = ret;
                reading_set_status("open_error", ret);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (s_solver_target_active) {
            vTaskDelay(pdMS_TO_TICKS(MEGA_SR04_POLL_MS));
            continue;
        }

        esp_err_t ret = mega_write_line((const char *)request, 500);
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_reading.request_count++;
            s_reading.mega_connected = s_mega_connected;
            xSemaphoreGive(s_mutex);
        }
        if (ret != ESP_OK) {
            s_last_error = ret;
            reading_set_status("request_error", ret);
        }
        vTaskDelay(pdMS_TO_TICKS(MEGA_SR04_POLL_MS));
    }
}

esp_err_t mega_sr04_init(void)
{
    if (s_mutex) return ESP_OK;
    s_mutex = xSemaphoreCreateMutex();
    s_tx_mutex = xSemaphoreCreateMutex();
    if (!s_mutex || !s_tx_mutex) return ESP_ERR_NO_MEM;

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_RETURN_ON_ERROR(usb_host_install(&host_config), TAG, "USB host install failed");
    xTaskCreate(usb_lib_task, "usb_host", 4096, NULL, 5, NULL);

    const cdc_acm_host_driver_config_t cdc_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 4,
        .xCoreID = tskNO_AFFINITY,
        .new_dev_cb = NULL,
    };
    ESP_RETURN_ON_ERROR(cdc_acm_host_install(&cdc_config), TAG, "CDC host install failed");
    xTaskCreate(mega_sr04_task, "mega_sr04", 4096, NULL, 4, NULL);
    return ESP_OK;
}

void mega_sr04_get_reading(mega_sr04_reading_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->distance_mm = -1;
    snprintf(out->status, sizeof(out->status), "not_initialized");
    if (!s_mutex) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        *out = s_reading;
        out->mega_connected = s_mega_connected;
        int64_t now = esp_timer_get_time();
        out->sample_age_us = out->timestamp_us > 0 ? now - out->timestamp_us : -1;
        if (out->valid && out->sample_age_us > MEGA_SR04_STALE_US) {
            out->valid = false;
            snprintf(out->status, sizeof(out->status), "stale");
        }
        xSemaphoreGive(s_mutex);
    }
}

esp_err_t mega_solver_send_targets(uint16_t fl_mm, uint16_t fr_mm,
                                   uint16_t rl_mm, uint16_t rr_mm)
{
    const uint16_t lengths[] = {fl_mm, fr_mm, rl_mm, rr_mm};
    for (size_t i = 0; i < sizeof(lengths) / sizeof(lengths[0]); ++i) {
        if (lengths[i] < 250 || lengths[i] > 390) return ESP_ERR_INVALID_ARG;
    }

    char command[64];
    int length = snprintf(command, sizeof(command), "SET_TARGET %u %u %u %u\n",
                          (unsigned)fl_mm, (unsigned)fr_mm,
                          (unsigned)rl_mm, (unsigned)rr_mm);
    if (length <= 0 || length >= (int)sizeof(command)) return ESP_ERR_INVALID_SIZE;

    esp_err_t ret = mega_write_line(command, 1000);
    if (ret == ESP_OK) {
        s_solver_target_active = true;
        ESP_LOGI(TAG, "solver targets sent FL=%u FR=%u RL=%u RR=%u",
                 (unsigned)fl_mm, (unsigned)fr_mm, (unsigned)rl_mm, (unsigned)rr_mm);
    } else {
        ESP_LOGW(TAG, "solver target send failed: 0x%x", ret);
    }
    return ret;
}

/* ================================================================
 * IMU controller command API
 * ================================================================ */

/* Compile-time IMU command gate is defined in main.c and defaults to 0. */
static esp_err_t imu_send_command(const char *cmd, bool emergency_stop_path)
{
    if (!emergency_stop_path && !mega_imu_control_is_enabled()) return ESP_ERR_NOT_SUPPORTED;
    return mega_write_line(cmd, MEGA_IMU_CMD_TIMEOUT_MS);
}

esp_err_t mega_remote_arm(void)
{
    return imu_send_command("ARM REMOTE\n", false);
}

esp_err_t mega_remote_set_absolute(uint32_t seq,
                                   uint16_t fl, uint16_t fr,
                                   uint16_t rl, uint16_t rr)
{
    const uint16_t values[] = {fl, fr, rl, rr};
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        if (values[i] > 400) return ESP_ERR_INVALID_ARG;
    }
    char cmd[96];
    int len = snprintf(cmd, sizeof(cmd), "SET_REMOTE %lu %u %u %u %u\n",
                       (unsigned long)seq, (unsigned)fl, (unsigned)fr,
                       (unsigned)rl, (unsigned)rr);
    if (seq == 0 || len <= 0 || len >= (int)sizeof(cmd)) return ESP_ERR_INVALID_ARG;
    return imu_send_command(cmd, false);
}

esp_err_t mega_imu_arm(void)
{
    return imu_send_command("ARM IMU\n", false);
}

esp_err_t mega_imu_stop(void)
{
    // STOP/ESTOP must remain available even if the motion compile gate is closed
    // after a fault or during rollback.
    return imu_send_command("STOP\n", true);
}

esp_err_t mega_imu_estop(void)
{
    return imu_send_command("ESTOP\n", true);
}

esp_err_t mega_imu_heartbeat(uint32_t seq)
{
    char cmd[32];
    int len = snprintf(cmd, sizeof(cmd), "HEARTBEAT %lu\n", (unsigned long)seq);
    if (len <= 0 || len >= (int)sizeof(cmd)) return ESP_ERR_INVALID_SIZE;
    return imu_send_command(cmd, false);
}

esp_err_t mega_imu_set_delta(uint32_t seq,
                             int16_t fl, int16_t fr,
                             int16_t rl, int16_t rr)
{
    if (fl < -350 || fl > 350 || fr < -350 || fr > 350 ||
        rl < -350 || rl > 350 || rr < -350 || rr > 350) {
        return ESP_ERR_INVALID_ARG;
    }
    char cmd[96];
    int len = snprintf(cmd, sizeof(cmd), "SET_DELTA %lu %d %d %d %d\n",
                       (unsigned long)seq, (int)fl, (int)fr, (int)rl, (int)rr);
    if (len <= 0 || len >= (int)sizeof(cmd)) return ESP_ERR_INVALID_SIZE;
    return imu_send_command(cmd, false);
}

void mega_imu_get_state(mega_imu_state_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!s_mutex) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        *out = s_imu_state;
        out->connected = s_mega_connected;
        int64_t now = esp_timer_get_time();
        if (out->timestamp_us > 0 && (now - out->timestamp_us) > MEGA_SR04_STALE_US) {
            out->connected = false;
        }
        xSemaphoreGive(s_mutex);
    }
}
