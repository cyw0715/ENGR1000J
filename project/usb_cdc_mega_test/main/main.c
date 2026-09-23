/*
 * P4-NANO PC <-> Arduino Mega framed USB bridge.
 *
 * USB path: P4 USB-A Host -> CH340 -> Mega USB-B.
 * PC path:  P4 UART0/CH343 (COM5), framed protocol only.
 *
 * Safety scope: transport, baud, DTR/RTS and reset pulse only. No GPIO,
 * camera, Wi-Fi, IMU, PWM, motor, actuator, STK500, or Mega flash writes.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.h"

#define PC_UART UART_NUM_0
#define PC_BAUD 115200
#define FRAME_SOF0 0xA5
#define FRAME_SOF1 0x5A
#define FRAME_MAX_PAYLOAD 512

#define CMD_HELLO       0x01
#define RSP_HELLO       0x02
#define CMD_MEGA_TX     0x10
#define RSP_MEGA_RX     0x11
#define CMD_SET_LINE    0x12
#define CMD_SET_CONTROL 0x13
#define CMD_RESET_MEGA  0x14
#define CMD_GET_STATUS  0x15
#define RSP_STATUS      0x16
#define RSP_ERROR       0x7F

static SemaphoreHandle_t s_pc_tx_lock;
static volatile bool s_usb_device_seen = false;
static volatile uint16_t s_usb_vid = 0;
static volatile uint16_t s_usb_pid = 0;
static volatile uint8_t s_usb_addr = 0;
static volatile esp_err_t s_ch34x_open_error = ESP_ERR_NOT_FOUND;
static cdc_acm_dev_hdl_t s_mega = NULL;
static volatile bool s_mega_connected = false;
static volatile esp_err_t s_last_error = ESP_OK;

static uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void pc_send_frame(uint8_t type, const uint8_t *payload, uint16_t length)
{
    if (length > FRAME_MAX_PAYLOAD) return;
    uint8_t header[5] = { FRAME_SOF0, FRAME_SOF1, type, (uint8_t)length, (uint8_t)(length >> 8) };
    uint8_t crc_input[3 + FRAME_MAX_PAYLOAD];
    crc_input[0] = type;
    crc_input[1] = header[3];
    crc_input[2] = header[4];
    if (length && payload) memcpy(&crc_input[3], payload, length);
    uint16_t crc = crc16_ccitt(crc_input, 3 + length);
    if (s_pc_tx_lock && xSemaphoreTake(s_pc_tx_lock, pdMS_TO_TICKS(1000)) != pdTRUE) return;
    uart_write_bytes(PC_UART, (const char *)header, sizeof(header));
    if (length && payload) uart_write_bytes(PC_UART, (const char *)payload, length);
    const uint8_t crc_bytes[2] = { (uint8_t)crc, (uint8_t)(crc >> 8) };
    uart_write_bytes(PC_UART, (const char *)crc_bytes, sizeof(crc_bytes));
    if (s_pc_tx_lock) xSemaphoreGive(s_pc_tx_lock);
}

static void pc_send_error(uint8_t command, esp_err_t error)
{
    uint8_t payload[5] = { command, (uint8_t)error, (uint8_t)(error >> 8),
                            (uint8_t)(error >> 16), (uint8_t)(error >> 24) };
    pc_send_frame(RSP_ERROR, payload, sizeof(payload));
}

static void usb_new_device_callback(usb_device_handle_t usb_dev)
{
    const usb_device_desc_t *desc = NULL;
    usb_device_info_t info = {0};
    if (usb_host_get_device_descriptor(usb_dev, &desc) == ESP_OK) {
        s_usb_device_seen = true;
        s_usb_vid = desc->idVendor;
        s_usb_pid = desc->idProduct;
        if (usb_host_device_info(usb_dev, &info) == ESP_OK) {
            s_usb_addr = info.dev_addr;
        }
    }
}

static void pc_send_status(void)
{
    int32_t last_error = s_last_error;
    int32_t open_error = s_ch34x_open_error;
    uint8_t payload[15] = {
        s_mega_connected ? 1 : 0,
        s_usb_device_seen ? 1 : 0,
        s_usb_addr,
        (uint8_t)s_usb_vid, (uint8_t)(s_usb_vid >> 8),
        (uint8_t)s_usb_pid, (uint8_t)(s_usb_pid >> 8),
        (uint8_t)last_error, (uint8_t)(last_error >> 8), (uint8_t)(last_error >> 16), (uint8_t)(last_error >> 24),
        (uint8_t)open_error, (uint8_t)(open_error >> 8), (uint8_t)(open_error >> 16), (uint8_t)(open_error >> 24),
    };
    pc_send_frame(RSP_STATUS, payload, sizeof(payload));
}

static bool mega_rx_callback(const uint8_t *data, size_t data_len, void *user_arg)
{
    while (data_len > 0) {
        uint16_t chunk = data_len > FRAME_MAX_PAYLOAD ? FRAME_MAX_PAYLOAD : (uint16_t)data_len;
        pc_send_frame(RSP_MEGA_RX, data, chunk);
        data += chunk;
        data_len -= chunk;
    }
    return true;
}

static void mega_event_callback(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        s_mega_connected = false;
        s_mega = NULL;
        s_last_error = ESP_ERR_NOT_FOUND;
    } else if (event->type == CDC_ACM_HOST_ERROR) {
        s_last_error = event->data.error;
    }
}

static void usb_lib_task(void *arg)
{
    while (true) {
        uint32_t flags = 0;
        esp_err_t ret = usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (ret != ESP_OK) {
            s_last_error = ret;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void mega_open_task(void *arg)
{
    const cdc_acm_host_device_config_t config = {
        .connection_timeout_ms = 3000,
        .out_buffer_size = 512,
        .in_buffer_size = 512,
        .event_cb = mega_event_callback,
        .data_cb = mega_rx_callback,
        .user_arg = NULL,
    };
    while (true) {
        if (!s_mega_connected) {
            cdc_acm_dev_hdl_t handle = NULL;
            esp_err_t ret = ch34x_vcp_open(CH34X_PID_AUTO, 0, &config, &handle);
            if (ret == ESP_OK) {
                s_ch34x_open_error = ESP_OK;
                s_mega = handle;
                s_mega_connected = true;
                s_last_error = ESP_OK;
                const cdc_acm_line_coding_t line = {
                    .dwDTERate = 115200, .bCharFormat = 0, .bParityType = 0, .bDataBits = 8,
                };
                s_last_error = cdc_acm_host_line_coding_set(s_mega, &line);
                if (s_last_error == ESP_OK) {
                    s_last_error = cdc_acm_host_set_control_line_state(s_mega, true, true);
                }
            } else {
                s_ch34x_open_error = ret;
                if (ret != ESP_ERR_TIMEOUT && ret != ESP_ERR_NOT_FOUND) {
                    s_last_error = ret;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void handle_command(uint8_t type, const uint8_t *payload, uint16_t length)
{
    if (type == CMD_HELLO) {
        static const uint8_t hello[] = { 'P','4','M','E','G','A','1' };
        pc_send_frame(RSP_HELLO, hello, sizeof(hello));
        pc_send_status();
        return;
    }
    if (type == CMD_GET_STATUS) {
        pc_send_status();
        return;
    }
    if (!s_mega_connected || s_mega == NULL) {
        pc_send_error(type, ESP_ERR_NOT_FOUND);
        return;
    }
    if (type == CMD_MEGA_TX) {
        if (length == 0) { pc_send_error(type, ESP_ERR_INVALID_ARG); return; }
        s_last_error = cdc_acm_host_data_tx_blocking(s_mega, payload, length, 1000);
        if (s_last_error != ESP_OK) pc_send_error(type, s_last_error); else pc_send_status();
        return;
    }
    if (type == CMD_SET_LINE) {
        if (length != 7) { pc_send_error(type, ESP_ERR_INVALID_SIZE); return; }
        const cdc_acm_line_coding_t line = {
            .dwDTERate = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
                         ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24),
            .bCharFormat = payload[4], .bParityType = payload[5], .bDataBits = payload[6],
        };
        s_last_error = cdc_acm_host_line_coding_set(s_mega, &line);
        if (s_last_error != ESP_OK) pc_send_error(type, s_last_error); else pc_send_status();
        return;
    }
    if (type == CMD_SET_CONTROL) {
        if (length != 1) { pc_send_error(type, ESP_ERR_INVALID_SIZE); return; }
        s_last_error = cdc_acm_host_set_control_line_state(s_mega, payload[0] & 1, payload[0] & 2);
        if (s_last_error != ESP_OK) pc_send_error(type, s_last_error); else pc_send_status();
        return;
    }
    if (type == CMD_RESET_MEGA) {
        if (length != 0) { pc_send_error(type, ESP_ERR_INVALID_SIZE); return; }
        s_last_error = cdc_acm_host_set_control_line_state(s_mega, false, true);
        if (s_last_error == ESP_OK) vTaskDelay(pdMS_TO_TICKS(100));
        if (s_last_error == ESP_OK) s_last_error = cdc_acm_host_set_control_line_state(s_mega, true, true);
        if (s_last_error != ESP_OK) pc_send_error(type, s_last_error); else pc_send_status();
        return;
    }
    pc_send_error(type, ESP_ERR_NOT_SUPPORTED);
}

static void pc_bridge_task(void *arg)
{
    enum { WAIT_SOF0, WAIT_SOF1, READ_HEADER, READ_PAYLOAD, READ_CRC } state = WAIT_SOF0;
    uint8_t header[3];
    uint8_t payload[FRAME_MAX_PAYLOAD];
    uint8_t crc_bytes[2];
    size_t pos = 0, expected = 0;
    uint8_t ch;

    while (true) {
        if (uart_read_bytes(PC_UART, &ch, 1, pdMS_TO_TICKS(100)) != 1) continue;
        if (state == WAIT_SOF0) { state = (ch == FRAME_SOF0) ? WAIT_SOF1 : WAIT_SOF0; continue; }
        if (state == WAIT_SOF1) { state = (ch == FRAME_SOF1) ? READ_HEADER : WAIT_SOF0; pos = 0; continue; }
        if (state == READ_HEADER) {
            header[pos++] = ch;
            if (pos == sizeof(header)) {
                expected = (size_t)header[1] | ((size_t)header[2] << 8);
                if (expected > FRAME_MAX_PAYLOAD) { state = WAIT_SOF0; } else { pos = 0; state = expected ? READ_PAYLOAD : READ_CRC; }
            }
            continue;
        }
        if (state == READ_PAYLOAD) {
            payload[pos++] = ch;
            if (pos == expected) { pos = 0; state = READ_CRC; }
            continue;
        }
        crc_bytes[pos++] = ch;
        if (pos == sizeof(crc_bytes)) {
            uint8_t crc_input[3 + FRAME_MAX_PAYLOAD];
            memcpy(crc_input, header, sizeof(header));
            if (expected) memcpy(&crc_input[3], payload, expected);
            uint16_t calculated = crc16_ccitt(crc_input, 3 + expected);
            uint16_t received = (uint16_t)crc_bytes[0] | ((uint16_t)crc_bytes[1] << 8);
            if (calculated == received) handle_command(header[0], payload, expected);
            state = WAIT_SOF0;
        }
    }
}

void app_main(void)
{
    s_pc_tx_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_pc_tx_lock ? ESP_OK : ESP_ERR_NO_MEM);
    uart_driver_delete(PC_UART); // console owns UART0 before app_main; bridge needs raw bytes.
    const uart_config_t pc_config = {
        .baud_rate = PC_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PC_UART, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PC_UART, &pc_config));
    ESP_ERROR_CHECK(uart_set_pin(PC_UART, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    const usb_host_config_t host_config = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 5, NULL);

    const cdc_acm_host_driver_config_t cdc_config = {
        .driver_task_stack_size = 4096, .driver_task_priority = 4,
        .xCoreID = tskNO_AFFINITY, .new_dev_cb = usb_new_device_callback,
    };
    ESP_ERROR_CHECK(cdc_acm_host_install(&cdc_config));
    xTaskCreate(mega_open_task, "mega_ch340", 4096, NULL, 4, NULL);
    xTaskCreate(pc_bridge_task, "pc_bridge", 4096, NULL, 5, NULL);
}
