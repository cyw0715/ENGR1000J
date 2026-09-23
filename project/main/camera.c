/*
 * OV5647 MIPI CSI Camera Driver - ESP-IDF
 * XCLK: GPIO40 (LEDC)  I2C: GPIO7(SDA), GPIO8(SCL)
 * 1920x1080 or 1280x720, PSRAM对齐缓冲
 *
 * 流程: LDO → XCLK → I2C → SCCB → 检测传感器 → 设置格式 → 启动流 → CSI → ISP
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_cam_sensor_xclk.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cam_sensor_types.h"
#include "esp_sccb_i2c.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_ldo_regulator.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "esp_heap_caps.h"
#include "camera.h"

static const char *TAG = "CAM";

// 引脚
#define CAM_I2C_PORT        0
#define CAM_I2C_SDA_PIN     7
#define CAM_I2C_SCL_PIN     8
#define CAM_I2C_FREQ        100000
#define CAM_XCLK_PIN        40
#define CAM_XCLK_FREQ       24000000
#define CAM_CSI_LDO_CHAN    3
#define CAM_CSI_LDO_MV      2500
#define CAM_CSI_DATA_LANES  2

// 状态
static esp_cam_ctlr_handle_t s_cam_handle = NULL;
static esp_cam_sensor_xclk_handle_t s_xclk_handle = NULL;
static esp_ldo_channel_handle_t s_ldo_handle = NULL;
static isp_proc_handle_t s_isp_handle = NULL;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static esp_sccb_io_handle_t s_sccb_handle = NULL;
static esp_cam_sensor_device_t *s_sensor_dev = NULL;
static uint8_t *s_frame_buffer = NULL;
static size_t s_frame_size = 0;
static SemaphoreHandle_t s_frame_ready = NULL;
static uint8_t *s_latest_frame = NULL;
static size_t s_latest_frame_len = 0;
static bool s_initialized = false;
static uint32_t s_width = 0;
static uint32_t s_height = 0;

// CSI回调
static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle,
                                        esp_cam_ctlr_trans_t *trans, void *user_data)
{
    trans->buffer = s_frame_buffer;
    trans->buflen = s_frame_size;
    return false;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle,
                                         esp_cam_ctlr_trans_t *trans, void *user_data)
{
    s_latest_frame = (uint8_t *)trans->buffer;
    s_latest_frame_len = trans->received_size;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_frame_ready, &xHigherPriorityTaskWoken);
    // 注意: IRAM_ATTR回调里不能用ESP_LOG, 但可以设置标记
    return xHigherPriorityTaskWoken;
}

// 检测传感器
static esp_cam_sensor_device_t *detect_sensor(esp_cam_sensor_config_t *cfg)
{
    esp_cam_sensor_detect_fn_t *start = NULL;
    esp_cam_sensor_detect_fn_t *end = NULL;
    esp_cam_sensor_detect_get_array(&start, &end);

    for (esp_cam_sensor_detect_fn_t *fn = start; fn < end; fn++) {
        if (fn->detect) {
            esp_cam_sensor_device_t *dev = fn->detect(cfg);
            if (dev) {
                ESP_LOGI(TAG, "Sensor detected via: port=%d, addr=0x%02X", fn->port, fn->sccb_addr);
                return dev;
            }
        }
    }
    return NULL;
}

esp_err_t camera_init(void)
{
    if (s_initialized) return ESP_OK;
    esp_err_t ret;

    s_frame_ready = xSemaphoreCreateBinary();
    ESP_LOGI(TAG, "Initializing camera...");

    // Step 1: LDO 2.5V for MIPI PHY
    esp_ldo_channel_config_t ldo_cfg = { .chan_id = CAM_CSI_LDO_CHAN, .voltage_mv = CAM_CSI_LDO_MV };
    ret = esp_ldo_acquire_channel(&ldo_cfg, &s_ldo_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "LDO failed: 0x%x", ret); return ret; }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 2: XCLK 24MHz
    esp_cam_sensor_xclk_config_t xclk_cfg = {
        .ledc_cfg = { .timer = LEDC_TIMER_0, .clk_cfg = LEDC_AUTO_CLK,
                      .channel = LEDC_CHANNEL_0, .xclk_freq_hz = CAM_XCLK_FREQ, .xclk_pin = CAM_XCLK_PIN },
    };
    ret = esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_LEDC, &s_xclk_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "XCLK alloc failed: 0x%x", ret); return ret; }
    ret = esp_cam_sensor_xclk_start(s_xclk_handle, &xclk_cfg);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "XCLK start failed: 0x%x", ret); return ret; }
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "XCLK 24MHz OK");

    // Step 3: I2C bus
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = CAM_I2C_PORT, .sda_io_num = CAM_I2C_SDA_PIN, .scl_io_num = CAM_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .intr_priority = 0, .trans_queue_depth = 0, .flags = { .enable_internal_pullup = 1 },
    };
    ret = i2c_new_master_bus(&i2c_cfg, &s_i2c_bus);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "I2C failed: 0x%x", ret); return ret; }

    // Step 4: SCCB I2C IO (sensor I2C handle)
    sccb_i2c_config_t sccb_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,  // OV5647 default
        .scl_speed_hz = CAM_I2C_FREQ,
        .addr_bits_width = 16,   // OV5647 uses 16-bit register addresses
        .val_bits_width = 8,     // 8-bit register values
    };
    ret = sccb_new_i2c_io(s_i2c_bus, &sccb_cfg, &s_sccb_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "SCCB init failed: 0x%x", ret); return ret; }

    // Step 5: 检测传感器
    esp_cam_sensor_config_t sensor_cfg = {
        .sccb_handle = s_sccb_handle,
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,       // XCLK已通过LEDC单独配置
        .xclk_freq_hz = 0,
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };
    s_sensor_dev = detect_sensor(&sensor_cfg);
    if (!s_sensor_dev) {
        ESP_LOGE(TAG, "No camera sensor detected!");
        ESP_LOGI(TAG, "Check: CSI cable, XCLK(GPIO40), I2C(GPIO7/8)");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Sensor: %s", esp_cam_sensor_get_name(s_sensor_dev));

    // Step 6: 查询格式，优先选1080p
    esp_cam_sensor_format_array_t fmt_array = {0};
    ret = esp_cam_sensor_query_format(s_sensor_dev, &fmt_array);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Query format failed: 0x%x", ret);
        return ret;
    }

    const esp_cam_sensor_format_t *selected_fmt = NULL;
    ESP_LOGI(TAG, "Available formats (%d):", fmt_array.count);
    for (int i = 0; i < fmt_array.count; i++) {
        const esp_cam_sensor_format_t *f = &fmt_array.format_array[i];
        ESP_LOGI(TAG, "  [%d] %s %dx%d", i, f->name ? f->name : "?", f->width, f->height);
        // 优先1080p
        if (f->height == 1080 && !selected_fmt) selected_fmt = f;
    }
    // fallback to 720p
    if (!selected_fmt) {
        for (int i = 0; i < fmt_array.count; i++) {
            if (fmt_array.format_array[i].height == 720) {
                selected_fmt = &fmt_array.format_array[i];
                break;
            }
        }
    }
    // fallback to first
    if (!selected_fmt && fmt_array.count > 0) {
        selected_fmt = &fmt_array.format_array[0];
    }

    if (!selected_fmt) {
        ESP_LOGE(TAG, "No format available!");
        return ESP_ERR_NOT_FOUND;
    }

    s_width = selected_fmt->width;
    s_height = selected_fmt->height;
    ESP_LOGI(TAG, "Selected: %s %dx%d", selected_fmt->name ? selected_fmt->name : "?", s_width, s_height);

    ret = esp_cam_sensor_set_format(s_sensor_dev, selected_fmt);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Set format failed: 0x%x", ret); return ret; }

    // Step 7: 启动传感器数据流
    bool stream_on = true;
    ret = esp_cam_sensor_ioctl(s_sensor_dev, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_on);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Start stream failed: 0x%x", ret); return ret; }
    ESP_LOGI(TAG, "Sensor streaming ON");

    // Step 8: CSI控制器
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id = 0, .h_res = s_width, .v_res = s_height,
        .data_lane_num = CAM_CSI_DATA_LANES, .lane_bit_rate_mbps = 200,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8, .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .queue_items = 1, .bk_buffer_dis = true, .byte_swap_en = false,
    };
    ret = esp_cam_new_csi_ctlr(&csi_cfg, &s_cam_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "CSI failed: 0x%x", ret); return ret; }

    esp_cam_ctlr_evt_cbs_t cbs = { .on_get_new_trans = on_get_new_trans, .on_trans_finished = on_trans_finished };
    esp_cam_ctlr_register_event_callbacks(s_cam_handle, &cbs, NULL);

    // Step 9: 帧缓冲 (PSRAM, 64字节对齐)
    s_frame_size = s_width * s_height * 2;
    s_frame_buffer = heap_caps_aligned_alloc(64, s_frame_size, MALLOC_CAP_SPIRAM);
    if (!s_frame_buffer) s_frame_buffer = heap_caps_malloc(s_frame_size, MALLOC_CAP_SPIRAM);
    if (!s_frame_buffer) { ESP_LOGE(TAG, "Buffer alloc failed! (%dKB)", s_frame_size/1024); return ESP_ERR_NO_MEM; }
    memset(s_frame_buffer, 0, s_frame_size);
    ESP_LOGI(TAG, "Buffer: %dKB @ %p (%dx%d RGB565)", s_frame_size/1024, s_frame_buffer, s_width, s_height);

    // Step 10: ISP (RAW8 → RGB565)
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80000000, .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8, .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false, .has_line_end_packet = false,
        .h_res = s_width, .v_res = s_height,
    };
    ret = esp_isp_new_processor(&isp_cfg, &s_isp_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "ISP failed: 0x%x", ret); return ret; }
    esp_isp_enable(s_isp_handle);

    // Step 11: 使能CSI
    esp_cam_ctlr_enable(s_cam_handle);

    s_initialized = true;
    ESP_LOGI(TAG, "Camera ready! %dx%d", s_width, s_height);
    return ESP_OK;
}

esp_err_t camera_start(void) { return s_initialized ? esp_cam_ctlr_start(s_cam_handle) : ESP_ERR_INVALID_STATE; }
esp_err_t camera_stop(void) { return s_initialized ? esp_cam_ctlr_stop(s_cam_handle) : ESP_ERR_INVALID_STATE; }

uint8_t *camera_get_frame(size_t *out_len, uint32_t timeout_ms)
{
    if (!s_initialized) return NULL;
    if (timeout_ms > 0 && xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return NULL;
    if (out_len) *out_len = s_latest_frame_len;
    return s_latest_frame;
}

bool camera_is_ready(void) { return s_initialized; }
uint32_t camera_get_width(void) { return s_width; }
uint32_t camera_get_height(void) { return s_height; }
