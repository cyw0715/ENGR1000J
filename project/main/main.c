/*
 * Mars Lander - 800x800 灰度 三缓冲
 * 1. 摄像头DMA写入frame_buf
 * 2. 拷贝到local_buf (防止DMA覆盖)
 * 3. 转灰度到send_buf
 * 4. 发送
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "camera.h"

static const char *TAG = "STREAM";
static uint32_t SYNC = 0xAA55AA55;

#define SW 800
#define SH 800
#define FRAME_RGB (SW * SH * 2)
#define FRAME_GRAY (SW * SH)

// 三缓冲
static uint8_t *s_local_buf = NULL;   // 拷贝RGB565
static uint8_t *s_gray_a = NULL;      // 灰度A
static uint8_t *s_gray_b = NULL;      // 灰度B
static uint8_t *s_write_gray = NULL;  // 当前写入灰度
static uint8_t *s_send_gray = NULL;   // 当前发送灰度

static SemaphoreHandle_t s_frame_ready = NULL;
static SemaphoreHandle_t s_buf_free = NULL;

// 采集任务 (CPU1)
static void capture_task(void *arg)
{
    uint32_t fc = 0;

    while (1) {
        size_t len = 0;
        uint8_t *frame = camera_get_frame(&len, 5000);

        if (frame && len > 0) {
            // 等待缓冲区空闲
            if (xSemaphoreTake(s_buf_free, pdMS_TO_TICKS(1000)) == pdTRUE) {
                // 1. 先拷贝RGB565到本地缓冲 (防止DMA覆盖)
                memcpy(s_local_buf, frame, FRAME_RGB);

                // 2. RGB565 → 灰度
                uint16_t *src = (uint16_t *)s_local_buf;
                for (int i = 0; i < SW * SH; i++) {
                    uint16_t px = src[i];
                    uint8_t r8 = ((px >> 11) & 0x1F) * 255 / 31;
                    uint8_t g8 = ((px >> 5) & 0x3F) * 255 / 63;
                    uint8_t b8 = (px & 0x1F) * 255 / 31;
                    s_write_gray[i] = (uint8_t)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
                }

                // 3. 交换灰度缓冲
                uint8_t *tmp = s_write_gray;
                s_write_gray = s_send_gray;
                s_send_gray = tmp;

                xSemaphoreGive(s_frame_ready);
                fc++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Mars Lander - 800x800 Triple Buffer");

    esp_err_t ret = camera_init();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "fail: 0x%x", ret); while(1) vTaskDelay(1000); }
    camera_start();
    ESP_LOGI(TAG, "Camera %dx%d", SW, SH);

    vTaskDelay(pdMS_TO_TICKS(2000));

    // 分配缓冲
    s_local_buf = heap_caps_aligned_alloc(64, FRAME_RGB, MALLOC_CAP_SPIRAM);
    s_gray_a = heap_caps_aligned_alloc(64, FRAME_GRAY, MALLOC_CAP_SPIRAM);
    s_gray_b = heap_caps_aligned_alloc(64, FRAME_GRAY, MALLOC_CAP_SPIRAM);
    s_write_gray = s_gray_a;
    s_send_gray = s_gray_b;

    s_frame_ready = xSemaphoreCreateBinary();
    s_buf_free = xSemaphoreCreateBinary();
    xSemaphoreGive(s_buf_free);

    ESP_LOGI(TAG, "Buffers ready");

    xTaskCreatePinnedToCore(capture_task, "cap", 8192, NULL, 5, NULL, 1);

    esp_log_level_set("*", ESP_LOG_NONE);
    setvbuf(stdout, NULL, _IOFBF, 65536);

    uint32_t fc = 0;
    int64_t t0 = esp_timer_get_time();
    uint32_t out_size = FRAME_GRAY;

    while (1) {
        if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(5000)) == pdTRUE) {
            fwrite(&SYNC, 4, 1, stdout);
            fwrite(&out_size, 4, 1, stdout);
            fwrite(s_send_gray, out_size, 1, stdout);
            fflush(stdout);

            xSemaphoreGive(s_buf_free);

            fc++;
            if (fc % 5 == 0) {
                int64_t now = esp_timer_get_time();
                float fps = fc * 1000000.0f / (now - t0);
                esp_log_level_set("*", ESP_LOG_INFO);
                ESP_LOGI(TAG, "Frame %u %.2fFPS", fc, fps);
                esp_log_level_set("*", ESP_LOG_NONE);
            }
        }
    }
}
