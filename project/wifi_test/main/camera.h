/*
 * OV5647 MIPI CSI Camera Driver - ESP-IDF
 */

#ifndef CAMERA_H
#define CAMERA_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

esp_err_t camera_init(void);
esp_err_t camera_start(void);
esp_err_t camera_stop(void);
uint8_t *camera_get_frame(size_t *out_len, uint32_t timeout_ms);
bool camera_is_ready(void);
uint32_t camera_get_width(void);
uint32_t camera_get_height(void);

#endif
