#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "esp_err.h"
#include <stdint.h>

esp_err_t ultrasonic_init(void);
int32_t ultrasonic_read_mm(void);
const char *ultrasonic_last_status(void);

#endif
