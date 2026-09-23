#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "esp_err.h"
#include <stdint.h>

typedef enum {
    ACT_FL = 0,
    ACT_FR,
    ACT_RL,
    ACT_RR,
    ACT_COUNT
} actuator_id_t;

typedef enum {
    ACT_STOP = 0,
    ACT_EXTEND,
    ACT_RETRACT
} actuator_dir_t;

esp_err_t actuator_init(void);
esp_err_t actuator_set(actuator_id_t id, actuator_dir_t dir, uint8_t duty);
esp_err_t actuator_set_length(actuator_id_t id, uint32_t length_mm);
esp_err_t actuator_stop_all(void);

#endif
