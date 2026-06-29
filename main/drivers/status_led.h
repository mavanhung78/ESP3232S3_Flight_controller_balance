#pragma once

#include "esp_err.h"
#include "interface/stabilizer_types.h"

esp_err_t status_led_init(void);
void status_led_update(arm_state_t state, bool sensor_ok);