#pragma once

#include "esp_err.h"
#include "interface/stabilizer_types.h"

esp_err_t commander_sbus_init(void);
void commander_get_setpoint(setpoint_t *setpoint);
