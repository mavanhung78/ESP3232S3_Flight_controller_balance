#pragma once

#include "esp_err.h"
#include "interface/stabilizer_types.h"

esp_err_t gy85_init(void);
esp_err_t gy85_read_accel_gyro(vec3f_t *acc_g, vec3f_t *gyro_dps);
