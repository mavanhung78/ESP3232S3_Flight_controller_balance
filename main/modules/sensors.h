#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "interface/stabilizer_types.h"

esp_err_t sensors_init(void);
esp_err_t sensors_calibrate(void);
bool sensors_are_calibrated(void);
esp_err_t sensors_acquire(sensorData_t *sensor);

// Thêm 2 dòng này
uint32_t sensors_get_error_count(void);
bool sensors_is_healthy(void);