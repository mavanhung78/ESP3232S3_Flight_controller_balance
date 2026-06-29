#pragma once

#include "esp_err.h"
#include "interface/stabilizer_types.h"

esp_err_t esc_oneshot_init(void);
void esc_oneshot_write_all(const motor_power_t *motors);
void esc_oneshot_stop_all(void);
