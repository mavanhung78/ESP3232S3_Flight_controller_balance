#pragma once

#include "interface/stabilizer_types.h"

void power_distribution_init(void);
void power_distribution_mix(const control_t *control, const setpoint_t *setpoint, motor_power_t *motors);
void power_distribution_apply(const motor_power_t *motors);
void power_distribution_stop(void);
