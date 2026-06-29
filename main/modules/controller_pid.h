#pragma once

#include "interface/stabilizer_types.h"

void controller_pid_init(void);
void controller_pid_reset(void);
void controller_pid_update(control_t *control,
                           const setpoint_t *setpoint,
                           const sensorData_t *sensor,
                           const state_t *state,
                           float dt);
