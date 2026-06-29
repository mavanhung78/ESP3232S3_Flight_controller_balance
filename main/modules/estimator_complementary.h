#pragma once

#include "interface/stabilizer_types.h"

void estimator_complementary_init(state_t *state);
void estimator_complementary_update(state_t *state, const sensorData_t *sensor, float dt);
