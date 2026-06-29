#include "estimator_complementary.h"

#include <math.h>

#define RAD_TO_DEG_F 57.2957795131f

static float wrap_180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void estimator_complementary_init(state_t *state)
{
    state->attitude.roll = 0.0f;
    state->attitude.pitch = 0.0f;
    state->attitude.yaw = 0.0f;
    state->gyro.x = state->gyro.y = state->gyro.z = 0.0f;
}

void estimator_complementary_update(state_t *state, const sensorData_t *sensor, float dt)
{
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 1.0f / 500.0f;
    }

    const float ax = sensor->acc.x;
    const float ay = sensor->acc.y;
    const float az = sensor->acc.z;

    const float roll_acc = atan2f(ay, az) * RAD_TO_DEG_F;
    const float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;

    // Time-constant based alpha. Increase tau for more gyro trust, decrease for more accel trust.
    const float tau = 0.50f;
    const float alpha = tau / (tau + dt);

    state->attitude.roll = alpha * (state->attitude.roll + sensor->gyro.x * dt) +
                           (1.0f - alpha) * roll_acc;
    state->attitude.pitch = alpha * (state->attitude.pitch + sensor->gyro.y * dt) +
                            (1.0f - alpha) * pitch_acc;
    state->attitude.yaw = wrap_180(state->attitude.yaw + sensor->gyro.z * dt);

    state->gyro = sensor->gyro;
}
