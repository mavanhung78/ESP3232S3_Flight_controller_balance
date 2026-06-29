#include "power_distribution.h"

#include "board_config.h"
#include "drivers/esc_oneshot_ledc.h"

static float constrainf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void power_distribution_init(void)
{
    esc_oneshot_init();
}

void power_distribution_mix(const control_t *control, const setpoint_t *setpoint, motor_power_t *motors)
{
    if (!setpoint->armed || setpoint->failsafe || control->thrust < THROTTLE_CUTOFF) {
        motors->m1 = motors->m2 = motors->m3 = motors->m4 = 0.0f;
        return;
    }

    // Crazyflie-style X quad mixer. Change signs/order if your motor layout is different.
    // M1 = front-left, M2 = front-right, M3 = rear-right, M4 = rear-left is one common mapping.
    float t = control->thrust;
    if (t > 0.0f && t < MOTOR_IDLE_POWER) {
        t = MOTOR_IDLE_POWER;
    }

    motors->m1 = t - control->roll + control->pitch + control->yaw;
    motors->m2 = t - control->roll - control->pitch - control->yaw;
    motors->m3 = t + control->roll - control->pitch + control->yaw;
    motors->m4 = t + control->roll + control->pitch - control->yaw;

    motors->m1 = constrainf_local(motors->m1, 0.0f, 1.0f);
    motors->m2 = constrainf_local(motors->m2, 0.0f, 1.0f);
    motors->m3 = constrainf_local(motors->m3, 0.0f, 1.0f);
    motors->m4 = constrainf_local(motors->m4, 0.0f, 1.0f);
}

void power_distribution_apply(const motor_power_t *motors)
{
    esc_oneshot_write_all(motors);
}

void power_distribution_stop(void)
{
    esc_oneshot_stop_all();
}
