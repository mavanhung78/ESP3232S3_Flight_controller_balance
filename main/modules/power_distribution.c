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

void power_distribution_mix(const control_t *control,
                            const setpoint_t *setpoint,
                            motor_power_t *motors)
{
    motors->m1 = 0.0f;
    motors->m2 = 0.0f;
    motors->m3 = 0.0f;
    motors->m4 = 0.0f;

    // Failsafe hoặc disarm -> motor về 0
    if (setpoint->failsafe || !setpoint->armed) {
        return;
    }

    // Đã arm nhưng chưa cất cánh -> giữ motor ở mức cố định
    if (setpoint->armed && !setpoint->flight_enabled) {
        motors->m1 = MOTOR_ARM_IDLE_POWER;
        motors->m2 = MOTOR_ARM_IDLE_POWER;
        motors->m3 = MOTOR_ARM_IDLE_POWER;
        motors->m4 = MOTOR_ARM_IDLE_POWER;
        return;
    }

    // Đang flying nhưng thrust quá thấp -> quay về idle, không bật/tắt liên tục
    if (control->thrust < THROTTLE_CUTOFF) {
        motors->m1 = MOTOR_ARM_IDLE_POWER;
        motors->m2 = MOTOR_ARM_IDLE_POWER;
        motors->m3 = MOTOR_ARM_IDLE_POWER;
        motors->m4 = MOTOR_ARM_IDLE_POWER;
        return;
    }

    float t = control->thrust;
    if (t > 0.0f && t < MOTOR_IDLE_POWER) {
        t = MOTOR_IDLE_POWER;
    }

// X quad mixer theo layout thực tế:
//
//                 FRONT
//                  ↑
//
//          M2 CCW          M4 CW
//          GPIO4           GPIO5
//
//
//          M1 CW           M3 CCW
//          GPIO7           GPIO6
//
//                  ↓
//                 REAR
//

    motors->m1 = t + control->roll - control->pitch - control->yaw;  // M1 Rear-Left  CW
    motors->m2 = t + control->roll + control->pitch + control->yaw;  // M2 Front-Left CCW
    motors->m3 = t - control->roll - control->pitch + control->yaw;  // M3 Rear-Right CCW
    motors->m4 = t - control->roll + control->pitch - control->yaw;  // M4 Front-Right CW

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