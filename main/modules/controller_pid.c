#include "controller_pid.h"

#include <string.h>

#include "board_config.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_limit;
    float integ;
    float prev_error;
    float d_lpf;
} pid_axis_t;

static pid_axis_t pid_roll;
static pid_axis_t pid_pitch;
static pid_axis_t pid_yaw;

static float constrainf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void pid_axis_init(pid_axis_t *p, float kp, float ki, float kd, float i_limit)
{
    memset(p, 0, sizeof(*p));
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_limit = i_limit;
}

void controller_pid_init(void)
{
    // PID khởi đầu an toàn cho drone ~190g, motor 1404, prop 3025, frame ~100mm
    pid_axis_init(&pid_roll,  0.0022f, 0.00045f, 0.000012f, 0.06f);
    pid_axis_init(&pid_pitch, 0.0022f, 0.00045f, 0.000012f, 0.06f);

    // Yaw để thấp trước, vì yaw sai dễ làm drone xoay mạnh
    pid_axis_init(&pid_yaw,   0.0010f, 0.00020f, 0.000000f, 0.04f);
}

void controller_pid_reset(void)
{
    pid_roll.integ = 0.0f;
    pid_roll.prev_error = 0.0f;
    pid_roll.d_lpf = 0.0f;

    pid_pitch.integ = 0.0f;
    pid_pitch.prev_error = 0.0f;
    pid_pitch.d_lpf = 0.0f;

    pid_yaw.integ = 0.0f;
    pid_yaw.prev_error = 0.0f;
    pid_yaw.d_lpf = 0.0f;
}

static float pid_update(pid_axis_t *p, float error, float dt)
{
    if (dt <= 0.0f || dt > 0.02f) {
        dt = 1.0f / (float)STABILIZER_RATE_HZ;
    }

    p->integ += error * dt;
    p->integ = constrainf_local(p->integ, -p->i_limit, p->i_limit);

    float derivative = (error - p->prev_error) / dt;
    p->prev_error = error;

    // D-term low-pass filter, giảm giật motor do nhiễu gyro
    const float d_alpha = 0.18f;
    p->d_lpf += d_alpha * (derivative - p->d_lpf);

    return p->kp * error + p->ki * p->integ + p->kd * p->d_lpf;
}

void controller_pid_update(control_t *control,
                           const setpoint_t *setpoint,
                           const sensorData_t *sensor,
                           const state_t *state,
                           float dt)
{
    memset(control, 0, sizeof(*control));

    if (!setpoint->armed || !setpoint->flight_enabled || setpoint->failsafe) {
        controller_pid_reset();
        return;
    }

    if (setpoint->thrust < THROTTLE_CUTOFF) {
        controller_pid_reset();
        return;
    }

    const float angle_kp = 2.8f;
    const float max_rate_rp = 120.0f;

    const float roll_error_deg = setpoint->attitude.roll - state->attitude.roll;
    const float pitch_error_deg = setpoint->attitude.pitch - state->attitude.pitch;

    const float roll_rate_sp = constrainf_local(angle_kp * roll_error_deg, -max_rate_rp, max_rate_rp);
    const float pitch_rate_sp = constrainf_local(angle_kp * pitch_error_deg, -max_rate_rp, max_rate_rp);
    const float yaw_rate_sp = constrainf_local(setpoint->attitudeRate.z, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);

    const float roll_rate_error = roll_rate_sp - sensor->gyro.x;
    const float pitch_rate_error = pitch_rate_sp - sensor->gyro.y;
    const float yaw_rate_error = yaw_rate_sp - sensor->gyro.z;

    control->roll  = constrainf_local(pid_update(&pid_roll,  roll_rate_error,  dt), -0.08f, 0.08f);
    control->pitch = constrainf_local(pid_update(&pid_pitch, pitch_rate_error, dt), -0.08f, 0.08f);
    control->yaw   = constrainf_local(pid_update(&pid_yaw,   yaw_rate_error,   dt), -0.05f, 0.05f);
    control->thrust = constrainf_local(setpoint->thrust, 0.0f, 1.0f);
}