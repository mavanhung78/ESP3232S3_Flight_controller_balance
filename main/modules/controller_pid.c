#include "controller_pid.h"

#include <string.h>
#include "board_config.h"

// This is a simple Crazyflie-style cascaded controller for first hover tests:
// angle P loop -> desired angular rate -> rate PID loop -> motor mixer command.

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_limit;
    float integ;
    float prev_error;
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
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_limit = i_limit;
    p->integ = 0.0f;
    p->prev_error = 0.0f;
}

void controller_pid_init(void)
{
    // Initial conservative gains for a small quad. Tune with props removed first.
    // Output unit is normalized motor correction, not PWM us.
    pid_axis_init(&pid_roll,  0.0040f, 0.0015f, 0.000035f, 0.18f);
    pid_axis_init(&pid_pitch, 0.0040f, 0.0015f, 0.000035f, 0.18f);
    pid_axis_init(&pid_yaw,   0.0022f, 0.0008f, 0.000000f, 0.12f);
}

void controller_pid_reset(void)
{
    pid_roll.integ = pid_pitch.integ = pid_yaw.integ = 0.0f;
    pid_roll.prev_error = pid_pitch.prev_error = pid_yaw.prev_error = 0.0f;
}

static float pid_update(pid_axis_t *p, float error, float dt)
{
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 1.0f / STABILIZER_RATE_HZ;
    }

    p->integ += error * dt;
    p->integ = constrainf_local(p->integ, -p->i_limit, p->i_limit);

    const float derivative = (error - p->prev_error) / dt;
    p->prev_error = error;

    return p->kp * error + p->ki * p->integ + p->kd * derivative;
}

void controller_pid_update(control_t *control,
                           const setpoint_t *setpoint,
                           const sensorData_t *sensor,
                           const state_t *state,
                           float dt)
{
    memset(control, 0, sizeof(*control));

    if (!setpoint->armed || setpoint->failsafe || setpoint->thrust < THROTTLE_CUTOFF) {
        controller_pid_reset();
        return;
    }

    // Outer attitude loop: desired angle -> desired angular rate.
    const float angle_kp = 4.5f;       // deg/s per deg error
    const float max_rate_rp = 220.0f;  // deg/s

    const float roll_error_deg = setpoint->attitude.roll - state->attitude.roll;
    const float pitch_error_deg = setpoint->attitude.pitch - state->attitude.pitch;

    const float roll_rate_sp = constrainf_local(angle_kp * roll_error_deg, -max_rate_rp, max_rate_rp);
    const float pitch_rate_sp = constrainf_local(angle_kp * pitch_error_deg, -max_rate_rp, max_rate_rp);
    const float yaw_rate_sp = constrainf_local(setpoint->attitudeRate.z, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);

    // Inner rate loop: desired angular rate -> normalized correction.
    const float roll_rate_error = roll_rate_sp - sensor->gyro.x;
    const float pitch_rate_error = pitch_rate_sp - sensor->gyro.y;
    const float yaw_rate_error = yaw_rate_sp - sensor->gyro.z;

    control->roll = constrainf_local(pid_update(&pid_roll, roll_rate_error, dt), -0.35f, 0.35f);
    control->pitch = constrainf_local(pid_update(&pid_pitch, pitch_rate_error, dt), -0.35f, 0.35f);
    control->yaw = constrainf_local(pid_update(&pid_yaw, yaw_rate_error, dt), -0.25f, 0.25f);
    control->thrust = constrainf_local(setpoint->thrust, 0.0f, 1.0f);
}
