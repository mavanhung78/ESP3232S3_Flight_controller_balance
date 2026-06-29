#include "controller_pid.h"

#include <string.h>

#include "board_config.h"

typedef struct {
    float kp;
    float ki;
    float kd;

    float output_limit;
    float iterm_limit;

    float prev_error;
    float prev_iterm;
    float d_lpf;
} pid_axis_t;

static pid_axis_t angle_roll;
static pid_axis_t angle_pitch;

static pid_axis_t rate_roll;
static pid_axis_t rate_pitch;
static pid_axis_t rate_yaw;

static float constrainf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void pid_axis_init(pid_axis_t *p,
                          float kp,
                          float ki,
                          float kd,
                          float output_limit,
                          float iterm_limit)
{
    memset(p, 0, sizeof(*p));

    p->kp = kp;
    p->ki = ki;
    p->kd = kd;

    p->output_limit = output_limit;
    p->iterm_limit = iterm_limit;
}

static void pid_axis_reset(pid_axis_t *p)
{
    p->prev_error = 0.0f;
    p->prev_iterm = 0.0f;
    p->d_lpf = 0.0f;
}

/*
 * PID equation kiểu Carbon:
 *
 * Pterm = P * error
 * Iterm = PrevIterm + I * (error + prev_error) * dt / 2
 * Dterm = D * (error - prev_error) / dt
 *
 * Khác Carbon một chút:
 * - dùng dt thực tế thay vì cố định 0.004s
 * - thêm lọc D-term để giảm nhiễu GY-85
 * - output tính theo normalized motor correction
 */
static float pid_update(pid_axis_t *p, float error, float dt)
{
    if (dt <= 0.0f || dt > 0.02f) {
        dt = 1.0f / (float)STABILIZER_RATE_HZ;
    }

    float p_term = p->kp * error;

    float i_term = p->prev_iterm + p->ki * (error + p->prev_error) * dt * 0.5f;
    i_term = constrainf_local(i_term, -p->iterm_limit, p->iterm_limit);

    float derivative = (error - p->prev_error) / dt;

    // D low-pass filter, quan trọng với GY-85 vì motor gây nhiễu
    const float d_alpha = 0.15f;
    p->d_lpf += d_alpha * (derivative - p->d_lpf);

    float d_term = p->kd * p->d_lpf;

    float output = p_term + i_term + d_term;
    output = constrainf_local(output, -p->output_limit, p->output_limit);

    p->prev_error = error;
    p->prev_iterm = i_term;

    return output;
}

void controller_pid_init(void)
{
    /*
     * Angle loop giống Carbon:
     * Angle error -> Desired rate
     *
     * Output của angle PID là deg/s.
     * I và D để 0 trước cho an toàn.
     */
    pid_axis_init(&angle_roll,  2.4f, 0.0f, 0.0f, 120.0f, 0.0f);
    pid_axis_init(&angle_pitch, 2.4f, 0.0f, 0.0f, 120.0f, 0.0f);

    /*
     * Rate loop:
     * Desired rate - gyro rate -> motor correction
     *
     * Drone 190g, 1404, prop 3025, frame ~100mm:
     * để gain mềm trước, tránh rung/nóng motor.
     */
    pid_axis_init(&rate_roll,  0.0020f, 0.00030f, 0.000000f, 0.08f, 0.04f);
    pid_axis_init(&rate_pitch, 0.0020f, 0.00030f, 0.000000f, 0.08f, 0.04f);

    /*
     * Yaw rate PID:
     * Bắt đầu rất nhẹ.
     * Nếu yaw trôi chậm, tăng I yaw.
     * Nếu yaw giật/quay mạnh, kiểm tra dấu yaw trước, không tăng PID.
     */
    pid_axis_init(&rate_yaw,   0.0008f, 0.00008f, 0.000000f, 0.04f, 0.025f);
}

void controller_pid_reset(void)
{
    pid_axis_reset(&angle_roll);
    pid_axis_reset(&angle_pitch);

    pid_axis_reset(&rate_roll);
    pid_axis_reset(&rate_pitch);
    pid_axis_reset(&rate_yaw);
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

    /*
     * ================= Angle loop =================
     * Giống Carbon:
     * Desired angle - estimated angle -> desired angular rate
     */
    float error_angle_roll = setpoint->attitude.roll - state->attitude.roll;
    float error_angle_pitch = setpoint->attitude.pitch - state->attitude.pitch;

    float desired_rate_roll = pid_update(&angle_roll, error_angle_roll, dt);
    float desired_rate_pitch = pid_update(&angle_pitch, error_angle_pitch, dt);

    desired_rate_roll = constrainf_local(desired_rate_roll, -120.0f, 120.0f);
    desired_rate_pitch = constrainf_local(desired_rate_pitch, -120.0f, 120.0f);

    /*
     * Yaw không dùng angle hold.
     * Yaw chỉ dùng rate command từ tay điều khiển.
     */
    float desired_rate_yaw = constrainf_local(setpoint->attitudeRate.z,
                                              -MAX_YAW_RATE_DPS,
                                              MAX_YAW_RATE_DPS);

    /*
     * ================= Rate loop =================
     */
    float error_rate_roll = desired_rate_roll - sensor->gyro.x;
    float error_rate_pitch = desired_rate_pitch - sensor->gyro.y;
    float error_rate_yaw = desired_rate_yaw - sensor->gyro.z;

    control->roll = pid_update(&rate_roll, error_rate_roll, dt);
    control->pitch = pid_update(&rate_pitch, error_rate_pitch, dt);
    control->yaw = pid_update(&rate_yaw, error_rate_yaw, dt);

    control->thrust = constrainf_local(setpoint->thrust, 0.0f, 1.0f);
}