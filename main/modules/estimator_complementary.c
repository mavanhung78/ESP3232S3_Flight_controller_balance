#include "estimator_complementary.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "EST_KALMAN";

#define RAD_TO_DEG_F 57.2957795f

// Thông số Kalman khởi đầu
// GY-85 + drone nhỏ dễ nhiễu khi motor chạy, nên measurement noise để hơi cao.
#define KALMAN_INITIAL_UNCERTAINTY   4.0f     // deg^2, tương đương 2 deg ban đầu
#define KALMAN_GYRO_NOISE_DPS        4.0f     // gyro process noise
#define KALMAN_ACC_NOISE_DEG         5.0f     // accelerometer angle noise

typedef struct {
    float angle_deg;
    float uncertainty;
} kalman_1d_t;

static kalman_1d_t kalman_roll;
static kalman_1d_t kalman_pitch;

static float constrainf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float wrap_180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void kalman_1d_init(kalman_1d_t *k, float initial_angle_deg)
{
    k->angle_deg = initial_angle_deg;
    k->uncertainty = KALMAN_INITIAL_UNCERTAINTY;
}

static float kalman_1d_update(kalman_1d_t *k,
                              float gyro_rate_dps,
                              float acc_angle_deg,
                              float dt)
{
    if (dt <= 0.0f || dt > 0.02f) {
        dt = 0.002f;   // fallback cho loop 500 Hz
    }

    /*
     * Prediction:
     * angle = angle + gyro_rate * dt
     */
    k->angle_deg += gyro_rate_dps * dt;

    /*
     * Uncertainty prediction:
     * P = P + dt^2 * gyro_noise^2
     */
    const float gyro_noise_var = KALMAN_GYRO_NOISE_DPS * KALMAN_GYRO_NOISE_DPS;
    k->uncertainty += dt * dt * gyro_noise_var;

    /*
     * Update bằng góc đo từ accelerometer.
     * K = P / (P + R)
     */
    const float acc_noise_var = KALMAN_ACC_NOISE_DEG * KALMAN_ACC_NOISE_DEG;
    float kalman_gain = k->uncertainty / (k->uncertainty + acc_noise_var);

    k->angle_deg += kalman_gain * (acc_angle_deg - k->angle_deg);
    k->uncertainty = (1.0f - kalman_gain) * k->uncertainty;

    return k->angle_deg;
}

void estimator_complementary_init(state_t *state)
{
    memset(state, 0, sizeof(*state));

    kalman_1d_init(&kalman_roll, 0.0f);
    kalman_1d_init(&kalman_pitch, 0.0f);

    ESP_LOGI(TAG, "Kalman 1D attitude estimator init OK");
}

void estimator_complementary_update(state_t *state,
                                    const sensorData_t *sensor,
                                    float dt)
{
    float ax = sensor->acc.x;
    float ay = sensor->acc.y;
    float az = sensor->acc.z;

    /*
     * Tính góc từ accelerometer.
     * Giữ cùng quy ước với project hiện tại:
     * roll  = xoay trái/phải
     * pitch = chúi/ngửa
     */
    float roll_acc = atan2f(ay, az) * RAD_TO_DEG_F;
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;

    // Giới hạn tránh giá trị bất thường khi rung mạnh
    roll_acc = constrainf_local(roll_acc, -85.0f, 85.0f);
    pitch_acc = constrainf_local(pitch_acc, -85.0f, 85.0f);

    /*
     * Kalman fusion:
     * gyro.x -> roll rate
     * gyro.y -> pitch rate
     *
     * Lưu ý: sensors.c của bạn đã remap GY-85 sang body axis.
     */
    float roll = kalman_1d_update(&kalman_roll,
                                  sensor->gyro.x,
                                  roll_acc,
                                  dt);

    float pitch = kalman_1d_update(&kalman_pitch,
                                   sensor->gyro.y,
                                   pitch_acc,
                                   dt);

    state->attitude.roll = roll;
    state->attitude.pitch = pitch;

    // Yaw hiện tại vẫn chỉ tích phân gyro, chưa có magnetometer correction
    state->attitude.yaw = wrap_180(state->attitude.yaw + sensor->gyro.z * dt);

    state->gyro = sensor->gyro;
}