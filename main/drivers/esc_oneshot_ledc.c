#include "esc_oneshot_ledc.h"

#include "board_config.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "ESC";

#define LEDC_MODE_USED        LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_USED       LEDC_TIMER_0
#define LEDC_DUTY_RES         LEDC_TIMER_10_BIT
#define LEDC_DUTY_MAX         ((1U << 10) - 1U)

static const ledc_channel_t channels[4] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
};

static const int motor_gpios[4] = {
    MOTOR1_GPIO,
    MOTOR2_GPIO,
    MOTOR3_GPIO,
    MOTOR4_GPIO,
};

static float constrainf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint32_t pulse_us_to_duty(float pulse_us)
{
    const float period_us = 1000000.0f / (float)ESC_PWM_FREQ_HZ;
    pulse_us = constrainf_local(pulse_us, ESC_PULSE_STOP_US, ESC_PULSE_MAX_US);
    return (uint32_t)((pulse_us / period_us) * (float)LEDC_DUTY_MAX);
}

static void write_channel(int index, float power)
{
    power = constrainf_local(power, 0.0f, 1.0f);
    const float pulse_us = ESC_PULSE_MIN_US + power * (ESC_PULSE_MAX_US - ESC_PULSE_MIN_US);
    const uint32_t duty = pulse_us_to_duty(pulse_us);
    ledc_set_duty(LEDC_MODE_USED, channels[index], duty);
    ledc_update_duty(LEDC_MODE_USED, channels[index]);
}

esp_err_t esc_oneshot_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE_USED,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER_USED,
        .freq_hz = ESC_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = motor_gpios[i],
            .speed_mode = LEDC_MODE_USED,
            .channel = channels[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_USED,
            .duty = pulse_us_to_duty(ESC_PULSE_STOP_US),
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }

    esc_oneshot_stop_all();
    ESP_LOGI(TAG, "ESC OneShot125-like LEDC init OK: %d Hz, %.0f-%.0f us",
             ESC_PWM_FREQ_HZ, ESC_PULSE_MIN_US, ESC_PULSE_MAX_US);
    return ESP_OK;
}

void esc_oneshot_write_all(const motor_power_t *motors)
{
    write_channel(0, motors->m1);
    write_channel(1, motors->m2);
    write_channel(2, motors->m3);
    write_channel(3, motors->m4);
}

void esc_oneshot_stop_all(void)
{
    motor_power_t stop = {0};
    esc_oneshot_write_all(&stop);
}
