#include "stabilizer.h"

#include "board_config.h"
#include "commander_sbus.h"
#include "controller_pid.h"
#include "drivers/status_led.h"
#include "estimator_complementary.h"
#include "power_distribution.h"
#include "sensors.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STABILIZER";

static sensorData_t sensorData;
static state_t state;
static setpoint_t setpoint;
static control_t control;
static motor_power_t motors;

static const char *arm_state_str(arm_state_t s)
{
    switch (s) {
    case ARM_STATE_DISARMED:     return "DISARMED";
    case ARM_STATE_ARMING:       return "ARMING";
    case ARM_STATE_ARMED_IDLE:   return "ARMED_IDLE";
    case ARM_STATE_FLYING:       return "FLYING";
    case ARM_STATE_FAILSAFE:     return "FAILSAFE";
    case ARM_STATE_SENSOR_ERROR: return "SENSOR_ERROR";
    default:                     return "UNKNOWN";
    }
}

static void set_all_motors_zero(void)
{
    motors.m1 = 0.0f;
    motors.m2 = 0.0f;
    motors.m3 = 0.0f;
    motors.m4 = 0.0f;
    power_distribution_apply(&motors);
}

static void stabilizer_task(void *arg)
{
    ESP_LOGI(TAG, "Waiting for sensor calibration...");

    while (!sensors_are_calibrated()) {
        status_led_update(ARM_STATE_ARMING, true);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    estimator_complementary_init(&state);
    controller_pid_reset();
    power_distribution_stop();

    ESP_LOGI(TAG, "Starting stabilizer loop at %d Hz", STABILIZER_RATE_HZ);

    const TickType_t period_ticks = pdMS_TO_TICKS(1000 / STABILIZER_RATE_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t last_us = esp_timer_get_time();
    uint32_t loop_count = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period_ticks);

        const uint64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;

        if (dt <= 0.0f || dt > 0.02f) {
            dt = 1.0f / (float)STABILIZER_RATE_HZ;
        }

        esp_err_t sensor_err = sensors_acquire(&sensorData);
        bool sensor_ok = (sensor_err == ESP_OK) && sensors_is_healthy();

        commander_get_setpoint(&setpoint);

        if (!sensor_ok) {
            ESP_LOGW(TAG, "sensor read failed -> stop motors");
            setpoint.armed = false;
            setpoint.flight_enabled = false;
            setpoint.failsafe = true;
            setpoint.arm_state = ARM_STATE_SENSOR_ERROR;

            controller_pid_reset();
            power_distribution_stop();
            set_all_motors_zero();

            status_led_update(ARM_STATE_SENSOR_ERROR, false);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (setpoint.failsafe || setpoint.arm_state == ARM_STATE_FAILSAFE) {
            controller_pid_reset();
            power_distribution_stop();
            set_all_motors_zero();
            status_led_update(ARM_STATE_FAILSAFE, true);
        } else {
            estimator_complementary_update(&state, &sensorData, dt);

            controller_pid_update(&control, &setpoint, &sensorData, &state, dt);
            power_distribution_mix(&control, &setpoint, &motors);
            power_distribution_apply(&motors);

            status_led_update(setpoint.arm_state, true);
        }

        if (++loop_count >= (STABILIZER_RATE_HZ / LOG_RATE_HZ)) {
            loop_count = 0;

            ESP_LOGI(TAG,
                     "state=%s arm=%d fly=%d fs=%d thr=%.2f roll=%.2f pitch=%.2f gyro=[%.1f %.1f %.1f] m=[%.3f %.3f %.3f %.3f]",
                     arm_state_str(setpoint.arm_state),
                     setpoint.armed,
                     setpoint.flight_enabled,
                     setpoint.failsafe,
                     setpoint.thrust,
                     state.attitude.roll,
                     state.attitude.pitch,
                     sensorData.gyro.x,
                     sensorData.gyro.y,
                     sensorData.gyro.z,
                     motors.m1,
                     motors.m2,
                     motors.m3,
                     motors.m4);
        }
    }
}

esp_err_t stabilizer_init(void)
{
    ESP_LOGI(TAG, "Init modules");

    ESP_ERROR_CHECK(status_led_init());

    ESP_ERROR_CHECK(sensors_init());
    ESP_ERROR_CHECK(commander_sbus_init());

    controller_pid_init();
    power_distribution_init();

    ESP_ERROR_CHECK(sensors_calibrate());

    xTaskCreatePinnedToCore(stabilizer_task, "stabilizer", 8192, NULL, 10, NULL, 1);

    return ESP_OK;
}