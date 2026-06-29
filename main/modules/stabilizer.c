#include "stabilizer.h"

#include "board_config.h"
#include "commander_sbus.h"
#include "controller_pid.h"
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

static void stabilizer_task(void *arg)
{
    ESP_LOGI(TAG, "Waiting for sensor calibration...");
    while (!sensors_are_calibrated()) {
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

        if (sensors_acquire(&sensorData) != ESP_OK) {
            ESP_LOGW(TAG, "sensor read failed -> stop motors");
            power_distribution_stop();
            continue;
        }

        estimator_complementary_update(&state, &sensorData, dt);
        commander_get_setpoint(&setpoint);
        controller_pid_update(&control, &setpoint, &sensorData, &state, dt);
        power_distribution_mix(&control, &setpoint, &motors);
        power_distribution_apply(&motors);

        if (++loop_count >= (STABILIZER_RATE_HZ / LOG_RATE_HZ)) {
            loop_count = 0;
            ESP_LOGI(TAG,
                     "arm=%d fs=%d thr=%.2f roll=%.2f pitch=%.2f gyro=[%.1f %.1f %.1f] m=[%.2f %.2f %.2f %.2f]",
                     setpoint.armed, setpoint.failsafe, setpoint.thrust,
                     state.attitude.roll, state.attitude.pitch,
                     sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z,
                     motors.m1, motors.m2, motors.m3, motors.m4);
        }
    }
}

esp_err_t stabilizer_init(void)
{
    ESP_LOGI(TAG, "Init modules: sensors -> estimator -> commander -> controller -> power distribution");

    ESP_ERROR_CHECK(sensors_init());
    ESP_ERROR_CHECK(commander_sbus_init());
    controller_pid_init();
    power_distribution_init();

    ESP_ERROR_CHECK(sensors_calibrate());

    xTaskCreatePinnedToCore(stabilizer_task, "stabilizer", 8192, NULL, 10, NULL, 1);
    return ESP_OK;
}
