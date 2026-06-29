#include "sensors.h"

#include <string.h>
#include <math.h>

#include "board_config.h"
#include "drivers/gy85.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SENSORS";

static vec3f_t gyro_offset = {0};
static bool calibrated = false;

static bool filter_initialized = false;
static vec3f_t acc_lpf = {0};
static vec3f_t gyro_lpf = {0};

static uint32_t sensor_error_count = 0;

static float lpf_update(float old_value, float new_value, float alpha)
{
    return old_value + alpha * (new_value - old_value);
}

static vec3f_t vec_lpf(vec3f_t old_v, vec3f_t new_v, float alpha)
{
    vec3f_t out;
    out.x = lpf_update(old_v.x, new_v.x, alpha);
    out.y = lpf_update(old_v.y, new_v.y, alpha);
    out.z = lpf_update(old_v.z, new_v.z, alpha);
    return out;
}

esp_err_t sensors_init(void)
{
    ESP_ERROR_CHECK(gy85_init());
    return ESP_OK;
}

esp_err_t sensors_calibrate(void)
{
    const int samples = 1000;

    vec3f_t acc;
    vec3f_t gyro;
    vec3f_t gyro_sum = {0};

    ESP_LOGI(TAG, "Keep drone level and still. Calibrating gyro...");

    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int i = 0; i < samples; i++) {
        esp_err_t err = gy85_read_accel_gyro(&acc, &gyro);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Gyro calibration read failed");
            return err;
        }

        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum.z += gyro.z;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gyro_offset.x = gyro_sum.x / samples;
    gyro_offset.y = gyro_sum.y / samples;
    gyro_offset.z = gyro_sum.z / samples;

    calibrated = true;
    filter_initialized = false;
    sensor_error_count = 0;

    ESP_LOGI(TAG,
             "Gyro offset [dps]: x=%.3f y=%.3f z=%.3f",
             gyro_offset.x,
             gyro_offset.y,
             gyro_offset.z);

    return ESP_OK;
}

bool sensors_are_calibrated(void)
{
    return calibrated;
}

uint32_t sensors_get_error_count(void)
{
    return sensor_error_count;
}

bool sensors_is_healthy(void)
{
    return sensor_error_count < SENSOR_MAX_ERROR_COUNT;
}

esp_err_t sensors_acquire(sensorData_t *sensor)
{
    vec3f_t acc;
    vec3f_t gyro;

    esp_err_t err = gy85_read_accel_gyro(&acc, &gyro);
    if (err != ESP_OK) {
        sensor_error_count++;
        return err;
    }

    sensor_error_count = 0;

    gyro.x -= gyro_offset.x;
    gyro.y -= gyro_offset.y;
    gyro.z -= gyro_offset.z;

    /*
    * Remap GY-85 axis to drone body axis.
    *
    * Drone body axis:
    *   X = front
    *   Y = right
    *   Z = up/down
    *
    * Theo log hiện tại:
    *   Cúi đầu về trước -> roll đổi mạnh, pitch gần 0
    *
    * Nghĩa là trục X/Y đang bị mapping sai.
    * Dùng mapping này để đưa chuyển động chúi/ngửa về pitch.
    */

    vec3f_t acc_body;
    vec3f_t gyro_body;

    acc_body.x = -acc.x;
    acc_body.y =  acc.y;
    acc_body.z =  acc.z;

    gyro_body.x =  gyro.x;
    gyro_body.y = -gyro.y;
    gyro_body.z =  gyro.z;

    if (!filter_initialized) {
        acc_lpf = acc_body;
        gyro_lpf = gyro_body;
        filter_initialized = true;
    } else {
        acc_lpf = vec_lpf(acc_lpf, acc_body, SENSOR_ACC_LPF_ALPHA);
        gyro_lpf = vec_lpf(gyro_lpf, gyro_body, SENSOR_GYRO_LPF_ALPHA);
    }

    sensor->acc = acc_lpf;
    sensor->gyro = gyro_lpf;
    sensor->timestamp_us = esp_timer_get_time();

    return ESP_OK;
}