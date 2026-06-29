#include "sensors.h"

#include "drivers/gy85.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SENSORS";

static vec3f_t gyro_offset = {0};
static bool calibrated = false;

esp_err_t sensors_init(void)
{
    ESP_ERROR_CHECK(gy85_init());
    return ESP_OK;
}

esp_err_t sensors_calibrate(void)
{
    const int samples = 1000;
    vec3f_t acc, gyro;
    vec3f_t gyro_sum = {0};

    ESP_LOGI(TAG, "Keep drone level and still. Calibrating gyro...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int i = 0; i < samples; i++) {
        esp_err_t err = gy85_read_accel_gyro(&acc, &gyro);
        if (err != ESP_OK) {
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

    ESP_LOGI(TAG, "Gyro offset [dps]: x=%.3f y=%.3f z=%.3f",
             gyro_offset.x, gyro_offset.y, gyro_offset.z);
    return ESP_OK;
}

bool sensors_are_calibrated(void)
{
    return calibrated;
}

esp_err_t sensors_acquire(sensorData_t *sensor)
{
    vec3f_t acc, gyro;
    esp_err_t err = gy85_read_accel_gyro(&acc, &gyro);
    if (err != ESP_OK) {
        return err;
    }

    // IMPORTANT: axis direction depends on how GY-85 is mounted.
    // If control reacts in the wrong direction, change signs here first.
    sensor->acc.x = acc.x;
    sensor->acc.y = acc.y;
    sensor->acc.z = acc.z;

    sensor->gyro.x = gyro.x - gyro_offset.x;
    sensor->gyro.y = gyro.y - gyro_offset.y;
    sensor->gyro.z = gyro.z - gyro_offset.z;

    sensor->timestamp_us = esp_timer_get_time();
    return ESP_OK;
}
