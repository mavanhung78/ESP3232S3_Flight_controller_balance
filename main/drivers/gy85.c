#include "gy85.h"

#include <string.h>
#include "board_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GY85";

#define ADXL345_ADDR      0x53
#define ADXL345_DEVID     0x00
#define ADXL345_BW_RATE   0x2C
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FMT  0x31
#define ADXL345_DATAX0    0x32

#define ITG3205_ADDR      0x68
#define ITG3205_SMPLRT    0x15
#define ITG3205_DLPF_FS   0x16
#define ITG3205_INT_CFG   0x17
#define ITG3205_PWR_MGM   0x3E
#define ITG3205_GYRO_X_H  0x1D

static esp_err_t i2c_write_u8(uint8_t dev, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_PORT_NUM, dev, buf, sizeof(buf), pdMS_TO_TICKS(50));
}

static esp_err_t i2c_read(uint8_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT_NUM, dev, &reg, 1, data, len, pdMS_TO_TICKS(50));
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

esp_err_t gy85_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_NUM, &conf));
    esp_err_t err = i2c_driver_install(I2C_PORT_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    uint8_t id = 0;
    err = i2c_read(ADXL345_ADDR, ADXL345_DEVID, &id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 not found at 0x53");
        return err;
    }
    ESP_LOGI(TAG, "ADXL345 DEVID=0x%02X", id);

    // ADXL345: full resolution, +/-2g default range, measurement mode, 400 Hz output.
    ESP_ERROR_CHECK(i2c_write_u8(ADXL345_ADDR, ADXL345_DATA_FMT, 0x08));
    ESP_ERROR_CHECK(i2c_write_u8(ADXL345_ADDR, ADXL345_BW_RATE, 0x0C));
    ESP_ERROR_CHECK(i2c_write_u8(ADXL345_ADDR, ADXL345_POWER_CTL, 0x08));

    // ITG3205: reset then set PLL clock, 1 kHz internal / (1+0) sample, DLPF ~42 Hz, full scale.
    ESP_ERROR_CHECK(i2c_write_u8(ITG3205_ADDR, ITG3205_PWR_MGM, 0x80));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(i2c_write_u8(ITG3205_ADDR, ITG3205_PWR_MGM, 0x01));
    ESP_ERROR_CHECK(i2c_write_u8(ITG3205_ADDR, ITG3205_SMPLRT, 0x00));
    ESP_ERROR_CHECK(i2c_write_u8(ITG3205_ADDR, ITG3205_DLPF_FS, 0x1B));
    ESP_ERROR_CHECK(i2c_write_u8(ITG3205_ADDR, ITG3205_INT_CFG, 0x00));

    ESP_LOGI(TAG, "GY-85 accel+gyro init OK");
    return ESP_OK;
}

esp_err_t gy85_read_accel_gyro(vec3f_t *acc_g, vec3f_t *gyro_dps)
{
    uint8_t a[6];
    uint8_t g[6];

    esp_err_t err = i2c_read(ADXL345_ADDR, ADXL345_DATAX0, a, sizeof(a));
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_read(ITG3205_ADDR, ITG3205_GYRO_X_H, g, sizeof(g));
    if (err != ESP_OK) {
        return err;
    }

    const int16_t ax = le16(&a[0]);
    const int16_t ay = le16(&a[2]);
    const int16_t az = le16(&a[4]);

    // ADXL345 full-resolution scale is about 3.9 mg/LSB.
    acc_g->x = ax * 0.0039f;
    acc_g->y = ay * 0.0039f;
    acc_g->z = az * 0.0039f;

    const int16_t gx = be16(&g[0]);
    const int16_t gy = be16(&g[2]);
    const int16_t gz = be16(&g[4]);

    // ITG3205 sensitivity is 14.375 LSB/(deg/s).
    gyro_dps->x = gx / 14.375f;
    gyro_dps->y = gy / 14.375f;
    gyro_dps->z = gz / 14.375f;

    return ESP_OK;
}
