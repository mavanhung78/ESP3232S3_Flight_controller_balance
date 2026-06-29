#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"

// ===================== Board pins =====================
#define I2C_PORT_NUM        I2C_NUM_0
#define I2C_SDA_GPIO        GPIO_NUM_8
#define I2C_SCL_GPIO        GPIO_NUM_9
#define I2C_FREQ_HZ         400000

// SBUS receiver input
#define SBUS_UART_NUM       UART_NUM_1
#define SBUS_RX_GPIO        GPIO_NUM_16
#define SBUS_RX_INVERT      1

// ESC outputs
#define MOTOR1_GPIO         GPIO_NUM_4   // M1
#define MOTOR2_GPIO         GPIO_NUM_5   // M2
#define MOTOR3_GPIO         GPIO_NUM_6   // M3
#define MOTOR4_GPIO         GPIO_NUM_7   // M4

// WS2812 status LED on ESP32-S3 Dev Module
#define STATUS_LED_GPIO     GPIO_NUM_48
#define STATUS_LED_NUM      1

// ===================== Stabilizer timing =====================
#define STABILIZER_RATE_HZ  500
#define LOG_RATE_HZ         10

// ===================== RC / SBUS mapping =====================
#define SBUS_CH_ROLL        0
#define SBUS_CH_PITCH       1
#define SBUS_CH_THROTTLE    2
#define SBUS_CH_YAW         3
#define SBUS_CH_ARM         4

#define SBUS_VALUE_MIN      172
#define SBUS_VALUE_MID      992
#define SBUS_VALUE_MAX      1811

// Nếu quá thời gian này không có SBUS frame mới -> failsafe
#define SBUS_FAILSAFE_TIMEOUT_MS  150

// ===================== Arm / flight state =====================
// CH arm > 0.35 thì coi như bật arm
#define ARM_SWITCH_ON_THRESHOLD   0.35f

// Chỉ cho arm khi throttle nhỏ hơn ngưỡng này
#define ARM_THROTTLE_LOW_MAX      0.06f

// Phải giữ switch arm ổn định trong khoảng này mới arm
#define ARM_HOLD_TIME_MS          500

// Sau khi arm, motor quay idle cố định ở mức này
#define MOTOR_ARM_IDLE_POWER      0.10f

// Tăng throttle vượt mức này mới chuyển từ ARMED_IDLE sang FLYING
#define TAKEOFF_THROTTLE_START    0.12f

// Khi đang flying mà kéo ga xuống dưới mức này thì quay lại idle
#define LAND_THROTTLE_BACK_IDLE   0.10f

// ===================== Flight limits =====================
#define MAX_ROLL_PITCH_DEG        20.0f
#define MAX_YAW_RATE_DPS          120.0f

#define MOTOR_IDLE_POWER          0.055f
#define THROTTLE_CUTOFF           0.060f

// ===================== Sensor filtering =====================
// Alpha càng nhỏ càng lọc mạnh nhưng phản ứng chậm hơn
#define SENSOR_ACC_LPF_ALPHA      0.18f
#define SENSOR_GYRO_LPF_ALPHA     0.25f

// Nếu đọc sensor lỗi liên tiếp quá số này thì stop motor
#define SENSOR_MAX_ERROR_COUNT    5

// ===================== ESC output =====================
#define ESC_PWM_FREQ_HZ           2000
#define ESC_PULSE_MIN_US          125.0f
#define ESC_PULSE_MAX_US          250.0f
#define ESC_PULSE_STOP_US         125.0f

// ===================== Yaw tuning =====================
#define MAX_YAW_RATE_DPS          60.0f
#define YAW_STICK_DEADBAND        0.05f

// Nếu drone quay yaw sai chiều, đổi 1.0f thành -1.0f
#define YAW_MIX_SIGN              1.0f

// Nếu log gyro.z sai chiều, đổi 1.0f thành -1.0f
#define IMU_GYRO_Z_SIGN           1.0f