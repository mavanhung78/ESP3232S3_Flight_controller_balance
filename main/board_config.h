#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"

// ===================== Board pins =====================
// ESP32-S3 DevKit common I2C pins. Change here if your wiring is different.
#define I2C_PORT_NUM               I2C_NUM_0
#define I2C_SDA_GPIO               GPIO_NUM_8
#define I2C_SCL_GPIO               GPIO_NUM_9
#define I2C_FREQ_HZ                400000

// SBUS receiver input. Standard SBUS is inverted, 100000 baud, 8E2.
#define SBUS_UART_NUM              UART_NUM_1
#define SBUS_RX_GPIO               GPIO_NUM_16
#define SBUS_RX_INVERT             1

// 4 ESC outputs. Output is OneShot125-like: 2 kHz period, pulse 125-250 us.
#define MOTOR1_GPIO                GPIO_NUM_4
#define MOTOR2_GPIO                GPIO_NUM_5
#define MOTOR3_GPIO                GPIO_NUM_6
#define MOTOR4_GPIO                GPIO_NUM_7

// ===================== Stabilizer timing =====================
#define STABILIZER_RATE_HZ         500
#define LOG_RATE_HZ                10

// ===================== RC / SBUS mapping =====================
// SBUS channel index starts from 0.
#define SBUS_CH_ROLL               0
#define SBUS_CH_PITCH              1
#define SBUS_CH_THROTTLE           2
#define SBUS_CH_YAW                3
#define SBUS_CH_ARM                4

// Typical SBUS values: min 172, mid 992, max 1811.
#define SBUS_VALUE_MIN             172
#define SBUS_VALUE_MID             992
#define SBUS_VALUE_MAX             1811
#define SBUS_FAILSAFE_TIMEOUT_MS   300

// ===================== Flight limits =====================
#define MAX_ROLL_PITCH_DEG         20.0f
#define MAX_YAW_RATE_DPS           120.0f
#define MOTOR_IDLE_POWER           0.055f
#define THROTTLE_CUTOFF            0.035f

// ===================== ESC output =====================
#define ESC_PWM_FREQ_HZ            2000
#define ESC_PULSE_MIN_US           125.0f
#define ESC_PULSE_MAX_US           250.0f
#define ESC_PULSE_STOP_US          125.0f
