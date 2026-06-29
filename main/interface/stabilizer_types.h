#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x;
    float y;
    float z;
} vec3f_t;

typedef struct {
    float roll;   // deg
    float pitch;  // deg
    float yaw;    // deg
} attitude_t;

typedef struct {
    vec3f_t acc;       // G
    vec3f_t gyro;      // deg/s
    uint64_t timestamp_us;
} sensorData_t;

typedef struct {
    attitude_t attitude;
    vec3f_t gyro;
} state_t;

typedef enum {
    ARM_STATE_DISARMED = 0,
    ARM_STATE_ARMING,
    ARM_STATE_ARMED_IDLE,
    ARM_STATE_FLYING,
    ARM_STATE_FAILSAFE,
    ARM_STATE_SENSOR_ERROR
} arm_state_t;

typedef struct {
    attitude_t attitude;
    vec3f_t attitudeRate;

    float thrust;          // 0.0 - 1.0
    bool armed;            // true khi đã arm
    bool flight_enabled;   // true khi cho phép PID điều khiển motor
    bool failsafe;

    arm_state_t arm_state;
} setpoint_t;

typedef struct {
    float thrust;
    float roll;
    float pitch;
    float yaw;
} control_t;

typedef struct {
    float m1;
    float m2;
    float m3;
    float m4;
} motor_power_t;