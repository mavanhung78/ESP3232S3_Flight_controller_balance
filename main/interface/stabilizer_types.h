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
    vec3f_t acc;     // G
    vec3f_t gyro;    // deg/s
    uint64_t timestamp_us;
} sensorData_t;

typedef struct {
    attitude_t attitude; // estimated roll/pitch/yaw [deg]
    vec3f_t gyro;        // gyro rates copied from sensor [deg/s]
} state_t;

typedef struct {
    attitude_t attitude;     // desired roll/pitch [deg]
    vec3f_t attitudeRate;    // desired rate, yaw uses z [deg/s]
    float thrust;            // normalized 0.0 - 1.0 from RC throttle
    bool armed;
    bool failsafe;
} setpoint_t;

typedef struct {
    float thrust;  // normalized 0.0 - 1.0
    float roll;    // normalized correction
    float pitch;   // normalized correction
    float yaw;     // normalized correction
} control_t;

typedef struct {
    float m1;
    float m2;
    float m3;
    float m4;
} motor_power_t;
