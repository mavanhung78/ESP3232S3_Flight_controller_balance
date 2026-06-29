#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define SBUS_CHANNELS 16

typedef struct {
    uint16_t ch[SBUS_CHANNELS];
    bool frame_lost;
    bool failsafe;
    uint64_t timestamp_us;
} sbus_frame_t;

esp_err_t sbus_driver_init(void);
bool sbus_read_frame(sbus_frame_t *out, uint32_t timeout_ms);
