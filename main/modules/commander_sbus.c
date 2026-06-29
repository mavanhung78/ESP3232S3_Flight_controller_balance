#include "commander_sbus.h"

#include <string.h>
#include "board_config.h"
#include "drivers/sbus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "COMMANDER";

static SemaphoreHandle_t sp_lock;
static setpoint_t current_sp;
static uint16_t last_channels[SBUS_CHANNELS];
static uint64_t last_frame_us;

static float constrainf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float sbus_norm_center(uint16_t v)
{
    return constrainf_local(((float)v - (float)SBUS_VALUE_MID) / 820.0f, -1.0f, 1.0f);
}

static float sbus_norm_throttle(uint16_t v)
{
    return constrainf_local(((float)v - (float)SBUS_VALUE_MIN) /
                            ((float)SBUS_VALUE_MAX - (float)SBUS_VALUE_MIN), 0.0f, 1.0f);
}

static void update_setpoint_from_sbus(const sbus_frame_t *frame)
{
    setpoint_t sp = {0};

    const float roll = sbus_norm_center(frame->ch[SBUS_CH_ROLL]);
    const float pitch = sbus_norm_center(frame->ch[SBUS_CH_PITCH]);
    const float throttle = sbus_norm_throttle(frame->ch[SBUS_CH_THROTTLE]);
    const float yaw = sbus_norm_center(frame->ch[SBUS_CH_YAW]);
    const float arm = sbus_norm_center(frame->ch[SBUS_CH_ARM]);

    sp.attitude.roll = roll * MAX_ROLL_PITCH_DEG;
    sp.attitude.pitch = -pitch * MAX_ROLL_PITCH_DEG;
    sp.attitude.yaw = 0.0f;
    sp.attitudeRate.z = yaw * MAX_YAW_RATE_DPS;
    sp.thrust = throttle;
    sp.armed = (arm > 0.35f) && !frame->failsafe;
    sp.failsafe = frame->failsafe || frame->frame_lost;

    if (!sp.armed || sp.failsafe) {
        sp.thrust = 0.0f;
    }

    xSemaphoreTake(sp_lock, portMAX_DELAY);
    current_sp = sp;
    memcpy(last_channels, frame->ch, sizeof(last_channels));
    last_frame_us = frame->timestamp_us;
    xSemaphoreGive(sp_lock);
}

static void sbus_task(void *arg)
{
    sbus_frame_t frame;
    while (1) {
        if (sbus_read_frame(&frame, 20)) {
            update_setpoint_from_sbus(&frame);
        }
    }
}

esp_err_t commander_sbus_init(void)
{
    sp_lock = xSemaphoreCreateMutex();
    if (!sp_lock) {
        return ESP_ERR_NO_MEM;
    }

    memset(&current_sp, 0, sizeof(current_sp));
    last_frame_us = 0;

    ESP_ERROR_CHECK(sbus_driver_init());
    xTaskCreatePinnedToCore(sbus_task, "sbus_task", 4096, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "SBUS commander init OK. CH%d roll, CH%d pitch, CH%d throttle, CH%d yaw, CH%d arm",
             SBUS_CH_ROLL + 1, SBUS_CH_PITCH + 1, SBUS_CH_THROTTLE + 1,
             SBUS_CH_YAW + 1, SBUS_CH_ARM + 1);
    return ESP_OK;
}

void commander_get_setpoint(setpoint_t *setpoint)
{
    xSemaphoreTake(sp_lock, portMAX_DELAY);
    *setpoint = current_sp;
    uint64_t last_us = last_frame_us;
    xSemaphoreGive(sp_lock);

    const uint64_t now = esp_timer_get_time();
    if (last_us == 0 || (now - last_us) > ((uint64_t)SBUS_FAILSAFE_TIMEOUT_MS * 1000ULL)) {
        memset(setpoint, 0, sizeof(*setpoint));
        setpoint->failsafe = true;
        setpoint->armed = false;
        setpoint->thrust = 0.0f;
    }
}
