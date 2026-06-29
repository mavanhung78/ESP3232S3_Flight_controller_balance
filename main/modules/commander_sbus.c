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
static uint64_t last_frame_us = 0;

static arm_state_t arm_state = ARM_STATE_DISARMED;
static uint64_t arm_switch_start_us = 0;

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
    return constrainf_local(
        ((float)v - (float)SBUS_VALUE_MIN) / ((float)SBUS_VALUE_MAX - (float)SBUS_VALUE_MIN),
        0.0f,
        1.0f
    );
}

static void fill_safe_setpoint(setpoint_t *sp, arm_state_t state)
{
    memset(sp, 0, sizeof(*sp));
    sp->armed = false;
    sp->flight_enabled = false;
    sp->failsafe = (state == ARM_STATE_FAILSAFE);
    sp->arm_state = state;
    sp->thrust = 0.0f;
}

static void update_arm_state(bool arm_switch_on, bool throttle_low, float throttle, bool failsafe)
{
    const uint64_t now = esp_timer_get_time();

    if (failsafe) {
        arm_state = ARM_STATE_FAILSAFE;
        arm_switch_start_us = 0;
        return;
    }

    if (!arm_switch_on) {
        arm_state = ARM_STATE_DISARMED;
        arm_switch_start_us = 0;
        return;
    }

    switch (arm_state) {
    case ARM_STATE_DISARMED:
    case ARM_STATE_FAILSAFE:
        if (throttle_low) {
            if (arm_switch_start_us == 0) {
                arm_switch_start_us = now;
                arm_state = ARM_STATE_ARMING;
            } else if ((now - arm_switch_start_us) >= ((uint64_t)ARM_HOLD_TIME_MS * 1000ULL)) {
                arm_state = ARM_STATE_ARMED_IDLE;
            }
        } else {
            arm_state = ARM_STATE_DISARMED;
            arm_switch_start_us = 0;
        }
        break;

    case ARM_STATE_ARMING:
        if (!throttle_low) {
            arm_state = ARM_STATE_DISARMED;
            arm_switch_start_us = 0;
        } else if ((now - arm_switch_start_us) >= ((uint64_t)ARM_HOLD_TIME_MS * 1000ULL)) {
            arm_state = ARM_STATE_ARMED_IDLE;
        }
        break;

    case ARM_STATE_ARMED_IDLE:
        if (throttle > TAKEOFF_THROTTLE_START) {
            arm_state = ARM_STATE_FLYING;
        }
        break;

    case ARM_STATE_FLYING:
        if (throttle < LAND_THROTTLE_BACK_IDLE) {
            arm_state = ARM_STATE_ARMED_IDLE;
        }
        break;

    case ARM_STATE_SENSOR_ERROR:
    default:
        arm_state = ARM_STATE_DISARMED;
        arm_switch_start_us = 0;
        break;
    }
}

static void update_setpoint_from_sbus(const sbus_frame_t *frame)
{
    setpoint_t sp;
    memset(&sp, 0, sizeof(sp));

    const bool frame_failsafe = frame->failsafe || frame->frame_lost;

    const float roll = sbus_norm_center(frame->ch[SBUS_CH_ROLL]);
    const float pitch = sbus_norm_center(frame->ch[SBUS_CH_PITCH]);
    const float throttle = sbus_norm_throttle(frame->ch[SBUS_CH_THROTTLE]);
    const float yaw = sbus_norm_center(frame->ch[SBUS_CH_YAW]);
    const float arm = sbus_norm_center(frame->ch[SBUS_CH_ARM]);

    const bool arm_switch_on = (arm > ARM_SWITCH_ON_THRESHOLD);
    const bool throttle_low = (throttle < ARM_THROTTLE_LOW_MAX);

    update_arm_state(arm_switch_on, throttle_low, throttle, frame_failsafe);

    sp.attitude.roll = roll * MAX_ROLL_PITCH_DEG;
    sp.attitude.pitch = -pitch * MAX_ROLL_PITCH_DEG;
    sp.attitude.yaw = 0.0f;

    sp.attitudeRate.x = 0.0f;
    sp.attitudeRate.y = 0.0f;
    sp.attitudeRate.z = yaw * MAX_YAW_RATE_DPS;

    sp.failsafe = frame_failsafe;

    if (arm_state == ARM_STATE_ARMED_IDLE) {
        sp.armed = true;
        sp.flight_enabled = false;
        sp.thrust = 0.0f;
    } else if (arm_state == ARM_STATE_FLYING) {
        sp.armed = true;
        sp.flight_enabled = true;
        sp.thrust = throttle;
    } else {
        sp.armed = false;
        sp.flight_enabled = false;
        sp.thrust = 0.0f;
    }

    sp.arm_state = arm_state;

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
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

esp_err_t commander_sbus_init(void)
{
    sp_lock = xSemaphoreCreateMutex();
    if (!sp_lock) {
        return ESP_ERR_NO_MEM;
    }

    fill_safe_setpoint(&current_sp, ARM_STATE_DISARMED);
    memset(last_channels, 0, sizeof(last_channels));
    last_frame_us = 0;
    arm_state = ARM_STATE_DISARMED;
    arm_switch_start_us = 0;

    ESP_ERROR_CHECK(sbus_driver_init());

    xTaskCreatePinnedToCore(sbus_task, "sbus_task", 4096, NULL, 5, NULL, 0);

    ESP_LOGI(TAG,
             "SBUS commander init OK. CH%d roll, CH%d pitch, CH%d throttle, CH%d yaw, CH%d arm",
             SBUS_CH_ROLL + 1,
             SBUS_CH_PITCH + 1,
             SBUS_CH_THROTTLE + 1,
             SBUS_CH_YAW + 1,
             SBUS_CH_ARM + 1);

    return ESP_OK;
}

void commander_get_setpoint(setpoint_t *setpoint)
{
    xSemaphoreTake(sp_lock, portMAX_DELAY);
    *setpoint = current_sp;
    uint64_t last_us = last_frame_us;
    xSemaphoreGive(sp_lock);

    const uint64_t now = esp_timer_get_time();

    if (last_us == 0 ||
        (now - last_us) > ((uint64_t)SBUS_FAILSAFE_TIMEOUT_MS * 1000ULL)) {
        fill_safe_setpoint(setpoint, ARM_STATE_FAILSAFE);

        xSemaphoreTake(sp_lock, portMAX_DELAY);
        current_sp = *setpoint;
        arm_state = ARM_STATE_FAILSAFE;
        xSemaphoreGive(sp_lock);
    }
}