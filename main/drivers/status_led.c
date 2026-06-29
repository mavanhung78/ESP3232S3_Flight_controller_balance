#include "status_led.h"

#include "board_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"

static const char *TAG = "STATUS_LED";

static led_strip_handle_t led_strip = NULL;

static void set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (!led_strip) {
        return;
    }

    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

esp_err_t status_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = STATUS_LED_GPIO,
        .max_leds = STATUS_LED_NUM,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "status led init failed: %s", esp_err_to_name(err));
        return err;
    }

    led_strip_clear(led_strip);
    ESP_LOGI(TAG, "WS2812 status LED init OK on GPIO%d", STATUS_LED_GPIO);
    return ESP_OK;
}

void status_led_update(arm_state_t state, bool sensor_ok)
{
    const uint64_t now_ms = esp_timer_get_time() / 1000ULL;
    const bool blink_fast = ((now_ms / 150) % 2) == 0;
    const bool blink_slow = ((now_ms / 500) % 2) == 0;

    if (!sensor_ok || state == ARM_STATE_SENSOR_ERROR) {
        if (blink_fast) set_rgb(80, 0, 80);
        else set_rgb(0, 0, 0);
        return;
    }

    switch (state) {
    case ARM_STATE_DISARMED:
        // vàng: có nguồn, chưa arm
        if (blink_slow) set_rgb(60, 35, 0);
        else set_rgb(0, 0, 0);
        break;

    case ARM_STATE_ARMING:
        // xanh dương nháy nhanh: đang giữ arm
        if (blink_fast) set_rgb(0, 0, 80);
        else set_rgb(0, 0, 0);
        break;

    case ARM_STATE_ARMED_IDLE:
        // xanh lá: đã arm, motor idle
        set_rgb(0, 70, 0);
        break;

    case ARM_STATE_FLYING:
        // xanh dương: PID đang chạy
        set_rgb(0, 0, 70);
        break;

    case ARM_STATE_FAILSAFE:
        // đỏ nháy nhanh
        if (blink_fast) set_rgb(100, 0, 0);
        else set_rgb(0, 0, 0);
        break;

    default:
        set_rgb(20, 20, 20);
        break;
    }
}