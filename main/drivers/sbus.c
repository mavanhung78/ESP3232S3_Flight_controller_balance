#include "sbus.h"

#include <string.h>
#include "board_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "SBUS";

#define SBUS_FRAME_LEN 25
#define SBUS_START_BYTE 0x0F

static void sbus_parse_frame(const uint8_t *buf, sbus_frame_t *out)
{
    memset(out, 0, sizeof(*out));

    out->ch[0]  = ((buf[1]    | buf[2] << 8) & 0x07FF);
    out->ch[1]  = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    out->ch[2]  = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    out->ch[3]  = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    out->ch[4]  = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    out->ch[5]  = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    out->ch[6]  = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    out->ch[7]  = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    out->ch[8]  = ((buf[12] | buf[13] << 8) & 0x07FF);
    out->ch[9]  = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
    out->ch[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
    out->ch[11] = ((buf[16] >> 1 | buf[17] << 7) & 0x07FF);
    out->ch[12] = ((buf[17] >> 4 | buf[18] << 4) & 0x07FF);
    out->ch[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF);
    out->ch[14] = ((buf[20] >> 2 | buf[21] << 6) & 0x07FF);
    out->ch[15] = ((buf[21] >> 5 | buf[22] << 3) & 0x07FF);

    const uint8_t flags = buf[23];
    out->frame_lost = flags & (1 << 2);
    out->failsafe = flags & (1 << 3);
    out->timestamp_us = esp_timer_get_time();
}

esp_err_t sbus_driver_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 100000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(SBUS_UART_NUM, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SBUS_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(SBUS_UART_NUM, UART_PIN_NO_CHANGE, SBUS_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#if SBUS_RX_INVERT
    ESP_ERROR_CHECK(uart_set_line_inverse(SBUS_UART_NUM, UART_SIGNAL_RXD_INV));
#endif

    ESP_LOGI(TAG, "SBUS UART init OK: RX GPIO=%d, inverted=%d", SBUS_RX_GPIO, SBUS_RX_INVERT);
    return ESP_OK;
}

bool sbus_read_frame(sbus_frame_t *out, uint32_t timeout_ms)
{
    static uint8_t frame[SBUS_FRAME_LEN];
    static int idx = 0;

    uint8_t byte;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while (uart_read_bytes(SBUS_UART_NUM, &byte, 1, timeout_ticks) == 1) {
        if (idx == 0) {
            if (byte != SBUS_START_BYTE) {
                continue;
            }
            frame[idx++] = byte;
        } else {
            frame[idx++] = byte;
            if (idx >= SBUS_FRAME_LEN) {
                idx = 0;
                // Most SBUS frames end with 0x00, but some receivers use other footer bytes.
                if (frame[0] == SBUS_START_BYTE) {
                    sbus_parse_frame(frame, out);
                    return true;
                }
            }
        }
    }

    return false;
}
