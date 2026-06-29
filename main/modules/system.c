#include "system.h"

#include "stabilizer.h"
#include "esp_log.h"

static const char *TAG = "SYSTEM";

void system_launch(void)
{
    ESP_LOGI(TAG, "ESP32-S3 Crazyflie-style balance firmware");
    ESP_ERROR_CHECK(stabilizer_init());
}
