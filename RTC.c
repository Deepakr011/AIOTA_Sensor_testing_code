#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ds3231.h"

static const char *TAG = "RTC_DEMO";

void app_main(void)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    // Initialize I2C and DS3231 descriptor
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(ds3231_init_desc(&dev, I2C_NUM_0, 21, 22));

    // --- SECTION: SETTING THE TIME ---
    // Manually setting to: January 30, 2026, 17:15:00
    struct tm time_to_set = {
        .tm_year = 2026 - 1900, // Year since 1900
        .tm_mon  = 0,            // 0 = January, 1 = February...
        .tm_mday = 30,           // Day of month
        .tm_hour = 17,
        .tm_min  = 15,
        .tm_sec  = 0
    };

    ESP_LOGI(TAG, "Setting RTC time...");
    if (ds3231_set_time(&dev, &time_to_set) != ESP_OK) {
        ESP_LOGE(TAG, "Could not set time!");
    } else {
        ESP_LOGI(TAG, "Time set successfully!");
    }
    // ---------------------------------

    while (1) {
        struct tm rtc_time;
        float temp;

        // --- SECTION: READING THE TIME ---
        if (ds3231_get_time(&dev, &rtc_time) == ESP_OK) {
            ESP_LOGI(TAG, "Current RTC Time: %04d-%02d-%02d %02d:%02d:%02d",
                     rtc_time.tm_year + 1900, 
                     rtc_time.tm_mon + 1, 
                     rtc_time.tm_mday,
                     rtc_time.tm_hour, 
                     rtc_time.tm_min, 
                     rtc_time.tm_sec);
        } else {
            ESP_LOGE(TAG, "Could not get time from RTC");
        }

        if (ds3231_get_temp_float(&dev, &temp) == ESP_OK) {
            ESP_LOGI(TAG, "Internal Temp: %.2f°C", temp);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
