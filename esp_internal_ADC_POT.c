#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// Define Tag for logging
#define TAG "POT_RAW"

// GPIO 34 corresponds to ADC1 Channel 6
#define ADC_CHANNEL ADC_CHANNEL_6 

void app_main(void)
{
    // 1. Initialize ADC Unit (ADC1)
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // 2. Configure Channel 
    // Attenuation 12dB allows reading higher voltages (up to ~3.1V)
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Default is 12-bit (0 - 4095)
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));

    int adc_raw = 0;

    // 3. Loop and Read
    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw));
        
        // Print strictly the raw integer value
        ESP_LOGI(TAG, "Raw Value: %d", adc_raw);

        // Delay 100ms for readability
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
