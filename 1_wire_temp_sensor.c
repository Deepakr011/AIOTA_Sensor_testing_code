#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "onewire_bus.h"
#include "ds18b20.h"

static const char *TAG = "DS18B20_APP";
#define ONEWIRE_GPIO 21

void app_main(void)
{
    // 1. Initialize 1-Wire Bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = { .bus_gpio_num = ONEWIRE_GPIO };
    onewire_bus_rmt_config_t rmt_config = { .max_rx_bytes = 10 };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    // 2. Find the Device
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_device;
    ds18b20_device_handle_t ds18b20 = NULL;

    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    if (onewire_device_iter_get_next(iter, &next_device) == ESP_OK) {
        ds18b20_config_t ds_cfg = {}; 
        ESP_ERROR_CHECK(ds18b20_new_device(&next_device, &ds_cfg, &ds18b20));
        ESP_LOGI(TAG, "Sensor found!");
    } else {
        ESP_LOGE(TAG, "No sensor found! Check wiring and pull-up resistor.");
    }
    onewire_del_device_iter(iter);

    // 3. Read Loop
    while (ds18b20) {
        float temperature;
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20));
        vTaskDelay(pdMS_TO_TICKS(800)); // Delay for 12-bit resolution
        
        if (ds18b20_get_temperature(ds18b20, &temperature) == ESP_OK) {
            ESP_LOGI(TAG, "Temp: %.2f °C", temperature);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
