#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TRIG_PIN GPIO_NUM_5
#define ECHO_PIN GPIO_NUM_18

void ultrasonic_init()
{
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
}

float ultrasonic_get_distance_cm()
{
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);

    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);

    gpio_set_level(TRIG_PIN, 0);

    // wait for echo HIGH
    while (gpio_get_level(ECHO_PIN) == 0);

    int64_t start = esp_timer_get_time();

    // wait for echo LOW
    while (gpio_get_level(ECHO_PIN) == 1);

    int64_t end = esp_timer_get_time();

    int64_t pulse = end - start;

    return pulse * 0.0343f / 2.0f;
}

void app_main(void)
{
    ultrasonic_init();

    while (1) {
        float dist = ultrasonic_get_distance_cm();
        printf("Distance: %.2f cm\n", dist);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
