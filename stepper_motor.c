#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

// ---------------- PIN DEFINITIONS ----------------

#define STEP_PIN 18
#define DIR_PIN  19
#define EN_PIN   21

// ---------------- MOTOR SETTINGS ----------------

// Your board is using 8 microsteps
// 200 full steps × 8 microsteps = 1600
#define STEPS_PER_REV 1600

// Motion goals
#define FWD_ROTATIONS 3
#define BWD_ROTATIONS 2

#define FWD_STEPS (FWD_ROTATIONS * STEPS_PER_REV)
#define BWD_STEPS (BWD_ROTATIONS * STEPS_PER_REV)

// Timing
#define FWD_DELAY_US 100     // ≈2 seconds
#define BWD_DELAY_US 80      // ≈1 second

// ---------------- FUNCTIONS ---------------------

void motor_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEP_PIN) |
                        (1ULL << DIR_PIN)  |
                        (1ULL << EN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Enable driver (LOW = enabled)
    gpio_set_level(EN_PIN, 0);
}

void step_motor(int steps, int delay_us)
{
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(delay_us);

        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(delay_us);
    }
}

// ---------------- MAIN --------------------------

void app_main(void)
{
    motor_gpio_init();

    while (1)
    {
        // ---- Forward 3 rotations ----
        gpio_set_level(DIR_PIN, 1);
        step_motor(FWD_STEPS, FWD_DELAY_US);

        vTaskDelay(pdMS_TO_TICKS(500));

        // ---- Backward 2 rotations ----
        gpio_set_level(DIR_PIN, 0);
        step_motor(BWD_STEPS, BWD_DELAY_US);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
