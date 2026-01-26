#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// GPIO pin connected to flow sensor output
#define FLOW_SENSOR_PIN GPIO_NUM_4

// Flow sensor calibration factor (pulses per liter)
#define CALIBRATION_FACTOR 540.0

// Variables for pulse counting
volatile uint32_t pulse_count = 0;
float flow_rate = 0.0;           // L/min
float total_volume = 0.0;        // Total liters
unsigned long old_time = 0;

// ISR for pulse counting
static void IRAM_ATTR flow_sensor_isr(void* arg) {
    pulse_count++;
}

void setup_flow_sensor(void) {
    // Configure GPIO as input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FLOW_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE  // Interrupt on rising edge
    };
    gpio_config(&io_conf);
    
    // Install ISR service
    gpio_install_isr_service(0);
    
    // Attach interrupt handler
    gpio_isr_handler_add(FLOW_SENSOR_PIN, flow_sensor_isr, NULL);
    
    printf("Flow sensor initialized on GPIO %d\n", FLOW_SENSOR_PIN);
}

void calculate_flow(void) {
    unsigned long current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    unsigned long elapsed_time = current_time - old_time;
    
    // Calculate every 1 second
    if (elapsed_time >= 1000) {
        // Disable interrupts while reading
        gpio_intr_disable(FLOW_SENSOR_PIN);
        uint32_t pulses = pulse_count;
        pulse_count = 0;
        gpio_intr_enable(FLOW_SENSOR_PIN);
        
        // Calculate flow rate in L/min
        flow_rate = (pulses / CALIBRATION_FACTOR) * 60.0;
        
        // Calculate total volume (L)
        total_volume += (pulses / CALIBRATION_FACTOR);
        
        // Print results
        printf("Flow Rate: %.2f L/min | Total Volume: %.2f L | Pulses: %lu\n", 
               flow_rate, total_volume, pulses);
        
        old_time = current_time;
    }
}

void app_main(void) {
    printf("=== ESP32 Flow Sensor Reading ===\n");
    printf("Sensor: DFRobot Gravity G1/4\n");
    printf("Calibration: %.0f pulses/L\n\n", CALIBRATION_FACTOR);
    
    // Initialize flow sensor
    setup_flow_sensor();
    
    // Initialize timer
    old_time = esp_timer_get_time() / 1000;
    
    // Main loop
    while (1) {
        calculate_flow();
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}  
