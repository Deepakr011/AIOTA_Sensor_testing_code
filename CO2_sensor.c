#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// --- Configuration ---
#define UART_PORT_NUM      UART_NUM_2
#define TXD_PIN            (GPIO_NUM_17) // Connect to Sensor RX
#define RXD_PIN            (GPIO_NUM_16) // Connect to Sensor TX
#define BAUD_RATE          9600
#define BUF_SIZE           1024

static const char *TAG = "MH-Z16";

// --- Commands ---
const uint8_t CMD_READ_CO2[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

// --- Helper: Calculate Checksum ---
// Rule: (NOT (Sum of Byte 1 to Byte 7)) + 1
uint8_t calculate_checksum(const uint8_t *packet) {
    uint8_t checksum = 0;
    for (int i = 1; i < 8; i++) {
        checksum += packet[i];
    }
    checksum = 0xFF - checksum;
    checksum += 1;
    return checksum;
}

// --- Helper: Send Command & Wait for Response ---
// Returns: CO2 PPM value or -1 if failed
int send_and_receive_co2() {
    uint8_t data[9];
    
    // 1. Flush Buffer
    uart_flush_input(UART_PORT_NUM);

    // 2. Send Command
    uart_write_bytes(UART_PORT_NUM, (const char *)CMD_READ_CO2, 9);

    // 3. Read Response (Wait 1000ms max)
    int len = uart_read_bytes(UART_PORT_NUM, data, 9, pdMS_TO_TICKS(1000));

    if (len == 9) {
        // 4. Verify Headers
        if (data[0] != 0xFF || data[1] != 0x86) {
            ESP_LOGE(TAG, "Invalid Header: %02X %02X", data[0], data[1]);
            return -1;
        }

        // 5. Verify Checksum
        uint8_t cal_sum = calculate_checksum(data);
        if (cal_sum != data[8]) {
            ESP_LOGE(TAG, "Checksum Mismatch! Calc: %02X, Recv: %02X", cal_sum, data[8]);
            return -1;
        }

        // 6. Calculate CO2: High * 256 + Low
        int co2_ppm = (data[2] * 256) + data[3];
        return co2_ppm;
    } else {
        ESP_LOGE(TAG, "UART Timeout or incomplete data");
        return -1;
    }
}

// --- Utility: Set Detection Range (0x99) ---
// For 0-1% volume, you might need to set range to 10000
void mhz16_set_range(int range) {
    uint8_t cmd[9] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    cmd[4] = (range >> 24) & 0xFF;
    cmd[5] = (range >> 16) & 0xFF;
    cmd[6] = (range >> 8) & 0xFF; // High byte of lower 16 bits
    cmd[7] = range & 0xFF;        // Low byte
    
    cmd[8] = calculate_checksum(cmd); // Calculate checksum for the new command
    
    uart_write_bytes(UART_PORT_NUM, (const char *)cmd, 9);
    ESP_LOGI(TAG, "Set Range Command Sent: %d ppm", range);
    // Note: No return value from sensor for this command
}

// --- Utility: Toggle Auto Calibration (0x79) ---
// enable = 1 (ON - 0xA0), enable = 0 (OFF - 0x00)
void mhz16_set_abc(uint8_t enable) {
    uint8_t cmd[9] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    cmd[3] = enable ? 0xA0 : 0x00;
    cmd[8] = calculate_checksum(cmd);
    
    uart_write_bytes(UART_PORT_NUM, (const char *)cmd, 9);
    ESP_LOGI(TAG, "ABC Logic Set to: %s", enable ? "ON" : "OFF");
}

void app_main(void) {
    // UART Configuration
    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // OPTIONAL: Uncomment to set range to 10,000ppm (1% Vol) once
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // mhz16_set_range(10000); 

    // OPTIONAL: Disable ABC if you are not in fresh air every 24h
    // mhz16_set_abc(0);

    while (1) {
        int co2 = send_and_receive_co2();
        
        if (co2 != -1) {
            ESP_LOGI(TAG, "CO2 Concentration: %d ppm (%.2f%%)", co2, co2 / 10000.0);
        }

        // Wait 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
