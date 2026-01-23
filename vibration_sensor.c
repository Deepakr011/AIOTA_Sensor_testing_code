#include <stdio.h>
#include <string.h>
#include <math.h> // Added for fabs()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// --- PIN CONFIG ---
#define TXD 17
#define RXD 16
#define DE  4
#define RE  5

// --- DATA TUNING ---
// RDL Sensors sometimes swap High/Low words. 
// If values look weird (e.g. 10^-30 or 10^38), toggle this to 0.
#define SWAP_WORDS  1  

// If X/Y/Z are correct but Vib is negative, the sensor might be sending
// Instantaneous values. We will take absolute value (fabs) to show magnitude.
#define FORCE_POSITIVE_VIB 1 

// --- UART CONFIG ---
#define UART_NUM UART_NUM_2
#define BUF_SIZE 128

// --- Helper: Convert Bytes to Float ---
float get_float(uint8_t *data, int index) {
    uint16_t reg1 = (data[index] << 8) | data[index + 1];
    uint16_t reg2 = (data[index + 2] << 8) | data[index + 3];
    
    uint32_t combined;
    
    // Toggle Word Order based on define
    if (SWAP_WORDS) {
        // Little Endian (CDAB) - Common for RDL
        // Reg1 is Low Word, Reg2 is High Word
        combined = ((uint32_t)reg2 << 16) | reg1;
    } else {
        // Big Endian (ABCD) - Standard Modbus
        // Reg1 is High Word, Reg2 is Low Word
        combined = ((uint32_t)reg1 << 16) | reg2;
    }

    float res;
    memcpy(&res, &combined, sizeof(res));
    return res;
}

// --- Helper: CRC16 ---
uint16_t crc16(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 1) != 0) { crc >>= 1; crc ^= 0xA001; }
            else { crc >>= 1; }
        }
    }
    return crc;
}

void app_main(void) {
    // 1. Setup UART
    uart_config_t cfg = { .baud_rate = 9600, .data_bits = UART_DATA_8_BITS, 
                          .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, 
                          .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_APB };
    uart_param_config(UART_NUM, &cfg);
    uart_set_pin(UART_NUM, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 2. Setup DE/RE
    gpio_set_direction(DE, GPIO_MODE_OUTPUT);
    gpio_set_direction(RE, GPIO_MODE_OUTPUT);
    gpio_set_level(DE, 0); gpio_set_level(RE, 0);

    uint8_t rx_buf[BUF_SIZE];

    printf("--- RDL850D Monitor Started ---\n");

    while (1) {
        // 3. Request Data (Read 8 Registers = 16 Bytes)
        // Slave 1, Func 03, Start 0x0000, Len 8
        uint8_t req[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00};
        uint16_t crc = crc16(req, 6);
        req[6] = crc & 0xFF; req[7] = (crc >> 8) & 0xFF;

        // Send
        gpio_set_level(DE, 1); gpio_set_level(RE, 1);
        uart_write_bytes(UART_NUM, (const char *)req, 8);
        uart_wait_tx_done(UART_NUM, 20);
        gpio_set_level(DE, 0); gpio_set_level(RE, 0);

        // 4. Read Response
        int len = uart_read_bytes(UART_NUM, rx_buf, BUF_SIZE, pdMS_TO_TICKS(500));

        // Check if we got valid Modbus packet (ID + Func + ByteCount)
        // ByteCount for 8 regs should be 16 (0x10)
        if (len > 5 && rx_buf[0] == 0x01 && rx_buf[1] == 0x03) {
            
            float x_val = get_float(rx_buf, 3);
            float y_val = get_float(rx_buf, 7);
            float z_val = get_float(rx_buf, 11);
            float v_val = get_float(rx_buf, 15);

            // Post-Processing: Convert Negative Vibration to Positive Magnitude
            if (FORCE_POSITIVE_VIB) {
                v_val = fabsf(v_val);
            }

            // Print with unit assumption (m/s^2 based on your logs)
            printf("X: %6.2f m/s2 | Y: %6.2f m/s2 | Z: %6.2f m/s2 | Vib: %6.2f mm/s\n", 
                   x_val, y_val, z_val, v_val);
                   
        } else {
            // Only print error if it's not just a silent timeout (reduces log spam)
            if (len > 0) printf("Modbus CRC/Frame Error. Len: %d\n", len);
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Update every 500ms
    }
}
