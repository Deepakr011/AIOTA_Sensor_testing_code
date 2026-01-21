#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

/* ---------------- Configuration ---------------- */
#define UART_NUM    UART_NUM_2
#define TXD_PIN     GPIO_NUM_17
#define RXD_PIN     GPIO_NUM_16
#define DE_PIN      GPIO_NUM_5   // Driver Enable
#define RE_PIN      GPIO_NUM_4  // Receiver Enable

#define BAUD_RATE   9600
#define BUF_SIZE    128

/* ---------------- MODBUS Query for Light Sensor ---------------- */
uint8_t luxQuery[] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x65, 0xCB};

/* ---------------- RS485 Control Functions ---------------- */
void rs485_set_transmit_mode(void)
{
    gpio_set_level(DE_PIN, 1);  // DE = HIGH (enable driver)
    gpio_set_level(RE_PIN, 1);  // RE = HIGH (disable receiver)
    esp_rom_delay_us(10);
}

void rs485_set_receive_mode(void)
{
    gpio_set_level(DE_PIN, 0);  // DE = LOW (disable driver)
    gpio_set_level(RE_PIN, 0);  // RE = LOW (enable receiver)
    esp_rom_delay_us(10);
}

/* ---------------- GPIO Init ---------------- */
void gpio_init_rs485(void)
{
    gpio_reset_pin(DE_PIN);
    gpio_reset_pin(RE_PIN);
    
    gpio_set_direction(DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RE_PIN, GPIO_MODE_OUTPUT);
    
    gpio_set_level(DE_PIN, 0);
    gpio_set_level(RE_PIN, 0);
    
    printf("RS485 initialized: DE=GPIO%d, RE=GPIO%d\n", DE_PIN, RE_PIN);
}

/* ---------------- UART Init ---------------- */
void uart_init_rs485(void)
{
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    
    printf("UART initialized: TX=GPIO%d, RX=GPIO%d, Baud=%d\n", TXD_PIN, RXD_PIN, BAUD_RATE);
}

/* ---------------- CRC16 Calculation (Modbus) ---------------- */
uint16_t crc16_modbus(uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    // Swap bytes (big-endian to little-endian)
    crc = ((crc & 0x00FF) << 8) | ((crc & 0xFF00) >> 8);
    
    return crc;
}

/* ---------------- Read N Bytes with Timeout ---------------- */
uint8_t read_n_bytes(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t offset = 0;
    size_t left = len;
    uint8_t *buffer = buf;
    
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    while (left > 0) {
        size_t available = 0;
        uart_get_buffered_data_len(UART_NUM, &available);
        
        if (available > 0) {
            int read_len = uart_read_bytes(UART_NUM, &buffer[offset], 1, pdMS_TO_TICKS(10));
            if (read_len > 0) {
                offset++;
                left--;
            }
        }
        
        // Check timeout
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    return offset;
}

/* ---------------- Read Lux Value ---------------- */
bool read_lux(float *lux_value)
{
    uint8_t data[10] = {0};
    uint8_t ch = 0;
    bool success = false;
    int max_retries = 3;
    
    for (int retry = 0; retry < max_retries && !success; retry++) {
        // Clear UART buffer
        uart_flush_input(UART_NUM);
        
        // Send query
        rs485_set_transmit_mode();
        vTaskDelay(pdMS_TO_TICKS(10));
        uart_write_bytes(UART_NUM, (char *)luxQuery, 8);
        uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(100));
        rs485_set_receive_mode();
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // Read response byte by byte
        // Expected response: [0x01][0x03][0x04][DATA1][DATA2][DATA3][DATA4][CRC_L][CRC_H]
        // Total: 9 bytes
        
        if (read_n_bytes(&ch, 1, 500) == 1) {
            if (ch == 0x01) {  // Address
                data[0] = ch;
                
                if (read_n_bytes(&ch, 1, 500) == 1) {
                    if (ch == 0x03) {  // Function code
                        data[1] = ch;
                        
                        if (read_n_bytes(&ch, 1, 500) == 1) {
                            if (ch == 0x04) {  // Byte count
                                data[2] = ch;
                                
                                // Read remaining 6 bytes: 4 data bytes + 2 CRC bytes
                                if (read_n_bytes(&data[3], 6, 500) == 6) {
                                    // Verify CRC
                                    uint16_t received_crc = (data[7] << 8) | data[8];
                                    uint16_t calculated_crc = crc16_modbus(data, 7);
                                    
                                    if (received_crc == calculated_crc) {
                                        // Calculate LUX value
                                        uint32_t raw_value = ((uint32_t)data[3] << 24) | 
                                                            ((uint32_t)data[4] << 16) | 
                                                            ((uint32_t)data[5] << 8) | 
                                                            (uint32_t)data[6];
                                        
                                        *lux_value = raw_value / 1000.0;
                                        success = true;
                                    } else {
                                        printf("CRC Error! Received: 0x%04X, Calculated: 0x%04X\n", 
                                               received_crc, calculated_crc);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        if (!success && retry < max_retries - 1) {
            printf("Retry %d/%d...\n", retry + 1, max_retries);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
    return success;
}

/* ---------------- Main Task ---------------- */
void light_sensor_task(void *pvParameters)
{
    printf("\nRS485 Ambient Light Sensor Started\n");
    printf("Reading light intensity every second...\n\n");
    
    while (1) {
        float lux = 0.0;
        
        if (read_lux(&lux)) {
            printf("Lux = %.3f lux\n", lux);
        } else {
            printf("Failed to read light sensor\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---------------- app_main ---------------- */
void app_main(void)
{
    printf("\n\n");
    printf("==========================================\n");
    printf("  RS485 Ambient Light Sensor Reader\n");
    printf("  Range: 0-200,000 lux\n");
    printf("==========================================\n\n");
    
    // Initialize GPIO for RS485 control
    gpio_init_rs485();
    
    // Initialize UART
    uart_init_rs485();
    
    printf("\nHardware Configuration:\n");
    printf("  TX  : GPIO %d\n", TXD_PIN);
    printf("  RX  : GPIO %d\n", RXD_PIN);
    printf("  DE  : GPIO %d\n", DE_PIN);
    printf("  RE  : GPIO %d\n", RE_PIN);
    printf("  Baud: %d\n", BAUD_RATE);
    printf("\n");
    
    // Create light sensor reading task
    xTaskCreate(light_sensor_task, "light_sensor", 4096, NULL, 5, NULL);
}
