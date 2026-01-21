#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/* ---------------- Configuration ---------------- */
#define UART_NUM    UART_NUM_1
#define TXD_PIN     GPIO_NUM_17
#define RXD_PIN     GPIO_NUM_16
#define DE_PIN      GPIO_NUM_4   // Driver Enable (active HIGH for transmit)
#define RE_PIN      GPIO_NUM_5   // Receiver Enable (active LOW for receive)

#define BAUD_RATE   9600
#define BUF_SIZE    256

/* ---------------- CRC16 (Modbus) ---------------- */
uint16_t modbus_crc16(uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ---------------- RS485 Control Functions ---------------- */
void rs485_set_transmit_mode(void)
{
    gpio_set_level(DE_PIN, 1);  // DE = HIGH (enable driver)
    gpio_set_level(RE_PIN, 1);  // RE = HIGH (disable receiver)
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for mode switching
}

void rs485_set_receive_mode(void)
{
    gpio_set_level(DE_PIN, 0);  // DE = LOW (disable driver)
    gpio_set_level(RE_PIN, 0);  // RE = LOW (enable receiver)
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for mode switching
}

/* ---------------- GPIO Init ---------------- */
void gpio_init_rs485_control(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DE_PIN) | (1ULL << RE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Start in receive mode
    rs485_set_receive_mode();
}

/* ---------------- UART Init ---------------- */
void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
  
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    
    uart_set_mode(UART_NUM, UART_MODE_UART);

    uart_set_rx_timeout(UART_NUM, 3);
}

/* ---------------- app_main ---------------- */
void app_main(void)
{
    // Initialize GPIO control pins first
    gpio_init_rs485_control();
    
    // Then initialize UART
    uart_init();

    uint8_t request[8] = {
        0x00, 0x03, 0x00, 0x01, 0x00, 0x02
    };

    uint8_t rx_buf[BUF_SIZE];

    uint16_t crc = modbus_crc16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = crc >> 8;

    while (1) {

        /* -------- Send request -------- */
        printf("TX: ");
        for (int i = 0; i < 8; i++) {
            printf("0x%02X ", request[i]);
        }
        printf("\n");

        // Switch to transmit mode
        rs485_set_transmit_mode();
        
        uart_write_bytes(UART_NUM, (char *)request, 8);
        uart_wait_tx_done(UART_NUM, portMAX_DELAY);
        
        // Switch to receive mode
        rs485_set_receive_mode();

        /* -------- Read response -------- */
        int len = uart_read_bytes(
            UART_NUM,
            rx_buf,
            sizeof(rx_buf),
            pdMS_TO_TICKS(1000)
        );

        if (len > 0) {
            printf("RX (%d bytes): ", len);
            for (int i = 0; i < len; i++) {
                printf("0x%02X ", rx_buf[i]);
            }
            printf("\n");

            /* Expected response:
               00 03 04 00 FA 02 58 CRC_L CRC_H
            */
            if (len >= 9 && rx_buf[1] == 0x03 && rx_buf[2] == 0x04) {

                uint16_t rawTemp = (rx_buf[3] << 8) | rx_buf[4];
                uint16_t rawHum  = (rx_buf[5] << 8) | rx_buf[6];

                float temperature = rawTemp / 10.0;
                float humidity    = rawHum / 10.0;

                printf("Temperature: %.1f C\n", temperature);
                printf("Humidity   : %.1f %%RH\n", humidity);
            }
        } else {
            printf("No response received\n");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
