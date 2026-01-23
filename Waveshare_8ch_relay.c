#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_NUM    UART_NUM_2
#define TXD_PIN     17
#define RXD_PIN     16
#define DE_PIN      5
#define RE_PIN      4

uint16_t crc16(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (int i = 8; i != 0; i--) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else { crc >>= 1; }
        }
    }
    return crc;
}

void init(void) {
    gpio_set_direction(DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DE_PIN, 0);
    gpio_set_level(RE_PIN, 0);
    
    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM, &cfg);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, -1, -1);
    uart_driver_install(UART_NUM, 256, 256, 0, NULL, 0);
}

void relay(uint8_t ch, uint8_t state) {  // ch: 0-7, state: 0=OFF, 1=ON
    uint8_t cmd[8] = {0x01, 0x05, 0x00, ch, state ? 0xFF : 0x00, 0x00};
    uint16_t crc = crc16(cmd, 6);
    cmd[6] = crc & 0xFF;
    cmd[7] = crc >> 8;
    
    gpio_set_level(DE_PIN, 1);
    gpio_set_level(RE_PIN, 1);
    uart_write_bytes(UART_NUM, (char*)cmd, 8);
    uart_wait_tx_done(UART_NUM, 100);
    gpio_set_level(DE_PIN, 0);
    gpio_set_level(RE_PIN, 0);
}

void app_main(void) {
    init();
    
    while(1) {
        relay(0, 1);  // CH1 ON
        vTaskDelay(1000);
        relay(0, 0);  // CH1 OFF
        vTaskDelay(1000);
    }
}
