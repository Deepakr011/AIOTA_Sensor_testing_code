#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define TAG "UART_FULL_DUPLEX"

/* UART ports */
#define UART1 UART_NUM_1
#define UART2 UART_NUM_2

/* YOUR PIN SELECTION */
#define UART1_TX_PIN 17
#define UART1_RX_PIN 16

#define UART2_TX_PIN 25
#define UART2_RX_PIN 26

#define BUF_SIZE 1024

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    // UART1
    uart_driver_install(UART1, BUF_SIZE*2, BUF_SIZE*2, 0, NULL, 0);
    uart_param_config(UART1, &cfg);
    uart_set_pin(UART1, UART1_TX_PIN, UART1_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // UART2
    uart_driver_install(UART2, BUF_SIZE*2, BUF_SIZE*2, 0, NULL, 0);
    uart_param_config(UART2, &cfg);
    uart_set_pin(UART2, UART2_TX_PIN, UART2_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/* UART1 sends, UART2 receives */
static void uart1_to_uart2_task(void *arg)
{
    uint8_t rx_buf[BUF_SIZE];

    while (1) {
        const char *msg = "Hello from UART1\r\n";

        uart_write_bytes(UART1, msg, strlen(msg));
        ESP_LOGI(TAG, "UART1 Sent: %s", msg);

        int len = uart_read_bytes(UART2, rx_buf,
                                  BUF_SIZE-1,
                                  pdMS_TO_TICKS(1000));

        if (len > 0) {
            rx_buf[len] = '\0';
            ESP_LOGI(TAG, "UART2 Received: %s", rx_buf);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* UART2 sends, UART1 receives */
static void uart2_to_uart1_task(void *arg)
{
    uint8_t rx_buf[BUF_SIZE];

    while (1) {
        const char *msg = "Hello from UART2\r\n";

        uart_write_bytes(UART2, msg, strlen(msg));
        ESP_LOGI(TAG, "UART2 Sent: %s", msg);

        int len = uart_read_bytes(UART1, rx_buf,
                                  BUF_SIZE-1,
                                  pdMS_TO_TICKS(1000));

        if (len > 0) {
            rx_buf[len] = '\0';
            ESP_LOGI(TAG, "UART1 Received: %s", rx_buf);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    uart_init();

    xTaskCreate(uart1_to_uart2_task, "uart1_to_uart2",
                4096, NULL, 10, NULL);

    xTaskCreate(uart2_to_uart1_task, "uart2_to_uart1",
                4096, NULL, 10, NULL);
}
