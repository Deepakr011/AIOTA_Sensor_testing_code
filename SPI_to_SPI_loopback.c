#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "SPI_LOOP"

/* MASTER PINS (VSPI) */
#define PIN_MOSI_M 23
#define PIN_MISO_M 19
#define PIN_CLK_M  18
#define PIN_CS_M   5

/* SLAVE PINS (HSPI) */
#define PIN_MOSI_S 13
#define PIN_MISO_S 12
#define PIN_CLK_S  14
#define PIN_CS_S   15

#define BUF_SIZE 32

uint8_t master_tx[BUF_SIZE] = "Hello from Master!";
uint8_t master_rx[BUF_SIZE];

uint8_t slave_rx[BUF_SIZE];
uint8_t slave_tx[BUF_SIZE] = "Hello from Slave!";

/* ================= MASTER TASK ================= */
void spi_master_task(void *arg)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI_M,
        .miso_io_num = PIN_MISO_M,
        .sclk_io_num = PIN_CLK_M,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_CS_M,
        .queue_size = 1,
    };

    spi_device_handle_t handle;
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &handle);

    while (1) {
        spi_transaction_t t = {
            .length = BUF_SIZE * 8,
            .tx_buffer = master_tx,
            .rx_buffer = master_rx,
        };

        esp_err_t ret = spi_device_transmit(handle, &t);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Master RX: %s", master_rx);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================= SLAVE TASK ================= */
void spi_slave_task(void *arg)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI_S,
        .miso_io_num = PIN_MISO_S,
        .sclk_io_num = PIN_CLK_S,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = PIN_CS_S,
        .mode = 0,
        .queue_size = 1,
    };

    spi_slave_initialize(SPI3_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

    while (1) {
        spi_slave_transaction_t t = {
            .length = BUF_SIZE * 8,
            .tx_buffer = slave_tx,
            .rx_buffer = slave_rx,
        };

        spi_slave_transmit(SPI3_HOST, &t, portMAX_DELAY);
        ESP_LOGI(TAG, "Slave RX: %s", slave_rx);
    }
}

/* ================= APP MAIN ================= */
void app_main(void)
{
    xTaskCreate(spi_slave_task, "spi_slave", 4096, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));   // ensure slave ready
    xTaskCreate(spi_master_task, "spi_master", 4096, NULL, 5, NULL);
}
