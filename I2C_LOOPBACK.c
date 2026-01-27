#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "I2C_LOOPBACK"

/* ================= PIN CONFIG ================= */

/* Master (I2C0) */
#define I2C_MASTER_SDA 21
#define I2C_MASTER_SCL 22

/* Slave (I2C1) */
#define I2C_SLAVE_SDA  18
#define I2C_SLAVE_SCL  19

/* ================= CONFIG ================= */

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_SLAVE_NUM  I2C_NUM_1

#define I2C_FREQ_HZ    100000
#define SLAVE_ADDR    0x28

#define RX_BUF_LEN    128
#define TX_BUF_LEN    128

#define FRAME_START   0xAA

/* ================= BUFFERS ================= */

uint8_t slave_rx_buf[RX_BUF_LEN];
uint8_t slave_tx_buf[] = "Hello from I2C Slave!";
uint8_t master_rx_buf[RX_BUF_LEN];

/* ================================================= */
/*                 SLAVE SECTION                     */
/* ================================================= */

static void i2c_slave_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA,
        .scl_io_num = I2C_SLAVE_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .slave_addr = SLAVE_ADDR,
            .addr_10bit_en = 0
        }
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(
        I2C_SLAVE_NUM,
        I2C_MODE_SLAVE,
        RX_BUF_LEN,
        TX_BUF_LEN,
        0
    ));

    ESP_LOGI(TAG, "I2C Slave Initialized");
}

void i2c_slave_task(void *arg)
{
    i2c_slave_init();

    while (1) {

        int len = i2c_slave_read_buffer(
            I2C_SLAVE_NUM,
            slave_rx_buf,
            RX_BUF_LEN,
            portMAX_DELAY
        );

        if (len >= 2 && slave_rx_buf[0] == FRAME_START) {

            uint8_t msg_len = slave_rx_buf[1];

            if (msg_len <= (len - 2)) {

                char msg[64];
                memcpy(msg, &slave_rx_buf[2], msg_len);
                msg[msg_len] = 0;

                ESP_LOGI(TAG, "Slave RX: %s", msg);
            }
        }

        /* Send reply */
        i2c_slave_write_buffer(
            I2C_SLAVE_NUM,
            slave_tx_buf,
            strlen((char *)slave_tx_buf),
            portMAX_DELAY
        );
    }
}

/* ================================================= */
/*                 MASTER SECTION                    */
/* ================================================= */

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(
        I2C_MASTER_NUM,
        I2C_MODE_MASTER,
        0,
        0,
        0
    ));

    ESP_LOGI(TAG, "I2C Master Initialized");
}

void i2c_master_task(void *arg)
{
    i2c_master_init();

    uint8_t payload[] = "Hello from I2C Master!";
    uint8_t tx_buf[64];

    while (1) {

        tx_buf[0] = FRAME_START;
        tx_buf[1] = strlen((char*)payload);
        memcpy(&tx_buf[2], payload, tx_buf[1]);

        ESP_ERROR_CHECK(
            i2c_master_write_to_device(
                I2C_MASTER_NUM,
                SLAVE_ADDR,
                tx_buf,
                tx_buf[1] + 2,
                pdMS_TO_TICKS(1000)
            )
        );

        vTaskDelay(pdMS_TO_TICKS(50));

        int len = i2c_master_read_from_device(
            I2C_MASTER_NUM,
            SLAVE_ADDR,
            master_rx_buf,
            RX_BUF_LEN,
            pdMS_TO_TICKS(1000)
        );

        if (len > 0) {
            master_rx_buf[len] = 0;
            ESP_LOGI(TAG, "Master RX: %s", master_rx_buf);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================================================= */
/*                    MAIN                            */
/* ================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting I2C Loopback (Framed)");

    xTaskCreate(i2c_slave_task, "i2c_slave", 4096, NULL, 10, NULL);
    vTaskDelay(pdMS_TO_TICKS(300));
    xTaskCreate(i2c_master_task, "i2c_master", 4096, NULL, 10, NULL);
}
