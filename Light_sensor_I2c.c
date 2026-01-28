#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM  I2C_NUM_0
#define SDA_PIN         21
#define SCL_PIN         22
#define I2C_FREQ        100000

#define BH1750_ADDR     0x23
#define BH1750_POWER_ON 0x01
#define BH1750_CONT_HIRES 0x10

static const char *TAG = "BH1750";

/*------------------------------------------------*/
void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/*------------------------------------------------*/
esp_err_t bh1750_init(void)
{
    uint8_t cmd = BH1750_POWER_ON;

    esp_err_t ret = i2c_master_write_to_device(
        I2C_MASTER_NUM, BH1750_ADDR,
        &cmd, 1, pdMS_TO_TICKS(100)
    );

    if (ret != ESP_OK) return ret;

    cmd = BH1750_CONT_HIRES;

    return i2c_master_write_to_device(
        I2C_MASTER_NUM, BH1750_ADDR,
        &cmd, 1, pdMS_TO_TICKS(100)
    );
}

/*------------------------------------------------*/
esp_err_t bh1750_read(float *lux)
{
    uint8_t data[2];

    esp_err_t ret = i2c_master_read_from_device(
        I2C_MASTER_NUM,
        BH1750_ADDR,
        data,
        2,
        pdMS_TO_TICKS(100)
    );

    if (ret != ESP_OK) return ret;

    uint16_t raw = (data[0] << 8) | data[1];
    *lux = raw / 1.2f;

    return ESP_OK;
}

/*------------------------------------------------*/
void app_main(void)
{
    i2c_master_init();

    if (bh1750_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "BH1750 Init Failed");
        return;
    }

    ESP_LOGI(TAG, "BH1750 Initialized");

    float lux;

    while (1)
    {
        if (bh1750_read(&lux) == ESP_OK)
        {
            ESP_LOGI(TAG, "Ambient Light: %.2f lux", lux);
        }
        else
        {
            ESP_LOGE(TAG, "Read error");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
