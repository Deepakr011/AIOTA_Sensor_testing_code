#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO  21
#define I2C_MASTER_SCL_IO  22
#define I2C_MASTER_FREQ_HZ 100000

#define ADS1115_ADDR 0x48

#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01

// ================= I2C INIT =================
void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ================= ADS1115 WRITE CONFIG =================
void ads1115_start_conversion(void)
{
    uint8_t data[3];

    data[0] = ADS1115_REG_CONFIG;
    data[1] = 0xC2;   // MSB
    data[2] = 0x83;   // LSB
    // 0xC283:
    // AIN0, Single-shot, ±4.096V, 128 SPS

    i2c_master_write_to_device(
        I2C_MASTER_NUM,
        ADS1115_ADDR,
        data,
        3,
        pdMS_TO_TICKS(100)
    );
}

// ================= ADS1115 READ =================
int16_t ads1115_read(void)
{
    uint8_t reg = ADS1115_REG_CONVERSION;
    uint8_t data[2];

    i2c_master_write_to_device(
        I2C_MASTER_NUM,
        ADS1115_ADDR,
        &reg,
        1,
        pdMS_TO_TICKS(100)
    );

    i2c_master_read_from_device(
        I2C_MASTER_NUM,
        ADS1115_ADDR,
        data,
        2,
        pdMS_TO_TICKS(100)
    );

    return (int16_t)((data[0] << 8) | data[1]);
}

// ================= MAIN =================
void app_main(void)
{
    i2c_master_init();

    printf("ADS1115 POT Reading Started...\n");

    while (1)
    {
        ads1115_start_conversion();
        vTaskDelay(pdMS_TO_TICKS(10));   // wait for conversion

        int16_t raw = ads1115_read();

        float voltage = (raw * 4.096) / 32768.0;
        float percent = (voltage / 3.3) * 100.0;

        printf("RAW: %d  Voltage: %.2f V  POT: %.1f %%\n",
               raw, voltage, percent);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
