#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ---------------- CONFIG ----------------
#define I2C_NUM         I2C_NUM_0
#define SDA_PIN         21
#define SCL_PIN         22
#define MCP4725_ADDR   0x60

#define ADC_CH          ADC1_CHANNEL_6   // GPIO34

// ------------- GLOBALS ------------------
static esp_adc_cal_characteristics_t adc_chars;

// ------------- I2C INIT -----------------
void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM, &conf);
    i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);
}

// ----------- MCP4725 WRITE --------------
void dac_write(uint16_t value)
{
    uint8_t high = value >> 4;
    uint8_t low  = (value & 0x0F) << 4;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP4725_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true);   // fast mode
    i2c_master_write_byte(cmd, high, true);
    i2c_master_write_byte(cmd, low, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// --------------- ADC INIT ---------------
void adc_init(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CH, ADC_ATTEN_DB_11);

    // Characterize ADC
    esp_adc_cal_characterize(ADC_UNIT_1,
                             ADC_ATTEN_DB_11,
                             ADC_WIDTH_BIT_12,
                             1100,
                             &adc_chars);
}

// ----------- READ ADC AVERAGE -----------
int adc_read_avg(void)
{
    int sum = 0;
    for(int i=0;i<8;i++)
        sum += adc1_get_raw(ADC_CH);

    return sum / 8;
}

// ---------------- MAIN ------------------
void app_main(void)
{
    i2c_init();
    adc_init();

    while (1)
    {
        for (int val = 0; val <= 4095; val += 512)
        {
            dac_write(val);
            vTaskDelay(10 / portTICK_PERIOD_MS);

            int raw = adc_read_avg();
            uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

            printf("DAC:%4d  ADC:%4d  %4lu mV\n", val, raw, (unsigned long)mv);

            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}
