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
#define DE_PIN      GPIO_NUM_5    // Separate DE pin
#define RE_PIN      GPIO_NUM_4    // Separate RE pin

#define BAUD_RATE   9600
#define BUF_SIZE    128

/* ---------------- MODBUS Queries (from your working Arduino code) ---------------- */
uint8_t npkQuery[]             = {0x01, 0x03, 0x00, 0x1E, 0x00, 0x03, 0x65, 0xCD};
uint8_t phQuery[]              = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0B};
uint8_t soilMoistureQuery[]    = {0x01, 0x03, 0x00, 0x12, 0x00, 0x01, 0x24, 0x0F};
uint8_t soilTemperatureQuery[] = {0x01, 0x03, 0x00, 0x13, 0x00, 0x01, 0x75, 0xCF};
uint8_t conductivityQuery[]    = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01, 0x95, 0xCE};

/* ---------------- RS485 Control Functions ---------------- */
void rs485_set_transmit_mode(void)
{
    gpio_set_level(DE_PIN, 1);  // DE = HIGH (enable driver)
    gpio_set_level(RE_PIN, 1);  // RE = HIGH (disable receiver)
    esp_rom_delay_us(10);       // 10 microseconds delay
}

void rs485_set_receive_mode(void)
{
    gpio_set_level(DE_PIN, 0);  // DE = LOW (disable driver)
    gpio_set_level(RE_PIN, 0);  // RE = LOW (enable receiver)
    esp_rom_delay_us(10);       // 10 microseconds delay
}

/* ---------------- GPIO Init ---------------- */
void gpio_init_rs485(void)
{
    // Configure DE pin
    gpio_reset_pin(DE_PIN);
    gpio_set_direction(DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DE_PIN, 0);
    
    // Configure RE pin
    gpio_reset_pin(RE_PIN);
    gpio_set_direction(RE_PIN, GPIO_MODE_OUTPUT);
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

/* ---------------- Send Query Function ---------------- */
void send_query(uint8_t *query, uint8_t qlen)
{
    // Flush any pending data
    uart_flush_input(UART_NUM);
    
    // Set to transmit mode
    rs485_set_transmit_mode();
    esp_rom_delay_us(10);
    
    // Send query
    uart_write_bytes(UART_NUM, (char *)query, qlen);
    uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(100));
    
    // Set to receive mode
    rs485_set_receive_mode();
}

/* ---------------- Read Single Register (7 bytes response) ---------------- */
uint16_t read_register(void)
{
    uint8_t resp[7];
    int len = uart_read_bytes(UART_NUM, resp, 7, pdMS_TO_TICKS(1000));
    
    if (len == 7) {
        // Response format: [addr][func][len][data_H][data_L][crc_L][crc_H]
        uint16_t value = (resp[3] << 8) | resp[4];
        return value;
    }
    
    return 0;
}

/* ---------------- Read NPK (11 bytes response) ---------------- */
bool read_npk(uint16_t *nitrogen, uint16_t *phosphorus, uint16_t *potassium)
{
    uint8_t npkResp[11];
    int len = uart_read_bytes(UART_NUM, npkResp, 11, pdMS_TO_TICKS(1000));
    
    if (len == 11) {
        // Response format: [addr][func][len][N_H][N_L][P_H][P_L][K_H][K_L][crc_L][crc_H]
        *nitrogen   = (npkResp[3] << 8) | npkResp[4];
        *phosphorus = (npkResp[5] << 8) | npkResp[6];
        *potassium  = (npkResp[7] << 8) | npkResp[8];
        return true;
    }
    
    return false;
}

/* ---------------- Main Task ---------------- */
void soil_sensor_task(void *pvParameters)
{
    printf("\nESP32 Soil Sensor Modbus Started\n");
    printf("Waiting 5 seconds for sensor warm-up...\n\n");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    while (1) {
        // -------- pH --------
        send_query(phQuery, sizeof(phQuery));
        vTaskDelay(pdMS_TO_TICKS(200));
        float pH = read_register() / 100.0;
        
        // -------- Moisture --------
        send_query(soilMoistureQuery, sizeof(soilMoistureQuery));
        vTaskDelay(pdMS_TO_TICKS(200));
        float moisture = read_register() / 10.0;
        
        // -------- Temperature --------
        send_query(soilTemperatureQuery, sizeof(soilTemperatureQuery));
        vTaskDelay(pdMS_TO_TICKS(200));
        float temperature = read_register() / 10.0;
        
        // -------- EC (Conductivity) --------
        send_query(conductivityQuery, sizeof(conductivityQuery));
        vTaskDelay(pdMS_TO_TICKS(200));
        uint16_t ec = read_register(); // μS/cm
        
        // -------- NPK --------
        send_query(npkQuery, sizeof(npkQuery));
        vTaskDelay(pdMS_TO_TICKS(200));
        
        uint16_t nitrogen = 0, phosphorus = 0, potassium = 0;
        bool npk_success = read_npk(&nitrogen, &phosphorus, &potassium);
        
        // -------- PRINT ALL VALUES --------
        printf("------------ SOIL DATA ------------\n");
        printf("pH          : %.2f\n", pH);
        printf("Moisture    : %.1f %%\n", moisture);
        printf("Temperature : %.1f °C\n", temperature);
        printf("EC          : %u μS/cm\n", ec);
        
        if (npk_success) {
            printf("Nitrogen    : %u mg/kg\n", nitrogen);
            printf("Phosphorus  : %u mg/kg\n", phosphorus);
            printf("Potassium   : %u mg/kg\n", potassium);
        } else {
            printf("NPK         : Read Failed\n");
        }
        
        printf("-----------------------------------\n\n");
        
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ---------------- app_main ---------------- */
void app_main(void)
{
    printf("\n\n");
    printf("==========================================\n");
    printf("  NPK 7-in-1 Soil Sensor Reader\n");
    printf("  (Converted from working Arduino code)\n");
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
    
    // Create sensor reading task
    xTaskCreate(soil_sensor_task, "soil_sensor", 4096, NULL, 5, NULL);
}
