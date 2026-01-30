#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

static const char *TAG = "LORA_FINAL";

// --- STRUCT ---
typedef struct {
    spi_device_handle_t spi_handle;
    int pin_cs;
    int pin_rst;
    int pin_dio0;
} lora_dev_t;

// --- REGISTERS ---
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_PA_CONFIG            0x09
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E // <--- THE MISSING LINK
#define REG_FIFO_RX_BASE_ADDR    0x0F // <--- THE MISSING LINK
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_SYNC_WORD            0x39
#define REG_VERSION              0x42

#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05

// --- MANUAL CS CONTROL (Most Stable) ---
void cs_low(lora_dev_t *dev) { gpio_set_level(dev->pin_cs, 0); }
void cs_high(lora_dev_t *dev) { gpio_set_level(dev->pin_cs, 1); }

// --- SPI TRANSACTIONS ---
void lora_write_reg(lora_dev_t *dev, int reg, int val) {
    uint8_t out[2] = { reg | 0x80, val };
    uint8_t in[2];
    
    spi_transaction_t t = { .flags = 0, .length = 16, .tx_buffer = out, .rx_buffer = in };
    
    cs_low(dev);
    spi_device_transmit(dev->spi_handle, &t);
    cs_high(dev);
}

int lora_read_reg(lora_dev_t *dev, int reg) {
    uint8_t out[2] = { reg & 0x7F, 0xFF };
    uint8_t in[2];
    
    spi_transaction_t t = { .flags = 0, .length = 16, .tx_buffer = out, .rx_buffer = in };
    
    cs_low(dev);
    spi_device_transmit(dev->spi_handle, &t);
    cs_high(dev);
    return in[1];
}

void lora_write_fifo(lora_dev_t *dev, uint8_t *data, int len) {
    uint8_t *tx_buf = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    tx_buf[0] = REG_FIFO | 0x80;
    memcpy(&tx_buf[1], data, len);

    spi_transaction_t t = { .length = 8 * (len + 1), .tx_buffer = tx_buf, .rx_buffer = NULL };
    
    cs_low(dev);
    spi_device_transmit(dev->spi_handle, &t);
    cs_high(dev);
    
    free(tx_buf);
}

void lora_read_fifo(lora_dev_t *dev, uint8_t *data, int len) {
    uint8_t *tx_buf = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    memset(tx_buf, 0, len + 1);
    tx_buf[0] = REG_FIFO & 0x7F;

    spi_transaction_t t = { .length = 8 * (len + 1), .tx_buffer = tx_buf, .rx_buffer = rx_buf };
    
    cs_low(dev);
    spi_device_transmit(dev->spi_handle, &t);
    cs_high(dev);
    
    memcpy(data, &rx_buf[1], len);
    free(tx_buf); free(rx_buf);
}

// --- LOGIC ---
void lora_init(lora_dev_t *dev, spi_host_device_t host_id, int miso, int mosi, int clk) {
    // GPIO CS SETUP (Manual)
    gpio_reset_pin(dev->pin_cs); gpio_set_direction(dev->pin_cs, GPIO_MODE_OUTPUT); cs_high(dev);
    gpio_reset_pin(dev->pin_rst); gpio_set_direction(dev->pin_rst, GPIO_MODE_OUTPUT);
    gpio_reset_pin(dev->pin_dio0); gpio_set_direction(dev->pin_dio0, GPIO_MODE_INPUT);

    // SPI SETUP
    spi_bus_config_t buscfg = {
        .miso_io_num = miso, .mosi_io_num = mosi, .sclk_io_num = clk,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 0
    };
    spi_bus_initialize(host_id, &buscfg, SPI_DMA_CH_AUTO); 

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = -1, .queue_size = 7
    };
    spi_bus_add_device(host_id, &devcfg, &dev->spi_handle);

    // RESET & CONFIG
    gpio_set_level(dev->pin_rst, 0); vTaskDelay(2); gpio_set_level(dev->pin_rst, 1); vTaskDelay(2);

    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(2);

    // FIX: FORCE BASE ADDRESSES TO 0
    lora_write_reg(dev, REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_reg(dev, REG_FIFO_RX_BASE_ADDR, 0);

    // FREQ 433MHz
    uint64_t frf = ((uint64_t)433000000 << 19) / 32000000;
    lora_write_reg(dev, REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(dev, 0x07, (uint8_t)(frf >> 8));
    lora_write_reg(dev, 0x08, (uint8_t)(frf >> 0));

    // CONFIG: Explicit Header, SF7, CRC On
    lora_write_reg(dev, REG_MODEM_CONFIG_1, 0x72); 
    lora_write_reg(dev, REG_MODEM_CONFIG_2, 0x74); 
    lora_write_reg(dev, REG_MODEM_CONFIG_3, 0x04); 
    lora_write_reg(dev, REG_SYNC_WORD, 0xF3);
    lora_write_reg(dev, REG_PA_CONFIG, 0x80); 

    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    ESP_LOGI(TAG, "CS %d Init OK. Ver: 0x%02x", dev->pin_cs, lora_read_reg(dev, REG_VERSION));
}

void lora_send(lora_dev_t *dev, uint8_t *data, int len) {
    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    
    // Reset FIFO Ptr to 0 (Start of memory)
    lora_write_reg(dev, REG_FIFO_ADDR_PTR, 0);
    
    lora_write_fifo(dev, data, len);
    lora_write_reg(dev, 0x22, len); 
    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    
    while ((lora_read_reg(dev, REG_IRQ_FLAGS) & 0x08) == 0) vTaskDelay(1);
    lora_write_reg(dev, REG_IRQ_FLAGS, 0xFF); 
}

int lora_receive(lora_dev_t *dev, uint8_t *buf, int max_len) {
    int irq = lora_read_reg(dev, REG_IRQ_FLAGS);
    if ((irq & 0x40) == 0) return 0;

    lora_write_reg(dev, REG_IRQ_FLAGS, 0xFF);
    int len = lora_read_reg(dev, REG_RX_NB_BYTES);
    
    // Read from where the packet was stored
    int currentAddr = lora_read_reg(dev, REG_FIFO_RX_CURRENT_ADDR);
    lora_write_reg(dev, REG_FIFO_ADDR_PTR, currentAddr);
    
    if (len > max_len) len = max_len;
    lora_read_fifo(dev, buf, len);
    
    return len;
}

void app_main(void) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    
    // Module 1 (Sender - VSPI)
    lora_dev_t sender = { .pin_cs = 5, .pin_rst = 4, .pin_dio0 = 2 };
    lora_init(&sender, SPI3_HOST, 19, 23, 18);

    // Module 2 (Receiver - HSPI)
    lora_dev_t receiver = { .pin_cs = 15, .pin_rst = 26, .pin_dio0 = 27 };
    lora_init(&receiver, SPI2_HOST, 12, 13, 14);

    lora_write_reg(&receiver, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    int counter = 0;
    char tx_buf[32];
    uint8_t rx_buf[64];

    while (1) {
        sprintf(tx_buf, "IDF_TEST_%d", counter++);
        ESP_LOGI(TAG, "Sending: %s", tx_buf);
        lora_send(&sender, (uint8_t*)tx_buf, strlen(tx_buf));

        for(int i=0; i<50; i++) {
            int len = lora_receive(&receiver, rx_buf, sizeof(rx_buf));
            if (len > 0) {
                rx_buf[len] = 0;
                int rssi = lora_read_reg(&receiver, REG_PKT_RSSI_VALUE) - 157;
                ESP_LOGI(TAG, "RECEIVED: %s | RSSI: %d", rx_buf, rssi);
                break; 
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
