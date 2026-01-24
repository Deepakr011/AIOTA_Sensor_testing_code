/*
 * ESP32 MCP2515 Dual CAN Bus - Simple Loopback Test
 * 
 * Hardware Connections:
 * MCP2515 Module 1: CS=GPIO5,  MISO=GPIO19, MOSI=GPIO23, SCK=GPIO18
 * MCP2515 Module 2: CS=GPIO15, MISO=GPIO19, MOSI=GPIO23, SCK=GPIO18
 * 
 * CAN Bus: Module1(CANH/CANL) <-> Module2(CANH/CANL) + 120Ω termination
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "CAN_TEST";

// Pin Configuration
#define CAN1_CS     5
#define CAN2_CS     15
#define SPI_MISO    19
#define SPI_MOSI    23
#define SPI_SCK     18

// MCP2515 Commands
#define CMD_RESET       0xC0
#define CMD_READ        0x03
#define CMD_WRITE       0x02
#define CMD_RTS         0x80
#define CMD_BIT_MODIFY  0x05

// Key Registers
#define REG_CANCTRL     0x0F
#define REG_CANSTAT     0x0E
#define REG_CANINTF     0x2C
#define REG_CNF1        0x2A
#define REG_CNF2        0x29
#define REG_CNF3        0x28
#define REG_RXB0CTRL    0x60

// Modes
#define MODE_CONFIG     0x80
#define MODE_NORMAL     0x00

spi_device_handle_t spi1, spi2;

// Basic SPI functions
void mcp_reset(spi_device_handle_t spi) {
    uint8_t cmd = CMD_RESET;
    spi_transaction_t t = {.length = 8, .tx_buffer = &cmd};
    spi_device_transmit(spi, &t);
    vTaskDelay(pdMS_TO_TICKS(10));
}

uint8_t mcp_read(spi_device_handle_t spi, uint8_t reg) {
    uint8_t tx[3] = {CMD_READ, reg, 0};
    uint8_t rx[3];
    spi_transaction_t t = {.length = 24, .tx_buffer = tx, .rx_buffer = rx};
    spi_device_transmit(spi, &t);
    return rx[2];
}

void mcp_write(spi_device_handle_t spi, uint8_t reg, uint8_t val) {
    uint8_t tx[3] = {CMD_WRITE, reg, val};
    spi_transaction_t t = {.length = 24, .tx_buffer = tx};
    spi_device_transmit(spi, &t);
}

void mcp_bit_modify(spi_device_handle_t spi, uint8_t reg, uint8_t mask, uint8_t val) {
    uint8_t tx[4] = {CMD_BIT_MODIFY, reg, mask, val};
    spi_transaction_t t = {.length = 32, .tx_buffer = tx};
    spi_device_transmit(spi, &t);
}

// Initialize MCP2515
bool mcp_init(spi_device_handle_t spi, const char* name) {
    mcp_reset(spi);
    
    // Enter config mode
    mcp_bit_modify(spi, REG_CANCTRL, 0xE0, MODE_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Verify config mode
    if ((mcp_read(spi, REG_CANSTAT) & 0xE0) != MODE_CONFIG) {
        ESP_LOGE(TAG, "%s: Config mode failed", name);
        return false;
    }
    
    // 125kbps @ 8MHz crystal
    mcp_write(spi, REG_CNF1, 0x01);
    mcp_write(spi, REG_CNF2, 0xB1);
    mcp_write(spi, REG_CNF3, 0x05);
    
    // Disable filters (accept all)
    mcp_write(spi, REG_RXB0CTRL, 0x60);
    
    // Enter normal mode
    mcp_bit_modify(spi, REG_CANCTRL, 0xE0, MODE_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Verify normal mode
    if ((mcp_read(spi, REG_CANSTAT) & 0xE0) != MODE_NORMAL) {
        ESP_LOGE(TAG, "%s: Normal mode failed", name);
        return false;
    }
    
    ESP_LOGI(TAG, "%s initialized ✓", name);
    return true;
}

// Send CAN message
void mcp_send(spi_device_handle_t spi, uint16_t id, uint8_t *data, uint8_t len) {
    // Write to TX buffer 0 (base address 0x31)
    mcp_write(spi, 0x31, id >> 3);           // SIDH
    mcp_write(spi, 0x32, id << 5);           // SIDL
    mcp_write(spi, 0x35, len);               // DLC
    for (int i = 0; i < len; i++) {
        mcp_write(spi, 0x36 + i, data[i]);   // Data
    }
    
    // Request to send
    uint8_t cmd = CMD_RTS | 0x01;
    spi_transaction_t t = {.length = 8, .tx_buffer = &cmd};
    spi_device_transmit(spi, &t);
}

// Receive CAN message
bool mcp_receive(spi_device_handle_t spi, uint16_t *id, uint8_t *data, uint8_t *len) {
    // Check for message
    if (!(mcp_read(spi, REG_CANINTF) & 0x01)) {
        return false;
    }
    
    // Read from RX buffer 0 (base address 0x61)
    uint8_t sidh = mcp_read(spi, 0x61);
    uint8_t sidl = mcp_read(spi, 0x62);
    *id = (sidh << 3) | (sidl >> 5);
    *len = mcp_read(spi, 0x65) & 0x0F;
    
    for (int i = 0; i < *len; i++) {
        data[i] = mcp_read(spi, 0x66 + i);
    }
    
    // Clear interrupt
    mcp_bit_modify(spi, REG_CANINTF, 0x01, 0x00);
    return true;
}

void app_main(void) {
    ESP_LOGI(TAG, "=== MCP2515 CAN Loopback Test ===");
    
    // Init SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    
    // Add CAN1
    spi_device_interface_config_t dev1_cfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = CAN1_CS,
        .queue_size = 3,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev1_cfg, &spi1));
    
    // Add CAN2
    spi_device_interface_config_t dev2_cfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = CAN2_CS,
        .queue_size = 3,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev2_cfg, &spi2));
    
    // Initialize both modules
    if (!mcp_init(spi1, "CAN1") || !mcp_init(spi2, "CAN2")) {
        ESP_LOGE(TAG, "Initialization failed!");
        return;
    }
    
    ESP_LOGI(TAG, "Starting loopback test...\n");
    
    // Test loop
    uint32_t count = 0;
    while (1) {
        // Prepare message
        uint8_t tx_data[8];
        for (int i = 0; i < 8; i++) {
            tx_data[i] = (count + i) & 0xFF;
        }
        
        // Send from CAN1
        ESP_LOGI(TAG, "[%lu] TX: ID=0x123, Data=%02X %02X %02X %02X %02X %02X %02X %02X",
                 count, tx_data[0], tx_data[1], tx_data[2], tx_data[3],
                 tx_data[4], tx_data[5], tx_data[6], tx_data[7]);
        
        mcp_send(spi1, 0x123, tx_data, 8);
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Receive on CAN2
        uint16_t rx_id;
        uint8_t rx_data[8], rx_len;
        
        if (mcp_receive(spi2, &rx_id, rx_data, &rx_len)) {
            ESP_LOGI(TAG, "[%lu] RX: ID=0x%03X, Data=%02X %02X %02X %02X %02X %02X %02X %02X",
                     count, rx_id, rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                     rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
            
            // Verify
            bool ok = (rx_id == 0x123 && rx_len == 8);
            for (int i = 0; i < 8 && ok; i++) {
                if (rx_data[i] != tx_data[i]) ok = false;
            }
            
            if (ok) {
                ESP_LOGI(TAG, "✓ PASS\n");
            } else {
                ESP_LOGE(TAG, "✗ FAIL\n");
            }
        } else {
            ESP_LOGW(TAG, "✗ No message received\n");
        }
        
        count++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
