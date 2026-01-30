#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"

// Pin definitions
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define MOUNT_POINT "/sdcard"

// --- FUNCTION PROTOTYPES ---
esp_err_t sd_write_file(const char *path, const char *data);
esp_err_t sd_read_file(const char *path);

void app_main(void) {
    esp_err_t ret;

    // 1. Mount Configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // 2. Host Configuration (Required for IDF v4.2+)
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // 3. SPI Bus Configuration
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        printf("Failed to initialize SPI bus.\n");
        return;
    }

    // 4. Slot Configuration
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    // 5. Mounting (The 5-argument version)
    sdmmc_card_t *card;
    printf("Mounting filesystem...\n");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        printf("Mount failed: %s\n", esp_err_to_name(ret));
        return;
    }
    printf("Filesystem mounted.\n");

    // 6. Practical usage
    sd_write_file(MOUNT_POINT "/test.txt", "Initial LoRa Log Entry...\n");
    sd_write_file(MOUNT_POINT "/test.txt", "Updating with new sensor data.\n");
    
    sd_read_file(MOUNT_POINT "/test.txt");
}

// --- HELPER FUNCTIONS ---

esp_err_t sd_write_file(const char *path, const char *data) {
    printf("Writing to %s... ", path);
    FILE *f = fopen(path, "a"); // "a" appends to the file
    if (f == NULL) {
        printf("Fail!\n");
        return ESP_FAIL;
    }
    fprintf(f, "%s", data);
    fclose(f);
    printf("Done.\n");
    return ESP_OK;
}

esp_err_t sd_read_file(const char *path) {
    printf("Reading %s:\n", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        printf("Failed to open file.\n");
        return ESP_FAIL;
    }

    char line[128];
    while (fgets(line, sizeof(line), f) != NULL) {
        printf(" > %s", line);
    }
    fclose(f);
    return ESP_OK;
}
