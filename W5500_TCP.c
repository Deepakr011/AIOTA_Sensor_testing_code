#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "W5500_LOOPBACK";

/* ===================== SPI ===================== */
#define ETH_SPI_HOST        SPI3_HOST
#define ETH_SPI_MISO_GPIO   19
#define ETH_SPI_MOSI_GPIO   23
#define ETH_SPI_SCLK_GPIO   18
#define ETH_SPI_CLOCK_MHZ   8

/* ===================== W5500 #1 ===================== */
#define ETH1_CS_GPIO        5
#define ETH1_INT_GPIO       32
#define ETH1_RST_GPIO       14

/* ===================== W5500 #2 ===================== */
#define ETH2_CS_GPIO        4
#define ETH2_INT_GPIO       33
#define ETH2_RST_GPIO       13

/* ===================== NETWORK ===================== */
#define PORT        3333
#define SERVER_IP   "192.168.2.10"
#define CLIENT_IP   "192.168.2.11"
#define NETMASK     "255.255.255.0"
#define GATEWAY     "192.168.2.1"

/* ===================== GLOBALS ===================== */
static esp_netif_t *eth1_netif = NULL;
static esp_netif_t *eth2_netif = NULL;
static bool eth1_ready = false;
static bool eth2_ready = false;

/* ===================== EVENTS ===================== */
static void eth_event_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data)
{
    if (id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Up");
    } else if (id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Down");
    }
}

static void ip_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;

    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

    if (event->esp_netif == eth1_netif) {
        eth1_ready = true;
        ESP_LOGI(TAG, "ETH1 ready");
    } else if (event->esp_netif == eth2_netif) {
        eth2_ready = true;
        ESP_LOGI(TAG, "ETH2 ready");
    }
}

/* ===================== W5500 INIT ===================== */
static esp_netif_t *init_w5500(int cs, int irq, int rst,
                               const char *ip, const char *mask, const char *gw,
                               const char *if_key, const uint8_t *mac_addr)
{
    /* Reset */
    gpio_set_direction(rst, GPIO_MODE_OUTPUT);
    gpio_set_level(rst, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(rst, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* INT pin */
    gpio_set_direction(irq, GPIO_MODE_INPUT);
    gpio_set_pull_mode(irq, GPIO_PULLUP_ONLY);

    spi_device_interface_config_t devcfg = {
        .command_bits = 16,
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = cs,
        .queue_size = 20
    };

    eth_w5500_config_t w5500_cfg =
        ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &devcfg);
    w5500_cfg.int_gpio_num = irq;

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.reset_gpio_num = -1;

    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_cfg);

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &eth_handle));

    /* Set MAC */
    ESP_ERROR_CHECK(esp_eth_ioctl(
        eth_handle, ETH_CMD_S_MAC_ADDR, (void *)mac_addr));

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_inherent_config_t base_cfg = *netif_cfg.base;
    base_cfg.if_key = if_key;
    base_cfg.if_desc = if_key;
    netif_cfg.base = &base_cfg;

    esp_netif_t *netif = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(esp_netif_attach(
        netif, esp_eth_new_netif_glue(eth_handle)));

    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(netif));

    esp_netif_ip_info_t info;
    ip4addr_aton(ip, (ip4_addr_t *)&info.ip);
    ip4addr_aton(mask, (ip4_addr_t *)&info.netmask);
    ip4addr_aton(gw, (ip4_addr_t *)&info.gw);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &info));

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    return netif;
}

/* ===================== TCP SERVER ===================== */
static void tcp_server_task(void *arg)
{
    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    listen(s, 1);
    ESP_LOGI(TAG, "[Server] Listening");

    while (1) {
        int c = accept(s, NULL, NULL);
        char rx[128];

        while (1) {
            int len = recv(c, rx, sizeof(rx) - 1, 0);
            if (len <= 0) break;
            rx[len] = 0;
            ESP_LOGI(TAG, "[Server] RX: %s", rx);
            send(c, "ACK", 3, 0);
        }
        close(c);
    }
}

/* ===================== TCP CLIENT ===================== */
static void tcp_client_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(6000));

    while (1) {
        int s = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

        struct sockaddr_in local = {
            .sin_family = AF_INET,
            .sin_addr.s_addr = inet_addr(CLIENT_IP),
            .sin_port = 0
        };
        bind(s, (struct sockaddr *)&local, sizeof(local));

        struct sockaddr_in dest = {
            .sin_family = AF_INET,
            .sin_port = htons(PORT),
            .sin_addr.s_addr = inet_addr(SERVER_IP)
        };

        if (connect(s, (struct sockaddr *)&dest, sizeof(dest)) == 0) {
            send(s, "Hello from ETH2", 15, 0);
            char rx[64];
            int len = recv(s, rx, sizeof(rx) - 1, 0);
            if (len > 0) {
                rx[len] = 0;
                ESP_LOGI(TAG, "[Client] RX: %s", rx);
            }
        }
        close(s);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ===================== APP MAIN ===================== */
void app_main(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_event_handler_register(
        ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_ETH_GOT_IP, ip_event_handler, NULL));

    gpio_install_isr_service(0);

    spi_bus_config_t buscfg = {
        .miso_io_num = ETH_SPI_MISO_GPIO,
        .mosi_io_num = ETH_SPI_MOSI_GPIO,
        .sclk_io_num = ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(
        ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    uint8_t mac1[6] = {0x02,0x00,0x00,0x00,0x00,0x01};
    uint8_t mac2[6] = {0x02,0x00,0x00,0x00,0x00,0x02};

    eth1_netif = init_w5500(
        ETH1_CS_GPIO, ETH1_INT_GPIO, ETH1_RST_GPIO,
        SERVER_IP, NETMASK, GATEWAY, "ETH1", mac1);

    eth2_netif = init_w5500(
        ETH2_CS_GPIO, ETH2_INT_GPIO, ETH2_RST_GPIO,
        CLIENT_IP, NETMASK, GATEWAY, "ETH2", mac2);

    while (!eth1_ready || !eth2_ready) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
}
    
