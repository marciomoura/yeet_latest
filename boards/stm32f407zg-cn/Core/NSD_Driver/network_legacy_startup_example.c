/**
 * @file network_legacy_startup_example.c
 * @brief Example showing how to use legacy compatibility helpers to start a TCP server.
 * @note  Excluded from build unless explicitly added; serves as reference.
 */
#include "w5200_driver.h"
#include "w5200_legacy_compat.h"

/* Static instance and buffers (adjust sizes to your DMA size if used) */
static w5200_instance_t w5200_spi2;
static uint8_t spi2_tx_buf[1024];
static uint8_t spi2_rx_buf[1024];

/* Example configuration (adapt IP/MAC/gateway for your network) */
static const w5200_network_config_t net_cfg = {.ip_address = {{192, 168, 2, 100}},
    .subnet_mask = {{255, 255, 255, 0}},
    .gateway = {{192, 168, 2, 1}},
    .mac_address = {{0x34, 0x24, 0xE8, 0xB9, 0x8B, 0x31}},
    .retry_time = 0x0FA0, /* 400 ms */
    .retry_count = 1};

static const w5200_socket_config_t sock0_cfg = {.enabled = true,
    .protocol = W5200_PROTO_TCP,
    .mode = W5200_MODE_SERVER,
    .source_port = 7000,
    .dest_port = 0, /* Not used for server; still set */
    .dest_ip = {{0, 0, 0, 0}}};

/* Call this early in main after clocks are set */
bool Legacy_Network_Start(void)
{
    if (!W5200_Init(&w5200_spi2, SPI2, spi2_tx_buf, spi2_rx_buf, W5200_DMA_SIZE_1K)) {
        return false;
    }

    if (!W5200_LegacyConfigureCommon(&w5200_spi2, &net_cfg)) {
        return false;
    }

    if (!W5200_LegacyConfigureSocket(&w5200_spi2, 0, &sock0_cfg)) {
        return false;
    }

    if (!W5200_LegacyOpenTCPServer(&w5200_spi2, 0)) {
        return false;
    }

    return true; /* Socket 0 now listening */
}
