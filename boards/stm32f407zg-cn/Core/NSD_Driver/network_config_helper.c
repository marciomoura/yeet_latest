/**
 ******************************************************************************
 * @file    network_config_helper.c
 * @brief   Helper functions for W5200 network configuration matching legacy behavior
 * @date    2025
 ******************************************************************************
 */

#include "network_config_helper.h"

#include <string.h>

/* ============================================================================
 * Public Function Implementations
 * ============================================================================ */

/**
 * @brief Helper to populate network config structure for SPI2
 */
void NetCfg_PopulateSPI2Config(w5200_network_config_t* config, uint8_t node_id)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(w5200_network_config_t));

    // IP Address: 192.168.2.(100 + node_id)
    config->ip_address.bytes[0] = 192;
    config->ip_address.bytes[1] = 168;
    config->ip_address.bytes[2] = 2;
    config->ip_address.bytes[3] = NETCFG_SPI2_BASE_IP_OCTET + node_id;

    // Gateway: 192.168.2.1
    config->gateway.bytes[0] = 192;
    config->gateway.bytes[1] = 168;
    config->gateway.bytes[2] = 2;
    config->gateway.bytes[3] = 1;

    // Subnet Mask: 255.255.255.0
    config->subnet_mask.bytes[0] = 255;
    config->subnet_mask.bytes[1] = 255;
    config->subnet_mask.bytes[2] = 255;
    config->subnet_mask.bytes[3] = 0;

    // MAC Address: Legacy pattern
    // If node_id == 0: MAC = 0x34:24:E8:B9:8B:31
    // Otherwise: MAC[4] = 0x8B + node_id
    config->mac_address.bytes[0] = 0x34;
    config->mac_address.bytes[1] = 0x24;
    config->mac_address.bytes[2] = 0xE8;
    config->mac_address.bytes[3] = 0xB9;

    if (node_id == 0) {
        config->mac_address.bytes[4] = 0x8B;
        config->mac_address.bytes[5] = 0x31;
    }
    else {
        config->mac_address.bytes[4] = 0x8B + node_id;
        config->mac_address.bytes[5] = 0x30;
    }

    // Retry configuration
    config->retry_time = NETCFG_DEFAULT_RETRY_TIME;
    config->retry_count = NETCFG_DEFAULT_RETRY_COUNT;
}

/**
 * @brief Helper to populate network config structure for SPI3
 */
void NetCfg_PopulateSPI3Config(w5200_network_config_t* config, uint8_t node_id)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(w5200_network_config_t));

    // IP Address: 192.168.10.(31 + node_id)
    config->ip_address.bytes[0] = 192;
    config->ip_address.bytes[1] = 168;
    config->ip_address.bytes[2] = 10;
    config->ip_address.bytes[3] = NETCFG_SPI3_BASE_IP_OCTET + node_id;

    // Gateway: 192.168.10.1
    config->gateway.bytes[0] = 192;
    config->gateway.bytes[1] = 168;
    config->gateway.bytes[2] = 10;
    config->gateway.bytes[3] = 1;

    // Subnet Mask: 255.255.255.0
    config->subnet_mask.bytes[0] = 255;
    config->subnet_mask.bytes[1] = 255;
    config->subnet_mask.bytes[2] = 255;
    config->subnet_mask.bytes[3] = 0;

    // MAC Address: Legacy pattern
    // MAC[3] = 0xB9 + node_id
    config->mac_address.bytes[0] = 0x34;
    config->mac_address.bytes[1] = 0x24;
    config->mac_address.bytes[2] = 0xE8;
    config->mac_address.bytes[3] = 0xB9 + node_id;
    config->mac_address.bytes[4] = 0x8B;
    config->mac_address.bytes[5] = 0x30;

    // Retry configuration
    config->retry_time = NETCFG_DEFAULT_RETRY_TIME;
    config->retry_count = NETCFG_DEFAULT_RETRY_COUNT;
}

/**
 * @brief Configure SPI2 network with legacy-compatible settings
 */
bool NetCfg_ConfigureSPI2(w5200_instance_t* w5200, uint8_t node_id)
{
    if (w5200 == NULL || node_id > 15) {
        return false;
    }

    w5200_network_config_t config;
    NetCfg_PopulateSPI2Config(&config, node_id);
    return W5200_ConfigureNetwork(w5200, &config);
}

/**
 * @brief Configure SPI3 network with legacy-compatible settings
 */
bool NetCfg_ConfigureSPI3(w5200_instance_t* w5200, uint8_t node_id)
{
    if (w5200 == NULL || node_id > 15) {
        return false;
    }

    w5200_network_config_t config;
    NetCfg_PopulateSPI3Config(&config, node_id);
    return W5200_ConfigureNetwork(w5200, &config);
}

/**
 * @brief Configure a socket as TCP server (legacy pattern)
 */
bool NetCfg_ConfigureTCPServer(w5200_instance_t* w5200, uint8_t socket_num, uint16_t port)
{
    if (w5200 == NULL || socket_num >= W5200_ACTIVE_SOCKETS) {
        return false;
    }

    w5200_socket_config_t socket_config;
    memset(&socket_config, 0, sizeof(w5200_socket_config_t));

    socket_config.enabled = true;
    socket_config.protocol = W5200_PROTO_TCP;
    socket_config.mode = W5200_MODE_SERVER;
    socket_config.source_port = port;

    // Destination IP/port not used for server mode
    socket_config.dest_ip.bytes[0] = 0;
    socket_config.dest_ip.bytes[1] = 0;
    socket_config.dest_ip.bytes[2] = 0;
    socket_config.dest_ip.bytes[3] = 0;
    socket_config.dest_port = 0;

    return W5200_ConfigureSocket(w5200, socket_num, &socket_config);
}
