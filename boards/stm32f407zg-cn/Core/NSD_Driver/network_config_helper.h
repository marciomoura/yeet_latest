/**
 ******************************************************************************
 * @file    network_config_helper.h
 * @brief   Helper functions for W5200 network configuration matching legacy behavior
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * This module provides simplified network configuration matching the legacy
 * implementation's IP addressing scheme based on DIP switch node ID.
 *
 * IP Address Scheme:
 * - SPI2 (W5200 #1): 192.168.2.(100 + uchNodeNum)
 * - SPI3 (W5200 #2): 192.168.10.(31 + uchNodeNum)
 *
 ******************************************************************************
 */

#ifndef __NETWORK_CONFIG_HELPER_H
#define __NETWORK_CONFIG_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "w5200_driver.h"

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Default network configurations */
#define NETCFG_SPI2_BASE_IP_OCTET  100  /**< SPI2 base IP: 192.168.2.(100+node) */
#define NETCFG_SPI3_BASE_IP_OCTET  31   /**< SPI3 base IP: 192.168.10.(31+node) */
#define NETCFG_DEFAULT_TCP_PORT    7000 /**< Default TCP server port */
#define NETCFG_DEFAULT_RETRY_TIME  4000 /**< 400ms retry time (units of 100us) */
#define NETCFG_DEFAULT_RETRY_COUNT 8    /**< Number of retries */

/* ============================================================================
 * Public Function Prototypes
 * ============================================================================ */

/**
 * @brief Configure SPI2 network with legacy-compatible settings
 * @param w5200 Pointer to W5200 instance
 * @param node_id Node ID from DIP switches (0-15)
 * @retval true if successful, false otherwise
 *
 * @note Configures:
 *       - IP: 192.168.2.(100 + node_id)
 *       - Gateway: 192.168.2.1
 *       - Subnet: 255.255.255.0
 *       - MAC: Based on node_id
 */
bool NetCfg_ConfigureSPI2(w5200_instance_t* w5200, uint8_t node_id);

/**
 * @brief Configure SPI3 network with legacy-compatible settings
 * @param w5200 Pointer to W5200 instance
 * @param node_id Node ID from DIP switches (0-15)
 * @retval true if successful, false otherwise
 *
 * @note Configures:
 *       - IP: 192.168.10.(31 + node_id)
 *       - Gateway: 192.168.10.1
 *       - Subnet: 255.255.255.0
 *       - MAC: Based on node_id
 */
bool NetCfg_ConfigureSPI3(w5200_instance_t* w5200, uint8_t node_id);

/**
 * @brief Configure a socket as TCP server (legacy pattern)
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @param port TCP port number (typically 7000)
 * @retval true if successful, false otherwise
 */
bool NetCfg_ConfigureTCPServer(w5200_instance_t* w5200, uint8_t socket_num, uint16_t port);

/**
 * @brief Helper to populate network config structure for SPI2
 * @param config Pointer to config structure to fill
 * @param node_id Node ID from DIP switches (0-15)
 */
void NetCfg_PopulateSPI2Config(w5200_network_config_t* config, uint8_t node_id);

/**
 * @brief Helper to populate network config structure for SPI3
 * @param config Pointer to config structure to fill
 * @param node_id Node ID from DIP switches (0-15)
 */
void NetCfg_PopulateSPI3Config(w5200_network_config_t* config, uint8_t node_id);

#ifdef __cplusplus
}
#endif

#endif /* __NETWORK_CONFIG_HELPER_H */
