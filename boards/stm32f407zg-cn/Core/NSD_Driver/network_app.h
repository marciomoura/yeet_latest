/**
 ******************************************************************************
 * @file    network_app.h
 * @brief   Network application layer for dual-channel communication
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * Application Layer Architecture:
 * - Socket 0: UDP for high-speed data transmission
 * - Socket 1: TCP for command/control and parametrization
 *
 * This module provides application-specific network operations built on top
 * of the W5200 hardware driver. It manages two independent communication
 * channels for different purposes.
 *
 ******************************************************************************
 */

#ifndef __NETWORK_APP_H
#define __NETWORK_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "w5200_driver.h"

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Socket assignments */
#define NETAPP_SOCKET_UDP_DATA    0 /**< UDP socket for data transmission */
#define NETAPP_SOCKET_TCP_CONTROL 1 /**< TCP socket for control/parametrization */

/** @brief Maximum packet sizes */
#define NETAPP_MAX_UDP_PACKET 1472 /**< Max UDP payload (Ethernet MTU - headers) */
#define NETAPP_MAX_TCP_PACKET 1460 /**< Max TCP payload (MSS) */
#define NETAPP_MAX_CMD_LENGTH 256  /**< Max command/parameter length */

/** @brief Command protocol constants */
#define NETAPP_CMD_HEADER_SIZE 4    /**< Command header size (bytes) */
#define NETAPP_CMD_START_BYTE  0xAA /**< Command start marker */
#define NETAPP_CMD_END_BYTE    0x55 /**< Command end marker */

/* ============================================================================
 * Command Types
 * ============================================================================ */

/**
 * @brief Command type enumeration for TCP control channel
 */
typedef enum {
    NETAPP_CMD_NONE = 0,         /**< No command */
    NETAPP_CMD_PING = 1,         /**< Ping request */
    NETAPP_CMD_GET_STATUS = 2,   /**< Get system status */
    NETAPP_CMD_SET_PARAM = 3,    /**< Set parameter */
    NETAPP_CMD_GET_PARAM = 4,    /**< Get parameter */
    NETAPP_CMD_START_STREAM = 5, /**< Start UDP data streaming */
    NETAPP_CMD_STOP_STREAM = 6,  /**< Stop UDP data streaming */
    NETAPP_CMD_RESET = 7,        /**< Reset command */
    NETAPP_CMD_ERROR = 0xFF      /**< Error response */
} netapp_cmd_type_t;

/**
 * @brief Command status codes
 */
typedef enum {
    NETAPP_STATUS_OK = 0,            /**< Success */
    NETAPP_STATUS_ERROR = 1,         /**< General error */
    NETAPP_STATUS_INVALID_CMD = 2,   /**< Invalid command */
    NETAPP_STATUS_INVALID_PARAM = 3, /**< Invalid parameter */
    NETAPP_STATUS_BUSY = 4,          /**< System busy */
    NETAPP_STATUS_TIMEOUT = 5        /**< Operation timeout */
} netapp_status_t;

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Command packet structure for TCP control channel
 */
typedef struct {
    uint8_t start_byte;                  /**< Start marker (0xAA) */
    netapp_cmd_type_t cmd_type;          /**< Command type */
    uint16_t data_length;                /**< Payload length */
    uint8_t data[NETAPP_MAX_CMD_LENGTH]; /**< Payload data */
    uint8_t end_byte;                    /**< End marker (0x55) */
    uint8_t checksum;                    /**< Simple XOR checksum */
} netapp_cmd_packet_t;

/**
 * @brief UDP data packet header
 */
typedef struct {
    uint32_t sequence_number; /**< Packet sequence number */
    uint32_t timestamp;       /**< Timestamp in milliseconds */
    uint16_t data_length;     /**< Payload length */
    uint16_t reserved;        /**< Reserved for future use */
} netapp_udp_header_t;

/**
 * @brief UDP transmission statistics
 */
typedef struct {
    uint32_t packets_sent;    /**< Total packets sent */
    uint32_t bytes_sent;      /**< Total bytes sent */
    uint32_t send_errors;     /**< Send error count */
    uint32_t sequence_number; /**< Current sequence number */
} netapp_udp_stats_t;

/**
 * @brief TCP control statistics
 */
typedef struct {
    uint32_t commands_received; /**< Total commands received */
    uint32_t commands_sent;     /**< Total responses sent */
    uint32_t parse_errors;      /**< Parse error count */
    uint32_t checksum_errors;   /**< Checksum error count */
} netapp_tcp_stats_t;

/**
 * @brief Application configuration structure
 */
typedef struct {
    w5200_ip_addr_t local_ip;     /**< Local IP address */
    w5200_ip_addr_t subnet_mask;  /**< Subnet mask */
    w5200_ip_addr_t gateway;      /**< Gateway address */
    w5200_mac_addr_t mac_address; /**< MAC address */

    uint16_t udp_local_port;       /**< UDP local port */
    uint16_t udp_remote_port;      /**< UDP remote port */
    w5200_ip_addr_t udp_remote_ip; /**< UDP remote IP */

    uint16_t tcp_local_port;       /**< TCP local port */
    bool tcp_server_mode;          /**< TCP server (true) or client (false) */
    uint16_t tcp_remote_port;      /**< TCP remote port (client mode) */
    w5200_ip_addr_t tcp_remote_ip; /**< TCP remote IP (client mode) */
} netapp_config_t;

/**
 * @brief Command callback function type
 * @param cmd_type Command type received
 * @param data Pointer to command data
 * @param length Length of command data
 * @param response_data Pointer to response data buffer (to be filled by callback)
 * @param response_length Pointer to response length (to be set by callback)
 * @return Status code
 */
typedef netapp_status_t (*netapp_cmd_callback_t)(netapp_cmd_type_t cmd_type,
    const uint8_t* data,
    uint16_t length,
    uint8_t* response_data,
    uint16_t* response_length);

/**
 * @brief Network application instance structure
 */
typedef struct {
    w5200_instance_t* w5200; /**< Pointer to W5200 driver instance */
    netapp_config_t config;  /**< Application configuration */

    bool udp_streaming_enabled;   /**< UDP streaming enable flag */
    netapp_udp_stats_t udp_stats; /**< UDP statistics */
    netapp_tcp_stats_t tcp_stats; /**< TCP statistics */

    netapp_cmd_callback_t cmd_callback; /**< Command handler callback */

    uint8_t rx_buffer[NETAPP_MAX_TCP_PACKET]; /**< RX buffer for TCP commands */
    uint16_t rx_buffer_length;                /**< Current RX buffer length */

    bool initialized; /**< Initialization complete flag */
} netapp_instance_t;

/* ============================================================================
 * Function Prototypes - Initialization
 * ============================================================================ */

/**
 * @brief Initialize network application layer
 * @param netapp Pointer to network app instance structure
 * @param w5200 Pointer to initialized W5200 driver instance
 * @param config Pointer to application configuration
 * @retval true if initialization successful
 * @retval false if initialization failed
 */
bool NetApp_Init(netapp_instance_t* netapp, w5200_instance_t* w5200, const netapp_config_t* config);

/**
 * @brief Register command handler callback
 * @param netapp Pointer to network app instance
 * @param callback Pointer to callback function
 * @note Callback will be invoked when TCP commands are received
 */
void NetApp_RegisterCommandCallback(netapp_instance_t* netapp, netapp_cmd_callback_t callback);

/**
 * @brief Reset network application
 * @param netapp Pointer to network app instance
 */
void NetApp_Reset(netapp_instance_t* netapp);

/* ============================================================================
 * Function Prototypes - UDP Data Channel
 * ============================================================================ */

/**
 * @brief Enable UDP data streaming
 * @param netapp Pointer to network app instance
 * @retval true if streaming enabled successfully
 * @retval false if enable failed
 */
bool NetApp_EnableUDPStream(netapp_instance_t* netapp);

/**
 * @brief Disable UDP data streaming
 * @param netapp Pointer to network app instance
 */
void NetApp_DisableUDPStream(netapp_instance_t* netapp);

/**
 * @brief Send data via UDP channel
 * @param netapp Pointer to network app instance
 * @param data Pointer to data buffer
 * @param length Length of data to send
 * @retval true if data sent successfully
 * @retval false if send failed
 * @note Automatically adds UDP header with sequence number and timestamp
 */
bool NetApp_SendUDPData(netapp_instance_t* netapp, const uint8_t* data, uint16_t length);

/**
 * @brief Get UDP transmission statistics
 * @param netapp Pointer to network app instance
 * @return Pointer to UDP statistics structure
 */
const netapp_udp_stats_t* NetApp_GetUDPStats(const netapp_instance_t* netapp);

/* ============================================================================
 * Function Prototypes - TCP Control Channel
 * ============================================================================ */

/**
 * @brief Process received TCP commands
 * @param netapp Pointer to network app instance
 * @note Call this function periodically to handle incoming commands
 */
void NetApp_ProcessTCPCommands(netapp_instance_t* netapp);

/**
 * @brief Send command via TCP control channel
 * @param netapp Pointer to network app instance
 * @param cmd_type Command type
 * @param data Pointer to command data (can be NULL if no data)
 * @param length Length of command data
 * @retval true if command sent successfully
 * @retval false if send failed
 */
bool NetApp_SendTCPCommand(netapp_instance_t* netapp, netapp_cmd_type_t cmd_type, const uint8_t* data, uint16_t length);

/**
 * @brief Send response to TCP command
 * @param netapp Pointer to network app instance
 * @param status Status code
 * @param data Pointer to response data (can be NULL if no data)
 * @param length Length of response data
 * @retval true if response sent successfully
 * @retval false if send failed
 */
bool NetApp_SendTCPResponse(netapp_instance_t* netapp, netapp_status_t status, const uint8_t* data, uint16_t length);

/**
 * @brief Get TCP control statistics
 * @param netapp Pointer to network app instance
 * @return Pointer to TCP statistics structure
 */
const netapp_tcp_stats_t* NetApp_GetTCPStats(const netapp_instance_t* netapp);

/* ============================================================================
 * Function Prototypes - Main Update Loop
 * ============================================================================ */

/**
 * @brief Update network application state
 * @param netapp Pointer to network app instance
 * @note Call this function periodically in main loop (e.g., every 1ms)
 *       This handles connection management, data reception, and state updates
 */
void NetApp_Update(netapp_instance_t* netapp);

/* ============================================================================
 * Function Prototypes - Utilities
 * ============================================================================ */

/**
 * @brief Get current system timestamp in milliseconds
 * @return Timestamp value
 * @note This function should be implemented by the application
 */
extern uint32_t NetApp_GetTimestamp(void);

/**
 * @brief Check if UDP streaming is enabled
 * @param netapp Pointer to network app instance
 * @return true if streaming enabled, false otherwise
 */
bool NetApp_IsUDPStreamEnabled(const netapp_instance_t* netapp);

/**
 * @brief Check if TCP control channel is connected
 * @param netapp Pointer to network app instance
 * @return true if connected, false otherwise
 */
bool NetApp_IsTCPConnected(const netapp_instance_t* netapp);

/**
 * @brief Parse IP address string to structure
 * @param ip_str IP address string (e.g., "192.168.1.100")
 * @param ip Pointer to IP address structure to fill
 * @retval true if parsing successful
 * @retval false if parsing failed
 */
bool NetApp_ParseIPAddress(const char* ip_str, w5200_ip_addr_t* ip);

/**
 * @brief Parse MAC address string to structure
 * @param mac_str MAC address string (e.g., "00:11:22:33:44:55")
 * @param mac Pointer to MAC address structure to fill
 * @retval true if parsing successful
 * @retval false if parsing failed
 */
bool NetApp_ParseMACAddress(const char* mac_str, w5200_mac_addr_t* mac);

#ifdef __cplusplus
}
#endif

#endif /* __NETWORK_APP_H */
