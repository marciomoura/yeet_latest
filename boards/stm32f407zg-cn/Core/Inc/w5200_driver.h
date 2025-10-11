/**
 ******************************************************************************
 * @file    w5200_driver.h
 * @brief   WIZnet W5200 Hardwired TCP/IP Ethernet Controller Driver (HAL)
 * @note    Ported from legacy SPL to STM32 HAL for STM32F407ZG-CN board
 ******************************************************************************
 * @attention
 *
 * Hardware Configuration:
 * - W5200 #1 (SPI2): 192.168.2.100+NodeID, Port 7000, TCP Server, No DMA
 * - W5200 #2 (SPI3): 192.168.10.31+NodeID, Port 7000, TCP Server, 8K DMA
 *
 * Features:
 * - Dual W5200 chip support (SPI2 + SPI3)
 * - TCP Server/Client modes
 * - UDP mode for telemetry
 * - DMA support for SPI3 (8K buffers)
 * - TLV protocol support for debugging and parameter configuration
 *
 ******************************************************************************
 */

#ifndef W5200_DRIVER_H
#define W5200_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "w5200_reg.h"


/* ==================== Configuration Constants ==================== */

#define W5200_MAX_SOCKETS           2     ///< Maximum sockets per chip (using 2 of 8)
#define W5200_MAX_CHIPS             2     ///< Number of W5200 chips (SPI2, SPI3)
#define W5200_RX_BUFFER_SIZE        8192  ///< RX buffer size per chip
#define W5200_SPI_DELAY_US          2     ///< Delay between SPI transactions (µs)
#define W5200_RESET_DELAY_MS        500   ///< Reset pulse duration (ms)
#define W5200_KEEPALIVE_INTERVAL_MS 1200  ///< Keep-alive packet interval (ms)

/* ==================== Chip Identifiers ==================== */

typedef enum {
    W5200_CHIP_SPI2 = 0,  ///< W5200 on SPI2 (192.168.2.x)
    W5200_CHIP_SPI3 = 1   ///< W5200 on SPI3 (192.168.10.x)
} W5200_ChipID_t;

/* ==================== Network Configuration Structure ==================== */

typedef struct {
    uint8_t ip[4];       ///< IP address
    uint8_t gateway[4];  ///< Gateway address
    uint8_t subnet[4];   ///< Subnet mask
    uint8_t mac[6];      ///< MAC address
} W5200_NetworkConfig_t;

/* ==================== Socket Configuration Structure ==================== */

typedef struct {
    bool enabled;         ///< Socket enabled flag
    uint8_t mode;         ///< W5200_MODE_SERVER or W5200_MODE_CLIENT
    uint8_t protocol;     ///< W5200_PROTO_TCP or W5200_PROTO_UDP
    uint16_t local_port;  ///< Local port number
    uint8_t dest_ip[4];   ///< Destination IP (client mode)
    uint16_t dest_port;   ///< Destination port (client mode)
} W5200_SocketConfig_t;

/* ==================== Socket State Structure ==================== */

typedef struct {
    uint8_t state;       ///< Socket state flags (S_INIT, S_CONN, S_OK)
    uint8_t data_flags;  ///< Data flags (DATA_RECEIVE, DATA_TX_OK)
    uint16_t tx_addr;    ///< TX buffer address
    uint16_t rx_addr;    ///< RX buffer address
    uint16_t tx_offset;  ///< TX buffer write offset
    uint16_t rx_offset;  ///< RX buffer read offset
} W5200_SocketState_t;

/* ====================Driver Instance Structure ==================== */

typedef struct {
    /* Hardware handles */
    SPI_HandleTypeDef *hspi;  ///< SPI handle (hspi2 or hspi3)
    GPIO_TypeDef *cs_port;    ///< Chip Select GPIO port
    uint16_t cs_pin;          ///< Chip Select GPIO pin
    GPIO_TypeDef *rst_port;   ///< Reset GPIO port
    uint16_t rst_pin;         ///< Reset GPIO pin

    /* DMA configuration */
    bool use_dma;                ///< DMA enabled flag
    uint16_t dma_size;           ///< DMA transfer size (0 = no DMA)
    DMA_HandleTypeDef *hdma_tx;  ///< DMA TX handle (optional)
    DMA_HandleTypeDef *hdma_rx;  ///< DMA RX handle (optional)

    /* Network configuration */
    W5200_NetworkConfig_t network;  ///< Network settings

    /* Socket configuration */
    W5200_SocketConfig_t sockets[W5200_MAX_SOCKETS];      ///< Socket configs
    W5200_SocketState_t socket_state[W5200_MAX_SOCKETS];  ///< Socket states

    /* Buffers */
    uint8_t rx_buffer[W5200_RX_BUFFER_SIZE];  ///< RX data buffer
    uint8_t tx_buffer[W5200_RX_BUFFER_SIZE];  ///< TX data buffer (for DMA)
    uint16_t rx_size;                         ///< Received data size
    uint16_t tx_size[W5200_MAX_SOCKETS];      ///< TX data size per socket

    /* Status flags */
    uint8_t tx_flags[W5200_MAX_SOCKETS];  ///< TX ready flags
    uint8_t rx_flags[W5200_MAX_SOCKETS];  ///< RX ready flags
    uint8_t net_state;                    ///< Network state (IDLE, CHECKING, SENDING, RECEIVING)
    uint8_t sending_socket;               ///< Currently sending socket ID
    uint8_t receiving_socket;             ///< Currently receiving socket ID

    /* Timing */
    uint16_t keepalive_counter;   ///< Keep-alive timer counter
    uint16_t rx_timeout_counter;  ///< RX timeout counter
    uint16_t tx_timeout_counter;  ///< TX timeout counter

    /* Chip ID */
    W5200_ChipID_t chip_id;  ///< Chip identifier (SPI2 or SPI3)
    uint8_t node_id;         ///< Node ID for IP address (0-255)

} W5200_Handle_t;

/* ==================== Initialization Functions ==================== */

/**
 * @brief  Initialize W5200 chip with network and socket configuration
 * @param  handle: Pointer to W5200 handle structure
 * @param  hspi: Pointer to SPI HAL handle (hspi2 or hspi3)
 * @param  chip_id: Chip identifier (W5200_CHIP_SPI2 or W5200_CHIP_SPI3)
 * @param  node_id: Node ID for IP addressing (0-255)
 * @param  use_dma: Enable DMA for SPI transfers (recommended for SPI3 only)
 * @param  dma_size: DMA buffer size (W5200_DMA_SIZE_8K recommended, 0 = no DMA)
 * @retval HAL_StatusTypeDef: HAL_OK on success
 */
HAL_StatusTypeDef W5200_Init(W5200_Handle_t *handle,
    SPI_HandleTypeDef *hspi,
    W5200_ChipID_t chip_id,
    uint8_t node_id,
    bool use_dma,
    uint16_t dma_size);

/**
 * @brief  Configure network settings (IP, gateway, subnet, MAC)
 * @param  handle: Pointer to W5200 handle
 * @param  config: Pointer to network configuration structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_ConfigureNetwork(W5200_Handle_t *handle, const W5200_NetworkConfig_t *config);

/**
 * @brief  Configure socket settings
 * @param  handle: Pointer to W5200 handle
 * @param  socket: Socket number (0-1)
 * @param  config: Pointer to socket configuration structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_ConfigureSocket(W5200_Handle_t *handle, uint8_t socket, const W5200_SocketConfig_t *config);

/* ==================== Low-Level SPI Functions ==================== */

/**
 * @brief  Write single byte to W5200 register
 * @param  handle: Pointer to W5200 handle
 * @param  addr: Register address
 * @param  data: Data byte to write
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_WriteByte(W5200_Handle_t *handle, uint16_t addr, uint8_t data);

/**
 * @brief  Read single byte from W5200 register
 * @param  handle: Pointer to W5200 handle
 * @param  addr: Register address
 * @param  data: Pointer to store read data
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_ReadByte(W5200_Handle_t *handle, uint16_t addr, uint8_t *data);

/**
 * @brief  Write multiple bytes to W5200
 * @param  handle: Pointer to W5200 handle
 * @param  addr: Starting register address
 * @param  data: Pointer to data buffer
 * @param  size: Number of bytes to write
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_Write(W5200_Handle_t *handle, uint16_t addr, const uint8_t *data, uint16_t size);

/**
 * @brief  Read multiple bytes from W5200
 * @param  handle: Pointer to W5200 handle
 * @param  addr: Starting register address
 * @param  data: Pointer to data buffer
 * @param  size: Number of bytes to read
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_Read(W5200_Handle_t *handle, uint16_t addr, uint8_t *data, uint16_t size);

/* ==================== Socket Management Functions ==================== */

/**
 * @brief  Open socket in specified mode (TCP Server/Client or UDP)
 * @param  handle: Pointer to W5200 handle
 * @param  socket: Socket number (0-1)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_SocketOpen(W5200_Handle_t *handle, uint8_t socket);

/**
 * @brief  Close socket
 * @param  handle: Pointer to W5200 handle
 * @param  socket: Socket number (0-1)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_SocketClose(W5200_Handle_t *handle, uint8_t socket);

/**
 * @brief  Get socket status
 * @param  handle: Pointer to W5200 handle
 * @param  socket: Socket number (0-1)
 * @param  status: Pointer to store socket status byte
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_SocketGetStatus(W5200_Handle_t *handle, uint8_t socket, uint8_t *status);

/**
 * @brief  Check if socket is connected (TCP) or ready (UDP)
 * @param  handle: Pointer to W5200 handle
 * @param  socket: Socket number (0-1)
 * @retval bool: true if socket is ready for communication
 */
bool W5200_SocketIsReady(W5200_Handle_t *handle, uint8_t socket);

/* ==================== Data Transfer Functions ==================== */

/**
 * @brief  Send data through socket
 * @param  handle: Pointer to W5200 handle
 * @param  socket: Socket number (0-1)
 * @param  data: Pointer to data buffer
 * @param  size: Number of bytes to send
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_SocketSend(W5200_Handle_t *handle, uint8_t socket, const uint8_t *data, uint16_t size);

/**
 * @brief  Receive data from socket
 * @param  handle: Pointer to W5200 handle
 * @param  socket: Socket number (0-1)
 * @param  data: Pointer to data buffer
 * @param  max_size: Maximum bytes to receive
 * @param  received_size: Pointer to store actual received size
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_SocketReceive(
    W5200_Handle_t *handle, uint8_t socket, uint8_t *data, uint16_t max_size, uint16_t *received_size);

/**
 * @brief  Check if data is available to receive
 * @param  handle: Pointer to W5200 handle
 * @param  socket: Socket number (0-1)
 * @param  available_size: Pointer to store available data size
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_SocketDataAvailable(W5200_Handle_t *handle, uint8_t socket, uint16_t *available_size);

/* ==================== Cyclic Update Functions ==================== */

/**
 * @brief  Process network events and interrupts
 * @param  handle: Pointer to W5200 handle
 * @note   Call this periodically (e.g., every 1ms) to handle socket events
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_ProcessInterrupts(W5200_Handle_t *handle);

/**
 * @brief  Manage socket states and connections
 * @param  handle: Pointer to W5200 handle
 * @note   Call this periodically to maintain socket connections
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_ManageSockets(W5200_Handle_t *handle);

/**
 * @brief  Send keep-alive packets for TCP connections
 * @param  handle: Pointer to W5200 handle
 * @note   Call this periodically (e.g., every 1ms) - handles internal timing
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_SendKeepAlive(W5200_Handle_t *handle);

/**
 * @brief  Main cyclic task - combines all periodic operations
 * @param  handle: Pointer to W5200 handle
 * @note   Call this in fast task (1ms recommended) for optimal performance
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_Cycle(W5200_Handle_t *handle);

/* ==================== Utility Functions ==================== */

/**
 * @brief  Hardware reset W5200 chip
 * @param  handle: Pointer to W5200 handle
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef W5200_HardwareReset(W5200_Handle_t *handle);

/**
 * @brief  Get chip identification string
 * @param  chip_id: Chip ID (W5200_CHIP_SPI2 or W5200_CHIP_SPI3)
 * @retval const char*: Chip name string
 */
const char *W5200_GetChipName(W5200_ChipID_t chip_id);

/**
 * @brief  Format MAC address for chip based on node ID
 * @param  chip_id: Chip ID
 * @param  node_id: Node ID (0-255)
 * @param  mac: Pointer to 6-byte MAC address buffer
 * @retval None
 */
void W5200_GenerateMAC(W5200_ChipID_t chip_id, uint8_t node_id, uint8_t *mac);

/**
 * @brief  Format IP address for chip based on node ID
 * @param  chip_id: Chip ID
 * @param  node_id: Node ID (0-255)
 * @param  ip: Pointer to 4-byte IP address buffer
 * @retval None
 */
void W5200_GenerateIP(W5200_ChipID_t chip_id, uint8_t node_id, uint8_t *ip);

#ifdef __cplusplus
}
#endif

#endif /* W5200_DRIVER_H */
