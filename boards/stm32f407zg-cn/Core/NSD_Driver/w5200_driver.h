/**
 ******************************************************************************
 * @file    w5200_driver.h
 * @brief   W5200 Ethernet controller hardware driver for STM32F407
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * Hardware Configuration:
 * - SPI2: SCK=PB13, MOSI=PB15, MISO=PB14, NSS=PB12, CS=PA11, Reset=PA12
 * - SPI3: SCK=PC10, MOSI=PC12, MISO=PC11, NSS=PA15, CS=PB3, Reset=PB4
 * - Each SPI interface supports independent W5200 chip with DMA
 * - 8KB TX/RX buffers per socket, up to 2 sockets per chip
 * - Support for TCP and UDP protocols
 *
 * This module provides low-level hardware abstraction for the W5200 Ethernet
 * controller. It handles SPI communication, DMA transfers, and register access.
 *
 ******************************************************************************
 */

#ifndef __W5200_DRIVER_H
#define __W5200_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx.h>

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Maximum number of sockets supported per W5200 chip */
#define W5200_MAX_SOCKETS 8

/** @brief Number of sockets actually used (2 for UDP + TCP) */
#define W5200_ACTIVE_SOCKETS 2

/** @brief Socket buffer size in bytes (8KB per socket) */
#define W5200_SOCKET_BUF_SIZE 8192

/** @brief DMA transfer size options */
#define W5200_DMA_SIZE_128 128
#define W5200_DMA_SIZE_256 256
#define W5200_DMA_SIZE_512 512
#define W5200_DMA_SIZE_1K  1024
#define W5200_DMA_SIZE_2K  2048
#define W5200_DMA_SIZE_4K  4096
#define W5200_DMA_SIZE_8K  8192

/** @brief SPI timeout for register access */
#define W5200_SPI_TIMEOUT 1200

/* ============================================================================
 * W5200 Register Definitions
 * ============================================================================ */

/** @brief Common register base addresses */
#define W5200_COMMON_BASE     0x0000
#define W5200_SOCKET_BASE     0x4000
#define W5200_SOCKET_REG_SIZE 0x0100
#define W5200_TX_MEM_BASE     0x8000
#define W5200_RX_MEM_BASE     0xC000

/** @brief Common registers */
#define W5200_GAR0  (W5200_COMMON_BASE + 0x0001) /**< Gateway address */
#define W5200_SUBR0 (W5200_COMMON_BASE + 0x0005) /**< Subnet mask */
#define W5200_SHAR0 (W5200_COMMON_BASE + 0x0009) /**< MAC address */
#define W5200_SIPR0 (W5200_COMMON_BASE + 0x000F) /**< Source IP */
#define W5200_RTR   (W5200_COMMON_BASE + 0x0017) /**< Retry time */
#define W5200_RCR   (W5200_COMMON_BASE + 0x0019) /**< Retry count */
#define W5200_IR2   (W5200_COMMON_BASE + 0x0034) /**< Interrupt register */

/** @brief Socket register offsets (add to socket base) */
#define W5200_Sn_MR(s)         (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0000) /**< Mode */
#define W5200_Sn_CR(s)         (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0001) /**< Command */
#define W5200_Sn_IR(s)         (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0002) /**< Interrupt */
#define W5200_Sn_SR(s)         (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0003) /**< Status */
#define W5200_Sn_PORT0(s)      (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0004) /**< Source port */
#define W5200_Sn_DIPR0(s)      (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x000C) /**< Dest IP */
#define W5200_Sn_DPORT0(s)     (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0010) /**< Dest port */
#define W5200_Sn_MSSR0(s)      (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0012) /**< Max segment size */
#define W5200_Sn_RXMEM_SIZE(s) (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x001E) /**< RX memory size */
#define W5200_Sn_TXMEM_SIZE(s) (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x001F) /**< TX memory size */
#define W5200_Sn_TX_FSR0(s)    (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0020) /**< TX free size */
#define W5200_Sn_TX_WR0(s)     (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0024) /**< TX write ptr */
#define W5200_Sn_RX_RSR0(s)    (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0026) /**< RX received size */
#define W5200_Sn_RX_RD0(s)     (W5200_SOCKET_BASE + (s) * W5200_SOCKET_REG_SIZE + 0x0028) /**< RX read ptr */

/* ============================================================================
 * W5200 Register Values
 * ============================================================================ */

/** @brief Socket mode register values */
#define W5200_Sn_MR_CLOSED 0x00 /**< Closed mode */
#define W5200_Sn_MR_TCP    0x01 /**< TCP mode */
#define W5200_Sn_MR_UDP    0x02 /**< UDP mode */
#define W5200_Sn_MR_IPRAW  0x03 /**< IP raw mode */
#define W5200_Sn_MR_MACRAW 0x04 /**< MAC raw mode */

/** @brief Socket command register values */
#define W5200_Sn_CR_OPEN      0x01 /**< Open socket */
#define W5200_Sn_CR_LISTEN    0x02 /**< Listen (TCP server) */
#define W5200_Sn_CR_CONNECT   0x04 /**< Connect (TCP client) */
#define W5200_Sn_CR_DISCON    0x08 /**< Disconnect */
#define W5200_Sn_CR_CLOSE     0x10 /**< Close socket */
#define W5200_Sn_CR_SEND      0x20 /**< Send data */
#define W5200_Sn_CR_SEND_KEEP 0x22 /**< Send with keep-alive */
#define W5200_Sn_CR_RECV      0x40 /**< Receive data */

/** @brief Socket status register values */
#define W5200_SOCK_CLOSED      0x00 /**< Socket closed */
#define W5200_SOCK_INIT        0x13 /**< Socket initialized */
#define W5200_SOCK_LISTEN      0x14 /**< Socket listening */
#define W5200_SOCK_ESTABLISHED 0x17 /**< TCP connection established */
#define W5200_SOCK_CLOSE_WAIT  0x1C /**< TCP close wait */
#define W5200_SOCK_UDP         0x22 /**< UDP socket open */

/** @brief Socket interrupt register values */
#define W5200_Sn_IR_CON     0x01 /**< Connection established */
#define W5200_Sn_IR_DISCON  0x02 /**< Disconnected */
#define W5200_Sn_IR_RECV    0x04 /**< Data received */
#define W5200_Sn_IR_TIMEOUT 0x08 /**< Timeout */
#define W5200_Sn_IR_SENDOK  0x10 /**< Send completed */

/* ============================================================================
 * Data Types
 * ============================================================================ */

/**
 * @brief Socket protocol type enumeration
 */
typedef enum {
    W5200_PROTO_TCP = 0, /**< TCP protocol */
    W5200_PROTO_UDP = 1  /**< UDP protocol */
} w5200_protocol_t;

/**
 * @brief Socket mode enumeration (client or server)
 */
typedef enum {
    W5200_MODE_CLIENT = 0, /**< Client mode */
    W5200_MODE_SERVER = 1  /**< Server mode */
} w5200_socket_mode_t;

/**
 * @brief Socket state enumeration
 */
typedef enum {
    W5200_STATE_IDLE = 0,     /**< Socket idle */
    W5200_STATE_CHECKING = 1, /**< Checking connection state */
    W5200_STATE_SENDING = 2,  /**< Sending data */
    W5200_STATE_RECEIVING = 3 /**< Receiving data */
} w5200_state_t;

/**
 * @brief IP address structure
 */
typedef struct {
    uint8_t bytes[4]; /**< IP address bytes */
} w5200_ip_addr_t;

/**
 * @brief MAC address structure
 */
typedef struct {
    uint8_t bytes[6]; /**< MAC address bytes */
} w5200_mac_addr_t;

/**
 * @brief Socket configuration structure
 */
typedef struct {
    bool enabled;              /**< Socket enabled flag */
    w5200_protocol_t protocol; /**< Protocol (TCP/UDP) */
    w5200_socket_mode_t mode;  /**< Client or server mode */
    uint16_t source_port;      /**< Source port number */
    uint16_t dest_port;        /**< Destination port number */
    w5200_ip_addr_t dest_ip;   /**< Destination IP address */
} w5200_socket_config_t;

/**
 * @brief Network configuration structure
 */
typedef struct {
    w5200_ip_addr_t ip_address;   /**< Device IP address */
    w5200_ip_addr_t subnet_mask;  /**< Subnet mask */
    w5200_ip_addr_t gateway;      /**< Gateway address */
    w5200_mac_addr_t mac_address; /**< MAC address */
    uint16_t retry_time;          /**< Retry time (400ms default) */
    uint8_t retry_count;          /**< Retry count */
} w5200_network_config_t;

/**
 * @brief Socket runtime state structure
 */
typedef struct {
    uint8_t status;          /**< Current socket status */
    w5200_state_t state;     /**< Driver state machine */
    bool tx_ready;           /**< TX ready flag */
    bool rx_ready;           /**< RX data available flag */
    uint16_t rx_size;        /**< Received data size */
    uint16_t tx_size;        /**< Data to transmit size */
    uint16_t tx_addr_offset; /**< TX buffer offset */
    uint16_t rx_addr_offset; /**< RX buffer offset */
} w5200_socket_state_t;

/**
 * @brief W5200 driver instance structure
 */
typedef struct {
    SPI_TypeDef* spi;         /**< SPI peripheral (SPI2 or SPI3) */
    GPIO_TypeDef* cs_port;    /**< Chip select GPIO port */
    uint16_t cs_pin;          /**< Chip select GPIO pin */
    GPIO_TypeDef* reset_port; /**< Reset GPIO port */
    uint16_t reset_pin;       /**< Reset GPIO pin */

    w5200_network_config_t network;                          /**< Network configuration */
    w5200_socket_config_t sockets[W5200_ACTIVE_SOCKETS];     /**< Socket configurations */
    w5200_socket_state_t socket_state[W5200_ACTIVE_SOCKETS]; /**< Socket runtime states */

    uint8_t* tx_buffer; /**< TX DMA buffer pointer */
    uint8_t* rx_buffer; /**< RX DMA buffer pointer */
    uint16_t dma_size;  /**< DMA transfer size */

    uint16_t check_delay; /**< Connection check delay counter */
    bool initialized;     /**< Initialization complete flag */
} w5200_instance_t;

/* ============================================================================
 * Function Prototypes - Initialization
 * ============================================================================ */

/**
 * @brief Initialize W5200 hardware driver
 * @param w5200 Pointer to W5200 instance structure
 * @param spi SPI peripheral to use (SPI2 or SPI3)
 * @param tx_buffer Pointer to TX buffer (must be at least dma_size + 4 bytes)
 * @param rx_buffer Pointer to RX buffer (must be at least dma_size + 4 bytes)
 * @param dma_size DMA transfer size (use W5200_DMA_SIZE_* constants)
 * @retval true if initialization successful
 * @retval false if initialization failed
 */
bool W5200_Init(w5200_instance_t* w5200, SPI_TypeDef* spi, uint8_t* tx_buffer, uint8_t* rx_buffer, uint16_t dma_size);

/**
 * @brief Configure network parameters
 * @param w5200 Pointer to W5200 instance
 * @param config Pointer to network configuration structure
 * @retval true if configuration successful
 * @retval false if configuration failed
 */
bool W5200_ConfigureNetwork(w5200_instance_t* w5200, const w5200_network_config_t* config);

/**
 * @brief Configure a socket
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @param config Pointer to socket configuration structure
 * @retval true if configuration successful
 * @retval false if configuration failed
 */
bool W5200_ConfigureSocket(w5200_instance_t* w5200, uint8_t socket_num, const w5200_socket_config_t* config);

/**
 * @brief Reset W5200 hardware
 * @param w5200 Pointer to W5200 instance
 */
void W5200_Reset(w5200_instance_t* w5200);

/* ============================================================================
 * Function Prototypes - Register Access
 * ============================================================================ */

/**
 * @brief Write single byte to W5200 register
 * @param w5200 Pointer to W5200 instance
 * @param addr Register address
 * @param data Data byte to write
 */
void W5200_WriteByte(w5200_instance_t* w5200, uint16_t addr, uint8_t data);

/**
 * @brief Read single byte from W5200 register
 * @param w5200 Pointer to W5200 instance
 * @param addr Register address
 * @return Data byte read
 */
uint8_t W5200_ReadByte(w5200_instance_t* w5200, uint16_t addr);

/**
 * @brief Write multiple bytes to W5200 registers
 * @param w5200 Pointer to W5200 instance
 * @param addr Starting register address
 * @param data Pointer to data buffer
 * @param length Number of bytes to write
 */
void W5200_Write(w5200_instance_t* w5200, uint16_t addr, const uint8_t* data, uint16_t length);

/**
 * @brief Read multiple bytes from W5200 registers
 * @param w5200 Pointer to W5200 instance
 * @param addr Starting register address
 * @param data Pointer to data buffer
 * @param length Number of bytes to read
 */
void W5200_Read(w5200_instance_t* w5200, uint16_t addr, uint8_t* data, uint16_t length);

/* ============================================================================
 * Function Prototypes - Socket Operations
 * ============================================================================ */

/**
 * @brief Open a socket
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @retval true if socket opened successfully
 * @retval false if socket open failed
 */
bool W5200_SocketOpen(w5200_instance_t* w5200, uint8_t socket_num);

/**
 * @brief Close a socket
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 */
void W5200_SocketClose(w5200_instance_t* w5200, uint8_t socket_num);

/**
 * @brief Listen on TCP socket (server mode)
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @retval true if listen command successful
 * @retval false if listen command failed
 */
bool W5200_SocketListen(w5200_instance_t* w5200, uint8_t socket_num);

/**
 * @brief Connect TCP socket (client mode)
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @retval true if connect command successful
 * @retval false if connect command failed
 */
bool W5200_SocketConnect(w5200_instance_t* w5200, uint8_t socket_num);

/**
 * @brief Disconnect TCP socket
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 */
void W5200_SocketDisconnect(w5200_instance_t* w5200, uint8_t socket_num);

/**
 * @brief Get socket status
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @return Socket status register value
 */
uint8_t W5200_GetSocketStatus(w5200_instance_t* w5200, uint8_t socket_num);

/**
 * @brief Get received data size
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @return Number of bytes available to read
 */
uint16_t W5200_GetRxSize(w5200_instance_t* w5200, uint8_t socket_num);

/**
 * @brief Get free TX buffer size
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @return Number of bytes available in TX buffer
 */
uint16_t W5200_GetTxFreeSize(w5200_instance_t* w5200, uint8_t socket_num);

/* ============================================================================
 * Function Prototypes - Data Transfer
 * ============================================================================ */

/**
 * @brief Send data through socket
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @retval true if send initiated successfully
 * @retval false if send failed
 */
bool W5200_Send(w5200_instance_t* w5200, uint8_t socket_num, const uint8_t* data, uint16_t length);

/**
 * @brief Receive data from socket
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @param data Pointer to data buffer
 * @param max_length Maximum number of bytes to receive
 * @return Number of bytes actually received
 */
uint16_t W5200_Receive(w5200_instance_t* w5200, uint8_t socket_num, uint8_t* data, uint16_t max_length);

/**
 * @brief Send data using DMA
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @retval true if DMA send initiated successfully
 * @retval false if DMA send failed
 */
bool W5200_SendDMA(w5200_instance_t* w5200, uint8_t socket_num, const uint8_t* data, uint16_t length);

/**
 * @brief Receive data using DMA
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @return true if data received successfully
 */
bool W5200_ReceiveDMA(w5200_instance_t* w5200, uint8_t socket_num);

/* ============================================================================
 * Function Prototypes - Interrupt Handling
 * ============================================================================ */

/**
 * @brief Process W5200 interrupts
 * @param w5200 Pointer to W5200 instance
 * @note Call this function periodically or from ISR
 */
void W5200_ProcessInterrupts(w5200_instance_t* w5200);

/**
 * @brief Check and update socket states
 * @param w5200 Pointer to W5200 instance
 * @note Call this function periodically in main loop
 */
void W5200_Update(w5200_instance_t* w5200);

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Convert IP address from bytes to string
 * @param ip Pointer to IP address structure
 * @param str Pointer to string buffer (must be at least 16 bytes)
 */
void W5200_IPToString(const w5200_ip_addr_t* ip, char* str);

/**
 * @brief Convert MAC address from bytes to string
 * @param mac Pointer to MAC address structure
 * @param str Pointer to string buffer (must be at least 18 bytes)
 */
void W5200_MACToString(const w5200_mac_addr_t* mac, char* str);

#ifdef __cplusplus
}
#endif

#endif /* __W5200_DRIVER_H */
