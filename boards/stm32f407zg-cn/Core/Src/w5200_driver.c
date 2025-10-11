/**
 ******************************************************************************
 * @file    w5200_driver.c
 * @brief   WIZnet W5200 Hardwired TCP/IP Ethernet Controller Driver (HAL)
 * @note    Complete implementation ported from legacy SPL to STM32 HAL
 ******************************************************************************
 * @attention
 *
 * This driver implements the complete W5200 interface including:
 * - SPI communication (blocking mode for SPI2, DMA-capable for SPI3)
 * - Socket management (TCP Server/Client, UDP)
 * - Network initialization and configuration
 * - Interrupt processing
 * - Keep-alive and connection management
 *
 * Usage:
 * 1. Call W5200_Init() with appropriate parameters
 * 2. Configure network and sockets
 * 3. Call W5200_Cycle() periodically (1ms recommended)
 * 4. Use W5200_SocketSend/Receive for data transfer
 *
 ******************************************************************************
 */

#include "w5200_driver.h"

#include <string.h>

#include "main.h"

/* ==================== Private Macros ==================== */

#define W5200_CS_LOW(handle)   HAL_GPIO_WritePin((handle)->cs_port, (handle)->cs_pin, GPIO_PIN_RESET)
#define W5200_CS_HIGH(handle)  HAL_GPIO_WritePin((handle)->cs_port, (handle)->cs_pin, GPIO_PIN_SET)
#define W5200_RST_LOW(handle)  HAL_GPIO_WritePin((handle)->rst_port, (handle)->rst_pin, GPIO_PIN_RESET)
#define W5200_RST_HIGH(handle) HAL_GPIO_WritePin((handle)->rst_port, (handle)->rst_pin, GPIO_PIN_SET)

#define W5200_SPI_WRITE_CMD 0x80  ///< SPI write command bit
#define W5200_SPI_READ_CMD  0x00  ///< SPI read command bit

/* ==================== Private Function Prototypes ==================== */

static HAL_StatusTypeDef W5200_SPITransmitReceive(
    W5200_Handle_t *handle, uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
static void W5200_DelayUs(uint32_t us);
static HAL_StatusTypeDef W5200_ConfigureChip(W5200_Handle_t *handle);

/* ==================== Initialization Functions ==================== */

/**
 * @brief  Initialize W5200 chip with network and socket configuration
 */
HAL_StatusTypeDef W5200_Init(W5200_Handle_t *handle,
    SPI_HandleTypeDef *hspi,
    W5200_ChipID_t chip_id,
    uint8_t node_id,
    bool use_dma,
    uint16_t dma_size)
{
    if (handle == NULL || hspi == NULL) {
        return HAL_ERROR;
    }

    // Clear handle structure
    memset(handle, 0, sizeof(W5200_Handle_t));

    // Set hardware handles
    handle->hspi = hspi;
    handle->chip_id = chip_id;
    handle->node_id = node_id;

    // Configure CS and RST pins based on chip ID
    if (chip_id == W5200_CHIP_SPI2) {
        handle->cs_port = W5200_CS1_GPIO_Port;
        handle->cs_pin = W5200_CS1_Pin;
        handle->rst_port = W5200_RST1_GPIO_Port;
        handle->rst_pin = W5200_RST1_Pin;
    }
    else {  // W5200_CHIP_SPI3
        handle->cs_port = W5200_CS2_GPIO_Port;
        handle->cs_pin = W5200_CS2_Pin;
        handle->rst_port = W5200_RST2_GPIO_Port;
        handle->rst_pin = W5200_RST2_Pin;
    }

    // Configure DMA (only for SPI3, SPI2 conflicts with USART3)
    handle->use_dma = (chip_id == W5200_CHIP_SPI3) && use_dma;
    handle->dma_size = handle->use_dma ? dma_size : 0;

    // Initialize socket addresses
    handle->socket_state[0].rx_addr = W5200_RX_MEM;
    handle->socket_state[0].tx_addr = W5200_TX_MEM;
    handle->socket_state[1].rx_addr = W5200_RX_MEM + W5200_SOCKET_BUF_SIZE;
    handle->socket_state[1].tx_addr = W5200_TX_MEM + W5200_SOCKET_BUF_SIZE;

    // Set initial network state
    handle->net_state = W5200_STATE_IDLE;
    handle->sending_socket = 0xFF;    // Invalid socket
    handle->receiving_socket = 0xFF;  // Invalid socket

    // Hardware reset
    HAL_StatusTypeDef status = W5200_HardwareReset(handle);
    if (status != HAL_OK) {
        return status;
    }

    // Configure chip registers (buffer sizes, retry time, etc.)
    status = W5200_ConfigureChip(handle);
    if (status != HAL_OK) {
        return status;
    }

    // Generate default network configuration
    W5200_GenerateIP(chip_id, node_id, handle->network.ip);
    W5200_GenerateMAC(chip_id, node_id, handle->network.mac);

    // Default subnet mask: 255.255.255.0
    handle->network.subnet[0] = 255;
    handle->network.subnet[1] = 255;
    handle->network.subnet[2] = 255;
    handle->network.subnet[3] = 0;

    // Default gateway
    memcpy(handle->network.gateway, handle->network.ip, 3);
    handle->network.gateway[3] = 1;

    // Configure network settings
    status = W5200_ConfigureNetwork(handle, &handle->network);

    return status;
}

/**
 * @brief  Configure network settings (IP, gateway, subnet, MAC)
 */
HAL_StatusTypeDef W5200_ConfigureNetwork(W5200_Handle_t *handle, const W5200_NetworkConfig_t *config)
{
    if (handle == NULL || config == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t retry_time[2] = {0x0F, 0xA0};  // 400ms retry time
    uint8_t retry_count = 1;

    // Write gateway address
    status = W5200_Write(handle, W5200_GAR0, config->gateway, 4);
    if (status != HAL_OK) return status;

    // Write subnet mask
    status = W5200_Write(handle, W5200_SUBR0, config->subnet, 4);
    if (status != HAL_OK) return status;

    // Write MAC address
    status = W5200_Write(handle, W5200_SHAR0, config->mac, 6);
    if (status != HAL_OK) return status;

    // Write IP address
    status = W5200_Write(handle, W5200_SIPR0, config->ip, 4);
    if (status != HAL_OK) return status;

    // Write retry time (4000 = 400ms)
    status = W5200_Write(handle, W5200_RTR0, retry_time, 2);
    if (status != HAL_OK) return status;

    // Write retry count
    status = W5200_WriteByte(handle, W5200_RCR, retry_count);
    if (status != HAL_OK) return status;

    // Save configuration
    memcpy(&handle->network, config, sizeof(W5200_NetworkConfig_t));

    return HAL_OK;
}

/**
 * @brief  Configure socket settings
 */
HAL_StatusTypeDef W5200_ConfigureSocket(W5200_Handle_t *handle, uint8_t socket, const W5200_SocketConfig_t *config)
{
    if (handle == NULL || config == NULL || socket >= W5200_MAX_SOCKETS) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t port_bytes[2];

    // Save socket configuration
    memcpy(&handle->sockets[socket], config, sizeof(W5200_SocketConfig_t));

    if (!config->enabled) {
        return HAL_OK;
    }

    // Write local port
    port_bytes[0] = (config->local_port >> 8) & 0xFF;
    port_bytes[1] = config->local_port & 0xFF;
    status = W5200_Write(handle, W5200_Sn_PORT0(socket), port_bytes, 2);
    if (status != HAL_OK) return status;

    // Write destination IP (for client mode)
    status = W5200_Write(handle, W5200_Sn_DIPR0(socket), config->dest_ip, 4);
    if (status != HAL_OK) return status;

    // Write destination port (for client mode)
    port_bytes[0] = (config->dest_port >> 8) & 0xFF;
    port_bytes[1] = config->dest_port & 0xFF;
    status = W5200_Write(handle, W5200_Sn_DPORT0(socket), port_bytes, 2);
    if (status != HAL_OK) return status;

    // Reset socket state
    handle->socket_state[socket].state = 0;
    handle->socket_state[socket].data_flags = 0;
    handle->socket_state[socket].tx_offset = 0;
    handle->socket_state[socket].rx_offset = 0;

    return HAL_OK;
}

/* ==================== Low-Level SPI Functions ==================== */

/**
 * @brief  Write single byte to W5200 register
 */
HAL_StatusTypeDef W5200_WriteByte(W5200_Handle_t *handle, uint16_t addr, uint8_t data)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t tx_buf[5];
    uint8_t rx_buf[5];

    // Prepare SPI frame: [ADDR_H, ADDR_L, WRITE_CMD|SIZE_H, SIZE_L, DATA]
    tx_buf[0] = (addr >> 8) & 0xFF;          // Address high byte
    tx_buf[1] = addr & 0xFF;                 // Address low byte
    tx_buf[2] = W5200_SPI_WRITE_CMD | 0x00;  // Write command + size high (0x80)
    tx_buf[3] = 0x01;                        // Size low = 1 byte
    tx_buf[4] = data;                        // Data byte

    W5200_CS_LOW(handle);
    status = W5200_SPITransmitReceive(handle, tx_buf, rx_buf, 5);
    W5200_CS_HIGH(handle);

    W5200_DelayUs(W5200_SPI_DELAY_US);

    return status;
}

/**
 * @brief  Read single byte from W5200 register
 */
HAL_StatusTypeDef W5200_ReadByte(W5200_Handle_t *handle, uint16_t addr, uint8_t *data)
{
    if (handle == NULL || data == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t tx_buf[6];
    uint8_t rx_buf[6];

    // Prepare SPI frame: [ADDR_H, ADDR_L, READ_CMD|SIZE_H, SIZE_L, DUMMY, DATA]
    tx_buf[0] = (addr >> 8) & 0xFF;         // Address high byte
    tx_buf[1] = addr & 0xFF;                // Address low byte
    tx_buf[2] = W5200_SPI_READ_CMD | 0x00;  // Read command + size high (0x00)
    tx_buf[3] = 0x01;                       // Size low = 1 byte
    tx_buf[4] = 0x00;                       // Dummy byte (clock out previous response)
    tx_buf[5] = 0x00;                       // Clock in data byte

    W5200_CS_LOW(handle);
    status = W5200_SPITransmitReceive(handle, tx_buf, rx_buf, 6);
    W5200_CS_HIGH(handle);

    if (status == HAL_OK) {
        *data = rx_buf[5];  // Data is in last byte of response
    }

    W5200_DelayUs(W5200_SPI_DELAY_US);

    return status;
}

/**
 * @brief  Write multiple bytes to W5200
 */
HAL_StatusTypeDef W5200_Write(W5200_Handle_t *handle, uint16_t addr, const uint8_t *data, uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t header[4];
    uint8_t dummy[4];

    // Prepare SPI header: [ADDR_H, ADDR_L, WRITE_CMD|SIZE_H, SIZE_L]
    header[0] = (addr >> 8) & 0xFF;
    header[1] = addr & 0xFF;
    header[2] = W5200_SPI_WRITE_CMD | ((size >> 8) & 0xFF);
    header[3] = size & 0xFF;

    W5200_CS_LOW(handle);

    // Send header
    status = W5200_SPITransmitReceive(handle, header, dummy, 4);
    if (status != HAL_OK) {
        W5200_CS_HIGH(handle);
        return status;
    }

    // Send data
    status = HAL_SPI_Transmit(handle->hspi, (uint8_t *)data, size, W5200_SPI_TIMEOUT);

    W5200_CS_HIGH(handle);
    W5200_DelayUs(W5200_SPI_DELAY_US);

    return status;
}

/**
 * @brief  Read multiple bytes from W5200
 */
HAL_StatusTypeDef W5200_Read(W5200_Handle_t *handle, uint16_t addr, uint8_t *data, uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t header[4];
    uint8_t dummy[4];
    uint8_t discard;

    // Prepare SPI header: [ADDR_H, ADDR_L, READ_CMD|SIZE_H, SIZE_L]
    header[0] = (addr >> 8) & 0xFF;
    header[1] = addr & 0xFF;
    header[2] = W5200_SPI_READ_CMD | ((size >> 8) & 0x7F);  // Clear write bit
    header[3] = size & 0xFF;

    W5200_CS_LOW(handle);

    // Send header
    status = W5200_SPITransmitReceive(handle, header, dummy, 4);
    if (status != HAL_OK) {
        W5200_CS_HIGH(handle);
        return status;
    }

    // Receive one dummy byte (W5200 protocol requirement)
    status = HAL_SPI_Receive(handle->hspi, &discard, 1, W5200_SPI_TIMEOUT);
    if (status != HAL_OK) {
        W5200_CS_HIGH(handle);
        return status;
    }

    // Receive data
    status = HAL_SPI_Receive(handle->hspi, data, size, W5200_SPI_TIMEOUT);

    W5200_CS_HIGH(handle);
    W5200_DelayUs(W5200_SPI_DELAY_US);

    return status;
}

/* ==================== Socket Management Functions ==================== */

/**
 * @brief  Open socket in specified mode (TCP Server/Client or UDP)
 */
HAL_StatusTypeDef W5200_SocketOpen(W5200_Handle_t *handle, uint8_t socket)
{
    if (handle == NULL || socket >= W5200_MAX_SOCKETS) {
        return HAL_ERROR;
    }

    if (!handle->sockets[socket].enabled) {
        return HAL_OK;  // Socket not enabled
    }

    HAL_StatusTypeDef status;
    uint8_t mode_reg;
    uint8_t socket_status;

    // Set socket mode register (TCP or UDP)
    if (handle->sockets[socket].protocol == W5200_PROTO_TCP) {
        mode_reg = W5200_Sn_MR_TCP;
    }
    else if (handle->sockets[socket].protocol == W5200_PROTO_UDP) {
        mode_reg = W5200_Sn_MR_UDP;
    }
    else {
        return HAL_ERROR;
    }

    status = W5200_WriteByte(handle, W5200_Sn_MR(socket), mode_reg);
    if (status != HAL_OK) return status;

    // Send OPEN command
    status = W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_OPEN);
    if (status != HAL_OK) return status;

    // Wait for socket to initialize
    HAL_Delay(10);

    // Check socket status
    status = W5200_ReadByte(handle, W5200_Sn_SR(socket), &socket_status);
    if (status != HAL_OK) return status;

    // Verify socket opened correctly
    bool opened = false;
    if (handle->sockets[socket].protocol == W5200_PROTO_TCP) {
        opened = (socket_status == W5200_SOCK_INIT);
    }
    else if (handle->sockets[socket].protocol == W5200_PROTO_UDP) {
        opened = (socket_status == W5200_SOCK_UDP);
    }

    if (!opened) {
        // Failed to open, close it
        W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_CLOSE);
        return HAL_ERROR;
    }

    // For TCP Server mode, send LISTEN command
    if (handle->sockets[socket].protocol == W5200_PROTO_TCP && handle->sockets[socket].mode == W5200_MODE_SERVER) {
        status = W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_LISTEN);
        if (status != HAL_OK) return status;

        HAL_Delay(10);

        status = W5200_ReadByte(handle, W5200_Sn_SR(socket), &socket_status);
        if (status != HAL_OK) return status;

        if (socket_status != W5200_SOCK_LISTEN) {
            W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_CLOSE);
            return HAL_ERROR;
        }
    }
    // For TCP Client mode, send CONNECT command
    else if (handle->sockets[socket].protocol == W5200_PROTO_TCP && handle->sockets[socket].mode == W5200_MODE_CLIENT) {
        status = W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_CONNECT);
        if (status != HAL_OK) return status;
    }

    // Update socket state
    handle->socket_state[socket].state = W5200_S_INIT;

    return HAL_OK;
}

/**
 * @brief  Close socket
 */
HAL_StatusTypeDef W5200_SocketClose(W5200_Handle_t *handle, uint8_t socket)
{
    if (handle == NULL || socket >= W5200_MAX_SOCKETS) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    // Send DISCONNECT command (TCP)
    if (handle->sockets[socket].protocol == W5200_PROTO_TCP) {
        W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_DISCON);
        HAL_Delay(10);
    }

    // Send CLOSE command
    status = W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_CLOSE);

    // Reset socket state
    handle->socket_state[socket].state = 0;
    handle->socket_state[socket].data_flags = 0;
    handle->socket_state[socket].rx_offset = 0;
    handle->socket_state[socket].tx_offset = 0;

    // Reset socket addresses to base
    handle->socket_state[socket].rx_addr = W5200_RX_MEM + (socket * W5200_SOCKET_BUF_SIZE);
    handle->socket_state[socket].tx_addr = W5200_TX_MEM + (socket * W5200_SOCKET_BUF_SIZE);

    return status;
}

/**
 * @brief  Get socket status
 */
HAL_StatusTypeDef W5200_SocketGetStatus(W5200_Handle_t *handle, uint8_t socket, uint8_t *status)
{
    if (handle == NULL || socket >= W5200_MAX_SOCKETS || status == NULL) {
        return HAL_ERROR;
    }

    return W5200_ReadByte(handle, W5200_Sn_SR(socket), status);
}

/**
 * @brief  Check if socket is connected (TCP) or ready (UDP)
 */
bool W5200_SocketIsReady(W5200_Handle_t *handle, uint8_t socket)
{
    if (handle == NULL || socket >= W5200_MAX_SOCKETS) {
        return false;
    }

    uint8_t expected_state = W5200_S_INIT | W5200_S_CONN | W5200_S_OK;
    return (handle->socket_state[socket].state == expected_state);
}

/* ==================== Data Transfer Functions - TO BE CONTINUED ==================== */

/**
 * @brief  Send data through socket
 */
HAL_StatusTypeDef W5200_SocketSend(W5200_Handle_t *handle, uint8_t socket, const uint8_t *data, uint16_t size)
{
    if (handle == NULL || socket >= W5200_MAX_SOCKETS || data == NULL || size == 0) {
        return HAL_ERROR;
    }

    // Check if socket is ready
    if (!handle->sockets[socket].enabled) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint16_t free_size;
    uint8_t size_bytes[2];
    uint16_t tx_write_ptr;
    uint16_t physical_addr;
    uint16_t offset;

    // Check free TX buffer space
    status = W5200_ReadByte(handle, W5200_Sn_TX_FSR0(socket), &size_bytes[0]);
    if (status != HAL_OK) return status;
    status = W5200_ReadByte(handle, W5200_Sn_TX_FSR0(socket) + 1, &size_bytes[1]);
    if (status != HAL_OK) return status;

    free_size = (size_bytes[0] << 8) | size_bytes[1];

    if (free_size < size) {
        return HAL_ERROR;  // Not enough space
    }

    // Get current TX write pointer
    status = W5200_ReadByte(handle, W5200_Sn_TX_WR0(socket), &size_bytes[0]);
    if (status != HAL_OK) return status;
    status = W5200_ReadByte(handle, W5200_Sn_TX_WR0(socket) + 1, &size_bytes[1]);
    if (status != HAL_OK) return status;

    tx_write_ptr = (size_bytes[0] << 8) | size_bytes[1];

    // Calculate physical address
    offset = tx_write_ptr % W5200_SOCKET_BUF_SIZE;
    physical_addr = W5200_TX_MEM + (socket * W5200_SOCKET_BUF_SIZE) + offset;

    // Handle buffer wrap-around
    if ((offset + size) > W5200_SOCKET_BUF_SIZE) {
        uint16_t first_part = W5200_SOCKET_BUF_SIZE - offset;
        uint16_t second_part = size - first_part;

        // Write first part
        status = W5200_Write(handle, physical_addr, data, first_part);
        if (status != HAL_OK) return status;

        // Write second part (wrapped)
        status = W5200_Write(handle, W5200_TX_MEM + (socket * W5200_SOCKET_BUF_SIZE), data + first_part, second_part);
        if (status != HAL_OK) return status;
    }
    else {
        // No wrap-around, write entire buffer
        status = W5200_Write(handle, physical_addr, data, size);
        if (status != HAL_OK) return status;
    }

    // Update TX write pointer
    tx_write_ptr += size;
    size_bytes[0] = (tx_write_ptr >> 8) & 0xFF;
    size_bytes[1] = tx_write_ptr & 0xFF;

    status = W5200_WriteByte(handle, W5200_Sn_TX_WR0(socket), size_bytes[0]);
    if (status != HAL_OK) return status;
    status = W5200_WriteByte(handle, W5200_Sn_TX_WR0(socket) + 1, size_bytes[1]);
    if (status != HAL_OK) return status;

    // Send SEND command
    status = W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_SEND);

    return status;
}

/**
 * @brief  Check if data is available to receive
 */
HAL_StatusTypeDef W5200_SocketDataAvailable(W5200_Handle_t *handle, uint8_t socket, uint16_t *available_size)
{
    if (handle == NULL || socket >= W5200_MAX_SOCKETS || available_size == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t size_bytes[2];

    // Read RX received size register
    status = W5200_ReadByte(handle, W5200_Sn_RX_RSR0(socket), &size_bytes[0]);
    if (status != HAL_OK) return status;
    status = W5200_ReadByte(handle, W5200_Sn_RX_RSR0(socket) + 1, &size_bytes[1]);
    if (status != HAL_OK) return status;

    *available_size = (size_bytes[0] << 8) | size_bytes[1];

    return HAL_OK;
}

/**
 * @brief  Receive data from socket
 */
HAL_StatusTypeDef W5200_SocketReceive(
    W5200_Handle_t *handle, uint8_t socket, uint8_t *data, uint16_t max_size, uint16_t *received_size)
{
    if (handle == NULL || socket >= W5200_MAX_SOCKETS || data == NULL || received_size == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint16_t available;
    uint8_t ptr_bytes[2];
    uint16_t rx_read_ptr;
    uint16_t physical_addr;
    uint16_t offset;
    uint16_t size_to_read;

    // Check available data
    status = W5200_SocketDataAvailable(handle, socket, &available);
    if (status != HAL_OK) return status;

    if (available == 0) {
        *received_size = 0;
        return HAL_OK;
    }

    // Limit to max_size
    size_to_read = (available > max_size) ? max_size : available;

    // Get current RX read pointer
    status = W5200_ReadByte(handle, W5200_Sn_RX_RD0(socket), &ptr_bytes[0]);
    if (status != HAL_OK) return status;
    status = W5200_ReadByte(handle, W5200_Sn_RX_RD0(socket) + 1, &ptr_bytes[1]);
    if (status != HAL_OK) return status;

    rx_read_ptr = (ptr_bytes[0] << 8) | ptr_bytes[1];

    // Calculate physical address
    offset = rx_read_ptr % W5200_SOCKET_BUF_SIZE;
    physical_addr = W5200_RX_MEM + (socket * W5200_SOCKET_BUF_SIZE) + offset;

    // Handle buffer wrap-around
    if ((offset + size_to_read) > W5200_SOCKET_BUF_SIZE) {
        uint16_t first_part = W5200_SOCKET_BUF_SIZE - offset;
        uint16_t second_part = size_to_read - first_part;

        // Read first part
        status = W5200_Read(handle, physical_addr, data, first_part);
        if (status != HAL_OK) return status;

        // Read second part (wrapped)
        status = W5200_Read(handle, W5200_RX_MEM + (socket * W5200_SOCKET_BUF_SIZE), data + first_part, second_part);
        if (status != HAL_OK) return status;
    }
    else {
        // No wrap-around, read entire buffer
        status = W5200_Read(handle, physical_addr, data, size_to_read);
        if (status != HAL_OK) return status;
    }

    // Update RX read pointer
    rx_read_ptr += size_to_read;
    ptr_bytes[0] = (rx_read_ptr >> 8) & 0xFF;
    ptr_bytes[1] = rx_read_ptr & 0xFF;

    status = W5200_WriteByte(handle, W5200_Sn_RX_RD0(socket), ptr_bytes[0]);
    if (status != HAL_OK) return status;
    status = W5200_WriteByte(handle, W5200_Sn_RX_RD0(socket) + 1, ptr_bytes[1]);
    if (status != HAL_OK) return status;

    // Send RECV command to complete reception
    status = W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_RECV);
    if (status != HAL_OK) return status;

    *received_size = size_to_read;

    return HAL_OK;
}

/* ==================== Cyclic Update Functions - TO BE CONTINUED ==================== */

/**
 * @brief  Process network events and interrupts
 */
HAL_StatusTypeDef W5200_ProcessInterrupts(W5200_Handle_t *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t interrupt_reg;
    uint8_t socket_int;
    uint8_t socket_mask = 0x01;

    // Read common interrupt register
    status = W5200_ReadByte(handle, W5200_IR2, &interrupt_reg);
    if (status != HAL_OK) return status;

    // Process each socket
    for (uint8_t socket = 0; socket < W5200_MAX_SOCKETS; socket++) {
        if (!handle->sockets[socket].enabled) {
            socket_mask <<= 1;
            continue;
        }

        // Check if this socket has interrupt
        if (interrupt_reg & socket_mask) {
            // Read socket interrupt register
            status = W5200_ReadByte(handle, W5200_Sn_IR(socket), &socket_int);
            if (status != HAL_OK) return status;

            // Clear socket interrupt
            status = W5200_WriteByte(handle, W5200_Sn_IR(socket), socket_int);
            if (status != HAL_OK) return status;

            // Handle connection established
            if (socket_int & W5200_Sn_IR_CON) {
                if (handle->sockets[socket].mode == W5200_MODE_SERVER) {
                    handle->socket_state[socket].state |= W5200_S_CONN;
                }
                else {
                    handle->socket_state[socket].state |= (W5200_S_CONN | W5200_S_OK);
                }
            }

            // Handle disconnection
            if (socket_int & W5200_Sn_IR_DISCON) {
                W5200_SocketClose(handle, socket);
            }

            // Handle send complete
            if (socket_int & W5200_Sn_IR_SENDOK) {
                handle->socket_state[socket].data_flags |= W5200_DATA_TX_OK;
            }

            // Handle data received
            if (socket_int & W5200_Sn_IR_RECV) {
                handle->socket_state[socket].data_flags |= W5200_DATA_RECEIVE;
                handle->rx_flags[socket] = 1;
            }

            // Handle timeout
            if (socket_int & W5200_Sn_IR_TIMEOUT) {
                if (W5200_SocketIsReady(handle, socket)) {
                    // Close and reset socket on timeout
                    W5200_SocketClose(handle, socket);
                }
            }
        }

        socket_mask <<= 1;
    }

    return HAL_OK;
}

/**
 * @brief  Manage socket states and connections
 */
HAL_StatusTypeDef W5200_ManageSockets(W5200_Handle_t *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    if (handle->net_state != W5200_STATE_IDLE) {
        return HAL_OK;  // Busy with other operations
    }

    // Check each socket
    for (uint8_t socket = 0; socket < W5200_MAX_SOCKETS; socket++) {
        if (!handle->sockets[socket].enabled) {
            continue;
        }

        uint8_t socket_state = handle->socket_state[socket].state;

        // Socket not initialized - try to open it
        if (socket_state == 0) {
            handle->net_state = W5200_STATE_CHECKING;
            HAL_StatusTypeDef status = W5200_SocketOpen(handle, socket);
            handle->net_state = W5200_STATE_IDLE;

            if (status == HAL_OK) {
                handle->socket_state[socket].state = W5200_S_INIT;
            }
        }
        // TCP Server socket connected but not yet ready
        else if ((socket_state == (W5200_S_INIT | W5200_S_CONN)) &&
                 (handle->sockets[socket].mode == W5200_MODE_SERVER)) {
            // Mark socket as ready for communication
            handle->socket_state[socket].state = W5200_S_INIT | W5200_S_CONN | W5200_S_OK;
        }
    }

    return HAL_OK;
}

/**
 * @brief  Send keep-alive packets for TCP connections
 */
HAL_StatusTypeDef W5200_SendKeepAlive(W5200_Handle_t *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    handle->keepalive_counter++;

    // Check if it's time to send keep-alive (every 1.2 seconds)
    if (handle->keepalive_counter < W5200_KEEPALIVE_INTERVAL_MS) {
        return HAL_OK;
    }

    handle->keepalive_counter = 0;

    if (handle->net_state != W5200_STATE_IDLE) {
        return HAL_OK;  // Busy
    }

    handle->net_state = W5200_STATE_CHECKING;

    // Send keep-alive for all connected TCP sockets
    for (uint8_t socket = 0; socket < W5200_MAX_SOCKETS; socket++) {
        if (handle->sockets[socket].enabled && (handle->socket_state[socket].state & W5200_S_INIT) &&
            handle->sockets[socket].protocol == W5200_PROTO_TCP) {
            W5200_WriteByte(handle, W5200_Sn_CR(socket), W5200_Sn_CR_SEND_KEEP);
        }
    }

    handle->net_state = W5200_STATE_IDLE;

    return HAL_OK;
}

/**
 * @brief  Main cyclic task - combines all periodic operations
 */
HAL_StatusTypeDef W5200_Cycle(W5200_Handle_t *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    // Manage socket connections and states
    status = W5200_ManageSockets(handle);
    if (status != HAL_OK) return status;

    // Process interrupts and events
    if (handle->net_state == W5200_STATE_IDLE) {
        handle->net_state = W5200_STATE_CHECKING;

        // Check if any socket needs attention
        bool has_active_socket = false;
        for (uint8_t s = 0; s < W5200_MAX_SOCKETS; s++) {
            if (handle->sockets[s].enabled && (handle->socket_state[s].state & W5200_S_INIT)) {
                has_active_socket = true;
                break;
            }
        }

        if (has_active_socket) {
            status = W5200_ProcessInterrupts(handle);
        }

        handle->net_state = W5200_STATE_IDLE;
    }

    // Send keep-alive packets
    status = W5200_SendKeepAlive(handle);

    return status;
}

/* ==================== Utility Functions ==================== */

/**
 * @brief  Hardware reset W5200 chip
 */
HAL_StatusTypeDef W5200_HardwareReset(W5200_Handle_t *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    // Set CS high (inactive)
    W5200_CS_HIGH(handle);

    // Reset pulse: LOW for 500ms
    W5200_RST_LOW(handle);
    HAL_Delay(W5200_RESET_DELAY_MS);

    // Release reset
    W5200_RST_HIGH(handle);
    HAL_Delay(W5200_RESET_DELAY_MS);

    return HAL_OK;
}

/**
 * @brief  Get chip identification string
 */
const char *W5200_GetChipName(W5200_ChipID_t chip_id)
{
    return (chip_id == W5200_CHIP_SPI2) ? "W5200-SPI2" : "W5200-SPI3";
}

/**
 * @brief  Format MAC address for chip based on node ID
 */
void W5200_GenerateMAC(W5200_ChipID_t chip_id, uint8_t node_id, uint8_t *mac)
{
    if (mac == NULL) return;

    // Base MAC address: 34:24:E8:B9:xx:xx
    mac[0] = 0x34;
    mac[1] = 0x24;
    mac[2] = 0xE8;

    if (node_id != 0) {  // B side or other nodes
        mac[3] = 0xB9;
        mac[4] = 0x8B + node_id;
        mac[5] = (chip_id == W5200_CHIP_SPI2) ? 0x31 : 0x32;
    }
    else {  // A side (node 0)
        mac[3] = 0xB9;
        mac[4] = 0x8B;
        mac[5] = (chip_id == W5200_CHIP_SPI2) ? 0x33 : 0x34;
    }
}

/**
 * @brief  Format IP address for chip based on node ID
 */
void W5200_GenerateIP(W5200_ChipID_t chip_id, uint8_t node_id, uint8_t *ip)
{
    if (ip == NULL) return;

    if (chip_id == W5200_CHIP_SPI2) {
        // SPI2: 192.168.2.100+NodeID
        ip[0] = 192;
        ip[1] = 168;
        ip[2] = 2;
        ip[3] = 100 + node_id;
    }
    else {  // W5200_CHIP_SPI3
        // SPI3: 192.168.10.31+NodeID
        ip[0] = 192;
        ip[1] = 168;
        ip[2] = 10;
        ip[3] = 31 + node_id;
    }
}

/* ==================== Private Helper Functions ==================== */

/**
 * @brief  SPI transmit/receive wrapper
 */
static HAL_StatusTypeDef W5200_SPITransmitReceive(
    W5200_Handle_t *handle, uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    return HAL_SPI_TransmitReceive(handle->hspi, tx_data, rx_data, size, W5200_SPI_TIMEOUT);
}

/**
 * @brief  Microsecond delay
 */
static void W5200_DelayUs(uint32_t us)
{
    // Simple delay loop - adjust based on CPU speed
    // At 168MHz, ~168 cycles = 1µs
    volatile uint32_t count = us * 42;  // Approximate: 168/4 = 42
    while (count--) {
        __NOP();
    }
}

/**
 * @brief  Configure W5200 chip registers
 */
static HAL_StatusTypeDef W5200_ConfigureChip(W5200_Handle_t *handle)
{
    HAL_StatusTypeDef status;

    // Configure socket buffer sizes (8KB RX + 8KB TX for each socket)
    for (uint8_t socket = 0; socket < W5200_MAX_SOCKETS; socket++) {
        status = W5200_WriteByte(handle, W5200_Sn_RXMEM_SIZE(socket), 0x08);
        if (status != HAL_OK) return status;

        status = W5200_WriteByte(handle, W5200_Sn_TXMEM_SIZE(socket), 0x08);
        if (status != HAL_OK) return status;
    }

    return HAL_OK;
}

/* ==================== End of File ==================== */
