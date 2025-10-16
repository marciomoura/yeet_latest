/**
 ******************************************************************************
 * @file    w5200_driver.c
 * @brief   W5200 Ethernet controller hardware driver implementation
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * This module provides low-level hardware abstraction for the W5200 Ethernet
 * controller. All hardware-specific operations are contained here.
 *
 * Hardware configuration extracted from original network.c implementation.
 *
 ******************************************************************************
 */

#include "w5200_driver.h"

#include <stdio.h>
#include <stm32f4xx.h>
#include <string.h>

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void W5200_InitGPIO(w5200_instance_t* w5200);
static void W5200_InitSPI(w5200_instance_t* w5200);
static void W5200_InitDMA(w5200_instance_t* w5200);
static void W5200_HardwareReset(w5200_instance_t* w5200);
static void W5200_Delay(uint32_t cycles);
static void W5200_SetCS(w5200_instance_t* w5200, bool state);
static void W5200_SetReset(w5200_instance_t* w5200, bool state);
static void W5200_ConfigureCommon(w5200_instance_t* w5200);
static uint16_t W5200_GetSocketTxBase(uint8_t socket_num);
static uint16_t W5200_GetSocketRxBase(uint8_t socket_num);

/* ============================================================================
 * Public Function Implementations - Initialization
 * ============================================================================ */

/**
 * @brief Initialize W5200 hardware driver
 */
bool W5200_Init(w5200_instance_t* w5200, SPI_TypeDef* spi, uint8_t* tx_buffer, uint8_t* rx_buffer, uint16_t dma_size)
{
    if (w5200 == NULL || spi == NULL || tx_buffer == NULL || rx_buffer == NULL) {
        return false;
    }

    // Validate DMA size
    if (dma_size != W5200_DMA_SIZE_128 && dma_size != W5200_DMA_SIZE_256 && dma_size != W5200_DMA_SIZE_512 &&
        dma_size != W5200_DMA_SIZE_1K && dma_size != W5200_DMA_SIZE_2K && dma_size != W5200_DMA_SIZE_4K &&
        dma_size != W5200_DMA_SIZE_8K) {
        return false;
    }

    // Clear instance structure
    memset(w5200, 0, sizeof(w5200_instance_t));

    // Store hardware configuration
    w5200->spi = spi;
    w5200->tx_buffer = tx_buffer;
    w5200->rx_buffer = rx_buffer;
    w5200->dma_size = dma_size;

    // Configure GPIO pins based on SPI interface
    if (spi == SPI2) {
        w5200->cs_port = GPIOA;
        w5200->cs_pin = GPIO_Pin_11;
        w5200->reset_port = GPIOA;
        w5200->reset_pin = GPIO_Pin_12;
    }
    else if (spi == SPI3) {
        w5200->cs_port = GPIOB;
        w5200->cs_pin = GPIO_Pin_3;
        w5200->reset_port = GPIOB;
        w5200->reset_pin = GPIO_Pin_4;
    }
    else {
        return false;
    }

    // Initialize hardware
    W5200_InitGPIO(w5200);
    W5200_InitSPI(w5200);
    W5200_InitDMA(w5200);

    // Reset W5200 chip
    W5200_HardwareReset(w5200);

    // Configure common registers
    W5200_ConfigureCommon(w5200);

    w5200->initialized = true;
    return true;
}

/**
 * @brief Configure network parameters
 */
bool W5200_ConfigureNetwork(w5200_instance_t* w5200, const w5200_network_config_t* config)
{
    if (w5200 == NULL || config == NULL || !w5200->initialized) {
        return false;
    }

    // Store configuration
    memcpy(&w5200->network, config, sizeof(w5200_network_config_t));

    // Write gateway address
    W5200_Write(w5200, W5200_GAR0, config->gateway.bytes, 4);

    // Write subnet mask
    W5200_Write(w5200, W5200_SUBR0, config->subnet_mask.bytes, 4);

    // Write MAC address
    W5200_Write(w5200, W5200_SHAR0, config->mac_address.bytes, 6);

    // Write IP address
    W5200_Write(w5200, W5200_SIPR0, config->ip_address.bytes, 4);

    // Configure retry time (400ms default = 0x0FA0)
    uint8_t retry_time[2];
    retry_time[0] = (config->retry_time >> 8) & 0xFF;
    retry_time[1] = config->retry_time & 0xFF;
    W5200_Write(w5200, W5200_RTR, retry_time, 2);

    // Configure retry count
    W5200_WriteByte(w5200, W5200_RCR, config->retry_count);

    return true;
}

/**
 * @brief Configure a socket
 */
bool W5200_ConfigureSocket(w5200_instance_t* w5200, uint8_t socket_num, const w5200_socket_config_t* config)
{
    if (w5200 == NULL || config == NULL || !w5200->initialized) {
        return false;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return false;
    }

    // Store configuration
    memcpy(&w5200->sockets[socket_num], config, sizeof(w5200_socket_config_t));

    if (!config->enabled) {
        return true;
    }

    // Configure source port
    uint8_t port_bytes[2];
    port_bytes[0] = (config->source_port >> 8) & 0xFF;
    port_bytes[1] = config->source_port & 0xFF;
    W5200_Write(w5200, W5200_Sn_PORT0(socket_num), port_bytes, 2);

    // Configure destination IP
    W5200_Write(w5200, W5200_Sn_DIPR0(socket_num), config->dest_ip.bytes, 4);

    // Configure destination port
    port_bytes[0] = (config->dest_port >> 8) & 0xFF;
    port_bytes[1] = config->dest_port & 0xFF;
    W5200_Write(w5200, W5200_Sn_DPORT0(socket_num), port_bytes, 2);

    // Configure max segment size for TCP
    if (config->protocol == W5200_PROTO_TCP) {
        uint8_t mss[2] = {0x05, 0xB4};  // 1460 bytes
        W5200_Write(w5200, W5200_Sn_MSSR0(socket_num), mss, 2);
    }

    // Initialize socket state
    w5200->socket_state[socket_num].status = W5200_SOCK_CLOSED;
    w5200->socket_state[socket_num].state = W5200_STATE_IDLE;
    w5200->socket_state[socket_num].tx_ready = false;
    w5200->socket_state[socket_num].rx_ready = false;
    w5200->socket_state[socket_num].rx_size = 0;
    w5200->socket_state[socket_num].tx_size = 0;

    return true;
}

/**
 * @brief Reset W5200 hardware
 */
void W5200_Reset(w5200_instance_t* w5200)
{
    if (w5200 == NULL) {
        return;
    }

    W5200_HardwareReset(w5200);
    W5200_ConfigureCommon(w5200);

    // Reset all socket states
    for (int i = 0; i < W5200_ACTIVE_SOCKETS; i++) {
        w5200->socket_state[i].status = W5200_SOCK_CLOSED;
        w5200->socket_state[i].state = W5200_STATE_IDLE;
        w5200->socket_state[i].tx_ready = false;
        w5200->socket_state[i].rx_ready = false;
    }
}

/* ============================================================================
 * Public Function Implementations - Register Access
 * ============================================================================ */

/**
 * @brief Write single byte to W5200 register
 */
void W5200_WriteByte(w5200_instance_t* w5200, uint16_t addr, uint8_t data)
{
    if (w5200 == NULL) {
        return;
    }

    int timeout = 0;

    SPI_Cmd(w5200->spi, ENABLE);

    // Send register address (high byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)(addr >> 8));
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Send register address (low byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)addr);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Send WRITE command (0x80 | 0x01)
    SPI_I2S_SendData(w5200->spi, 0x80);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    SPI_I2S_SendData(w5200->spi, 0x01);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Write data byte
    SPI_I2S_SendData(w5200->spi, data);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    SPI_Cmd(w5200->spi, DISABLE);
    W5200_Delay(2);
}

/**
 * @brief Read single byte from W5200 register
 */
uint8_t W5200_ReadByte(w5200_instance_t* w5200, uint16_t addr)
{
    if (w5200 == NULL) {
        return 0;
    }

    uint8_t result = 0;
    int timeout = 0;

    SPI_Cmd(w5200->spi, ENABLE);

    // Send register address (high byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)(addr >> 8));
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Send register address (low byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)addr);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Send READ command (0x00 | 0x01)
    SPI_I2S_SendData(w5200->spi, 0x00);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    SPI_I2S_SendData(w5200->spi, 0x01);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Wait for response
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_RXNE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);
    result = SPI_I2S_ReceiveData(w5200->spi);

    // Read data byte
    SPI_I2S_SendData(w5200->spi, 0x00);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_RXNE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);
    result = SPI_I2S_ReceiveData(w5200->spi);

    SPI_Cmd(w5200->spi, DISABLE);
    W5200_Delay(2);

    return result;
}

/**
 * @brief Write multiple bytes to W5200 registers
 */
void W5200_Write(w5200_instance_t* w5200, uint16_t addr, const uint8_t* data, uint16_t length)
{
    if (w5200 == NULL || data == NULL || length == 0) {
        return;
    }

    int timeout = 0;

    SPI_Cmd(w5200->spi, ENABLE);

    // Send register address (high byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)(addr >> 8));
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Send register address (low byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)addr);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Send WRITE command with length
    uint16_t cmd = length | 0x8000;
    SPI_I2S_SendData(w5200->spi, (uint8_t)(cmd >> 8));
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    SPI_I2S_SendData(w5200->spi, (uint8_t)cmd);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Write data bytes
    for (uint16_t i = 0; i < length; i++) {
        SPI_I2S_SendData(w5200->spi, data[i]);
        timeout = 0;
        while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);
    }

    SPI_Cmd(w5200->spi, DISABLE);
    W5200_Delay(2);
}

/**
 * @brief Read multiple bytes from W5200 registers
 */
void W5200_Read(w5200_instance_t* w5200, uint16_t addr, uint8_t* data, uint16_t length)
{
    if (w5200 == NULL || data == NULL || length == 0) {
        return;
    }

    int timeout = 0;

    SPI_Cmd(w5200->spi, ENABLE);

    // Send register address (high byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)(addr >> 8));
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Send register address (low byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)addr);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Send READ command with length
    uint16_t cmd = length & 0x7FFF;
    SPI_I2S_SendData(w5200->spi, (uint8_t)(cmd >> 8));
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    SPI_I2S_SendData(w5200->spi, (uint8_t)cmd);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);

    // Wait for first response
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_RXNE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);
    uint8_t dummy = SPI_I2S_ReceiveData(w5200->spi);
    (void)dummy;

    // Read data bytes
    for (uint16_t i = 0; i < length; i++) {
        SPI_I2S_SendData(w5200->spi, 0x00);
        timeout = 0;
        while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_RXNE) == RESET) && timeout++ < W5200_SPI_TIMEOUT);
        data[i] = SPI_I2S_ReceiveData(w5200->spi);
    }

    SPI_Cmd(w5200->spi, DISABLE);
    W5200_Delay(2);
}

/* ============================================================================
 * Public Function Implementations - Socket Operations
 * ============================================================================ */

/**
 * @brief Open a socket
 */
bool W5200_SocketOpen(w5200_instance_t* w5200, uint8_t socket_num)
{
    if (w5200 == NULL || !w5200->initialized) {
        return false;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return false;
    }

    if (!w5200->sockets[socket_num].enabled) {
        return false;
    }

    // Set socket mode (TCP or UDP)
    uint8_t mode = (w5200->sockets[socket_num].protocol == W5200_PROTO_TCP) ? W5200_Sn_MR_TCP : W5200_Sn_MR_UDP;
    W5200_WriteByte(w5200, W5200_Sn_MR(socket_num), mode);

    // Open socket
    W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_OPEN);

    // Wait for socket to open
    W5200_Delay(1000);

    // Check socket status
    uint8_t status = W5200_ReadByte(w5200, W5200_Sn_SR(socket_num));

    if (w5200->sockets[socket_num].protocol == W5200_PROTO_TCP) {
        if (status != W5200_SOCK_INIT) {
            return false;
        }

        // For TCP server mode, start listening
        if (w5200->sockets[socket_num].mode == W5200_MODE_SERVER) {
            return W5200_SocketListen(w5200, socket_num);
        }
        // For TCP client mode, connect
        else {
            return W5200_SocketConnect(w5200, socket_num);
        }
    }
    else {
        // UDP socket should be in UDP state
        if (status != W5200_SOCK_UDP) {
            return false;
        }
    }

    w5200->socket_state[socket_num].status = status;
    return true;
}

/**
 * @brief Close a socket
 */
void W5200_SocketClose(w5200_instance_t* w5200, uint8_t socket_num)
{
    if (w5200 == NULL || !w5200->initialized) {
        return;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return;
    }

    // Send close command
    W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_CLOSE);

    // Wait for close to complete
    W5200_Delay(1000);

    // Update state
    w5200->socket_state[socket_num].status = W5200_SOCK_CLOSED;
    w5200->socket_state[socket_num].state = W5200_STATE_IDLE;
}

/**
 * @brief Listen on TCP socket (server mode)
 */
bool W5200_SocketListen(w5200_instance_t* w5200, uint8_t socket_num)
{
    if (w5200 == NULL || !w5200->initialized) {
        return false;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return false;
    }

    // Send listen command
    W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_LISTEN);

    // Wait for listen state
    W5200_Delay(1000);

    // Check status
    uint8_t status = W5200_ReadByte(w5200, W5200_Sn_SR(socket_num));
    w5200->socket_state[socket_num].status = status;

    return (status == W5200_SOCK_LISTEN);
}

/**
 * @brief Connect TCP socket (client mode)
 */
bool W5200_SocketConnect(w5200_instance_t* w5200, uint8_t socket_num)
{
    if (w5200 == NULL || !w5200->initialized) {
        return false;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return false;
    }

    // Send connect command
    W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_CONNECT);

    // Status will be updated in interrupt handler or polling
    return true;
}

/**
 * @brief Disconnect TCP socket
 */
void W5200_SocketDisconnect(w5200_instance_t* w5200, uint8_t socket_num)
{
    if (w5200 == NULL || !w5200->initialized) {
        return;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return;
    }

    // Send disconnect command
    W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_DISCON);
}

/**
 * @brief Get socket status
 */
uint8_t W5200_GetSocketStatus(w5200_instance_t* w5200, uint8_t socket_num)
{
    if (w5200 == NULL || !w5200->initialized) {
        return W5200_SOCK_CLOSED;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return W5200_SOCK_CLOSED;
    }

    uint8_t status = W5200_ReadByte(w5200, W5200_Sn_SR(socket_num));
    w5200->socket_state[socket_num].status = status;
    return status;
}

/**
 * @brief Get received data size
 */
uint16_t W5200_GetRxSize(w5200_instance_t* w5200, uint8_t socket_num)
{
    if (w5200 == NULL || !w5200->initialized) {
        return 0;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return 0;
    }

    // Read RX received size register (2 bytes)
    uint8_t size_bytes[2];
    W5200_Read(w5200, W5200_Sn_RX_RSR0(socket_num), size_bytes, 2);

    uint16_t size = (size_bytes[0] << 8) | size_bytes[1];
    w5200->socket_state[socket_num].rx_size = size;
    w5200->socket_state[socket_num].rx_ready = (size > 0);

    return size;
}

/**
 * @brief Get free TX buffer size
 */
uint16_t W5200_GetTxFreeSize(w5200_instance_t* w5200, uint8_t socket_num)
{
    if (w5200 == NULL || !w5200->initialized) {
        return 0;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return 0;
    }

    // Read TX free size register (2 bytes)
    uint8_t size_bytes[2];
    W5200_Read(w5200, W5200_Sn_TX_FSR0(socket_num), size_bytes, 2);

    uint16_t size = (size_bytes[0] << 8) | size_bytes[1];
    w5200->socket_state[socket_num].tx_ready = (size > 0);

    return size;
}

/* ============================================================================
 * Public Function Implementations - Data Transfer
 * ============================================================================ */

/**
 * @brief Send data through socket
 */
bool W5200_Send(w5200_instance_t* w5200, uint8_t socket_num, const uint8_t* data, uint16_t length)
{
    if (w5200 == NULL || !w5200->initialized || data == NULL || length == 0) {
        return false;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return false;
    }

    // Check if socket is ready
    uint16_t free_size = W5200_GetTxFreeSize(w5200, socket_num);
    if (free_size < length) {
        return false;
    }

    // Get TX write pointer
    uint8_t ptr_bytes[2];
    W5200_Read(w5200, W5200_Sn_TX_WR0(socket_num), ptr_bytes, 2);
    uint16_t tx_ptr = (ptr_bytes[0] << 8) | ptr_bytes[1];

    // Calculate physical address in TX buffer
    uint16_t tx_base = W5200_GetSocketTxBase(socket_num);
    uint16_t offset = tx_ptr % W5200_SOCKET_BUF_SIZE;
    uint16_t physical_addr = tx_base + offset;

    // Write data to TX buffer
    W5200_Write(w5200, physical_addr, data, length);

    // Update TX write pointer
    tx_ptr += length;
    ptr_bytes[0] = (tx_ptr >> 8) & 0xFF;
    ptr_bytes[1] = tx_ptr & 0xFF;
    W5200_Write(w5200, W5200_Sn_TX_WR0(socket_num), ptr_bytes, 2);

    // Send SEND command
    W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_SEND);

    w5200->socket_state[socket_num].tx_size = length;
    w5200->socket_state[socket_num].state = W5200_STATE_SENDING;

    return true;
}

/**
 * @brief Receive data from socket
 */
uint16_t W5200_Receive(w5200_instance_t* w5200, uint8_t socket_num, uint8_t* data, uint16_t max_length)
{
    if (w5200 == NULL || !w5200->initialized || data == NULL || max_length == 0) {
        return 0;
    }

    if (socket_num >= W5200_ACTIVE_SOCKETS) {
        return 0;
    }

    // Get received data size
    uint16_t rx_size = W5200_GetRxSize(w5200, socket_num);
    if (rx_size == 0) {
        return 0;
    }

    // Limit to buffer size
    uint16_t read_size = (rx_size > max_length) ? max_length : rx_size;

    // Get RX read pointer
    uint8_t ptr_bytes[2];
    W5200_Read(w5200, W5200_Sn_RX_RD0(socket_num), ptr_bytes, 2);
    uint16_t rx_ptr = (ptr_bytes[0] << 8) | ptr_bytes[1];

    // Calculate physical address in RX buffer
    uint16_t rx_base = W5200_GetSocketRxBase(socket_num);
    uint16_t offset = rx_ptr % W5200_SOCKET_BUF_SIZE;
    uint16_t physical_addr = rx_base + offset;

    // Read data from RX buffer
    W5200_Read(w5200, physical_addr, data, read_size);

    // Update RX read pointer
    rx_ptr += read_size;
    ptr_bytes[0] = (rx_ptr >> 8) & 0xFF;
    ptr_bytes[1] = rx_ptr & 0xFF;
    W5200_Write(w5200, W5200_Sn_RX_RD0(socket_num), ptr_bytes, 2);

    // Send RECV command
    W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_RECV);

    w5200->socket_state[socket_num].state = W5200_STATE_IDLE;

    return read_size;
}

/**
 * @brief Send data using DMA
 */
bool W5200_SendDMA(w5200_instance_t* w5200, uint8_t socket_num, const uint8_t* data, uint16_t length)
{
    // DMA implementation would go here
    // For now, fall back to regular send
    return W5200_Send(w5200, socket_num, data, length);
}

/**
 * @brief Receive data using DMA
 */
bool W5200_ReceiveDMA(w5200_instance_t* w5200, uint8_t socket_num)
{
    // DMA implementation would go here
    // For now, return false to indicate DMA not implemented
    return false;
}

/* ============================================================================
 * Public Function Implementations - Interrupt Handling
 * ============================================================================ */

/**
 * @brief Process W5200 interrupts
 */
void W5200_ProcessInterrupts(w5200_instance_t* w5200)
{
    if (w5200 == NULL || !w5200->initialized) {
        return;
    }

    // Read interrupt register
    uint8_t ir = W5200_ReadByte(w5200, W5200_IR2);

    // Process each socket
    for (uint8_t i = 0; i < W5200_ACTIVE_SOCKETS; i++) {
        if (!w5200->sockets[i].enabled) {
            continue;
        }

        // Check if this socket has interrupt
        uint8_t mask = (1 << i);
        if (!(ir & mask)) {
            continue;
        }

        // Read socket interrupt register
        uint8_t sir = W5200_ReadByte(w5200, W5200_Sn_IR(i));

        // Handle connection event
        if (sir & W5200_Sn_IR_CON) {
            w5200->socket_state[i].status = W5200_SOCK_ESTABLISHED;
            W5200_WriteByte(w5200, W5200_Sn_IR(i), W5200_Sn_IR_CON);
        }

        // Handle disconnection event
        if (sir & W5200_Sn_IR_DISCON) {
            w5200->socket_state[i].status = W5200_SOCK_CLOSED;
            w5200->socket_state[i].state = W5200_STATE_IDLE;
            W5200_WriteByte(w5200, W5200_Sn_IR(i), W5200_Sn_IR_DISCON);
        }

        // Handle receive event
        if (sir & W5200_Sn_IR_RECV) {
            w5200->socket_state[i].rx_ready = true;
            w5200->socket_state[i].state = W5200_STATE_RECEIVING;
            W5200_WriteByte(w5200, W5200_Sn_IR(i), W5200_Sn_IR_RECV);
        }

        // Handle send complete event
        if (sir & W5200_Sn_IR_SENDOK) {
            w5200->socket_state[i].state = W5200_STATE_IDLE;
            W5200_WriteByte(w5200, W5200_Sn_IR(i), W5200_Sn_IR_SENDOK);
        }

        // Handle timeout event
        if (sir & W5200_Sn_IR_TIMEOUT) {
            w5200->socket_state[i].state = W5200_STATE_IDLE;
            W5200_WriteByte(w5200, W5200_Sn_IR(i), W5200_Sn_IR_TIMEOUT);
        }
    }
}

/**
 * @brief Check and update socket states
 */
void W5200_Update(w5200_instance_t* w5200)
{
    if (w5200 == NULL || !w5200->initialized) {
        return;
    }

    // Process interrupts
    W5200_ProcessInterrupts(w5200);

    // Periodic connection check (every 1.2 seconds)
    if (++w5200->check_delay > 1200) {
        w5200->check_delay = 0;

        for (uint8_t i = 0; i < W5200_ACTIVE_SOCKETS; i++) {
            if (!w5200->sockets[i].enabled) {
                continue;
            }

            // Update socket status
            W5200_GetSocketStatus(w5200, i);

            // Check RX data availability
            W5200_GetRxSize(w5200, i);

            // Check TX buffer availability
            W5200_GetTxFreeSize(w5200, i);
        }
    }
}

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Convert IP address from bytes to string
 */
void W5200_IPToString(const w5200_ip_addr_t* ip, char* str)
{
    if (ip == NULL || str == NULL) {
        return;
    }

    sprintf(str, "%d.%d.%d.%d", ip->bytes[0], ip->bytes[1], ip->bytes[2], ip->bytes[3]);
}

/**
 * @brief Convert MAC address from bytes to string
 */
void W5200_MACToString(const w5200_mac_addr_t* mac, char* str)
{
    if (mac == NULL || str == NULL) {
        return;
    }

    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X", mac->bytes[0], mac->bytes[1], mac->bytes[2], mac->bytes[3],
        mac->bytes[4], mac->bytes[5]);
}

/* ============================================================================
 * Private Function Implementations
 * ============================================================================ */

/**
 * @brief Initialize GPIO pins for W5200
 */
static void W5200_InitGPIO(w5200_instance_t* w5200)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    if (w5200->spi == SPI2) {
        // Configure SPI2 pins
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

        // SCK, MISO, MOSI
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        // NSS
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        // CS and Reset as outputs
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        // CS low, Reset off
        GPIOA->BSRRH = GPIO_Pin_11;
        GPIOA->BSRRH = GPIO_Pin_12;
    }
    else if (w5200->spi == SPI3) {
        // Configure SPI3 pins
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3);

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

        // SCK, MISO, MOSI
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        // NSS
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        // CS and Reset as outputs
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        // CS low, Reset off
        GPIOB->BSRRH = GPIO_Pin_3;
        GPIOB->BSRRH = GPIO_Pin_4;
    }
}

/**
 * @brief Initialize SPI peripheral for W5200
 */
static void W5200_InitSPI(w5200_instance_t* w5200)
{
    SPI_InitTypeDef SPI_InitStructure;

    // Enable SPI clock
    if (w5200->spi == SPI2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    }
    else if (w5200->spi == SPI3) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    }

    // Reset SPI
    SPI_I2S_DeInit(w5200->spi);

    // Configure SPI
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

    SPI_Init(w5200->spi, &SPI_InitStructure);

    SPI_Cmd(w5200->spi, DISABLE);
    SPI_SSOutputCmd(w5200->spi, ENABLE);
}

/**
 * @brief Initialize DMA for W5200
 */
static void W5200_InitDMA(w5200_instance_t* w5200)
{
    // DMA initialization would go here
    // This is a placeholder for future DMA implementation
    // The original code uses DMA1_Stream3/4 for SPI2 and DMA1_Stream2/7 for SPI3
}

/**
 * @brief Hardware reset of W5200 chip
 */
static void W5200_HardwareReset(w5200_instance_t* w5200)
{
    // Delay before reset
    W5200_Delay(50000);

    // Assert reset
    W5200_SetReset(w5200, true);

    // Hold reset for ~150ms
    for (int i = 0; i < 400; i++) {
        W5200_Delay(1000);
    }

    // Release reset
    W5200_SetReset(w5200, false);

    // Wait for chip to stabilize
    W5200_Delay(50000);
}

/**
 * @brief Software delay
 */
static void W5200_Delay(uint32_t cycles)
{
    for (uint32_t i = 0; i < cycles; i++) {
        __NOP();
    }
}

/**
 * @brief Set chip select state
 */
static void W5200_SetCS(w5200_instance_t* w5200, bool state)
{
    if (state) {
        w5200->cs_port->BSRRL = w5200->cs_pin;  // Set high
    }
    else {
        w5200->cs_port->BSRRH = w5200->cs_pin;  // Set low
    }
}

/**
 * @brief Set reset pin state
 */
static void W5200_SetReset(w5200_instance_t* w5200, bool state)
{
    if (state) {
        w5200->reset_port->BSRRL = w5200->reset_pin;  // Set high (reset active)
    }
    else {
        w5200->reset_port->BSRRH = w5200->reset_pin;  // Set low (reset inactive)
    }
}

/**
 * @brief Configure common W5200 registers
 */
static void W5200_ConfigureCommon(w5200_instance_t* w5200)
{
    // Configure TX/RX memory size for each socket (8KB each)
    for (uint8_t i = 0; i < W5200_ACTIVE_SOCKETS; i++) {
        W5200_WriteByte(w5200, W5200_Sn_RXMEM_SIZE(i), 0x03);  // 8KB
        W5200_WriteByte(w5200, W5200_Sn_TXMEM_SIZE(i), 0x03);  // 8KB
    }

    // Set default retry time (400ms = 0x0FA0)
    W5200_WriteByte(w5200, W5200_RTR, 0x0F);
    W5200_WriteByte(w5200, W5200_RTR + 1, 0xA0);
}

/**
 * @brief Get TX buffer base address for socket
 */
static uint16_t W5200_GetSocketTxBase(uint8_t socket_num)
{
    return W5200_TX_MEM_BASE + (socket_num * W5200_SOCKET_BUF_SIZE);
}

/**
 * @brief Get RX buffer base address for socket
 */
static uint16_t W5200_GetSocketRxBase(uint8_t socket_num)
{
    return W5200_RX_MEM_BASE + (socket_num * W5200_SOCKET_BUF_SIZE);
}
