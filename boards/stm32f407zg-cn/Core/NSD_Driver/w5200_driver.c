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

#include "tasks.h"  // For get_1ms_counter()

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
static bool W5200_WaitCommandComplete(w5200_instance_t* w5200, uint8_t socket_num, uint32_t timeout_cycles);

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

    // Validate DMA size (0 = no DMA, or valid DMA size)
    if (dma_size != 0 && dma_size != W5200_DMA_SIZE_128 && dma_size != W5200_DMA_SIZE_256 &&
        dma_size != W5200_DMA_SIZE_512 && dma_size != W5200_DMA_SIZE_1K && dma_size != W5200_DMA_SIZE_2K &&
        dma_size != W5200_DMA_SIZE_4K && dma_size != W5200_DMA_SIZE_8K) {
        return false;
    }

    // Clear instance structure
    memset(w5200, 0, sizeof(w5200_instance_t));

    // Store hardware configuration
    w5200->spi = spi;
    w5200->tx_buffer = tx_buffer;
    w5200->rx_buffer = rx_buffer;
    w5200->dma_size = dma_size;

    // Initialize DMA state
    w5200->dma_state = W5200_STATE_IDLE;
    w5200->dma_socket = 0xFF;  // Invalid socket (none selected)
    w5200->dma_tx_delay = 0;
    w5200->dma_rx_delay = 0;
    w5200->dma_rx_size = 0;

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
    bool timeout_occurred = false;

    // Enable SPI for this transaction (legacy pattern)
    SPI_Cmd(w5200->spi, ENABLE);

    // Assert CS (pull low to select chip)
    W5200_SetCS(w5200, false);

    // Send register address (high byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)(addr >> 8));
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: addr high byte TX timeout
    }

    // Send register address (low byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)addr);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: addr low byte TX timeout
    }

    // Send WRITE command (0x80 | 0x01)
    SPI_I2S_SendData(w5200->spi, 0x80);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: command high byte TX timeout
    }

    SPI_I2S_SendData(w5200->spi, 0x01);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: command low byte TX timeout
    }

    // Write data byte
    SPI_I2S_SendData(w5200->spi, data);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: data byte TX timeout
    }

    // Wait for the last byte to be fully transmitted before de-asserting CS
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_BSY) == SET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }

    // De-assert CS (pull high to deselect chip)
    W5200_SetCS(w5200, true);

    // Disable SPI after transaction (legacy pattern)
    SPI_Cmd(w5200->spi, DISABLE);
    W5200_Delay(2);

    if (timeout_occurred) {
        // Breakpoint here: SPI timeout occurred during write
        // Inspect: addr, data, timeout_occurred
        __NOP();
    }
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
    bool timeout_occurred = false;

    // Enable SPI for this transaction (legacy pattern)
    SPI_Cmd(w5200->spi, ENABLE);

    // Assert CS (pull low to select chip)
    W5200_SetCS(w5200, false);

    // Send register address (high byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)(addr >> 8));
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: addr high byte TX timeout
    }

    // Send register address (low byte)
    SPI_I2S_SendData(w5200->spi, (uint8_t)addr);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: addr low byte TX timeout
    }

    // Send READ command (0x00 | 0x01)
    SPI_I2S_SendData(w5200->spi, 0x00);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: command high byte TX timeout
    }

    SPI_I2S_SendData(w5200->spi, 0x01);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_TXE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: command low byte TX timeout
    }

    // Wait for response
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_RXNE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: first RX timeout
    }
    result = SPI_I2S_ReceiveData(w5200->spi);

    // Read data byte
    SPI_I2S_SendData(w5200->spi, 0x00);
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_RXNE) == RESET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }
    if (timeout >= W5200_SPI_TIMEOUT) {
        timeout_occurred = true;  // Breakpoint here: data byte RX timeout
    }
    result = SPI_I2S_ReceiveData(w5200->spi);

    // Wait for the last byte to be fully transmitted before de-asserting CS
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_BSY) == SET) && timeout < W5200_SPI_TIMEOUT) {
        timeout++;
    }

    // De-assert CS (pull high to deselect chip)
    W5200_SetCS(w5200, true);

    // Disable SPI after transaction (legacy pattern)
    SPI_Cmd(w5200->spi, DISABLE);
    W5200_Delay(2);

    if (timeout_occurred) {
        // Breakpoint here: SPI timeout occurred during read
        // Inspect: addr, result, timeout_occurred
        __NOP();
    }

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

    // Enable SPI for this transaction (legacy pattern)
    SPI_Cmd(w5200->spi, ENABLE);

    // Assert CS (pull low to select chip)
    W5200_SetCS(w5200, false);

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

    // Wait for the last byte to be fully transmitted before de-asserting CS
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_BSY) == SET) && timeout++ < W5200_SPI_TIMEOUT);

    // De-assert CS (pull high to deselect chip)
    W5200_SetCS(w5200, true);

    // Disable SPI after transaction (legacy pattern)
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

    // Enable SPI for this transaction (legacy pattern)
    SPI_Cmd(w5200->spi, ENABLE);

    // Assert CS (pull low to select chip)
    W5200_SetCS(w5200, false);

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

    // Wait for the last byte to be fully transmitted before de-asserting CS
    timeout = 0;
    while ((SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_BSY) == SET) && timeout++ < W5200_SPI_TIMEOUT);

    // De-assert CS (pull high to deselect chip)
    W5200_SetCS(w5200, true);

    // Disable SPI after transaction (legacy pattern)
    SPI_Cmd(w5200->spi, DISABLE);
    W5200_Delay(2);
}

/* ============================================================================
 * Public Function Implementations - Socket Operations
 * ============================================================================ */

/**
 * @brief Open a socket
 * @note Matches archive legacy behavior - no polling, immediate status check
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

    uint8_t status;

    // Set socket mode (TCP or UDP)
    uint8_t mode = (w5200->sockets[socket_num].protocol == W5200_PROTO_TCP) ? W5200_Sn_MR_TCP : W5200_Sn_MR_UDP;
    W5200_WriteByte(w5200, W5200_Sn_MR(socket_num), mode);

    // Open socket
    W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_OPEN);

    // Archive behavior: immediately check status (no waiting/polling)
    status = W5200_ReadByte(w5200, W5200_Sn_SR(socket_num));

    // Verify socket opened with correct status
    if (w5200->sockets[socket_num].protocol == W5200_PROTO_TCP) {
        if (status != W5200_SOCK_INIT) {
            // Failed to open, close it
            W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_CLOSE);
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
        // UDP socket should be in UDP state (0x22)
        if (status != W5200_SOCK_UDP) {
            W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_CLOSE);
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
 * @note Matches archive legacy behavior - immediate status check
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

    // Archive behavior: immediately check status (no delay)
    uint8_t status = W5200_ReadByte(w5200, W5200_Sn_SR(socket_num));

    if (status != W5200_SOCK_LISTEN) {
        // Failed to listen, close socket
        W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_CLOSE);
        return false;
    }

    w5200->socket_state[socket_num].status = status;
    return true;
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

    // Write data to TX buffer, handling wrap-around within ring buffer
    uint16_t remaining = W5200_SOCKET_BUF_SIZE - offset;
    if (length <= remaining) {
        W5200_Write(w5200, physical_addr, data, length);
    }
    else {
        W5200_Write(w5200, physical_addr, data, remaining);
        W5200_Write(w5200, tx_base, data + remaining, length - remaining);
    }

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

    // Read data from RX buffer, handling wrap-around within ring buffer
    uint16_t remaining = W5200_SOCKET_BUF_SIZE - offset;
    if (read_size <= remaining) {
        W5200_Read(w5200, physical_addr, data, read_size);
    }
    else {
        W5200_Read(w5200, physical_addr, data, remaining);
        W5200_Read(w5200, rx_base, data + remaining, read_size - remaining);
    }

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
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @param data Pointer to data to send
 * @param length Length of data to send
 * @retval true if DMA transfer initiated successfully
 * @retval false if DMA busy or parameters invalid
 *
 * @note This function initiates a DMA transfer. Call W5200_UpdateDMA()
 *       periodically to poll for completion.
 */
bool W5200_SendDMA(w5200_instance_t* w5200, uint8_t socket_num, const uint8_t* data, uint16_t length)
{
    if (w5200 == NULL || !w5200->initialized || data == NULL || length == 0) {
        return false;
    }

    // Check if DMA is disabled (dma_size == 0)
    if (w5200->dma_size == 0) {
        // Fall back to regular send
        return W5200_Send(w5200, socket_num, data, length);
    }

    // Check if length exceeds DMA buffer size
    if (length > w5200->dma_size) {
        return false;  // Data too large for DMA buffer
    }

    // Check if DMA is already busy
    if (w5200->dma_state != W5200_STATE_IDLE) {
        return false;  // DMA busy
    }

    // Get TX write pointer from W5200 registers
    uint16_t tx_wr = W5200_ReadByte(w5200, W5200_Sn_TX_WR0(socket_num)) << 8;
    tx_wr |= W5200_ReadByte(w5200, W5200_Sn_TX_WR0(socket_num) + 1);

    // Fallback to CPU transfer if write would wrap the ring buffer
    uint16_t offset = tx_wr % W5200_SOCKET_BUF_SIZE;
    if ((uint32_t)offset + length > W5200_SOCKET_BUF_SIZE) {
        return W5200_Send(w5200, socket_num, data, length);
    }

    // Calculate socket TX buffer address
    uint16_t socket_tx_base = W5200_TX_MEM_BASE + (socket_num * W5200_SOCKET_BUF_SIZE);
    uint16_t tx_addr = socket_tx_base + offset;

    // Build W5200 SPI command header in TX buffer
    // Header format: [ADDR_HIGH][ADDR_LOW][OPCODE|LENGTH_HIGH][LENGTH_LOW]
    w5200->tx_buffer[0] = (uint8_t)(tx_addr >> 8);          // Address high byte
    w5200->tx_buffer[1] = (uint8_t)(tx_addr & 0xFF);        // Address low byte
    w5200->tx_buffer[2] = (uint8_t)((length >> 8) | 0x80);  // Write opcode + length high
    w5200->tx_buffer[3] = (uint8_t)(length & 0xFF);         // Length low byte

    // Copy user data after header
    for (uint16_t i = 0; i < length; i++) {
        w5200->tx_buffer[4 + i] = data[i];
    }

    // Update DMA state
    w5200->dma_state = W5200_STATE_SENDING;
    w5200->dma_socket = socket_num;
    w5200->dma_tx_delay = 0;

    // Store TX info for completion handling
    w5200->socket_state[socket_num].tx_size = length;
    w5200->socket_state[socket_num].tx_addr_offset = tx_wr;

    // Enable SPI and start DMA transfer
    SPI_Cmd(w5200->spi, ENABLE);

    if (w5200->spi == SPI2) {
        // Update DMA transfer size and start
        DMA1_Stream4->NDTR = length + 4;  // Data + 4-byte header
        DMA_Cmd(DMA1_Stream4, ENABLE);
    }
    else if (w5200->spi == SPI3) {
        // Update DMA transfer size and start
        DMA1_Stream7->NDTR = length + 4;  // Data + 4-byte header
        DMA_Cmd(DMA1_Stream7, ENABLE);
    }

    return true;
}

/**
 * @brief Receive data using DMA
 * @param w5200 Pointer to W5200 instance
 * @param socket_num Socket number (0-7)
 * @retval true if DMA transfer initiated successfully
 * @retval false if DMA busy or no data available
 *
 * @note This function initiates a DMA receive. Call W5200_UpdateDMA()
 *       periodically to poll for completion. Data will be in rx_buffer
 *       starting at offset 4 (after protocol header).
 */
bool W5200_ReceiveDMA(w5200_instance_t* w5200, uint8_t socket_num)
{
    if (w5200 == NULL || !w5200->initialized) {
        return false;
    }

    // Check if DMA is disabled
    if (w5200->dma_size == 0) {
        return false;  // DMA not available
    }

    // Check if DMA is already busy
    if (w5200->dma_state != W5200_STATE_IDLE) {
        return false;  // DMA busy
    }

    // Read received data size from W5200
    uint16_t rx_size = W5200_ReadByte(w5200, W5200_Sn_RX_RSR0(socket_num)) << 8;
    rx_size |= W5200_ReadByte(w5200, W5200_Sn_RX_RSR0(socket_num) + 1);

    if (rx_size == 0) {
        return false;  // No data to receive
    }

    // Limit to DMA buffer size
    if (rx_size > w5200->dma_size) {
        rx_size = w5200->dma_size;
    }

    // Get RX read pointer from W5200 registers
    uint16_t rx_rd = W5200_ReadByte(w5200, W5200_Sn_RX_RD0(socket_num)) << 8;
    rx_rd |= W5200_ReadByte(w5200, W5200_Sn_RX_RD0(socket_num) + 1);

    // Calculate socket RX buffer address
    uint16_t socket_rx_base = W5200_RX_MEM_BASE + (socket_num * W5200_SOCKET_BUF_SIZE);
    uint16_t offset = rx_rd % W5200_SOCKET_BUF_SIZE;
    uint16_t rx_addr = socket_rx_base + offset;

    // Build W5200 SPI read command header in TX buffer
    // (SPI is full-duplex, so we transmit read command while receiving data)
    w5200->tx_buffer[0] = (uint8_t)(rx_addr >> 8);    // Address high byte
    w5200->tx_buffer[1] = (uint8_t)(rx_addr & 0xFF);  // Address low byte
    w5200->tx_buffer[2] = (uint8_t)(rx_size >> 8);    // Read opcode (0x00) + length high
    w5200->tx_buffer[3] = (uint8_t)(rx_size & 0xFF);  // Length low byte

    // Fill rest of TX buffer with dummy bytes (for clock generation)
    for (uint16_t i = 0; i < rx_size; i++) {
        w5200->tx_buffer[4 + i] = 0x00;
    }

    // Update DMA state
    w5200->dma_state = W5200_STATE_RECEIVING;
    w5200->dma_socket = socket_num;
    w5200->dma_rx_delay = 0;
    w5200->dma_rx_size = rx_size;

    // Store RX info for completion handling
    w5200->socket_state[socket_num].rx_size = rx_size;
    w5200->socket_state[socket_num].rx_addr_offset = rx_rd;

    // Enable SPI RX DMA (always enabled in circular mode, but ensure it's on)
    SPI_I2S_DMACmd(w5200->spi, SPI_I2S_DMAReq_Rx, ENABLE);

    // Enable SPI and start DMA transfer
    SPI_Cmd(w5200->spi, ENABLE);

    if (w5200->spi == SPI2) {
        // Update DMA transfer size and start
        DMA1_Stream4->NDTR = rx_size + 4;  // Data + 4-byte header
        DMA_Cmd(DMA1_Stream4, ENABLE);
    }
    else if (w5200->spi == SPI3) {
        // Update DMA transfer size and start
        DMA1_Stream7->NDTR = rx_size + 4;  // Data + 4-byte header
        DMA_Cmd(DMA1_Stream7, ENABLE);
    }

    return true;
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

/**
 * @brief Update DMA state and handle completion
 * @param w5200 Pointer to W5200 instance
 * @note Call this function periodically (e.g., from main loop) to poll for
 *       DMA completion and handle timeouts.
 */
void W5200_UpdateDMA(w5200_instance_t* w5200)
{
    if (w5200 == NULL || !w5200->initialized) {
        return;
    }

    // Skip if DMA not enabled
    if (w5200->dma_size == 0) {
        return;
    }

    // Handle TX DMA completion
    if (w5200->dma_state == W5200_STATE_SENDING) {
        bool dma_complete = false;
        DMA_Stream_TypeDef* tx_stream = (w5200->spi == SPI2) ? DMA1_Stream4 : DMA1_Stream7;
        uint32_t tcif_flag = (w5200->spi == SPI2) ? DMA_FLAG_TCIF4 : DMA_FLAG_TCIF7;

        // Check if DMA transfer complete
        if (DMA_GetFlagStatus(tx_stream, tcif_flag) == SET) {
            // Wait for SPI to finish (BSY flag clear)
            if (SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_BSY) == RESET) {
                dma_complete = true;

                // Clear DMA flag
                DMA_ClearFlag(tx_stream, tcif_flag);

                // Disable SPI
                SPI_Cmd(w5200->spi, DISABLE);

                // Update W5200 TX write pointer
                uint8_t socket_num = w5200->dma_socket;
                uint16_t tx_wr =
                    w5200->socket_state[socket_num].tx_addr_offset + w5200->socket_state[socket_num].tx_size;

                W5200_WriteByte(w5200, W5200_Sn_TX_WR0(socket_num), (tx_wr >> 8));
                W5200_WriteByte(w5200, W5200_Sn_TX_WR0(socket_num) + 1, (tx_wr & 0xFF));

                // Send SEND command to W5200
                W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_SEND);

                // Reset DMA state
                w5200->dma_state = W5200_STATE_IDLE;
                w5200->dma_socket = 0xFF;
                w5200->dma_tx_delay = 0;
            }
        }

        // Check for timeout (100ms @ 1kHz call rate)
        if (!dma_complete) {
            if (++w5200->dma_tx_delay >= 100) {
                // Timeout - abort DMA and reset socket
                DMA_Cmd(tx_stream, DISABLE);
                SPI_Cmd(w5200->spi, DISABLE);

                // Reset socket state
                uint8_t socket_num = w5200->dma_socket;
                W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_CLOSE);
                w5200->socket_state[socket_num].state = W5200_STATE_IDLE;
                w5200->socket_state[socket_num].status = W5200_SOCK_CLOSED;

                // Reset DMA state
                w5200->dma_state = W5200_STATE_IDLE;
                w5200->dma_socket = 0xFF;
                w5200->dma_tx_delay = 0;
            }
        }
    }

    // Handle RX DMA completion
    else if (w5200->dma_state == W5200_STATE_RECEIVING) {
        bool dma_complete = false;
        DMA_Stream_TypeDef* tx_stream = (w5200->spi == SPI2) ? DMA1_Stream4 : DMA1_Stream7;
        uint32_t tcif_flag = (w5200->spi == SPI2) ? DMA_FLAG_TCIF4 : DMA_FLAG_TCIF7;

        // Check if DMA transfer complete (TX complete flag, since RX follows TX)
        if (DMA_GetFlagStatus(tx_stream, tcif_flag) == SET) {
            // Wait for SPI to finish
            if (SPI_I2S_GetFlagStatus(w5200->spi, SPI_I2S_FLAG_BSY) == RESET) {
                dma_complete = true;

                // Clear DMA flag
                DMA_ClearFlag(tx_stream, tcif_flag);

                // Disable SPI
                SPI_Cmd(w5200->spi, DISABLE);

                // Data is now in rx_buffer[4..rx_size+3]
                // (First 4 bytes are SPI protocol overhead)

                // Update W5200 RX read pointer
                uint8_t socket_num = w5200->dma_socket;
                uint16_t rx_rd = w5200->socket_state[socket_num].rx_addr_offset + w5200->dma_rx_size;

                W5200_WriteByte(w5200, W5200_Sn_RX_RD0(socket_num), (rx_rd >> 8));
                W5200_WriteByte(w5200, W5200_Sn_RX_RD0(socket_num) + 1, (rx_rd & 0xFF));

                // Send RECV command to W5200
                W5200_WriteByte(w5200, W5200_Sn_CR(socket_num), W5200_Sn_CR_RECV);

                // Mark RX data ready
                w5200->socket_state[socket_num].rx_ready = true;

                // Reset DMA state
                w5200->dma_state = W5200_STATE_IDLE;
                w5200->dma_socket = 0xFF;
                w5200->dma_rx_delay = 0;
            }
        }

        // Check for timeout (100ms @ 1kHz call rate)
        if (!dma_complete) {
            if (++w5200->dma_rx_delay >= 100) {
                // Timeout - abort DMA and reset
                DMA_Cmd(tx_stream, DISABLE);
                SPI_Cmd(w5200->spi, DISABLE);

                // Reset DMA state
                w5200->dma_state = W5200_STATE_IDLE;
                w5200->dma_socket = 0xFF;
                w5200->dma_rx_delay = 0;
            }
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

        // Set initial state according to legacy option
        // Reset pin should start HIGH (released). CS legacy behavior may keep it LOW.
        if (W5200_LEGACY_CS_IDLE_LOW) {
            // CS LOW (asserted), Reset HIGH
            GPIOA->BSRRH = GPIO_Pin_11;  // CS low
            GPIOA->BSRRL = GPIO_Pin_12;  // Reset high
        }
        else {
            GPIOA->BSRRL = GPIO_Pin_11 | GPIO_Pin_12;  // CS high, Reset high
        }
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

        if (W5200_LEGACY_CS_IDLE_LOW) {
            GPIOB->BSRRH = GPIO_Pin_3;  // CS low
            GPIOB->BSRRL = GPIO_Pin_4;  // Reset high
        }
        else {
            GPIOB->BSRRL = GPIO_Pin_3 | GPIO_Pin_4;  // CS high, Reset high
        }
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
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;  // Hardware NSS to match legacy behavior
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

    SPI_Init(w5200->spi, &SPI_InitStructure);

    // Legacy pattern: Disable SPI initially, enable SPI_SSOutput, will enable/disable per transaction
    SPI_Cmd(w5200->spi, DISABLE);
    SPI_SSOutputCmd(w5200->spi, ENABLE);
}

/**
 * @brief Initialize DMA for W5200
 */
static void W5200_InitDMA(w5200_instance_t* w5200)
{
    DMA_InitTypeDef DMA_InitStructureTx, DMA_InitStructureRx;

    // Skip if DMA not enabled
    if (w5200->dma_size == 0) {
        return;
    }

    // Enable DMA1 clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    if (w5200->spi == SPI2) {
        // SPI2 TX: DMA1 Stream 4, Channel 0
        DMA_DeInit(DMA1_Stream4);
        DMA_InitStructureTx.DMA_Channel = DMA_Channel_0;
        DMA_InitStructureTx.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
        DMA_InitStructureTx.DMA_Memory0BaseAddr = (uint32_t)w5200->tx_buffer;
        DMA_InitStructureTx.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructureTx.DMA_BufferSize = w5200->dma_size + 4;
        DMA_InitStructureTx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructureTx.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructureTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructureTx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructureTx.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructureTx.DMA_Priority = DMA_Priority_Low;
        DMA_InitStructureTx.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructureTx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructureTx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructureTx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream4, &DMA_InitStructureTx);

        // Enable SPI2 DMA TX
        SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
        DMA_Cmd(DMA1_Stream4, DISABLE);

        // SPI2 RX: DMA1 Stream 3, Channel 0
        DMA_DeInit(DMA1_Stream3);
        DMA_StructInit(&DMA_InitStructureRx);
        DMA_InitStructureRx.DMA_Channel = DMA_Channel_0;
        DMA_InitStructureRx.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
        DMA_InitStructureRx.DMA_Memory0BaseAddr = (uint32_t)w5200->rx_buffer;
        DMA_InitStructureRx.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructureRx.DMA_BufferSize = w5200->dma_size + 4;
        DMA_InitStructureRx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructureRx.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructureRx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructureRx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructureRx.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructureRx.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructureRx.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructureRx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructureRx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructureRx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream3, &DMA_InitStructureRx);

        DMA_Cmd(DMA1_Stream3, ENABLE);
    }
    else if (w5200->spi == SPI3) {
        // SPI3 TX: DMA1 Stream 7, Channel 0
        DMA_DeInit(DMA1_Stream7);
        DMA_InitStructureTx.DMA_Channel = DMA_Channel_0;
        DMA_InitStructureTx.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;
        DMA_InitStructureTx.DMA_Memory0BaseAddr = (uint32_t)w5200->tx_buffer;
        DMA_InitStructureTx.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructureTx.DMA_BufferSize = w5200->dma_size + 4;
        DMA_InitStructureTx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructureTx.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructureTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructureTx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructureTx.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructureTx.DMA_Priority = DMA_Priority_Low;
        DMA_InitStructureTx.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructureTx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructureTx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructureTx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream7, &DMA_InitStructureTx);

        // Enable SPI3 DMA TX
        SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
        DMA_Cmd(DMA1_Stream7, DISABLE);

        // SPI3 RX: DMA1 Stream 2, Channel 0
        DMA_DeInit(DMA1_Stream2);
        DMA_StructInit(&DMA_InitStructureRx);
        DMA_InitStructureRx.DMA_Channel = DMA_Channel_0;
        DMA_InitStructureRx.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;
        DMA_InitStructureRx.DMA_Memory0BaseAddr = (uint32_t)w5200->rx_buffer;
        DMA_InitStructureRx.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructureRx.DMA_BufferSize = w5200->dma_size + 4;
        DMA_InitStructureRx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructureRx.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructureRx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructureRx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructureRx.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructureRx.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructureRx.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructureRx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructureRx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructureRx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream2, &DMA_InitStructureRx);

        DMA_Cmd(DMA1_Stream2, ENABLE);
    }
}

/**
 * @brief Hardware reset of W5200 chip
 */
static void W5200_HardwareReset(w5200_instance_t* w5200)
{
    uint32_t start_time;

    // Small delay before reset (ensure previous operations complete)
    W5200_Delay(100);  // ~100 CPU cycles for settling

    // Assert reset (active LOW - pull pin LOW to reset chip)
    W5200_SetReset(w5200, false);

    // Hold reset for 150ms using timer
    start_time = get_1ms_counter();
    while ((get_1ms_counter() - start_time) < 150) {
        __NOP();
    }

    // Release reset (pull pin HIGH to run chip)
    W5200_SetReset(w5200, true);

    // Wait for chip to stabilize (at least 50ms per datasheet)
    start_time = get_1ms_counter();
    while ((get_1ms_counter() - start_time) < 100) {  // 100ms for safety
        __NOP();
    }
}

/**
 * @brief Software delay (very short delays only, uses CPU cycles)
 * @note For delays > 1ms, use timer-based delays instead
 */
static void W5200_Delay(uint32_t cycles)
{
    volatile uint32_t i;  // Volatile prevents optimizer from removing loop
    for (i = 0; i < cycles; i++) {
        __NOP();
    }
}

/**
 * @brief Set chip select state
 */
static void W5200_SetCS(w5200_instance_t* w5200, bool state)
{
    if (state) {
        if (W5200_LEGACY_CS_IDLE_LOW) {
            // Legacy mode keeps CS asserted low even when 'state' requests de-assert.
            // Still, provide a short high pulse possibility if needed in future.
            // For now: do nothing (remain low).
        }
        else {
            w5200->cs_port->BSRRL = w5200->cs_pin;  // Set high (inactive)
        }
    }
    else {
        w5200->cs_port->BSRRH = w5200->cs_pin;  // Set low (active)
    }
}

/**
 * @brief Set reset pin state (W5200 reset is active-LOW)
 */
static void W5200_SetReset(w5200_instance_t* w5200, bool state)
{
    if (state) {
        // De-assert reset (run mode) - pull HIGH
        w5200->reset_port->BSRRL = w5200->reset_pin;
    }
    else {
        // Assert reset (hold in reset) - pull LOW
        w5200->reset_port->BSRRH = w5200->reset_pin;
    }
}

/**
 * @brief Configure common W5200 registers
 */
static void W5200_ConfigureCommon(w5200_instance_t* w5200)
{
    // Configure TX/RX memory size for each socket (8KB each)
    // Note: 0x08 = 8KB per socket (matching original implementation)
    for (uint8_t i = 0; i < W5200_ACTIVE_SOCKETS; i++) {
        W5200_WriteByte(w5200, W5200_Sn_RXMEM_SIZE(i), 0x08);  // 8KB
        W5200_WriteByte(w5200, W5200_Sn_TXMEM_SIZE(i), 0x08);  // 8KB
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

/**
 * @brief Wait for W5200 command to complete
 * @note The command register (Sn_CR) is cleared to 0x00 by W5200 when command completes
 */
inline bool W5200_WaitCommandComplete(w5200_instance_t* w5200, uint8_t socket_num, uint32_t timeout_cycles)
{
    while (timeout_cycles-- > 0) {
        uint8_t cmd_reg = W5200_ReadByte(w5200, W5200_Sn_CR(socket_num));
        if (cmd_reg == 0x00) {
            return true;  // Command completed
        }
        W5200_Delay(10);
    }
    return false;  // Timeout
}
