/**
 ******************************************************************************
 * @file    fiber_optic.h
 * @author  Techperts Team
 * @version V1.0.0
 * @date    18-Oct-2025
 * @brief   Fiber-Optic Communication Driver Header
 ******************************************************************************
 * @attention
 *
 * This driver provides a hardware abstraction layer for 6-channel fiber-optic
 * communication via FPGA and FSMC Bank 4. The FPGA handles the actual optical
 * transceiver interface and must be configured before use.
 *
 * Hardware Architecture:
 * - STM32 <--FSMC Bank 4--> FPGA <--Optical Interface--> Fiber Transceivers
 * - Base Address: 0x6C000000 (FSMC Bank 4)
 * - 6 independent channels + broadcast modes
 * - Configurable baud rate up to 1,382,400 baud
 *
 * Usage:
 * 1. Call FiberOptic_Init() once during system startup
 * 2. Use FiberOptic_Transmit() to send data on any channel
 * 3. Use FiberOptic_Receive() to read data from any channel
 * 4. Use FiberOptic_DataAvailable() to check for incoming data
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FIBER_OPTIC_H
#define __FIBER_OPTIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

/**
 * @brief FSMC Bank 4 base address for FPGA optical interface
 */
#define FIBER_OPTIC_BASE_ADDR 0x6C000000

/**
 * @brief Maximum data length per transmission
 */
#define FIBER_OPTIC_MAX_LENGTH 256

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Fiber-optic channel selection
 */
typedef enum {
    FIBER_CH1 = 1,               ///< Channel 1
    FIBER_CH2 = 2,               ///< Channel 2
    FIBER_CH3 = 3,               ///< Channel 3
    FIBER_CH4 = 4,               ///< Channel 4
    FIBER_CH5 = 5,               ///< Channel 5
    FIBER_CH6 = 6,               ///< Channel 6
    FIBER_BROADCAST_ALL = 0,     ///< Broadcast to channels 1-6
    FIBER_BROADCAST_1234 = 100,  ///< Broadcast to channels 1-4
    FIBER_BROADCAST_56 = 101     ///< Broadcast to channels 5-6
} fiber_channel_t;

/**
 * @brief Fiber-optic instance structure
 */
typedef struct {
    uint32_t baud_rate;  ///< Configured baud rate
    uint8_t node_id;     ///< Device node ID (from DIP switches)
    bool initialized;    ///< Initialization status flag
} fiber_optic_instance_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize fiber-optic communication system
 * @param  instance: Pointer to fiber-optic instance structure
 * @param  baud_rate: Communication baud rate (max 1,382,400)
 * @param  node_id: Device node ID (0-15, typically from DIP switches)
 * @retval true if initialization successful, false otherwise
 * @note   CRITICAL: This function assumes FPGA is already initialized!
 *         Must call FPGA_Init_Pins() and FPGA_Configure() BEFORE this function.
 *         Typically called from driver_init.c after FPGA initialization.
 *
 *         This function:
 *         - Configures FSMC Bank 4 for FPGA access
 *         - Sets baud rate divider register
 *         - Sets node ID register
 */
bool FiberOptic_Init(fiber_optic_instance_t* instance, uint32_t baud_rate, uint8_t node_id);

/**
 * @brief  Transmit data on specified channel
 * @param  instance: Pointer to fiber-optic instance structure
 * @param  channel: Target channel (FIBER_CH1..FIBER_CH6 or broadcast modes)
 * @param  data: Pointer to data buffer to transmit
 * @param  length: Number of bytes to transmit
 * @retval true if transmission successful, false otherwise
 * @note   Data is written byte-by-byte to FPGA TX FIFO.
 *         Maximum length is FIBER_OPTIC_MAX_LENGTH.
 */
bool FiberOptic_Transmit(
    fiber_optic_instance_t* instance, fiber_channel_t channel, const uint8_t* data, uint16_t length);

/**
 * @brief  Receive data from specified channel
 * @param  instance: Pointer to fiber-optic instance structure
 * @param  channel: Source channel (FIBER_CH1..FIBER_CH6)
 * @param  buffer: Pointer to buffer to store received data
 * @param  max_length: Maximum number of bytes to receive
 * @retval Number of bytes actually received (0 if no data available)
 * @note   Reads data from FPGA RX FIFO until empty or buffer full.
 *         Non-blocking - returns immediately if no data available.
 */
uint16_t FiberOptic_Receive(
    fiber_optic_instance_t* instance, fiber_channel_t channel, uint8_t* buffer, uint16_t max_length);

/**
 * @brief  Check if data is available on specified channel
 * @param  instance: Pointer to fiber-optic instance structure
 * @param  channel: Channel to check (FIBER_CH1..FIBER_CH6)
 * @retval true if data available, false otherwise
 * @note   Checks RX empty flag (bit 1) in channel status register.
 *         Use before calling FiberOptic_Receive() to avoid unnecessary reads.
 */
bool FiberOptic_DataAvailable(fiber_optic_instance_t* instance, fiber_channel_t channel);

/**
 * @brief  Get channel status register value
 * @param  instance: Pointer to fiber-optic instance structure
 * @param  channel: Channel to query (FIBER_CH1..FIBER_CH6)
 * @retval 16-bit status value from FPGA
 * @note   Status bits (from archive code):
 *         - Bit 1: RX buffer empty (1 = empty, 0 = data available)
 *         Other bits are FPGA-specific (see hardware documentation)
 */
uint16_t FiberOptic_GetChannelStatus(fiber_optic_instance_t* instance, fiber_channel_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __FIBER_OPTIC_H */

/******* (C) COPYRIGHT 2025 Techperts *****END OF FILE****/
