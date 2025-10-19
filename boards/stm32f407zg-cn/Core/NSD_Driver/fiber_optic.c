/**
 ******************************************************************************
 * @file    fiber_optic.c
 * @author  Techperts Team
 * @version V1.0.0
 * @date    18-Oct-2025
 * @brief   Fiber-Optic Communication Driver Implementation
 ******************************************************************************
 * @attention
 *
 * This driver provides hardware-level access to 6-channel fiber-optic
 * communication via FPGA and FSMC Bank 4.
 *
 * Memory Map (FPGA registers via FSMC Bank 4):
 * - Base: 0x6C000000
 * - 0x04: Baud rate divisor register
 * - 0x10: Node ID register
 * - Channel N data: Base + (N * 2)
 * - Channel N status: Base + ((N + 100) * 2)
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "fiber_optic.h"

#include "driver_init.h"     // For dip_switch_channel_number
#include "fpga.h"            // For FPGA configuration
#include "stm32f4xx.h"       // For __IO definition
#include "stm32f4xx_fsmc.h"  // For FSMC peripheral definitions
#include "stm32f4xx_rcc.h"   // For RCC clock control

/* Private defines -----------------------------------------------------------*/

/**
 * @brief Maximum baud rate supported by FPGA hardware
 */
#define FIBER_OPTIC_MAX_BAUD_RATE 1382400

/**
 * @brief FPGA register offsets (in 16-bit words)
 */
#define FPGA_REG_BAUD_DIVISOR 0x04  ///< Baud rate divisor register offset
#define FPGA_REG_NODE_ID      16    ///< Node ID register offset

/**
 * @brief Channel status register bits
 */
#define CHANNEL_STATUS_RX_EMPTY 0x0002  ///< Bit 1: RX buffer empty

/* Private function prototypes -----------------------------------------------*/
static void FiberOptic_FSMC_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize FSMC Bank 4 for FPGA optical interface access
 * @note   Configures FSMC timing parameters for 16-bit FPGA register access
 */
static void FiberOptic_FSMC_Init(void)
{
    FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef timing;

    // Enable FSMC clock
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

    // Configure timing (from legacy optical_fiber.c)
    // These values are tuned for reliable FPGA communication
    timing.FSMC_AddressSetupTime = 6;
    timing.FSMC_AddressHoldTime = 15;
    timing.FSMC_DataSetupTime = 21;
    timing.FSMC_BusTurnAroundDuration = 1;
    timing.FSMC_CLKDivision = 1;
    timing.FSMC_DataLatency = 0;
    timing.FSMC_AccessMode = FSMC_AccessMode_A;

    // Configure FSMC Bank 4 for FPGA access
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_PSRAM;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &timing;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &timing;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

    // Enable FSMC Bank 4
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize fiber-optic communication system
 */
bool FiberOptic_Init(fiber_optic_instance_t* instance, uint32_t baud_rate)
{
    uint32_t baud_divisor;

    if (!instance || baud_rate == 0 || baud_rate > FIBER_OPTIC_MAX_BAUD_RATE) {
        return false;
    }

    // NOTE: FPGA must already be initialized by driver_init.c before calling this function
    // This function assumes FPGA_Init_Pins() and FPGA_Configure() were already called

    // Step 1: Configure FSMC Bank 4 for FPGA access
    FiberOptic_FSMC_Init();

    // Step 2: Configure baud rate
    // Formula: divisor = 1,382,400 / desired_baud_rate
    baud_divisor = FIBER_OPTIC_MAX_BAUD_RATE / baud_rate;
    *(__IO uint16_t*)(FIBER_OPTIC_BASE_ADDR + FPGA_REG_BAUD_DIVISOR) = (uint16_t)baud_divisor;

    // Step 3: Set node ID (from DIP switches)
    *(__IO uint16_t*)(FIBER_OPTIC_BASE_ADDR + FPGA_REG_NODE_ID) = (uint16_t)dip_switch_channel_number;

    // Update instance structure
    instance->baud_rate = baud_rate;
    instance->node_id = dip_switch_channel_number;
    instance->initialized = true;

    return true;
}

/**
 * @brief  Transmit data on specified channel
 */
bool FiberOptic_Transmit(
    fiber_optic_instance_t* instance, fiber_channel_t channel, const uint8_t* data, uint16_t length)
{
    uint32_t channel_addr;
    uint16_t i;

    // Validate parameters
    if (!instance || !instance->initialized || !data || length == 0) {
        return false;
    }

    if (length > FIBER_OPTIC_MAX_LENGTH) {
        return false;
    }

    // Calculate channel TX register address
    // Channel offset: (channel + 100) * 2 bytes
    channel_addr = FIBER_OPTIC_BASE_ADDR + (channel + 100) * 2;

    // Write data byte-by-byte to FPGA TX FIFO
    // Each write to this address pushes one byte into the transmit queue
    for (i = 0; i < length; i++) {
        *(__IO uint16_t*)channel_addr = (uint16_t)data[i];
    }

    return true;
}

/**
 * @brief  Receive data from specified channel
 */
uint16_t FiberOptic_Receive(
    fiber_optic_instance_t* instance, fiber_channel_t channel, uint8_t* buffer, uint16_t max_length)
{
    uint16_t length = 0;
    uint16_t status;
    uint32_t data_addr;
    uint32_t status_addr;
    uint16_t safety_counter = 0;

    // Validate parameters
    if (!instance || !instance->initialized || !buffer || max_length == 0) {
        return 0;
    }

    // Calculate register addresses
    data_addr = FIBER_OPTIC_BASE_ADDR + channel * 2;            // Data register
    status_addr = FIBER_OPTIC_BASE_ADDR + (channel + 100) * 2;  // Status register

    // Read data until RX buffer empty or max length reached
    // Added safety counter (matching legacy implementation) to prevent infinite loops
    while (length < max_length && safety_counter < 255) {
        // Check if data available (bit 1 = 0 means data present)
        status = *(__IO uint16_t*)status_addr;
        if (status & CHANNEL_STATUS_RX_EMPTY) {
            // RX buffer empty - no more data
            break;
        }

        // Read one byte from RX FIFO
        buffer[length++] = (uint8_t)(*(__IO uint16_t*)data_addr);
        safety_counter++;
    }

    return length;
}

/**
 * @brief  Check if data is available on specified channel
 */
bool FiberOptic_DataAvailable(fiber_optic_instance_t* instance, fiber_channel_t channel)
{
    uint16_t status;
    uint32_t status_addr;

    if (!instance || !instance->initialized) {
        return false;
    }

    // Read channel status register
    status_addr = FIBER_OPTIC_BASE_ADDR + (channel + 100) * 2;
    status = *(__IO uint16_t*)status_addr;

    // Bit 1 = 0 means data available, bit 1 = 1 means empty
    return !(status & CHANNEL_STATUS_RX_EMPTY);
}

/**
 * @brief  Get channel status register value
 */
uint16_t FiberOptic_GetChannelStatus(fiber_optic_instance_t* instance, fiber_channel_t channel)
{
    uint32_t status_addr;

    if (!instance || !instance->initialized) {
        return 0xFFFF;  // Invalid status
    }

    status_addr = FIBER_OPTIC_BASE_ADDR + (channel + 100) * 2;
    return *(__IO uint16_t*)status_addr;
}

/******* (C) COPYRIGHT 2025 Techperts *****END OF FILE****/
