/******************************************************************************
 * @file    sram_driver.h
 * @brief   IS61WV25616BLL SRAM Driver for STM32F407
 * @author  Refactored from NSD (Nengshida) driver
 * @date    2025
 *
 * @details
 * Hardware abstraction layer for IS61WV25616BLL SRAM memory connected via
 * STM32 FSMC (Flexible Static Memory Controller) Bank 1.
 *
 * SRAM Specifications:
 * - Device: IS61WV25616BLL (ISSI)
 * - Capacity: 4 Mbit (256K × 16-bit organization)
 * - Data bus width: 16-bit
 * - Address range: 0x00000 to 0x3FFFF (256K words)
 * - Access time: 10ns typical
 * - Asynchronous operation
 *
 * FSMC Configuration:
 * - Bank: FSMC Bank 1, NOR/SRAM Region 1
 * - Base address: 0x68000000
 * - Memory type: SRAM
 * - Data width: 16-bit
 * - Access mode: Mode A (asynchronous)
 *
 * Pin Mapping (STM32F407):
 * - Data bus (D0-D15): PD14-15, PD0-1, PE7-15
 * - Address bus (A0-A17): PF0-5, PF12-15, PG0-5
 * - Control signals: PD4 (NOE/RD), PD5 (NWE/WR), PD7 (NE1/CS), PE0 (NBL0), PE1 (NBL1)
 *
 * Usage Notes:
 * - SRAM is volatile - loses data when power is removed
 * - No erase required - can write directly to any location
 * - Fast random access (10ns typical)
 * - Byte-enable signals (NBL0, NBL1) allow byte-level writes
 * - Ideal for temporary data storage, buffers, stack/heap extension
 *
 * @note This driver provides low-level hardware access. Memory management
 *       (allocation, deallocation) should be implemented at higher levels.
 *****************************************************************************/

#ifndef SRAM_DRIVER_H
#define SRAM_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx.h"

/******************************************************************************
 * SRAM Memory Layout
 *****************************************************************************/

/** @brief SRAM capacity in 16-bit words */
#define SRAM_SIZE_WORDS ((uint32_t)0x40000)  // 256K words

/** @brief SRAM capacity in bytes */
#define SRAM_SIZE_BYTES ((uint32_t)(SRAM_SIZE_WORDS * 2))  // 512 KB

/** @brief SRAM base address */
#define SRAM_BASE_ADDR ((uint32_t)0x68000000)

/** @brief Maximum valid address offset (in 16-bit words) */
#define SRAM_MAX_ADDRESS (SRAM_SIZE_WORDS - 1)

/******************************************************************************
 * Return Status Codes
 *****************************************************************************/

typedef enum {
    SRAM_OK = 0,              /**< Operation completed successfully */
    SRAM_ERROR = 1,           /**< Operation failed */
    SRAM_INVALID_ADDRESS = 2, /**< Address out of range */
    SRAM_INVALID_PARAM = 3    /**< Invalid parameter */
} sram_status_t;

/******************************************************************************
 * Data Structures
 *****************************************************************************/

/**
 * @brief SRAM driver instance
 *
 * Contains FSMC configuration and operational statistics.
 */
typedef struct {
    // Hardware configuration
    uint32_t base_address; /**< Memory-mapped base address */
    bool initialized;      /**< Initialization status */

    // Statistics
    uint32_t read_count;  /**< Number of read operations */
    uint32_t write_count; /**< Number of write operations */
} sram_instance_t;

/******************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/**
 * @brief Initialize SRAM driver and hardware
 *
 * Configures FSMC Bank 1 for SRAM operation:
 * - Sets up GPIO pins for data, address, and control lines
 * - Configures FSMC timing parameters for the SRAM device
 * - Enables asynchronous access mode
 *
 * @param instance Pointer to SRAM driver instance
 * @return SRAM_OK if successful, error code otherwise
 *
 * @note Must be called before any other SRAM operations
 * @note FSMC and GPIO clocks are enabled by this function
 */
sram_status_t SRAM_Init(sram_instance_t* instance);

/**
 * @brief Write 16-bit data to SRAM
 *
 * Writes an array of 16-bit words to the specified SRAM address.
 *
 * @param instance Pointer to SRAM driver instance
 * @param buffer Pointer to data buffer (16-bit words)
 * @param address Starting address (in 16-bit words, 0 to 0x3FFFF)
 * @param length Number of 16-bit words to write
 * @return SRAM_OK if successful, error code otherwise
 *
 * @note Address is in words, not bytes
 * @note No address boundary restrictions - can write to any location
 */
sram_status_t SRAM_Write(sram_instance_t* instance, const uint16_t* buffer, uint32_t address, uint32_t length);

/**
 * @brief Read 16-bit data from SRAM
 *
 * Reads an array of 16-bit words from the specified SRAM address.
 *
 * @param instance Pointer to SRAM driver instance
 * @param buffer Pointer to buffer to store read data (16-bit words)
 * @param address Starting address (in 16-bit words, 0 to 0x3FFFF)
 * @param length Number of 16-bit words to read
 * @return SRAM_OK if successful, error code otherwise
 *
 * @note Address is in words, not bytes
 * @note Access time is typically 10ns per word
 */
sram_status_t SRAM_Read(sram_instance_t* instance, uint16_t* buffer, uint32_t address, uint32_t length);

/**
 * @brief Write single 16-bit word to SRAM
 *
 * Convenience function to write a single word.
 *
 * @param instance Pointer to SRAM driver instance
 * @param address Address (in 16-bit words)
 * @param data 16-bit word to write
 * @return SRAM_OK if successful, error code otherwise
 */
sram_status_t SRAM_WriteSingle(sram_instance_t* instance, uint32_t address, uint16_t data);

/**
 * @brief Read single 16-bit word from SRAM
 *
 * Convenience function to read a single word.
 *
 * @param instance Pointer to SRAM driver instance
 * @param address Address (in 16-bit words)
 * @param data Pointer to store 16-bit word
 * @return SRAM_OK if successful, error code otherwise
 */
sram_status_t SRAM_ReadSingle(sram_instance_t* instance, uint32_t address, uint16_t* data);

/**
 * @brief Fill SRAM region with a pattern
 *
 * Writes the same 16-bit pattern to a range of addresses.
 * Useful for memory initialization or testing.
 *
 * @param instance Pointer to SRAM driver instance
 * @param address Starting address (in 16-bit words)
 * @param length Number of words to fill
 * @param pattern 16-bit pattern to write
 * @return SRAM_OK if successful, error code otherwise
 */
sram_status_t SRAM_Fill(sram_instance_t* instance, uint32_t address, uint32_t length, uint16_t pattern);

/**
 * @brief Test SRAM memory integrity
 *
 * Performs a non-destructive memory test by:
 * 1. Reading original data
 * 2. Writing test pattern
 * 3. Reading back and verifying
 * 4. Restoring original data
 *
 * @param instance Pointer to SRAM driver instance
 * @param address Starting address to test
 * @param length Number of words to test
 * @return SRAM_OK if test passes, SRAM_ERROR if failure detected
 *
 * @note This function is slower than normal read/write operations
 * @note Original data is restored after test
 */
sram_status_t SRAM_Test(sram_instance_t* instance, uint32_t address, uint32_t length);

/**
 * @brief Get driver statistics
 *
 * Returns operational statistics for monitoring and diagnostics.
 *
 * @param instance Pointer to SRAM driver instance
 * @param read_count Pointer to store read count (can be NULL)
 * @param write_count Pointer to store write count (can be NULL)
 * @return SRAM_OK if successful
 */
sram_status_t SRAM_GetStatistics(sram_instance_t* instance, uint32_t* read_count, uint32_t* write_count);

/**
 * @brief Validate address range
 *
 * Checks if the address and length are within valid SRAM bounds.
 *
 * @param address Starting address (in words)
 * @param length Number of words
 * @return true if valid, false if out of range
 */
bool SRAM_IsAddressValid(uint32_t address, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* SRAM_DRIVER_H */
