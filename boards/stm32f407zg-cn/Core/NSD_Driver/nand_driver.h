/******************************************************************************
 * @file    nand_driver.h
 * @brief   K9F2G08U0M NAND Flash Driver for STM32F407
 * @author  Refactored from NSD (Nengshida) driver
 * @date    2025
 *
 * @details
 * Hardware abstraction layer for K9F2G08U0M NAND Flash memory connected via
 * STM32 FSMC (Flexible Static Memory Controller) Bank 2.
 *
 * NAND Flash Specifications:
 * - Device: K9F2G08U0M (Samsung/compatible)
 * - Capacity: 2 Gbit (256 MB)
 * - Organization: 2048 blocks × 64 pages × 2048 bytes + 64 spare bytes
 * - Page size: 2048 bytes (main area) + 64 bytes (spare area)
 * - Block size: 128 KB (64 pages × 2048 bytes)
 * - Total blocks: 2048
 * - Data bus width: 8-bit
 *
 * FSMC Configuration:
 * - Bank: FSMC Bank 2 NAND
 * - Base address: 0x70000000
 * - Address lines: A16 (CLE), A17 (ALE)
 * - Control signals: NOE (read enable), NWE (write enable), NCE2 (chip select)
 * - Ready/Busy: NWAIT signal monitors flash status
 *
 * Pin Mapping (STM32F407):
 * - Data lines: PD14-PD15 (D0-D1), PD0-PD1 (D2-D3), PE7-PE10 (D4-D7)
 * - Address/Control: PD11 (A16/CLE), PD12 (A17/ALE)
 * - Control: PD4 (NOE), PD5 (NWE), PD7 (NCE2), PD6 (NWAIT)
 *
 * Usage Notes:
 * - Always erase blocks before writing (NAND Flash requirement)
 * - A block must be fully erased (all bits set to 0xFF) before programming
 * - Pages within a block should be written sequentially
 * - Use spare area for bad block management and ECC
 * - Hardware ECC enabled for 2048-byte page size
 *
 * @note This driver provides low-level hardware access. Higher-level functions
 *       like wear leveling, bad block management, and file systems should be
 *       implemented separately.
 *****************************************************************************/

#ifndef NAND_DRIVER_H
#define NAND_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx.h"

/******************************************************************************
 * NAND Flash Memory Layout
 *****************************************************************************/

/** @brief NAND Flash page size (main data area) */
#define NAND_PAGE_SIZE ((uint16_t)0x0800)  // 2048 bytes

/** @brief NAND Flash spare area size (OOB - Out Of Band data) */
#define NAND_SPARE_AREA_SIZE ((uint16_t)0x0040)  // 64 bytes

/** @brief Number of pages per block */
#define NAND_BLOCK_SIZE ((uint16_t)0x0040)  // 64 pages

/** @brief Number of blocks per zone */
#define NAND_ZONE_SIZE ((uint16_t)0x0800)  // 2048 blocks

/** @brief Number of zones in device */
#define NAND_MAX_ZONE ((uint16_t)0x0001)  // 1 zone

/** @brief Total number of blocks */
#define NAND_TOTAL_BLOCKS (NAND_ZONE_SIZE * NAND_MAX_ZONE)

/** @brief Total device capacity in bytes */
#define NAND_TOTAL_SIZE ((uint32_t)(NAND_TOTAL_BLOCKS * NAND_BLOCK_SIZE * NAND_PAGE_SIZE))

/******************************************************************************
 * NAND Flash Commands
 *****************************************************************************/

#define NAND_CMD_READ_1ST    ((uint8_t)0x00) /**< Read page cycle 1 */
#define NAND_CMD_READ_2ND    ((uint8_t)0x30) /**< Read page cycle 2 */
#define NAND_CMD_WRITE_1ST   ((uint8_t)0x80) /**< Write page cycle 1 */
#define NAND_CMD_WRITE_2ND   ((uint8_t)0x10) /**< Write page cycle 2 */
#define NAND_CMD_ERASE_1ST   ((uint8_t)0x60) /**< Erase block cycle 1 */
#define NAND_CMD_ERASE_2ND   ((uint8_t)0xD0) /**< Erase block cycle 2 */
#define NAND_CMD_READ_ID     ((uint8_t)0x90) /**< Read device ID */
#define NAND_CMD_READ_STATUS ((uint8_t)0x70) /**< Read status register */
#define NAND_CMD_RESET       ((uint8_t)0xFF) /**< Reset device */

/******************************************************************************
 * NAND Flash Status Flags
 *****************************************************************************/

#define NAND_STATUS_READY ((uint8_t)0x40) /**< Ready bit in status register */
#define NAND_STATUS_ERROR ((uint8_t)0x01) /**< Error bit in status register */

/******************************************************************************
 * Return Status Codes
 *****************************************************************************/

typedef enum {
    NAND_OK = 0,              /**< Operation completed successfully */
    NAND_ERROR = 1,           /**< Operation failed */
    NAND_TIMEOUT = 2,         /**< Operation timed out */
    NAND_BUSY = 3,            /**< Device is busy */
    NAND_INVALID_ADDRESS = 4, /**< Invalid address provided */
    NAND_WRITE_PROTECTED = 5  /**< Device is write protected */
} nand_status_t;

/******************************************************************************
 * Data Structures
 *****************************************************************************/

/**
 * @brief NAND Flash device identification structure
 *
 * Contains manufacturer and device identification codes read from the chip.
 */
typedef struct {
    uint8_t maker_id;  /**< Manufacturer ID (e.g., 0xEC for Samsung) */
    uint8_t device_id; /**< Device ID */
    uint8_t third_id;  /**< Third ID byte (organization info) */
    uint8_t fourth_id; /**< Fourth ID byte (page size, block size) */
} nand_id_t;

/**
 * @brief NAND Flash address structure
 *
 * Hierarchical addressing: Zone → Block → Page
 */
typedef struct {
    uint16_t zone;  /**< Zone number (0 to NAND_MAX_ZONE-1) */
    uint16_t block; /**< Block number within zone (0 to NAND_ZONE_SIZE-1) */
    uint16_t page;  /**< Page number within block (0 to NAND_BLOCK_SIZE-1) */
} nand_address_t;

/**
 * @brief NAND Flash driver instance
 *
 * Contains FSMC configuration, device information, and operational statistics.
 */
typedef struct {
    // Hardware configuration
    void* fsmc_bank;       /**< FSMC NAND bank registers (actually FSMC_Bank2_3_TypeDef*) */
    uint32_t base_address; /**< Memory-mapped base address */

    // Device information
    nand_id_t device_id; /**< Device identification */
    bool initialized;    /**< Initialization status */

    // Statistics
    uint32_t read_count;  /**< Number of page reads */
    uint32_t write_count; /**< Number of page writes */
    uint32_t erase_count; /**< Number of block erases */
    uint32_t error_count; /**< Number of errors encountered */
} nand_instance_t;

/******************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/**
 * @brief Initialize NAND Flash driver and hardware
 *
 * Configures FSMC Bank 2 for NAND Flash operation:
 * - Sets up GPIO pins for data, address, and control lines
 * - Configures FSMC timing parameters for the NAND device
 * - Enables hardware ECC for error detection/correction
 * - Resets the NAND Flash device
 *
 * @param instance Pointer to NAND driver instance
 * @return NAND_OK if successful, error code otherwise
 *
 * @note Must be called before any other NAND operations
 * @note FSMC and GPIO clocks are enabled by this function
 */
nand_status_t NAND_Init(nand_instance_t* instance);

/**
 * @brief Reset NAND Flash device
 *
 * Sends reset command to NAND Flash and waits for ready status.
 * This brings the device to a known idle state.
 *
 * @param instance Pointer to NAND driver instance
 * @return NAND_OK if successful, error code otherwise
 *
 * @note Reset time is typically 5-10ms
 */
nand_status_t NAND_Reset(nand_instance_t* instance);

/**
 * @brief Read NAND Flash device identification
 *
 * Reads the 4-byte ID sequence from the NAND Flash to identify
 * manufacturer, device type, and organization.
 *
 * @param instance Pointer to NAND driver instance
 * @param id Pointer to structure to store device ID
 * @return NAND_OK if successful, error code otherwise
 *
 * @note For K9F2G08U0M: Maker=0xEC, Device=0xDA
 */
nand_status_t NAND_ReadID(nand_instance_t* instance, nand_id_t* id);

/**
 * @brief Erase a NAND Flash block
 *
 * Erases all pages in the specified block (sets all bits to 0xFF).
 * This must be done before writing to any page in the block.
 *
 * @param instance Pointer to NAND driver instance
 * @param address Block address to erase (page field is ignored)
 * @return NAND_OK if successful, error code otherwise
 *
 * @warning This operation takes 2-3ms and cannot be interrupted
 * @note After erase, verify with status register before writing
 */
nand_status_t NAND_EraseBlock(nand_instance_t* instance, nand_address_t address);

/**
 * @brief Write data to NAND Flash page(s)
 *
 * Writes one or more pages to NAND Flash. The target pages must be
 * in an erased state (all 0xFF) before writing.
 *
 * @param instance Pointer to NAND driver instance
 * @param buffer Pointer to data buffer to write
 * @param address Starting page address
 * @param num_pages Number of pages to write
 * @return NAND_OK if successful, error code otherwise
 *
 * @warning Pages must be erased before writing
 * @note Write time is approximately 200-700μs per page
 * @note Spare area is not written by this function
 */
nand_status_t NAND_WritePage(
    nand_instance_t* instance, const uint8_t* buffer, nand_address_t address, uint32_t num_pages);

/**
 * @brief Read data from NAND Flash page(s)
 *
 * Reads one or more pages from NAND Flash into the provided buffer.
 *
 * @param instance Pointer to NAND driver instance
 * @param buffer Pointer to buffer to store read data
 * @param address Starting page address
 * @param num_pages Number of pages to read
 * @return NAND_OK if successful, error code otherwise
 *
 * @note Read time is approximately 25μs per page
 * @note Spare area is not read by this function
 */
nand_status_t NAND_ReadPage(nand_instance_t* instance, uint8_t* buffer, nand_address_t address, uint32_t num_pages);

/**
 * @brief Read NAND Flash status register
 *
 * Reads and returns the device status register, which contains
 * ready/busy and error flags.
 *
 * @param instance Pointer to NAND driver instance
 * @param status Pointer to store status byte
 * @return NAND_OK if successful, error code otherwise
 */
nand_status_t NAND_ReadStatus(nand_instance_t* instance, uint8_t* status);

/**
 * @brief Wait for NAND Flash to become ready
 *
 * Polls the status register until the device is ready or timeout occurs.
 *
 * @param instance Pointer to NAND driver instance
 * @param timeout_ms Maximum wait time in milliseconds
 * @return NAND_OK if ready, NAND_TIMEOUT if timeout
 *
 * @note Typical operations: Erase=2-3ms, Write=200-700μs, Read=25μs
 */
nand_status_t NAND_WaitReady(nand_instance_t* instance, uint32_t timeout_ms);

/**
 * @brief Get driver statistics
 *
 * Returns operational statistics for monitoring and diagnostics.
 *
 * @param instance Pointer to NAND driver instance
 * @param read_count Pointer to store read count (can be NULL)
 * @param write_count Pointer to store write count (can be NULL)
 * @param erase_count Pointer to store erase count (can be NULL)
 * @param error_count Pointer to store error count (can be NULL)
 * @return NAND_OK if successful
 */
nand_status_t NAND_GetStatistics(nand_instance_t* instance,
    uint32_t* read_count,
    uint32_t* write_count,
    uint32_t* erase_count,
    uint32_t* error_count);

/**
 * @brief Validate address bounds
 *
 * Checks if the address is within valid range.
 *
 * @param address Address to validate
 * @return true if valid, false if out of range
 */
bool NAND_IsAddressValid(nand_address_t address);

/**
 * @brief Increment address to next page
 *
 * Advances address to the next page, handling block and zone boundaries.
 *
 * @param address Pointer to address to increment
 * @return NAND_OK if incremented, NAND_INVALID_ADDRESS if overflow
 */
nand_status_t NAND_IncrementAddress(nand_address_t* address);

#ifdef __cplusplus
}
#endif

#endif /* NAND_DRIVER_H */
