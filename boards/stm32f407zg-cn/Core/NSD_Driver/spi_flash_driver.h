/******************************************************************************
 * @file    spi_flash_driver.h
 * @brief   M25P32 SPI Flash Driver for STM32F407
 * @author  Refactored from NSD (Nengshida) driver
 * @date    2025
 *
 * @details
 * Hardware abstraction layer for M25P32 SPI NOR Flash memory.
 *
 * Flash Specifications:
 * - Device: M25P32 (Micron/STMicroelectronics)
 * - Capacity: 32 Mbit (4 MB)
 * - Organization: 64 sectors × 64 KB
 * - Page size: 256 bytes (for programming)
 * - Interface: SPI (up to 75 MHz)
 * - Voltage: 2.7V to 3.6V
 *
 * SPI Configuration (SPI1):
 * - SCK: PA5
 * - MISO: PA6
 * - MOSI: PA7
 * - CS: PE5 (software controlled)
 * - Mode: Mode 0 or Mode 3
 * - Clock speed: Up to 75 MHz (configured for safety margin)
 *
 * Memory Operations:
 * - Sector erase: 64 KB at a time (required before writing)
 * - Page program: 256 bytes at a time
 * - Read: Unlimited length from any address
 * - Write protection: Software and hardware mechanisms available
 *
 * Usage Notes:
 * - Must erase sectors before writing (Flash requirement)
 * - Page writes must not cross page boundaries
 * - Device auto-increments address during multi-byte operations
 * - Status register indicates busy/ready and write enable latch
 * - Supports deep power-down mode for low power applications
 *
 * Typical Operation Flow:
 * 1. Initialize driver and verify ID
 * 2. Erase sector(s) containing target addresses
 * 3. Write data in page-sized chunks (256 bytes max)
 * 4. Read back to verify
 *
 * @note This driver uses blocking operations. For high-performance applications,
 *       consider implementing DMA transfers or interrupt-driven operation.
 *****************************************************************************/

#ifndef SPI_FLASH_DRIVER_H
#define SPI_FLASH_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx.h"

/******************************************************************************
 * Flash Memory Layout
 *****************************************************************************/

/** @brief M25P32 device ID */
#define M25P32_DEVICE_ID ((uint32_t)0x202016)

/** @brief M25P128 device ID (alternative larger variant) */
#define M25P128_DEVICE_ID ((uint32_t)0x202018)

/** @brief M25P64 device ID (alternative smaller variant) */
#define M25P64_DEVICE_ID ((uint32_t)0x202017)

/** @brief Flash page size for programming */
#define FLASH_PAGE_SIZE ((uint16_t)0x0100)  // 256 bytes

/** @brief Flash sector size for erasing */
#define FLASH_SECTOR_SIZE ((uint32_t)0x10000)  // 64 KB

/** @brief Number of sectors */
#define FLASH_SECTOR_COUNT ((uint16_t)64)

/** @brief Total flash capacity in bytes */
#define FLASH_TOTAL_SIZE ((uint32_t)(FLASH_SECTOR_SIZE * FLASH_SECTOR_COUNT))  // 4 MB

/******************************************************************************
 * Return Status Codes
 *****************************************************************************/

typedef enum {
    FLASH_OK = 0,            /**< Operation completed successfully */
    FLASH_ERROR = 1,         /**< Operation failed */
    FLASH_TIMEOUT = 2,       /**< Operation timed out */
    FLASH_BUSY = 3,          /**< Device is busy */
    FLASH_INVALID_PARAM = 4, /**< Invalid parameter */
    FLASH_VERIFY_ERROR = 5,  /**< Verification failed */
    FLASH_WRONG_ID = 6       /**< Device ID mismatch */
} flash_status_t;

/******************************************************************************
 * Data Structures
 *****************************************************************************/

/**
 * @brief SPI Flash driver instance
 *
 * Contains SPI configuration and operational statistics.
 */
typedef struct {
    // Hardware configuration
    SPI_TypeDef* spi;      /**< SPI peripheral (SPI1) */
    GPIO_TypeDef* cs_port; /**< Chip select GPIO port */
    uint16_t cs_pin;       /**< Chip select GPIO pin */
    bool initialized;      /**< Initialization status */

    // Device information
    uint32_t device_id; /**< Flash device ID */

    // Statistics
    uint32_t read_count;  /**< Number of read operations */
    uint32_t write_count; /**< Number of write operations */
    uint32_t erase_count; /**< Number of erase operations */
    uint32_t error_count; /**< Number of errors encountered */
} flash_instance_t;

/******************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/**
 * @brief Initialize SPI Flash driver and hardware
 *
 * Configures SPI1 peripheral and GPIO pins for M25P32 communication:
 * - Sets up SPI1 in master mode, Mode 0, 8-bit data
 * - Configures SCK, MISO, MOSI, and CS pins
 * - Sets appropriate clock speed (less than 75 MHz)
 * - Reads and validates device ID
 *
 * @param instance Pointer to flash driver instance
 * @return FLASH_OK if successful, error code otherwise
 *
 * @note Must be called before any other flash operations
 * @note SPI and GPIO clocks are enabled by this function
 */
flash_status_t Flash_Init(flash_instance_t* instance);

/**
 * @brief Read Flash device identification
 *
 * Reads the 3-byte JEDEC ID: Manufacturer ID + Memory Type + Capacity.
 *
 * @param instance Pointer to flash driver instance
 * @return 24-bit device ID, or 0 on error
 *
 * @note For M25P32: ID = 0x202016
 */
uint32_t Flash_ReadID(flash_instance_t* instance);

/**
 * @brief Erase a Flash sector
 *
 * Erases 64 KB sector containing the specified address.
 * All bytes in the sector are set to 0xFF.
 *
 * @param instance Pointer to flash driver instance
 * @param sector_address Any address within the sector to erase
 * @return FLASH_OK if successful, error code otherwise
 *
 * @warning This operation takes 0.6 to 3 seconds
 * @note Sector boundaries are at 64 KB intervals (0x00000, 0x10000, 0x20000, ...)
 * @note Watchdog must be fed during this long operation
 */
flash_status_t Flash_EraseSector(flash_instance_t* instance, uint32_t sector_address);

/**
 * @brief Write data to Flash page
 *
 * Writes up to 256 bytes to a Flash page. The target area must be
 * in an erased state (all 0xFF) before writing.
 *
 * @param instance Pointer to flash driver instance
 * @param buffer Pointer to data buffer to write
 * @param address Starting address (must be page-aligned for best practice)
 * @param length Number of bytes to write (max 256, must not cross page boundary)
 * @return FLASH_OK if successful, error code otherwise
 *
 * @warning Length must not exceed 256 bytes
 * @warning Address + length must not cross page boundary (256-byte aligned)
 * @note Write time is typically 0.6 to 5 ms
 */
flash_status_t Flash_WritePage(flash_instance_t* instance, const uint8_t* buffer, uint32_t address, uint16_t length);

/**
 * @brief Read data from Flash
 *
 * Reads any number of bytes from Flash starting at the specified address.
 *
 * @param instance Pointer to flash driver instance
 * @param buffer Pointer to buffer to store read data
 * @param address Starting address
 * @param length Number of bytes to read
 * @return FLASH_OK if successful, error code otherwise
 *
 * @note No length restriction - can read entire flash if needed
 * @note Read speed is limited by SPI clock (typically several MB/s)
 */
flash_status_t Flash_ReadBuffer(flash_instance_t* instance, uint8_t* buffer, uint32_t address, uint16_t length);

/**
 * @brief Write data with automatic sector erase
 *
 * High-level write function that automatically:
 * - Erases required sector(s)
 * - Writes data in page-sized chunks
 * - Verifies written data
 *
 * @param instance Pointer to flash driver instance
 * @param buffer Pointer to data buffer to write
 * @param address Starting address
 * @param length Number of bytes to write
 * @return FLASH_OK if successful, error code otherwise
 *
 * @warning This function erases entire sector(s), destroying all data within
 * @note This is a convenience function; use Flash_EraseSector + Flash_WritePage
 *       directly if you need more control
 */
flash_status_t Flash_WriteBuffer(flash_instance_t* instance, const uint8_t* buffer, uint32_t address, uint16_t length);

/**
 * @brief Wait for Flash to become ready
 *
 * Polls the status register until the write-in-progress bit is clear.
 *
 * @param instance Pointer to flash driver instance
 * @return FLASH_OK if ready, FLASH_TIMEOUT if timeout
 *
 * @note Typical times: Erase=0.6-3s, Write=0.6-5ms
 */
flash_status_t Flash_WaitReady(flash_instance_t* instance);

/**
 * @brief Enable write operations
 *
 * Sets the write enable latch in the status register.
 * Required before any erase or program operation.
 *
 * @param instance Pointer to flash driver instance
 * @return FLASH_OK if successful
 *
 * @note Automatically cleared after erase/program completion
 */
flash_status_t Flash_WriteEnable(flash_instance_t* instance);

/**
 * @brief Read Flash status register
 *
 * Returns the 8-bit status register containing:
 * - Bit 0: Write In Progress (WIP)
 * - Bit 1: Write Enable Latch (WEL)
 * - Bits 2-4: Block Protect bits
 * - Bit 7: Status Register Write Disable
 *
 * @param instance Pointer to flash driver instance
 * @param status Pointer to store status byte
 * @return FLASH_OK if successful
 */
flash_status_t Flash_ReadStatus(flash_instance_t* instance, uint8_t* status);

/**
 * @brief Perform Flash memory test
 *
 * Tests a sector by:
 * 1. Backing up original data
 * 2. Erasing sector
 * 3. Writing test patterns
 * 4. Reading and verifying
 * 5. Restoring original data
 *
 * @param instance Pointer to flash driver instance
 * @param sector_address Address within sector to test
 * @return FLASH_OK if test passes, error code otherwise
 *
 * @warning This test is destructive - temporarily erases the sector
 * @note Original data is restored after test
 */
flash_status_t Flash_TestSector(flash_instance_t* instance, uint32_t sector_address);

/**
 * @brief Get driver statistics
 *
 * Returns operational statistics for monitoring and diagnostics.
 *
 * @param instance Pointer to flash driver instance
 * @param read_count Pointer to store read count (can be NULL)
 * @param write_count Pointer to store write count (can be NULL)
 * @param erase_count Pointer to store erase count (can be NULL)
 * @param error_count Pointer to store error count (can be NULL)
 * @return FLASH_OK if successful
 */
flash_status_t Flash_GetStatistics(flash_instance_t* instance,
    uint32_t* read_count,
    uint32_t* write_count,
    uint32_t* erase_count,
    uint32_t* error_count);

#ifdef __cplusplus
}
#endif

#endif /* SPI_FLASH_DRIVER_H */
