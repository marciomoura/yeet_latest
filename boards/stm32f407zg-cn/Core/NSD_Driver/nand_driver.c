/******************************************************************************
 * @file    nand_driver.c
 * @brief   K9F2G08U0M NAND Flash Driver Implementation
 * @author  Refactored from NSD (Nengshida) driver
 * @date    2025
 *****************************************************************************/

#include "nand_driver.h"

#include <string.h>

/******************************************************************************
 * Private Definitions
 *****************************************************************************/

/** @brief FSMC Bank 2 NAND base address */
#define NAND_BASE_ADDR ((uint32_t)0x70000000)

/** @brief Command latch enable (CLE) - A16 */
#define NAND_CMD_AREA ((uint32_t)(1 << 16))

/** @brief Address latch enable (ALE) - A17 */
#define NAND_ADDR_AREA ((uint32_t)(1 << 17))

/** @brief Data area (normal access) */
#define NAND_DATA_AREA ((uint32_t)0x00000000)

/** @brief Default timeout for operations (milliseconds) */
#define NAND_DEFAULT_TIMEOUT_MS 5000

/** @brief Maximum status polling attempts */
#define NAND_STATUS_TIMEOUT 0x01000000

/******************************************************************************
 * Address Calculation Macros
 *****************************************************************************/

/** @brief Calculate row address from hierarchical address */
#define NAND_ROW_ADDRESS(addr) ((addr).page + ((addr).block + ((addr).zone * NAND_ZONE_SIZE)) * NAND_BLOCK_SIZE)

/** @brief Address cycle macros for 5-cycle addressing */
#define ADDR_1ST_CYCLE(addr) ((uint8_t)0x00)                 // Column address low (always 0 for page operations)
#define ADDR_2ND_CYCLE(addr) ((uint8_t)0x00)                 // Column address high (always 0 for page operations)
#define ADDR_3RD_CYCLE(addr) ((uint8_t)((addr) & 0x0000FF))  // Row address byte 0
#define ADDR_4TH_CYCLE(addr) ((uint8_t)(((addr) & 0x00FF00) >> 8))   // Row address byte 1
#define ADDR_5TH_CYCLE(addr) ((uint8_t)(((addr) & 0x010000) >> 16))  // Row address byte 2

/******************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

static void NAND_InitGPIO(void);
static void NAND_InitFSMC(void);
static void NAND_WriteCommand(uint8_t command);
static void NAND_WriteAddress(uint8_t address);
static uint8_t NAND_ReadData(void);
static void NAND_WriteData(uint8_t data);
static nand_status_t NAND_GetStatus_Internal(void);

/******************************************************************************
 * Private Functions
 *****************************************************************************/

/**
 * @brief Initialize GPIO pins for NAND Flash
 *
 * Configures all necessary GPIO pins for FSMC Bank 2 NAND interface:
 * - Data bus (D0-D7)
 * - Address lines (A16/CLE, A17/ALE)
 * - Control signals (NOE, NWE, NCE2, NWAIT)
 */
static void NAND_InitGPIO(void)
{
    GPIO_InitTypeDef gpio_init;

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

    // Common GPIO configuration for all FSMC pins
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

    // Configure GPIOD pins
    // D0-D3: PD14, PD15, PD0, PD1
    // Control: PD4 (NOE), PD5 (NWE), PD7 (NCE2)
    // Address: PD11 (A16/CLE), PD12 (A17/ALE)
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);   // D2
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);   // D3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);   // NOE (read enable)
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);   // NWE (write enable)
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);   // NCE2 (chip enable)
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);  // A16 (CLE)
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);  // A17 (ALE)
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);  // D0
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);  // D1

    gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_12 |
                         GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &gpio_init);

    // Configure GPIOE pins
    // D4-D7: PE7, PE8, PE9, PE10
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);   // D4
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);   // D5
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);   // D6
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);  // D7

    gpio_init.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOE, &gpio_init);

    // Configure NWAIT pin (PD6) with pull-up
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_FSMC);
    gpio_init.GPIO_Pin = GPIO_Pin_6;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;  // Pull-up for NWAIT
    GPIO_Init(GPIOD, &gpio_init);
}

/**
 * @brief Initialize FSMC for NAND Flash operation
 *
 * Configures FSMC Bank 2 with timing parameters optimized for K9F2G08U0M:
 * - Setup time, wait time, hold time
 * - Enables hardware ECC for 2048-byte pages
 * - Configures for 8-bit data width
 */
static void NAND_InitFSMC(void)
{
    FSMC_NANDInitTypeDef fsmc_init;
    FSMC_NAND_PCCARDTimingInitTypeDef timing;

    // Enable FSMC clock
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

    // Configure timing parameters
    // These values are optimized for K9F2G08U0M with 100MHz HCLK
    timing.FSMC_SetupTime = 0x00;      // Setup time: 1 HCLK cycle
    timing.FSMC_WaitSetupTime = 0x03;  // Wait time: 4 HCLK cycles
    timing.FSMC_HoldSetupTime = 0x02;  // Hold time: 3 HCLK cycles
    timing.FSMC_HiZSetupTime = 0x00;   // Hi-Z time: 1 HCLK cycle

    // Configure FSMC for NAND Flash
    fsmc_init.FSMC_Bank = FSMC_Bank2_NAND;
    fsmc_init.FSMC_Waitfeature = FSMC_Waitfeature_Disable;  // Disable wait feature for now
    fsmc_init.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
    fsmc_init.FSMC_ECC = FSMC_ECC_Enable;                     // Enable hardware ECC
    fsmc_init.FSMC_ECCPageSize = FSMC_ECCPageSize_2048Bytes;  // 2KB page size for ECC
    fsmc_init.FSMC_TCLRSetupTime = 0x00;
    fsmc_init.FSMC_TARSetupTime = 0x00;
    fsmc_init.FSMC_CommonSpaceTimingStruct = &timing;
    fsmc_init.FSMC_AttributeSpaceTimingStruct = &timing;

    // Initialize and enable FSMC Bank 2
    FSMC_NANDInit(&fsmc_init);
    FSMC_NANDCmd(FSMC_Bank2_NAND, ENABLE);
}

/**
 * @brief Write command to NAND Flash
 *
 * Sends a command byte to the NAND Flash via the command latch (CLE).
 *
 * @param command Command byte to send
 */
static void NAND_WriteCommand(uint8_t command) { *(__IO uint8_t*)(NAND_BASE_ADDR | NAND_CMD_AREA) = command; }

/**
 * @brief Write address byte to NAND Flash
 *
 * Sends an address byte to the NAND Flash via the address latch (ALE).
 *
 * @param address Address byte to send
 */
static void NAND_WriteAddress(uint8_t address) { *(__IO uint8_t*)(NAND_BASE_ADDR | NAND_ADDR_AREA) = address; }

/**
 * @brief Read data byte from NAND Flash
 *
 * Reads a data byte from the NAND Flash data area.
 *
 * @return Data byte read
 */
static uint8_t NAND_ReadData(void) { return *(__IO uint8_t*)(NAND_BASE_ADDR | NAND_DATA_AREA); }

/**
 * @brief Write data byte to NAND Flash
 *
 * Writes a data byte to the NAND Flash data area.
 *
 * @param data Data byte to write
 */
static void NAND_WriteData(uint8_t data) { *(__IO uint8_t*)(NAND_BASE_ADDR | NAND_DATA_AREA) = data; }

/**
 * @brief Get NAND Flash status (internal helper)
 *
 * Reads and interprets the status register.
 *
 * @return NAND_OK if ready and no error, error code otherwise
 */
static nand_status_t NAND_GetStatus_Internal(void)
{
    uint8_t status;
    uint32_t timeout = NAND_STATUS_TIMEOUT;

    // Send read status command
    NAND_WriteCommand(NAND_CMD_READ_STATUS);

    // Poll status register
    while (timeout--) {
        status = NAND_ReadData();

        // Check if device is ready
        if (status & NAND_STATUS_READY) {
            // Check for errors
            if (status & NAND_STATUS_ERROR) {
                return NAND_ERROR;
            }
            return NAND_OK;
        }
    }

    return NAND_TIMEOUT;
}

/******************************************************************************
 * Public Functions
 *****************************************************************************/

/**
 * @brief Initialize NAND Flash driver and hardware
 */
nand_status_t NAND_Init(nand_instance_t* instance)
{
    if (instance == NULL) {
        return NAND_ERROR;
    }

    // Clear instance structure
    memset(instance, 0, sizeof(nand_instance_t));

    // Initialize hardware
    NAND_InitGPIO();
    NAND_InitFSMC();

    // Store configuration
    instance->base_address = NAND_BASE_ADDR;
    instance->fsmc_bank = FSMC_Bank2_NAND;

    // Reset the NAND Flash
    nand_status_t status = NAND_Reset(instance);
    if (status != NAND_OK) {
        return status;
    }

    // Read device ID
    status = NAND_ReadID(instance, &instance->device_id);
    if (status != NAND_OK) {
        return status;
    }

    instance->initialized = true;

    return NAND_OK;
}

/**
 * @brief Reset NAND Flash device
 */
nand_status_t NAND_Reset(nand_instance_t* instance)
{
    if (instance == NULL) {
        return NAND_ERROR;
    }

    // Send reset command
    NAND_WriteCommand(NAND_CMD_RESET);

    // Wait for device to become ready
    return NAND_WaitReady(instance, 100);  // 100ms timeout
}

/**
 * @brief Read NAND Flash device identification
 */
nand_status_t NAND_ReadID(nand_instance_t* instance, nand_id_t* id)
{
    if (instance == NULL || id == NULL) {
        return NAND_ERROR;
    }

    // Send read ID command
    NAND_WriteCommand(NAND_CMD_READ_ID);

    // Send address 0x00
    NAND_WriteAddress(0x00);

    // Read 4-byte ID sequence
    id->maker_id = NAND_ReadData();
    id->device_id = NAND_ReadData();
    id->third_id = NAND_ReadData();
    id->fourth_id = NAND_ReadData();

    return NAND_OK;
}

/**
 * @brief Erase a NAND Flash block
 */
nand_status_t NAND_EraseBlock(nand_instance_t* instance, nand_address_t address)
{
    if (instance == NULL || !instance->initialized) {
        return NAND_ERROR;
    }

    // Validate address
    if (!NAND_IsAddressValid(address)) {
        return NAND_INVALID_ADDRESS;
    }

    // Calculate row address
    uint32_t row_addr = NAND_ROW_ADDRESS(address);

    // Send erase command (first cycle)
    NAND_WriteCommand(NAND_CMD_ERASE_1ST);

    // Send block address (3 address cycles for row address only)
    NAND_WriteAddress(ADDR_3RD_CYCLE(row_addr));
    NAND_WriteAddress(ADDR_4TH_CYCLE(row_addr));
    NAND_WriteAddress(ADDR_5TH_CYCLE(row_addr));

    // Send erase command (second cycle)
    NAND_WriteCommand(NAND_CMD_ERASE_2ND);

    // Wait for operation to complete
    nand_status_t status = NAND_GetStatus_Internal();

    if (status == NAND_OK) {
        instance->erase_count++;
    }
    else {
        instance->error_count++;
    }

    return status;
}

/**
 * @brief Write data to NAND Flash page(s)
 */
nand_status_t NAND_WritePage(
    nand_instance_t* instance, const uint8_t* buffer, nand_address_t address, uint32_t num_pages)
{
    if (instance == NULL || !instance->initialized || buffer == NULL) {
        return NAND_ERROR;
    }

    uint32_t page_index = 0;
    nand_status_t status = NAND_OK;

    while (num_pages > 0 && status == NAND_OK) {
        // Validate address
        if (!NAND_IsAddressValid(address)) {
            instance->error_count++;
            return NAND_INVALID_ADDRESS;
        }

        // Calculate row address
        uint32_t row_addr = NAND_ROW_ADDRESS(address);

        // Send write command (first cycle)
        NAND_WriteCommand(NAND_CMD_WRITE_1ST);

        // Send 5-cycle address (column + row)
        NAND_WriteAddress(ADDR_1ST_CYCLE(row_addr));  // Column low byte (0x00)
        NAND_WriteAddress(ADDR_2ND_CYCLE(row_addr));  // Column high byte (0x00)
        NAND_WriteAddress(ADDR_3RD_CYCLE(row_addr));  // Row address byte 0
        NAND_WriteAddress(ADDR_4TH_CYCLE(row_addr));  // Row address byte 1
        NAND_WriteAddress(ADDR_5TH_CYCLE(row_addr));  // Row address byte 2

        // Write page data
        for (uint32_t i = 0; i < NAND_PAGE_SIZE; i++) {
            NAND_WriteData(buffer[page_index * NAND_PAGE_SIZE + i]);
        }

        // Send write command (second cycle)
        NAND_WriteCommand(NAND_CMD_WRITE_2ND);

        // Wait for write to complete
        status = NAND_GetStatus_Internal();

        if (status == NAND_OK) {
            instance->write_count++;
            page_index++;
            num_pages--;

            // Increment to next page
            NAND_IncrementAddress(&address);
        }
        else {
            instance->error_count++;
            break;
        }
    }

    return status;
}

/**
 * @brief Read data from NAND Flash page(s)
 */
nand_status_t NAND_ReadPage(nand_instance_t* instance, uint8_t* buffer, nand_address_t address, uint32_t num_pages)
{
    if (instance == NULL || !instance->initialized || buffer == NULL) {
        return NAND_ERROR;
    }

    uint32_t page_index = 0;
    nand_status_t status = NAND_OK;

    while (num_pages > 0) {
        // Validate address
        if (!NAND_IsAddressValid(address)) {
            instance->error_count++;
            return NAND_INVALID_ADDRESS;
        }

        // Calculate row address
        uint32_t row_addr = NAND_ROW_ADDRESS(address);

        // Send read command (first cycle)
        NAND_WriteCommand(NAND_CMD_READ_1ST);

        // Send 5-cycle address (column + row)
        NAND_WriteAddress(ADDR_1ST_CYCLE(row_addr));  // Column low byte (0x00)
        NAND_WriteAddress(ADDR_2ND_CYCLE(row_addr));  // Column high byte (0x00)
        NAND_WriteAddress(ADDR_3RD_CYCLE(row_addr));  // Row address byte 0
        NAND_WriteAddress(ADDR_4TH_CYCLE(row_addr));  // Row address byte 1
        NAND_WriteAddress(ADDR_5TH_CYCLE(row_addr));  // Row address byte 2

        // Send read command (second cycle)
        NAND_WriteCommand(NAND_CMD_READ_2ND);

        // Wait for data to be ready (tR = ~25μs typical)
        // Simple delay loop - could be replaced with proper timing
        for (volatile uint32_t i = 0; i < 10000; i++);

        // Read page data
        for (uint32_t i = 0; i < NAND_PAGE_SIZE; i++) {
            buffer[page_index * NAND_PAGE_SIZE + i] = NAND_ReadData();
        }

        instance->read_count++;
        page_index++;
        num_pages--;

        // Increment to next page
        status = NAND_IncrementAddress(&address);
        if (status != NAND_OK) {
            instance->error_count++;
            break;
        }
    }

    return NAND_OK;
}

/**
 * @brief Read NAND Flash status register
 */
nand_status_t NAND_ReadStatus(nand_instance_t* instance, uint8_t* status)
{
    if (instance == NULL || status == NULL) {
        return NAND_ERROR;
    }

    NAND_WriteCommand(NAND_CMD_READ_STATUS);
    *status = NAND_ReadData();

    return NAND_OK;
}

/**
 * @brief Wait for NAND Flash to become ready
 */
nand_status_t NAND_WaitReady(nand_instance_t* instance, uint32_t timeout_ms)
{
    if (instance == NULL) {
        return NAND_ERROR;
    }

    uint8_t status;
    uint32_t timeout = timeout_ms * 10000;  // Rough conversion to loop iterations

    NAND_WriteCommand(NAND_CMD_READ_STATUS);

    while (timeout--) {
        status = NAND_ReadData();

        if (status & NAND_STATUS_READY) {
            return NAND_OK;
        }
    }

    return NAND_TIMEOUT;
}

/**
 * @brief Get driver statistics
 */
nand_status_t NAND_GetStatistics(nand_instance_t* instance,
    uint32_t* read_count,
    uint32_t* write_count,
    uint32_t* erase_count,
    uint32_t* error_count)
{
    if (instance == NULL) {
        return NAND_ERROR;
    }

    if (read_count != NULL) {
        *read_count = instance->read_count;
    }
    if (write_count != NULL) {
        *write_count = instance->write_count;
    }
    if (erase_count != NULL) {
        *erase_count = instance->erase_count;
    }
    if (error_count != NULL) {
        *error_count = instance->error_count;
    }

    return NAND_OK;
}

/**
 * @brief Validate address bounds
 */
bool NAND_IsAddressValid(nand_address_t address)
{
    if (address.zone >= NAND_MAX_ZONE) {
        return false;
    }
    if (address.block >= NAND_ZONE_SIZE) {
        return false;
    }
    if (address.page >= NAND_BLOCK_SIZE) {
        return false;
    }
    return true;
}

/**
 * @brief Increment address to next page
 */
nand_status_t NAND_IncrementAddress(nand_address_t* address)
{
    if (address == NULL) {
        return NAND_ERROR;
    }

    address->page++;

    // Check for block boundary
    if (address->page >= NAND_BLOCK_SIZE) {
        address->page = 0;
        address->block++;

        // Check for zone boundary
        if (address->block >= NAND_ZONE_SIZE) {
            address->block = 0;
            address->zone++;

            // Check for device overflow
            if (address->zone >= NAND_MAX_ZONE) {
                return NAND_INVALID_ADDRESS;
            }
        }
    }

    return NAND_OK;
}
