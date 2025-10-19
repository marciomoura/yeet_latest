/******************************************************************************
 * @file    sram_driver.c
 * @brief   IS61WV25616BLL SRAM Driver Implementation
 * @author  Refactored from NSD (Nengshida) driver
 * @date    2025
 *****************************************************************************/

#include "sram_driver.h"

#include <string.h>

/******************************************************************************
 * Private Definitions
 *****************************************************************************/

/** @brief Direct memory access pointer to SRAM */
#define SRAM_MEMORY_PTR ((volatile uint16_t*)SRAM_BASE_ADDR)

/******************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

static void SRAM_InitGPIO(void);
static void SRAM_InitFSMC(void);

/******************************************************************************
 * Private Functions
 *****************************************************************************/

/**
 * @brief Initialize GPIO pins for SRAM
 *
 * Configures all necessary GPIO pins for FSMC Bank 1 NOR/SRAM interface:
 * - Data bus (D0-D15)
 * - Address bus (A0-A17)
 * - Control signals (NOE, NWE, NE1, NBL0, NBL1)
 */
static void SRAM_InitGPIO(void)
{
    GPIO_InitTypeDef gpio_init;

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);

    // Common GPIO configuration for all FSMC pins
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

    //-------------------------------------------------------------------------
    // Configure GPIOD pins
    // Data: PD0 (D2), PD1 (D3), PD8-10 (D13-15), PD14-15 (D0-1)
    // Address: PD11 (A16), PD12 (A17), PD13 (A18)
    // Control: PD4 (NOE), PD5 (NWE), PD7 (NE1)
    //-------------------------------------------------------------------------
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

    gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                         GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &gpio_init);

    //-------------------------------------------------------------------------
    // Configure GPIOE pins
    // Data: PE7-15 (D4-12)
    // Control: PE0 (NBL0), PE1 (NBL1), PE3 (A19)
    //-------------------------------------------------------------------------
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);

    gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                         GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOE, &gpio_init);

    //-------------------------------------------------------------------------
    // Configure GPIOF pins
    // Address: PF0-5 (A0-5), PF12-15 (A6-9)
    //-------------------------------------------------------------------------
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource2, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource15, GPIO_AF_FSMC);

    gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 |
                         GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOF, &gpio_init);

    //-------------------------------------------------------------------------
    // Configure GPIOG pins
    // Address: PG0-5 (A10-15), PG10 (NE3)
    //-------------------------------------------------------------------------
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource2, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource10, GPIO_AF_FSMC);

    gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10;
    GPIO_Init(GPIOG, &gpio_init);
}

/**
 * @brief Initialize FSMC for SRAM operation
 *
 * Configures FSMC Bank 1 Region 1 with timing parameters optimized for
 * IS61WV25616BLL SRAM (10ns access time).
 */
static void SRAM_InitFSMC(void)
{
    FSMC_NORSRAMInitTypeDef fsmc_init;
    FSMC_NORSRAMTimingInitTypeDef timing;

    // Enable FSMC clock
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

    // Configure timing parameters for 10ns SRAM @ 168MHz HCLK
    // HCLK period = 6ns, SRAM access = 10ns -> 2 cycles minimum
    timing.FSMC_AddressSetupTime = 0x00;         // Address setup: 1 HCLK
    timing.FSMC_AddressHoldTime = 0x00;          // Address hold: 1 HCLK
    timing.FSMC_DataSetupTime = 0x02;            // Data setup: 3 HCLK cycles
    timing.FSMC_BusTurnAroundDuration = 0x00;    // Bus turnaround: 0 HCLK
    timing.FSMC_CLKDivision = 0x00;              // Not used in async mode
    timing.FSMC_DataLatency = 0x00;              // Not used in async mode
    timing.FSMC_AccessMode = FSMC_AccessMode_A;  // Asynchronous mode A

    // Configure FSMC for SRAM
    fsmc_init.FSMC_Bank = FSMC_Bank1_NORSRAM1;                        // Bank 1, Region 1
    fsmc_init.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;      // No mux
    fsmc_init.FSMC_MemoryType = FSMC_MemoryType_SRAM;                 // SRAM type
    fsmc_init.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;        // 16-bit bus
    fsmc_init.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;    // No burst
    fsmc_init.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;  // No wait
    fsmc_init.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    fsmc_init.FSMC_WrapMode = FSMC_WrapMode_Disable;
    fsmc_init.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    fsmc_init.FSMC_WriteOperation = FSMC_WriteOperation_Enable;  // Write enabled
    fsmc_init.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    fsmc_init.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;  // Same timing for R/W
    fsmc_init.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    fsmc_init.FSMC_ReadWriteTimingStruct = &timing;
    fsmc_init.FSMC_WriteTimingStruct = &timing;

    // Initialize and enable FSMC Bank 1
    FSMC_NORSRAMInit(&fsmc_init);
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

/******************************************************************************
 * Public Functions
 *****************************************************************************/

/**
 * @brief Initialize SRAM driver and hardware
 */
sram_status_t SRAM_Init(sram_instance_t* instance)
{
    if (instance == NULL) {
        return SRAM_ERROR;
    }

    // Clear instance structure
    memset(instance, 0, sizeof(sram_instance_t));

    // Initialize hardware
    SRAM_InitGPIO();
    SRAM_InitFSMC();

    // Store configuration
    instance->base_address = SRAM_BASE_ADDR;
    instance->initialized = true;

    return SRAM_OK;
}

/**
 * @brief Write 16-bit data to SRAM
 */
sram_status_t SRAM_Write(sram_instance_t* instance, const uint16_t* buffer, uint32_t address, uint32_t length)
{
    if (instance == NULL || !instance->initialized || buffer == NULL) {
        return SRAM_ERROR;
    }

    if (!SRAM_IsAddressValid(address, length)) {
        return SRAM_INVALID_ADDRESS;
    }

    // Direct memory writes
    volatile uint16_t* sram_ptr = SRAM_MEMORY_PTR + address;
    for (uint32_t i = 0; i < length; i++) {
        *sram_ptr++ = buffer[i];
    }

    instance->write_count++;

    return SRAM_OK;
}

/**
 * @brief Read 16-bit data from SRAM
 */
sram_status_t SRAM_Read(sram_instance_t* instance, uint16_t* buffer, uint32_t address, uint32_t length)
{
    if (instance == NULL || !instance->initialized || buffer == NULL) {
        return SRAM_ERROR;
    }

    if (!SRAM_IsAddressValid(address, length)) {
        return SRAM_INVALID_ADDRESS;
    }

    // Direct memory reads
    volatile uint16_t* sram_ptr = SRAM_MEMORY_PTR + address;
    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = *sram_ptr++;
    }

    instance->read_count++;

    return SRAM_OK;
}

/**
 * @brief Write single 16-bit word to SRAM
 */
sram_status_t SRAM_WriteSingle(sram_instance_t* instance, uint32_t address, uint16_t data)
{
    if (instance == NULL || !instance->initialized) {
        return SRAM_ERROR;
    }

    if (!SRAM_IsAddressValid(address, 1)) {
        return SRAM_INVALID_ADDRESS;
    }

    SRAM_MEMORY_PTR[address] = data;
    instance->write_count++;

    return SRAM_OK;
}

/**
 * @brief Read single 16-bit word from SRAM
 */
sram_status_t SRAM_ReadSingle(sram_instance_t* instance, uint32_t address, uint16_t* data)
{
    if (instance == NULL || !instance->initialized || data == NULL) {
        return SRAM_ERROR;
    }

    if (!SRAM_IsAddressValid(address, 1)) {
        return SRAM_INVALID_ADDRESS;
    }

    *data = SRAM_MEMORY_PTR[address];
    instance->read_count++;

    return SRAM_OK;
}

/**
 * @brief Fill SRAM region with a pattern
 */
sram_status_t SRAM_Fill(sram_instance_t* instance, uint32_t address, uint32_t length, uint16_t pattern)
{
    if (instance == NULL || !instance->initialized) {
        return SRAM_ERROR;
    }

    if (!SRAM_IsAddressValid(address, length)) {
        return SRAM_INVALID_ADDRESS;
    }

    volatile uint16_t* sram_ptr = SRAM_MEMORY_PTR + address;
    for (uint32_t i = 0; i < length; i++) {
        *sram_ptr++ = pattern;
    }

    instance->write_count++;

    return SRAM_OK;
}

/**
 * @brief Test SRAM memory integrity
 */
sram_status_t SRAM_Test(sram_instance_t* instance, uint32_t address, uint32_t length)
{
    if (instance == NULL || !instance->initialized) {
        return SRAM_ERROR;
    }

    if (!SRAM_IsAddressValid(address, length)) {
        return SRAM_INVALID_ADDRESS;
    }

    // Allocate temporary buffer for backup (limit test size)
    if (length > 1024) {
        length = 1024;  // Limit to 2KB test
    }

    uint16_t backup[1024];
    uint16_t readback[1024];

    // Step 1: Backup original data
    sram_status_t status = SRAM_Read(instance, backup, address, length);
    if (status != SRAM_OK) {
        return status;
    }

    // Step 2: Write test pattern 0xAAAA
    status = SRAM_Fill(instance, address, length, 0xAAAA);
    if (status != SRAM_OK) {
        return status;
    }

    // Step 3: Read back and verify
    status = SRAM_Read(instance, readback, address, length);
    if (status != SRAM_OK) {
        return status;
    }

    for (uint32_t i = 0; i < length; i++) {
        if (readback[i] != 0xAAAA) {
            // Restore original data before returning error
            SRAM_Write(instance, backup, address, length);
            return SRAM_ERROR;
        }
    }

    // Step 4: Write test pattern 0x5555
    status = SRAM_Fill(instance, address, length, 0x5555);
    if (status != SRAM_OK) {
        return status;
    }

    // Step 5: Read back and verify
    status = SRAM_Read(instance, readback, address, length);
    if (status != SRAM_OK) {
        return status;
    }

    for (uint32_t i = 0; i < length; i++) {
        if (readback[i] != 0x5555) {
            // Restore original data before returning error
            SRAM_Write(instance, backup, address, length);
            return SRAM_ERROR;
        }
    }

    // Step 6: Restore original data
    status = SRAM_Write(instance, backup, address, length);

    return status;
}

/**
 * @brief Get driver statistics
 */
sram_status_t SRAM_GetStatistics(sram_instance_t* instance, uint32_t* read_count, uint32_t* write_count)
{
    if (instance == NULL) {
        return SRAM_ERROR;
    }

    if (read_count != NULL) {
        *read_count = instance->read_count;
    }
    if (write_count != NULL) {
        *write_count = instance->write_count;
    }

    return SRAM_OK;
}

/**
 * @brief Validate address range
 */
bool SRAM_IsAddressValid(uint32_t address, uint32_t length)
{
    // Check for overflow
    if (length == 0) {
        return false;
    }

    // Check if address + length exceeds SRAM size
    if (address > SRAM_MAX_ADDRESS) {
        return false;
    }

    if ((address + length - 1) > SRAM_MAX_ADDRESS) {
        return false;
    }

    return true;
}
