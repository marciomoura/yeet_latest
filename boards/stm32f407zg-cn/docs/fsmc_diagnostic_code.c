/**
 * @file fsmc_diagnostic.c
 * @brief FSMC Diagnostic Functions for FPGA Communication Testing
 * @date 2025-10-14
 *
 * Add these functions to your main.c or create a separate diagnostic module
 */

#include <stdio.h>

#include "main.h"

/* Diagnostic function prototypes */
void FSMC_DiagnosticCheck(void);
void FSMC_TestPattern(void);
uint32_t FSMC_ReadConfig(void);

/**
 * @brief Comprehensive FSMC diagnostic check
 * @note Call this after MX_FSMC_Init() in main()
 */
void FSMC_DiagnosticCheck(void)
{
    // Check 1: FSMC Clock Enabled
    if (!__HAL_RCC_FSMC_IS_CLK_ENABLED()) {
        // ERROR: FSMC clock not enabled!
        // Set breakpoint here or light an error LED
        while (1);
    }

    // Check 2: Read FSMC Bank 4 Configuration
    uint32_t bcr4 = FSMC_Bank1->BTCR[6];  // Bank Control Register 4
    uint32_t btr4 = FSMC_Bank1->BTCR[7];  // Bank Timing Register 4

    // Verify Bank 4 is enabled (bit 0 = MBKEN)
    if ((bcr4 & FSMC_BCR1_MBKEN) == 0) {
        // ERROR: Bank 4 not enabled
        while (1);
    }

    // Verify Write Operation enabled (bit 12 = WREN)
    if ((bcr4 & FSMC_BCR1_WREN) == 0) {
        // ERROR: Write operation not enabled for Bank 4
        while (1);
    }

    // Check 3: Verify GPIO Alternate Function for NE4 (PG12)
    uint32_t gpiog_moder = GPIOG->MODER;
    uint32_t pg12_mode = (gpiog_moder >> (12 * 2)) & 0x3;
    if (pg12_mode != 0x2) {  // Should be 0b10 (Alternate Function)
        // ERROR: PG12 not in AF mode
        while (1);
    }

    // Check 4: Verify AF12 (FSMC) is selected for PG12
    uint32_t gpiog_afrh = GPIOG->AFR[1];  // High register (pins 8-15)
    uint32_t pg12_af = (gpiog_afrh >> ((12 - 8) * 4)) & 0xF;
    if (pg12_af != 12) {  // Should be AF12 (FSMC)
        // ERROR: PG12 not configured for FSMC
        while (1);
    }

    // Check 5: Verify timing parameters are reasonable
    uint32_t addset = (btr4 & FSMC_BTR1_ADDSET) >> 0;
    uint32_t datast = (btr4 & FSMC_BTR1_DATAST) >> 8;

    // Expected values: ADDSET=6, DATAST=21
    // If values are 0 or 255, something is wrong
    if (addset == 0 || addset > 15 || datast == 0 || datast > 255) {
        // ERROR: Invalid timing configuration
        while (1);
    }

    // All checks passed!
    // Optional: Blink LED or send debug message
}

/**
 * @brief Test FSMC with simple write patterns
 * @note Call this to verify FSMC writes are reaching FPGA
 */
void FSMC_TestPattern(void)
{
    volatile uint16_t *fpga_led = (volatile uint16_t *)0x6C00000E;

    // Test 1: All zeros
    *fpga_led = 0x0000;
    HAL_Delay(200);

    // Test 2: Each LED individually
    *fpga_led = 0x0001;  // LED0 only
    HAL_Delay(200);

    *fpga_led = 0x0002;  // LED1 only
    HAL_Delay(200);

    *fpga_led = 0x0004;  // LED2 only
    HAL_Delay(200);

    // Test 3: Binary counting pattern
    for (int i = 0; i < 8; i++) {
        *fpga_led = i;
        HAL_Delay(200);
    }

    // Test 4: Walking bit pattern
    for (int bit = 0; bit < 3; bit++) {
        *fpga_led = (1 << bit);
        HAL_Delay(200);
    }

    // Test 5: Alternating pattern
    for (int i = 0; i < 10; i++) {
        *fpga_led = 0x0005;  // LED0 and LED2
        HAL_Delay(100);
        *fpga_led = 0x0002;  // LED1
        HAL_Delay(100);
    }

    // Final: All on
    *fpga_led = 0x0007;
}

/**
 * @brief Read FSMC configuration and return diagnostic info
 * @return 32-bit diagnostic code:
 *         Bit 0: FSMC clock enabled
 *         Bit 1: Bank 4 enabled
 *         Bit 2: Write enabled
 *         Bit 3: PG12 in AF mode
 *         Bits 8-15: Address setup time
 *         Bits 16-23: Data setup time
 */
uint32_t FSMC_ReadConfig(void)
{
    uint32_t result = 0;

    // Bit 0: FSMC clock
    if (__HAL_RCC_FSMC_IS_CLK_ENABLED()) {
        result |= (1 << 0);
    }

    // Read Bank 4 registers
    uint32_t bcr4 = FSMC_Bank1->BTCR[6];
    uint32_t btr4 = FSMC_Bank1->BTCR[7];

    // Bit 1: Bank enabled
    if (bcr4 & FSMC_BCR1_MBKEN) {
        result |= (1 << 1);
    }

    // Bit 2: Write enabled
    if (bcr4 & FSMC_BCR1_WREN) {
        result |= (1 << 2);
    }

    // Bit 3: PG12 in AF mode
    uint32_t pg12_mode = (GPIOG->MODER >> (12 * 2)) & 0x3;
    if (pg12_mode == 0x2) {
        result |= (1 << 3);
    }

    // Bits 8-15: Address setup time
    uint32_t addset = (btr4 & FSMC_BTR1_ADDSET) >> 0;
    result |= (addset << 8);

    // Bits 16-23: Data setup time
    uint32_t datast = (btr4 & FSMC_BTR1_DATAST) >> 8;
    result |= (datast << 16);

    return result;
}

/**
 * @brief Try different addresses to find FPGA
 * @note Use this if you're not sure of the correct address
 */
void FSMC_AddressScan(void)
{
    // Scan common word offsets in Bank 4
    for (uint32_t offset = 0; offset < 256; offset += 2) {
        volatile uint16_t *addr = (volatile uint16_t *)(0x6C000000 + offset);

        // Try writing test pattern
        *addr = 0x5555;
        HAL_Delay(10);
        *addr = 0xAAAA;
        HAL_Delay(10);
        *addr = 0x0000;
        HAL_Delay(10);

        // If LEDs responded, this might be the right address
        // Set breakpoint here and check 'offset' value
    }
}

/**
 * @brief Read FPGA digital inputs (if implemented)
 * @note Use this to test FSMC read operations
 */
uint16_t FSMC_TestRead(void)
{
    // Try reading from input register address
    volatile uint16_t *fpga_input = (volatile uint16_t *)0x6C000192;

    // Perform read
    uint16_t value = *fpga_input;

    // If FPGA is connected and responding:
    // - value should contain input data
    // If FPGA is not connected:
    // - value will be 0xFFFF (floating bus) or 0x0000
    // - or CPU might hang (if FPGA doesn't respond)

    return value;
}

/**
 * @brief Minimal test - use this in main() for basic verification
 */
void FSMC_MinimalTest(void)
{
    // 1. Check if FSMC is initialized
    FSMC_DiagnosticCheck();

    // 2. Try simple LED pattern
    volatile uint16_t *led = (volatile uint16_t *)0x6C00000E;

    while (1) {
        // Blink pattern: 001 -> 010 -> 100 -> 111 -> 000
        *led = 0x0001;
        HAL_Delay(250);
        *led = 0x0002;
        HAL_Delay(250);
        *led = 0x0004;
        HAL_Delay(250);
        *led = 0x0007;
        HAL_Delay(250);
        *led = 0x0000;
        HAL_Delay(250);
    }
}

/*
 * Example usage in main():
 *
 * int main(void) {
 *     HAL_Init();
 *     SystemClock_Config();
 *     MX_GPIO_Init();
 *     MX_DMA_Init();
 *     MX_FSMC_Init();
 *
 *     // Run diagnostic
 *     FSMC_DiagnosticCheck();
 *
 *     // Get config info
 *     uint32_t config = FSMC_ReadConfig();
 *     // Expected: 0x00150F0F (if properly configured)
 *     // Bit 0-3 should all be 1
 *     // Bits 8-15 = 0x06 (AddressSetupTime)
 *     // Bits 16-23 = 0x15 (21 decimal, DataSetupTime)
 *
 *     // Test patterns
 *     FSMC_TestPattern();
 *
 *     // If nothing works, scan for correct address
 *     // FSMC_AddressScan();
 *
 *     // Or run minimal continuous test
 *     FSMC_MinimalTest();  // Never returns
 * }
 */
