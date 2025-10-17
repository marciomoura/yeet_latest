/**
 ******************************************************************************
 * @file    fpga.c
 * @author  Techperts Team
 * @version V1.0.0
 * @date    17-Oct-2025
 * @brief   FPGA Configuration Driver Implementation
 ******************************************************************************
 * @attention
 *
 * This module handles FPGA initialization and configuration for the
 * STM32F407ZG-CN board. The FPGA handles:
 * - Optical fiber communication interface
 * - FSMC Bank 4 peripherals (LEDs, I/O)
 * - Various control and data processing functions
 *
 * Configuration Process:
 * 1. Initialize control pins (PROGRAM_B, DONE, RST)
 * 2. Pulse PROGRAM_B low to trigger configuration
 * 3. Wait for DONE signal to go high (indicates success)
 * 4. Perform internal reset sequence
 *
 * The FPGA loads its bitstream from external flash memory. If the flash
 * is empty or corrupt, configuration will fail and timeout.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "fpga.h"

#include "stm32f4xx.h"
#include "tasks.h"  // For timer-based delays

/* Private defines -----------------------------------------------------------*/

/* FPGA Control Pins */
#define FPGA_PROGRAM_B_PORT GPIOG
#define FPGA_PROGRAM_B_PIN  GPIO_Pin_15  // PROGRAM_B: Configuration trigger (active low)

#define FPGA_DONE_PORT GPIOD
#define FPGA_DONE_PIN  GPIO_Pin_2  // DONE: Configuration complete indicator (active high)

#define FPGA_RST_PORT GPIOG
#define FPGA_RST_PIN  GPIO_Pin_11  // Internal FPGA reset

#define FPGA_INITB_PORT GPIOE
#define FPGA_INITB_PIN  GPIO_Pin_3  // INIT_B: Optional status pin (not used)

/* Configuration retry interval */
#define FPGA_RETRY_INTERVAL_MS 100  // Retry FPGA reset every 100ms

/* Private function prototypes -----------------------------------------------*/
static void delay_ms(uint32_t ms);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Timer-based millisecond delay function
 * @param  ms: Delay time in milliseconds
 * @note   Requires TIMx_Init() to be called first to start the 1ms timer
 */
static void delay_ms(uint32_t ms)
{
    uint32_t start = get_1ms_counter();
    while ((get_1ms_counter() - start) < ms) {
        __NOP();
    }
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize FPGA internal reset pin (PG11)
 * @note   Configures PG11 as push-pull output for internal FPGA reset control
 */
void FPGA_RST_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    GPIO_InitStructure.GPIO_Pin = FPGA_RST_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(FPGA_RST_PORT, &GPIO_InitStructure);
}

/**
 * @brief  Initialize FPGA INIT_B input pin (PE3)
 * @note   Configures PE3 as input for FPGA INIT_B status signal
 *         Currently not used in configuration logic
 */
void FPGA_INITB_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = FPGA_INITB_PIN;
    GPIO_Init(FPGA_INITB_PORT, &GPIO_InitStructure);
}

/**
 * @brief  Initialize FPGA DONE input pin (PD2)
 * @note   Configures PD2 as input for FPGA configuration complete signal
 *         DONE goes HIGH when FPGA has successfully loaded its bitstream
 */
void FPGA_DONE_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = FPGA_DONE_PIN;
    GPIO_Init(FPGA_DONE_PORT, &GPIO_InitStructure);
}

/**
 * @brief  Initialize FPGA PROGRAM_B output pin (PG15)
 * @note   Configures PG15 as push-pull output for FPGA configuration trigger
 *         Pulsing this pin LOW initiates FPGA reconfiguration from flash
 */
void FPGA_PROGRAMB_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    GPIO_InitStructure.GPIO_Pin = FPGA_PROGRAM_B_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(FPGA_PROGRAM_B_PORT, &GPIO_InitStructure);
}

/**
 * @brief  Initialize all FPGA control pins
 * @note   Convenience function that initializes RST, DONE, and PROGRAM_B pins
 */
void FPGA_Init_Pins(void)
{
    FPGA_RST_Init();
    FPGA_DONE_Init();
    FPGA_PROGRAMB_Init();
}

/**
 * @brief  Configure FPGA and wait for DONE signal with timeout protection
 * @param  timeout_ms: Maximum time to wait for FPGA configuration (milliseconds)
 * @retval FPGA_CONFIG_SUCCESS: FPGA configured successfully
 *         FPGA_CONFIG_TIMEOUT: FPGA failed to configure within timeout
 * @note   This function will:
 *         1. Trigger FPGA configuration by pulsing PROGRAM_B low
 *         2. Wait for DONE signal to go high (indicates success)
 *         3. Retry configuration every 100ms if DONE doesn't go high
 *         4. Return timeout error if FPGA doesn't configure within timeout_ms
 */
FPGA_Status_t FPGA_Configure(uint32_t timeout_ms)
{
    uint32_t start_time = get_1ms_counter();
    uint32_t elapsed_time;

    // Initial reset of FPGA via PROGRAM_B (pulse low for ~1ms)
    GPIO_ResetBits(FPGA_PROGRAM_B_PORT, FPGA_PROGRAM_B_PIN);
    delay_ms(1);
    GPIO_SetBits(FPGA_PROGRAM_B_PORT, FPGA_PROGRAM_B_PIN);

    // Wait for FPGA to configure with timeout protection
    while (GPIO_ReadInputDataBit(FPGA_DONE_PORT, FPGA_DONE_PIN) != 1) {
        // Check timeout
        elapsed_time = get_1ms_counter() - start_time;
        if (elapsed_time >= timeout_ms) {
            // Timeout - FPGA failed to configure
            return FPGA_CONFIG_TIMEOUT;
        }

        // Retry FPGA reset every 100ms
        if ((elapsed_time % FPGA_RETRY_INTERVAL_MS) == 0) {
            GPIO_ResetBits(FPGA_PROGRAM_B_PORT, FPGA_PROGRAM_B_PIN);
            delay_ms(1);
            GPIO_SetBits(FPGA_PROGRAM_B_PORT, FPGA_PROGRAM_B_PIN);
        }

        // Small delay to avoid tight polling
        delay_ms(10);
    }

    // FPGA configured successfully
    return FPGA_CONFIG_SUCCESS;
}

/**
 * @brief  Perform internal FPGA reset sequence
 * @note   This resets the internal logic of the FPGA after configuration
 *         Uses PG11 pin to pulse the internal reset signal
 */
void FPGA_Reset(void)
{
    // Assert reset (HIGH)
    GPIO_SetBits(FPGA_RST_PORT, FPGA_RST_PIN);
    delay_ms(1);

    // Release reset (LOW)
    GPIO_ResetBits(FPGA_RST_PORT, FPGA_RST_PIN);
    delay_ms(1);

    // Return to idle state (HIGH)
    GPIO_SetBits(FPGA_RST_PORT, FPGA_RST_PIN);
}

/**
 * @brief  Check if FPGA configuration is complete
 * @retval 1: FPGA is configured (DONE pin is HIGH)
 *         0: FPGA is not configured (DONE pin is LOW)
 */
uint8_t FPGA_Is_Configured(void) { return (GPIO_ReadInputDataBit(FPGA_DONE_PORT, FPGA_DONE_PIN) == 1) ? 1 : 0; }

/******************************************************************************
 * END OF FILE
 ******************************************************************************/
