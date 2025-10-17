/**
 ******************************************************************************
 * @file    driver_init.c
 * @author  Techperts Team
 * @version V1.0.0
 * @date    17-Oct-2025
 * @brief   Hardware Driver Initialization Implementation
 ******************************************************************************
 * @attention
 *
 * This module initializes all hardware peripherals for the STM32F407ZG-CN
 * board. Application-specific control logic has been removed.
 *
 * Initialization Order (CRITICAL):
 * 1. DIP switches (read node ID)
 * 2. Timers (required for delay functions)
 * 3. FPGA (if present, uses timer delays)
 * 4. FSMC GPIO
 * 5. SRAM
 * 6. ADC
 * 7. Flash memory
 * 8. Watchdog
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "driver_init.h"

#include "adc.h"               // ADC driver (AD7606)
#include "fpga.h"              // FPGA configuration
#include "fsmc_gpio_driver.h"  // FSMC GPIO driver
#include "nand_driver.h"       // NAND Flash driver
#include "sram_driver.h"       // SRAM driver
#include "stm32f4xx.h"
#include "tasks.h"            // Timer tasks
#include "watchdog_simple.h"  // Watchdog driver

/* Exported variables --------------------------------------------------------*/

/**
 * @brief Node ID read from DIP switches (0-15)
 */
uint8_t uchNodeNum = 0;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Convert 16-bit integer to two bytes (big-endian)
 * @param  n: 16-bit integer to convert
 * @param  Data: Pointer to 2-byte buffer for output
 */
void intToChar(int16_t n, uint8_t* Data)
{
    *Data++ = (n >> 8) & 0xFF;
    *Data = n & 0xFF;
}

/**
 * @brief  Convert two bytes to 16-bit integer (big-endian)
 * @param  Data: Pointer to 2-byte buffer containing data
 * @retval Converted 16-bit integer
 */
int16_t charToInt(uint8_t* Data)
{
    int16_t i;
    i = *Data++;
    i <<= 8;
    return i | *Data;
}

/**
 * @brief  Initialize DIP switch GPIO pins (PC6-PC9)
 * @note   DIP switches are configured as inputs with pull-ups
 *         PC6 = Bit 0, PC7 = Bit 1, PC8 = Bit 2, PC9 = Bit 3
 */
void DIP_Switch_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIOC clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // Configure PC6, PC7, PC8, PC9 as inputs with pull-ups
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 * @brief  Read node ID from DIP switches
 * @retval Node ID value (0-15)
 * @note   DIP switches use inverted logic:
 *         - Switch ON (closed) = GPIO reads LOW (0)
 *         - Switch OFF (open) = GPIO reads HIGH (1)
 *         This function inverts the logic so ON = 1 in the returned value
 */
uint8_t Read_DIP_Switch(void)
{
    uint8_t node_id = 0;

    // Read 4 bits from PC6-PC9 (inverted logic: LOW = ON = 1)
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6) == 0) {
        node_id |= 0x01;  // Bit 0
    }
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == 0) {
        node_id |= 0x02;  // Bit 1
    }
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) == 0) {
        node_id |= 0x04;  // Bit 2
    }
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == 0) {
        node_id |= 0x08;  // Bit 3
    }

    return node_id;
}

/**
 * @brief  Initialize all hardware peripherals
 * @note   CRITICAL: Initialization order matters!
 *         Timers must be initialized before FPGA (for delay functions)
 * @retval 0 if successful, non-zero if any initialization failed
 */
int Init_Hardware(void)
{
    FPGA_Status_t fpga_status;
    bool init_success = true;

    // 1. Read board node ID from DIP switches
    DIP_Switch_Init();
    uchNodeNum = Read_DIP_Switch();

    // 2. Initialize timers FIRST (required for FPGA delay functions)
    TIMx_Init();

    // 3. Initialize FPGA (if present, with timeout protection)
    FPGA_Init_Pins();
    fpga_status = FPGA_Configure(5000);  // 5 second timeout

    if (fpga_status == FPGA_CONFIG_SUCCESS) {
        // FPGA configured successfully - perform internal reset
        FPGA_Reset();
    }
    else {
        // FPGA configuration failed/timeout
        // System can continue but FPGA-dependent features won't work
        // (e.g., optical fiber, some LEDs)
        init_success = false;
    }

    // 4. Initialize FSMC GPIO (must be after FPGA for some boards)
    // This enables access to LEDs and digital I/O via FSMC Bank 4 (0x6C000000)
    if (!FSMC_GPIO_Init(&fsmc_gpio_instance)) {
        init_success = false;
    }

    // Set FAULT LED if any initialization failed
    if (!init_success) {
        FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
        FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
    }

    // 5. Initialize external SRAM
    // Note: This uses a shared global instance - to be refactored
    // if (!SRAM_Init(&sram)) {
    //     FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
    //     FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
    // }

    // 6. Initialize external ADC (AD7606)
    // Note: This uses a shared global instance - to be refactored
    // if (!AD7606_Init(&adc)) {
    //     FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
    //     FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
    // }

    // 7. Initialize NAND Flash
    // Note: This uses a shared global instance - to be refactored
    // if (NAND_Init(&nand) != NAND_OK) {
    //     FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
    //     FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
    // }

    // 8. Initialize watchdog (no parameters needed)
    // Watchdog_Init();

    // Note: The following were removed as they depend on archived files:
    // - Init_Net() - Network initialization (archived net.c)
    // - Init_T() - Temperature polling (depends on USART and optical drivers)
    // - Init_RS232_Comm() - Serial communication (archived)
    // - Init_RTC() - Real-time clock (implement if needed)

    return init_success ? 0 : -1;
}

/**
 * @brief  Initialize application data structures
 * @note   Minimal version - applications should implement their own initialization
 *         - Control algorithm parameters
 *         - Network buffers
 *         - Application state machines
 */
void Init_Data(void)
{
    // TODO: User applications should implement their own data initialization
    // Example:
    // - Initialize ADC buffers
    // - Initialize control parameters
    // - Initialize network buffers
}

/******************************************************************************
 * END OF FILE
 ******************************************************************************/
