/**
 ******************************************************************************
 * @file    driver_init.h
 * @author  Techperts Team
 * @version V1.0.0
 * @date    17-Oct-2025
 * @brief   Hardware Driver Initialization Header
 ******************************************************************************
 * @attention
 *
 * This module provides hardware peripheral initialization functions for the
 * STM32F407ZG-CN board. Application-specific initialization has been removed.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVER_INIT_H
#define __DRIVER_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "fsmc_gpio_driver.h"  // For fsmc_gpio_instance_t

/* Exported variables --------------------------------------------------------*/

/**
 * @brief Node ID read from DIP switches (0-15)
 */
extern uint8_t uchNodeNum;

/**
 * @brief FSMC GPIO driver instance (for LEDs and digital I/O via FSMC Bank 4)
 * @note  This should be defined in the application (e.g., main.c)
 */
extern fsmc_gpio_instance_t fsmc_gpio_instance;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize DIP switch GPIO pins (PC6-PC9)
 * @note   DIP switches are used to set the node ID (4-bit value)
 */
void DIP_Switch_Init(void);

/**
 * @brief  Read node ID from DIP switches
 * @retval Node ID value (0-15)
 * @note   DIP switches use inverted logic (ON = 0, OFF = 1)
 */
uint8_t Read_DIP_Switch(void);

/**
 * @brief  Initialize all hardware peripherals
 * @note   This function initializes:
 *         - DIP switches and reads node ID
 *         - Timers (must be first for delay functions)
 *         - FPGA (if present)
 *         - FSMC GPIO
 *         - SRAM
 *         - ADC (AD7606)
 *         - Flash memory (M25P32)
 *         - Watchdog
 * @note   Call this function before using any hardware peripherals
 * @note   Sets FAULT LED if any critical initialization fails
 * @retval 0 if successful, -1 if any initialization failed
 */
int Init_Hardware(void);

/**
 * @brief  Initialize application data structures
 * @note   Minimal version - user applications should implement their own
 *         Initializes ADC data buffers only
 */
void Init_Data(void);

/**
 * @brief  Convert 16-bit integer to two bytes (big-endian)
 * @param  n: 16-bit integer to convert
 * @param  Data: Pointer to 2-byte buffer for output
 */
void intToChar(int16_t n, uint8_t* Data);

/**
 * @brief  Convert two bytes to 16-bit integer (big-endian)
 * @param  Data: Pointer to 2-byte buffer containing data
 * @retval Converted 16-bit integer
 */
int16_t charToInt(uint8_t* Data);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_INIT_H */

/******************************************************************************
 * END OF FILE
 ******************************************************************************/
