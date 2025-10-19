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
 * @brief FSMC GPIO driver instance (for LEDs and digital I/O via FSMC Bank 4)
 * @note  This should be defined in the application (e.g., main.c)
 */
extern fsmc_gpio_instance_t fsmc_gpio_instance;

/* Exported functions --------------------------------------------------------*/

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
