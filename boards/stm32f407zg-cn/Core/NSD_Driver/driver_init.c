/**
 ******************************************************************************
 * @file    driver_init.c
 * @author  Techperts Team
 * @version V1.0.0
 * @date    17-Oct-2025
 * @brief   Hardware Driver Utility Functions
 ******************************************************************************
 * @attention
 *
 * This module provides utility functions for byte/integer conversion.
 * Hardware initialization has been moved to the application layer (main.c)
 * for better visibility and control.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "driver_init.h"

#include "stm32f4xx.h"

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

/******************************************************************************
 * END OF FILE
 ******************************************************************************/
