/**
 ******************************************************************************
 * @file    WatchDog.h (LEGACY)
 * @brief   Legacy watchdog interface for backward compatibility
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * This is a LEGACY compatibility wrapper. New code should use iwdg_driver.h
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _WATCHDOG_H
#define _WATCHDOG_H

#include <stdint.h>

/**
 * @brief Initialize watchdog timer (LEGACY)
 * @param time Timeout in milliseconds (max 0xFFF = 4095ms)
 * @note This function enables the watchdog immediately
 * @warning Once enabled, the watchdog CANNOT be disabled!
 */
void WatchDog_Init(uint16_t time);

/**
 * @brief Feed the watchdog to prevent reset (LEGACY)
 * @note This function MUST be called regularly before timeout
 */
void Feed_Dog(void);

#endif
