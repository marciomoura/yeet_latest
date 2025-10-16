/**
 ******************************************************************************
 * @file    watchdog.c (LEGACY)
 * @brief   Legacy watchdog wrapper for backward compatibility
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * This is a LEGACY compatibility wrapper. New code should use iwdg_driver.h
 *
 ******************************************************************************
 */

#include "iwdg_driver.h"
#include "stm32f4xx.h"

// Legacy global instance
static iwdg_instance_t legacy_iwdg_instance;

/********************************************************************************
 * WatchDog_Init (LEGACY)
 * Initialize watchdog with legacy interface
 * time: Reload time value, max 0xFFF, unit in ms
 ********************************************************************************/
void WatchDog_Init(uint16_t time)
{
    iwdg_config_t config;

    // Configure with legacy parameters
    config.prescaler = IWDG_PRESCALER_32;  // 32KHz/32 = 1KHz
    config.timeout_ms = time;
    config.stop_in_debug = true;

    // Initialize driver
    IWDG_Init(&legacy_iwdg_instance, &config);

    // Enable watchdog immediately (legacy behavior)
    IWDG_Enable(&legacy_iwdg_instance);
}

/********************************************************************************
 * Feed_Dog (LEGACY)
 * Feed the watchdog
 ********************************************************************************/
void Feed_Dog(void)
{
    // Feed without timestamp (legacy behavior)
    IWDG_Feed(&legacy_iwdg_instance, 0);
}

//===========================================================================
// End of file.
//===========================================================================
