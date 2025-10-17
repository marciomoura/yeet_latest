/**
 ******************************************************************************
 * @file    watchdog_simple.h
 * @brief   Simple watchdog interface for STM32F407
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * Simple watchdog driver with minimal configuration.
 * Just initialize once and feed regularly - that's it!
 *
 * Usage:
 *   1. Call Watchdog_Init() once at startup
 *   2. Call Watchdog_Feed() regularly (every 1ms, 10ms, or 100ms)
 *   3. If you forget to feed, the system resets automatically
 *
 * Example:
 *   Watchdog_Init();  // Initialize with 1 second timeout
 *
 *   while(1) {
 *       // Your application code here
 *       HAL_Delay(10);
 *       Watchdog_Feed();  // Feed every 10ms
 *   }
 *
 ******************************************************************************
 */

#ifndef WATCHDOG_SIMPLE_H
#define WATCHDOG_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize watchdog with default 1000ms timeout
 *
 * Sets up the independent watchdog with a 1 second timeout.
 * The watchdog starts immediately after calling this function.
 *
 * @note You MUST call Watchdog_Feed() regularly after this!
 * @warning Once enabled, the watchdog cannot be disabled!
 */
void Watchdog_Init(void);

/**
 * @brief Initialize watchdog with custom timeout
 *
 * @param timeout_ms Timeout in milliseconds (1 to 4095)
 *
 * @note Common values: 100ms, 500ms, 1000ms, 2000ms
 * @warning Once enabled, the watchdog cannot be disabled!
 */
void Watchdog_Init_Custom(uint16_t timeout_ms);

/**
 * @brief Feed the watchdog to prevent reset
 *
 * Call this function regularly (faster than the timeout period)
 * to prevent the watchdog from resetting the system.
 *
 * @note This should be called from your main loop
 * @note Safe to call from interrupts
 */
void Watchdog_Feed(void);

/**
 * @brief Check if last reset was caused by watchdog
 *
 * @return true if watchdog caused the last reset
 * @return false if reset was caused by something else
 *
 * @note Call this during initialization to detect watchdog resets
 */
bool Watchdog_WasLastResetByWatchdog(void);

/**
 * @brief Clear the watchdog reset flag
 *
 * Call this after checking the reset cause to clear the flag.
 */
void Watchdog_ClearResetFlag(void);

/**
 * @brief Get number of times watchdog was fed
 *
 * @return Feed counter value
 *
 * @note Useful for debugging and monitoring
 */
uint32_t Watchdog_GetFeedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_SIMPLE_H */
