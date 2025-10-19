/**
 ******************************************************************************
 * @file    watchdog_simple.c
 * @brief   Simple watchdog implementation for STM32F407
 * @date    2025
 ******************************************************************************
 */

#include "watchdog_simple.h"

#include "stm32f4xx.h"

/* ============================================================================
 * Private Constants
 * ============================================================================ */

/** @brief IWDG register keys */
#define IWDG_KEY_RELOAD       0xAAAA /**< Reload counter */
#define IWDG_KEY_ENABLE       0xCCCC /**< Enable watchdog */
#define IWDG_KEY_WRITE_ACCESS 0x5555 /**< Enable register writes */

/** @brief Default timeout (milliseconds) */
#define DEFAULT_TIMEOUT_MS 1000

/** @brief Prescaler value (LSI/32 = 1kHz) */
#define PRESCALER_VALUE 3 /**< Prescaler = 32 */

/* ============================================================================
 * Private Variables
 * ============================================================================ */

static uint32_t feed_counter = 0;
static bool initialized = false;

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Configure watchdog hardware
 * @param timeout_ms Timeout in milliseconds
 */
static void Configure_Watchdog(uint16_t timeout_ms)
{
    // Limit timeout to valid range
    if (timeout_ms < 1) {
        timeout_ms = 1;
    }
    if (timeout_ms > 4095) {
        timeout_ms = 4095;
    }

    // Calculate reload value (LSI = 32kHz, Prescaler = 32, so 1kHz counter)
    uint16_t reload = timeout_ms;

    // Enable write access to IWDG registers
    IWDG->KR = IWDG_KEY_WRITE_ACCESS;

    // Set prescaler to 32 (LSI/32 = 1kHz)
    IWDG->PR = PRESCALER_VALUE;

    // Set reload value
    IWDG->RLR = reload;

    // Reload counter
    IWDG->KR = IWDG_KEY_RELOAD;

    // Enable watchdog - THIS CANNOT BE UNDONE!
    IWDG->KR = IWDG_KEY_ENABLE;

    // Stop watchdog during debug to allow debugging
    DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, ENABLE);
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

/**
 * @brief Initialize watchdog with default 1000ms timeout
 */
void Watchdog_Init(void)
{
    if (initialized) {
        return;  // Already initialized
    }

    Configure_Watchdog(DEFAULT_TIMEOUT_MS);
    initialized = true;
    feed_counter = 0;
}

/**
 * @brief Initialize watchdog with custom timeout
 */
void Watchdog_Init_Custom(uint16_t timeout_ms)
{
    if (initialized) {
        return;  // Already initialized
    }

    Configure_Watchdog(timeout_ms);
    initialized = true;
    feed_counter = 0;
}

/**
 * @brief Feed the watchdog to prevent reset
 */
void Watchdog_Feed(void)
{
    if (!initialized) {
        return;  // Not initialized yet
    }

    // Reload the watchdog counter
    IWDG->KR = IWDG_KEY_RELOAD;

    // Increment feed counter
    feed_counter++;
}

/**
 * @brief Check if last reset was caused by watchdog
 */
bool Watchdog_WasLastResetByWatchdog(void)
{
    // Check RCC reset flags
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
        return true;
    }
    return false;
}

/**
 * @brief Clear the watchdog reset flag
 */
void Watchdog_ClearResetFlag(void)
{
    // Clear all reset flags
    RCC_ClearFlag();
}

/**
 * @brief Get number of times watchdog was fed
 */
uint32_t Watchdog_GetFeedCount(void) { return feed_counter; }
