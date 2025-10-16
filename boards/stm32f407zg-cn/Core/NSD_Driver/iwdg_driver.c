/**
 ******************************************************************************
 * @file    iwdg_driver.c
 * @brief   Independent Watchdog (IWDG) driver implementation
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * This module implements the Independent Watchdog driver for STM32F407.
 * The watchdog is a critical safety feature that monitors software execution.
 *
 ******************************************************************************
 */

#include "iwdg_driver.h"

#include <stm32f4xx.h>
#include <string.h>

/* ============================================================================
 * Private Constants
 * ============================================================================ */

/** @brief IWDG key register values */
#define IWDG_KEY_RELOAD       0xAAAA /**< Reload counter key */
#define IWDG_KEY_ENABLE       0xCCCC /**< Enable watchdog key */
#define IWDG_KEY_WRITE_ACCESS 0x5555 /**< Enable write access to PR and RLR */

/** @brief LSI frequency (approximate, can vary ±40%) */
#define LSI_FREQUENCY_HZ 32000

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void IWDG_ConfigureHardware(const iwdg_instance_t* iwdg);
static uint32_t IWDG_GetPrescalerValue(uint8_t prescaler);

/* ============================================================================
 * Public Function Implementations - Initialization
 * ============================================================================ */

/**
 * @brief Initialize watchdog driver
 */
bool IWDG_Init(iwdg_instance_t* iwdg, const iwdg_config_t* config)
{
    if (iwdg == NULL || config == NULL) {
        return false;
    }

    // Validate prescaler
    if (config->prescaler > IWDG_PRESCALER_256) {
        return false;
    }

    // Validate timeout
    if (config->timeout_ms < IWDG_MIN_TIMEOUT_MS || config->timeout_ms > IWDG_MAX_TIMEOUT_MS) {
        return false;
    }

    // Calculate and validate reload value
    uint16_t reload = IWDG_CalculateReloadValue(config->timeout_ms, config->prescaler);
    if (reload == 0 || reload > IWDG_MAX_RELOAD_VALUE) {
        return false;
    }

    // Clear instance structure
    memset(iwdg, 0, sizeof(iwdg_instance_t));

    // Store configuration
    memcpy(&iwdg->config, config, sizeof(iwdg_config_t));

    // Check if last reset was caused by watchdog
    iwdg->stats.was_reset_by_watchdog = IWDG_WasResetByWatchdog(iwdg);

    // Configure debug behavior
    if (config->stop_in_debug) {
        DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, ENABLE);
    }
    else {
        DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, DISABLE);
    }

    iwdg->initialized = true;
    return true;
}

/**
 * @brief Enable watchdog timer
 */
bool IWDG_Enable(iwdg_instance_t* iwdg)
{
    if (iwdg == NULL || !iwdg->initialized) {
        return false;
    }

    if (iwdg->enabled) {
        // Already enabled
        return true;
    }

    // Configure hardware registers
    IWDG_ConfigureHardware(iwdg);

    // Enable the watchdog (THIS CANNOT BE UNDONE!)
    IWDG->KR = IWDG_KEY_ENABLE;

    iwdg->enabled = true;
    return true;
}

/**
 * @brief Check if system was reset by watchdog
 */
bool IWDG_WasResetByWatchdog(iwdg_instance_t* iwdg)
{
    if (iwdg == NULL) {
        return false;
    }

    // Check RCC reset status register for IWDG reset flag
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
        iwdg->stats.was_reset_by_watchdog = true;
        return true;
    }

    return false;
}

/**
 * @brief Clear watchdog reset flag
 */
void IWDG_ClearResetFlag(iwdg_instance_t* iwdg)
{
    if (iwdg == NULL) {
        return;
    }

    // Clear all reset flags in RCC
    RCC_ClearFlag();
}

/* ============================================================================
 * Public Function Implementations - Operation
 * ============================================================================ */

/**
 * @brief Feed the watchdog (reload counter)
 */
void IWDG_Feed(iwdg_instance_t* iwdg, uint32_t timestamp)
{
    if (iwdg == NULL || !iwdg->initialized) {
        return;
    }

    // Calculate interval since last feed
    if (iwdg->stats.feed_count > 0) {
        uint32_t interval = timestamp - iwdg->stats.last_feed_time;
        if (interval > iwdg->stats.max_feed_interval) {
            iwdg->stats.max_feed_interval = interval;
        }
    }

    // Update statistics
    iwdg->stats.feed_count++;
    iwdg->stats.last_feed_time = timestamp;

    // Reload the watchdog counter
    IWDG->KR = IWDG_KEY_RELOAD;
}

/**
 * @brief Check if watchdog needs feeding soon
 */
bool IWDG_NeedsFeeding(const iwdg_instance_t* iwdg, uint32_t timestamp, uint32_t margin_ms)
{
    if (iwdg == NULL || !iwdg->initialized || !iwdg->enabled) {
        return false;
    }

    // Check if we're approaching the timeout
    uint32_t time_since_feed = IWDG_GetTimeSinceLastFeed(iwdg, timestamp);
    uint32_t timeout = iwdg->config.timeout_ms;

    // Return true if we're within the safety margin
    return (time_since_feed >= (timeout - margin_ms));
}

/**
 * @brief Get time since last feed
 */
uint32_t IWDG_GetTimeSinceLastFeed(const iwdg_instance_t* iwdg, uint32_t timestamp)
{
    if (iwdg == NULL || !iwdg->initialized) {
        return 0;
    }

    if (iwdg->stats.feed_count == 0) {
        return 0;
    }

    return timestamp - iwdg->stats.last_feed_time;
}

/* ============================================================================
 * Public Function Implementations - Statistics
 * ============================================================================ */

/**
 * @brief Get watchdog statistics
 */
const iwdg_stats_t* IWDG_GetStats(const iwdg_instance_t* iwdg)
{
    if (iwdg == NULL || !iwdg->initialized) {
        return NULL;
    }

    return &iwdg->stats;
}

/**
 * @brief Reset watchdog statistics
 */
void IWDG_ResetStats(iwdg_instance_t* iwdg)
{
    if (iwdg == NULL || !iwdg->initialized) {
        return;
    }

    iwdg->stats.feed_count = 0;
    iwdg->stats.max_feed_interval = 0;
    // Don't reset last_feed_time or was_reset_by_watchdog
}

/* ============================================================================
 * Public Function Implementations - Configuration Helpers
 * ============================================================================ */

/**
 * @brief Calculate reload value from timeout
 */
uint16_t IWDG_CalculateReloadValue(uint16_t timeout_ms, uint8_t prescaler)
{
    // Get prescaler divider value
    uint32_t prescaler_value = IWDG_GetPrescalerValue(prescaler);
    if (prescaler_value == 0) {
        return 0;
    }

    // Calculate counter frequency
    uint32_t counter_freq_hz = LSI_FREQUENCY_HZ / prescaler_value;

    // Calculate reload value: (timeout_ms / 1000) * counter_freq_hz
    uint32_t reload = ((uint32_t)timeout_ms * counter_freq_hz) / 1000;

    // Validate range
    if (reload > IWDG_MAX_RELOAD_VALUE) {
        return 0;
    }

    return (uint16_t)reload;
}

/**
 * @brief Get default configuration
 */
void IWDG_GetDefaultConfig(iwdg_config_t* config)
{
    if (config == NULL) {
        return;
    }

    config->prescaler = IWDG_DEFAULT_PRESCALER;
    config->timeout_ms = IWDG_RECOMMENDED_TIMEOUT_MS;
    config->stop_in_debug = true;
}

/* ============================================================================
 * Public Function Implementations - Utility
 * ============================================================================ */

/**
 * @brief Get configured timeout value
 */
uint16_t IWDG_GetTimeoutMs(const iwdg_instance_t* iwdg)
{
    if (iwdg == NULL || !iwdg->initialized) {
        return 0;
    }

    return iwdg->config.timeout_ms;
}

/**
 * @brief Check if watchdog is initialized
 */
bool IWDG_IsInitialized(const iwdg_instance_t* iwdg)
{
    if (iwdg == NULL) {
        return false;
    }

    return iwdg->initialized;
}

/**
 * @brief Check if watchdog is enabled
 */
bool IWDG_IsEnabled(const iwdg_instance_t* iwdg)
{
    if (iwdg == NULL || !iwdg->initialized) {
        return false;
    }

    return iwdg->enabled;
}

/* ============================================================================
 * Private Function Implementations
 * ============================================================================ */

/**
 * @brief Configure IWDG hardware registers
 */
static void IWDG_ConfigureHardware(const iwdg_instance_t* iwdg)
{
    // Enable write access to IWDG_PR and IWDG_RLR registers
    IWDG->KR = IWDG_KEY_WRITE_ACCESS;

    // Configure prescaler
    IWDG->PR = iwdg->config.prescaler;

    // Calculate and set reload value
    uint16_t reload = IWDG_CalculateReloadValue(iwdg->config.timeout_ms, iwdg->config.prescaler);
    IWDG->RLR = reload;

    // Reload counter to apply settings
    IWDG->KR = IWDG_KEY_RELOAD;
}

/**
 * @brief Get prescaler divider value
 */
static uint32_t IWDG_GetPrescalerValue(uint8_t prescaler)
{
    switch (prescaler) {
        case IWDG_PRESCALER_4:
            return 4;
        case IWDG_PRESCALER_8:
            return 8;
        case IWDG_PRESCALER_16:
            return 16;
        case IWDG_PRESCALER_32:
            return 32;
        case IWDG_PRESCALER_64:
            return 64;
        case IWDG_PRESCALER_128:
            return 128;
        case IWDG_PRESCALER_256:
            return 256;
        default:
            return 0;
    }
}
