/**
 ******************************************************************************
 * @file    iwdg_driver.h
 * @brief   Independent Watchdog (IWDG) driver for STM32F407
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * Watchdog Configuration:
 * - Uses STM32 Independent Watchdog (IWDG)
 * - Clock source: LSI (32 kHz internal oscillator)
 * - Prescaler: 32 (results in 1 kHz counter frequency)
 * - Timeout range: 1ms to 4095ms (0xFFF)
 * - Watchdog stops during debug to prevent resets while stepping
 *
 * The Independent Watchdog is a critical safety feature that monitors system
 * health by requiring periodic "feeding" (counter reload). If the software
 * fails to feed the watchdog within the configured timeout period, the
 * watchdog triggers a hardware reset to recover from software failures.
 *
 * CRITICAL SAFETY NOTES:
 * ----------------------
 * 1. The watchdog MUST be fed regularly by the main application loop
 * 2. Once enabled, the watchdog CANNOT be disabled except by hardware reset
 * 3. If the system hangs or enters an infinite loop, the watchdog will
 *    trigger a reset after the timeout period
 * 4. In debug mode, the watchdog is automatically stopped to allow debugging
 *
 * Common Failure Scenarios Prevented:
 * - Software infinite loops
 * - CPU lockup due to hardware faults
 * - Stack overflow causing execution to halt
 * - Memory corruption preventing normal execution
 * - External interference causing CPU malfunction
 *
 ******************************************************************************
 */

#ifndef __IWDG_DRIVER_H
#define __IWDG_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx.h>

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief IWDG prescaler values (LSI / prescaler = counter frequency) */
#define IWDG_PRESCALER_4   0 /**< LSI/4   = 8 kHz */
#define IWDG_PRESCALER_8   1 /**< LSI/8   = 4 kHz */
#define IWDG_PRESCALER_16  2 /**< LSI/16  = 2 kHz */
#define IWDG_PRESCALER_32  3 /**< LSI/32  = 1 kHz (default) */
#define IWDG_PRESCALER_64  4 /**< LSI/64  = 500 Hz */
#define IWDG_PRESCALER_128 5 /**< LSI/128 = 250 Hz */
#define IWDG_PRESCALER_256 6 /**< LSI/256 = 125 Hz */

/** @brief Maximum reload value (12-bit counter) */
#define IWDG_MAX_RELOAD_VALUE 0x0FFF /**< 4095 */

/** @brief Default prescaler for 1ms resolution */
#define IWDG_DEFAULT_PRESCALER IWDG_PRESCALER_32

/** @brief Minimum safe timeout (milliseconds) */
#define IWDG_MIN_TIMEOUT_MS 1

/** @brief Maximum timeout with default prescaler (milliseconds) */
#define IWDG_MAX_TIMEOUT_MS 4095

/** @brief Recommended timeout for typical applications (milliseconds) */
#define IWDG_RECOMMENDED_TIMEOUT_MS 1000

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Watchdog configuration structure
 */
typedef struct {
    uint8_t prescaler;   /**< Prescaler value (use IWDG_PRESCALER_* constants) */
    uint16_t timeout_ms; /**< Timeout in milliseconds */
    bool stop_in_debug;  /**< Stop watchdog when CPU is halted by debugger */
} iwdg_config_t;

/**
 * @brief Watchdog statistics structure
 */
typedef struct {
    uint32_t feed_count;        /**< Total number of feeds */
    uint32_t last_feed_time;    /**< Timestamp of last feed (milliseconds) */
    uint32_t max_feed_interval; /**< Maximum interval between feeds (milliseconds) */
    bool was_reset_by_watchdog; /**< True if last reset was caused by watchdog */
} iwdg_stats_t;

/**
 * @brief Watchdog driver instance structure
 */
typedef struct {
    iwdg_config_t config; /**< Watchdog configuration */
    iwdg_stats_t stats;   /**< Watchdog statistics */
    bool initialized;     /**< Initialization complete flag */
    bool enabled;         /**< Watchdog enabled flag */
} iwdg_instance_t;

/* ============================================================================
 * Function Prototypes - Initialization
 * ============================================================================ */

/**
 * @brief Initialize watchdog driver
 * @param iwdg Pointer to IWDG instance structure
 * @param config Pointer to configuration structure
 * @retval true if initialization successful
 * @retval false if initialization failed (invalid parameters)
 * @note This initializes the structure but does NOT enable the watchdog
 * @warning Once enabled via IWDG_Enable(), the watchdog CANNOT be disabled!
 */
bool IWDG_Init(iwdg_instance_t* iwdg, const iwdg_config_t* config);

/**
 * @brief Enable watchdog timer
 * @param iwdg Pointer to IWDG instance
 * @retval true if watchdog enabled successfully
 * @retval false if enable failed
 * @warning Once enabled, the watchdog CANNOT be disabled except by reset!
 * @warning You MUST call IWDG_Feed() regularly or the system will reset!
 */
bool IWDG_Enable(iwdg_instance_t* iwdg);

/**
 * @brief Check if system was reset by watchdog
 * @param iwdg Pointer to IWDG instance
 * @retval true if last reset was caused by watchdog timeout
 * @retval false if last reset was not caused by watchdog
 * @note This checks the RCC reset status register
 */
bool IWDG_WasResetByWatchdog(iwdg_instance_t* iwdg);

/**
 * @brief Clear watchdog reset flag
 * @param iwdg Pointer to IWDG instance
 * @note Call this after checking reset cause to clear the flag
 */
void IWDG_ClearResetFlag(iwdg_instance_t* iwdg);

/* ============================================================================
 * Function Prototypes - Operation
 * ============================================================================ */

/**
 * @brief Feed the watchdog (reload counter)
 * @param iwdg Pointer to IWDG instance
 * @param timestamp Current system timestamp in milliseconds
 * @note This function MUST be called regularly before timeout expires
 * @note Calling this resets the watchdog counter to prevent reset
 * @warning If this is not called within the timeout period, the system WILL reset!
 */
void IWDG_Feed(iwdg_instance_t* iwdg, uint32_t timestamp);

/**
 * @brief Check if watchdog needs feeding soon
 * @param iwdg Pointer to IWDG instance
 * @param timestamp Current system timestamp in milliseconds
 * @param margin_ms Safety margin in milliseconds
 * @retval true if watchdog should be fed soon
 * @retval false if watchdog was fed recently
 * @note Use this to implement proactive feeding strategy
 */
bool IWDG_NeedsFeeding(const iwdg_instance_t* iwdg, uint32_t timestamp, uint32_t margin_ms);

/**
 * @brief Get time since last feed
 * @param iwdg Pointer to IWDG instance
 * @param timestamp Current system timestamp in milliseconds
 * @return Time in milliseconds since last feed
 */
uint32_t IWDG_GetTimeSinceLastFeed(const iwdg_instance_t* iwdg, uint32_t timestamp);

/* ============================================================================
 * Function Prototypes - Statistics
 * ============================================================================ */

/**
 * @brief Get watchdog statistics
 * @param iwdg Pointer to IWDG instance
 * @return Pointer to statistics structure (read-only)
 */
const iwdg_stats_t* IWDG_GetStats(const iwdg_instance_t* iwdg);

/**
 * @brief Reset watchdog statistics
 * @param iwdg Pointer to IWDG instance
 * @note Resets feed count and interval tracking
 */
void IWDG_ResetStats(iwdg_instance_t* iwdg);

/* ============================================================================
 * Function Prototypes - Configuration Helpers
 * ============================================================================ */

/**
 * @brief Calculate reload value from timeout
 * @param timeout_ms Desired timeout in milliseconds
 * @param prescaler Prescaler value (use IWDG_PRESCALER_* constants)
 * @return Reload value for IWDG counter (0 if invalid)
 * @note Helper function to validate timeout values
 */
uint16_t IWDG_CalculateReloadValue(uint16_t timeout_ms, uint8_t prescaler);

/**
 * @brief Get default configuration
 * @param config Pointer to configuration structure to fill
 * @note Fills config with recommended default values
 */
void IWDG_GetDefaultConfig(iwdg_config_t* config);

/* ============================================================================
 * Function Prototypes - Utility
 * ============================================================================ */

/**
 * @brief Get configured timeout value
 * @param iwdg Pointer to IWDG instance
 * @return Timeout in milliseconds
 */
uint16_t IWDG_GetTimeoutMs(const iwdg_instance_t* iwdg);

/**
 * @brief Check if watchdog is initialized
 * @param iwdg Pointer to IWDG instance
 * @retval true if initialized
 * @retval false if not initialized
 */
bool IWDG_IsInitialized(const iwdg_instance_t* iwdg);

/**
 * @brief Check if watchdog is enabled
 * @param iwdg Pointer to IWDG instance
 * @retval true if enabled
 * @retval false if not enabled
 */
bool IWDG_IsEnabled(const iwdg_instance_t* iwdg);

#ifdef __cplusplus
}
#endif

#endif /* __IWDG_DRIVER_H */
