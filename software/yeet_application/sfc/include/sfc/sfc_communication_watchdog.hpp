/**
 * @file sfc_communication_watchdog.hpp
 * @brief Communication link watchdog for fiber-optic communication health monitoring
 *
 * This module provides timeout-based detection of communication link failures between
 * grid-side and machine-side controllers. It monitors the validity of received data
 * and triggers protection trips when communication is lost for a configurable duration.
 *
 * Key Features:
 * - Counter-based timeout detection (deterministic, no floating-point time)
 * - Configurable timeout threshold (cycles)
 * - Hysteresis: different thresholds for trip and reset
 * - Warning and trip severity levels
 * - Compatible with 200µs execution rate (5 kHz)
 *
 * Typical Usage:
 * - Grid-side: Monitor machine→grid link health
 * - Machine-side: Monitor grid→machine link health
 *
 * Integration Pattern:
 * 1. Call update() every cycle (200µs) with data_valid flag
 * 2. Check is_tripped() to determine if communication timeout occurred
 * 3. Call reset() to clear trip after communication restored
 *
 * Example:
 * @code
 * sfc_communication_watchdog watchdog;
 * watchdog.configure_timeout_cycles(250);  // 50ms timeout (250 cycles × 200µs)
 *
 * void protection_task() {
 *     bool rx_valid = decode_fiber_optic_rx(...);
 *     watchdog.update(rx_valid);
 *
 *     if (watchdog.is_tripped()) {
 *         // Trigger protection trip
 *     }
 * }
 * @endcode
 */

#pragma once

#include <cstdint>

#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Communication watchdog for fiber-optic link health monitoring
 *
 * Monitors the validity of received communication data and trips after a configurable
 * number of consecutive invalid cycles. Uses counter-based timeout for deterministic
 * real-time behavior.
 *
 * State Machine:
 * - NORMAL: Communication valid, counter at zero
 * - WARNING: Some invalid cycles detected, counter < trip threshold
 * - TRIPPED: Timeout exceeded, counter >= trip threshold
 *
 * The watchdog uses hysteresis to prevent chattering:
 * - Trip threshold: Number of invalid cycles before trip
 * - Reset threshold: Number of valid cycles required to clear trip
 */
class sfc_communication_watchdog {
public:
    /**
     * @brief Construct communication watchdog with specified sampling period
     *
     * @param sampling_period Execution period of the watchdog (e.g., 200µs for 5kHz control)
     * @param timeout_duration Timeout duration before trip (default: 50ms)
     * @param reset_duration Duration of valid communication required to reset (default: 5ms)
     *
     * The constructor automatically calculates cycle thresholds from time durations:
     * - timeout_cycles = timeout_duration / sampling_period
     * - reset_cycles = reset_duration / sampling_period
     *
     * Example:
     * @code
     * using namespace yeet::control;
     * sfc_communication_watchdog watchdog(duration_t{200e-6f});  // 200µs sampling, 50ms timeout
     * sfc_communication_watchdog watchdog(duration_t{200e-6f}, duration_t{100e-3f});  // 100ms timeout
     * @endcode
     */
    explicit sfc_communication_watchdog(yeet::duration_t sampling_period,
        yeet::duration_t timeout_duration = yeet::duration_t{50e-3f},
        yeet::duration_t reset_duration = yeet::duration_t{5e-3f}) noexcept;

    /**
     * @brief Configure timeout threshold in execution cycles
     *
     * Sets the number of consecutive invalid communication cycles before trip.
     * At 200µs sampling rate:
     * - 50 cycles = 10ms
     * - 250 cycles = 50ms
     * - 500 cycles = 100ms
     *
     * @param cycles Number of invalid cycles before trip (must be > 0)
     *
     * Example:
     * @code
     * watchdog.configure_timeout_cycles(250);  // 50ms timeout
     * @endcode
     */
    void configure_timeout_cycles(std::uint16_t cycles) noexcept;

    /**
     * @brief Configure timeout threshold using duration
     *
     * Sets the timeout threshold by specifying the desired timeout duration.
     * Automatically converts to cycles based on the sampling period provided
     * to the constructor.
     *
     * @param duration Timeout duration before trip
     *
     * Example:
     * @code
     * watchdog.configure_timeout_duration(duration_t{50e-3f});  // 50ms timeout
     * watchdog.configure_timeout_duration(duration_t{100e-3f}); // 100ms timeout
     * @endcode
     */
    void configure_timeout_duration(yeet::duration_t duration) noexcept;

    /**
     * @brief Configure reset threshold in execution cycles
     *
     * Sets the number of consecutive valid communication cycles required to
     * clear the trip condition. Provides hysteresis to prevent chattering.
     *
     * Typically set to 10-20% of timeout threshold.
     *
     * @param cycles Number of valid cycles required to reset (must be > 0)
     *
     * Example:
     * @code
     * watchdog.configure_reset_cycles(25);  // 5ms of valid data to reset
     * @endcode
     */
    void configure_reset_cycles(std::uint16_t cycles) noexcept;

    /**
     * @brief Configure reset threshold using duration
     *
     * Sets the reset threshold by specifying the desired duration of valid
     * communication required to clear the trip. Automatically converts to
     * cycles based on the sampling period provided to the constructor.
     *
     * @param duration Duration of valid communication required to reset
     *
     * Example:
     * @code
     * watchdog.configure_reset_duration(duration_t{5e-3f});  // 5ms to reset
     * watchdog.configure_reset_duration(duration_t{10e-3f}); // 10ms to reset
     * @endcode
     */
    void configure_reset_duration(yeet::duration_t duration) noexcept;

    /**
     * @brief Update watchdog with current communication status
     *
     * Call this method every execution cycle (typically 200µs) with the result
     * of the communication decode operation.
     *
     * Behavior:
     * - If data_valid == true: Decrement invalid counter (bounded at 0)
     * - If data_valid == false: Increment invalid counter
     * - Trip when invalid counter >= timeout threshold
     * - Reset when tripped && invalid counter <= reset threshold
     *
     * @param data_valid True if communication data was successfully received and decoded
     *
     * Example:
     * @code
     * bool rx_ok = decode_fiber_optic_rx_from_grid(...);
     * watchdog.update(rx_ok);
     * @endcode
     */
    void update(bool data_valid) noexcept;

    /**
     * @brief Manually reset the watchdog (clear trip and counters)
     *
     * Forces the watchdog back to normal state regardless of current counter value.
     * Use this for:
     * - System initialization
     * - Manual trip acknowledgment
     * - Forced reset after maintenance
     *
     * Note: If communication is still invalid, the watchdog will trip again
     * on subsequent update() calls.
     */
    void reset() noexcept;

    /**
     * @brief Check if communication timeout has occurred
     *
     * @return true if invalid counter >= timeout threshold
     *
     * Example:
     * @code
     * if (watchdog.is_tripped()) {
     *     trigger_protection_trip();
     * }
     * @endcode
     */
    [[nodiscard]] bool is_tripped() const noexcept;

    /**
     * @brief Check if communication is in warning state
     *
     * Warning state is defined as: invalid counter > (timeout_threshold / 2)
     * This provides early indication of degrading communication quality.
     *
     * @return true if in warning state (invalid cycles > 50% of trip threshold)
     *
     * Example:
     * @code
     * if (watchdog.is_warning()) {
     *     log_communication_degradation();
     * }
     * @endcode
     */
    [[nodiscard]] bool is_warning() const noexcept;

    /**
     * @brief Get current invalid cycle counter value
     *
     * @return Number of consecutive invalid cycles detected
     *
     * Use this for diagnostics and telemetry.
     */
    [[nodiscard]] std::uint16_t get_invalid_count() const noexcept;

    /**
     * @brief Get configured timeout threshold
     *
     * @return Number of invalid cycles before trip
     */
    [[nodiscard]] std::uint16_t get_timeout_threshold() const noexcept;

    /**
     * @brief Get configured reset threshold
     *
     * @return Number of valid cycles required to reset
     */
    [[nodiscard]] std::uint16_t get_reset_threshold() const noexcept;

private:
    yeet::duration_t _sampling_period{200e-6f};  ///< Execution period (default 200µs)
    std::uint16_t _invalid_count{0};             ///< Current count of consecutive invalid cycles
    std::uint16_t _timeout_threshold{250};       ///< Trip threshold (cycles) - default 50ms @ 200µs
    std::uint16_t _reset_threshold{25};          ///< Reset threshold (cycles) - default 5ms @ 200µs
    bool _is_tripped{false};                     ///< Trip state flag
};

}  // namespace yeet::application::sfc
