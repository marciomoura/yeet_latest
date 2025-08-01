#pragma once

#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @class elapsed_timer
 * @brief A generic elapsed time measurement component for real-time control applications.
 *
 * This component provides precise elapsed time measurement with configurable sampling time.
 * It is designed for use in control loops where timing information is critical, such as
 * state machine timeouts, pulse duration measurement, and event timing.
 *
 * The timer follows the yeet_control architectural patterns:
 * - Configuration via `configure_` methods
 * - Main execution via `update()` method
 * - State access via `get_` methods
 * - Real-time safety with no dynamic allocation
 *
 * @note This timer is incremental and relies on regular `update()` calls at the
 *       configured sampling period for accurate timing.
 */
class elapsed_timer {
public:
    /**
     * @brief Constructs an elapsed timer with specified sampling time.
     * @param sampling_time The period between consecutive update() calls.
     */
    explicit elapsed_timer(duration_t sampling_time);

    /**
     * @brief Configures the sampling time for the timer.
     *
     * This should match the actual period between consecutive update() calls
     * in the control loop for accurate timing measurement.
     *
     * @param sampling_time The sampling period in seconds.
     */
    void configure_sampling_time(duration_t sampling_time);

    /**
     * @brief Updates the timer by one sampling period.
     *
     * This method increments the internal elapsed time counter by the configured
     * sampling time. It must be called regularly at the sampling period for
     * accurate timing.
     *
     * @param enable Timer enable signal. When false, timer is paused.
     */
    void update(bool enable) noexcept;

    /**
     * @brief Resets the timer to zero.
     *
     * Clears the elapsed time counter and restarts timing from zero.
     * The timer state (enabled/disabled) is preserved.
     */
    void reset() noexcept;

    /**
     * @brief Resets the timer to a specific initial value.
     *
     * Sets the elapsed time counter to the specified initial value.
     * Useful for implementing countdown timers or offset timing.
     *
     * @param initial_time The initial elapsed time value.
     */
    void reset(duration_t initial_time) noexcept;

    /**
     * @brief Gets the current elapsed time.
     * @return The elapsed time since last reset in seconds.
     */
    duration_t get_elapsed_time() const noexcept;

    /**
     * @brief Checks if the elapsed time has exceeded a threshold.
     *
     * Convenient method for timeout detection without retrieving the actual
     * elapsed time value.
     *
     * @param threshold The time threshold to check against.
     * @return True if elapsed time >= threshold.
     */
    bool has_elapsed(duration_t threshold) const noexcept;

    /**
     * @brief Gets the remaining time until a threshold is reached.
     *
     * Useful for countdown applications. Returns zero if threshold has
     * already been exceeded.
     *
     * @param threshold The target time threshold.
     * @return Remaining time until threshold, or zero if already exceeded.
     */
    duration_t get_remaining_time(duration_t threshold) const noexcept;

    /**
     * @brief Checks if the timer is currently enabled.
     * @return True if timer is running (enabled), false if paused.
     */
    bool is_enabled() const noexcept;

private:
    duration_t _sampling_time;  ///< Configured sampling period
    duration_t _elapsed_time;   ///< Current elapsed time
    bool _enabled;              ///< Timer enable state
};

}  // namespace yeet::control