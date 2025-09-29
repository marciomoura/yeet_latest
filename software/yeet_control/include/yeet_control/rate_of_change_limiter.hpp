#pragma once

#include "types.hpp"

namespace yeet::control {

/**
 * @brief Limits the rate of change of a signal.
 *
 * This class implements a slew rate limiter. It ensures that the output signal
 * ramps towards the input signal at a specified maximum rate, preventing
 * instantaneous jumps. The rate limit is symmetrical for both rising and
 * falling signals. The limiter can be enabled or disabled; when disabled,
 * the input passes through directly to the output.
 */
class rate_of_change_limiter {
public:
    /**
     * @brief Constructs the rate of change limiter.
     *
     * @param sampling_time The execution interval (sampling time) in seconds.
     */
    rate_of_change_limiter(real_t sampling_time);

    /**
     * @brief Constructs the rate of change limiter.
     *
     * @param rate_limit_per_second The maximum rate of change in units per second. Must be positive.
     * @param sampling_time The execution interval (sampling time) in seconds.
     * @param initial_value The initial value of the output signal.
     */
    rate_of_change_limiter(real_t rate_limit_per_second, real_t sampling_time, real_t initial_value = 0.0f);

    /**
     * @brief Updates the limiter with a new input (target) value.
     *
     * This method should be called once per execution cycle. It calculates the
     * new output by moving the current output towards the input, respecting the
     * configured rate limit when enabled. When disabled, the input passes through
     * directly to the output.
     *
     * @param input The target value for the output signal.
     */
    void update(real_t input);

    /**
     * @brief Gets the current output value of the limiter.
     *
     * @return The current, rate-limited output value.
     */
    real_t get_output() const;

    /**
     * @brief Forcibly resets the output to a specific value.
     *
     * @param reset_value The value to which the output should be immediately set.
     */
    void reset(real_t reset_value = 0.0f);

    /**
     * @brief Configures or changes the rate limit.
     *
     * @param rate_limit_per_second The new maximum rate of change in units per second. Must be positive.
     */
    void configure(real_t rate_limit_per_second);

    /**
     * @brief Enables or disables the rate limiting functionality.
     *
     * When enabled (default), the rate limiter constrains the output to change at
     * the specified maximum rate. When disabled, the input passes through directly
     * to the output without any rate limiting.
     *
     * @param enable True to enable rate limiting, false to disable (pass-through mode).
     */
    void configure_enable(bool enable);

    /**
     * @brief Gets the current enable state of the rate limiter.
     *
     * @return True if rate limiting is enabled, false if in pass-through mode.
     */
    bool is_enabled() const;

private:
    real_t _sampling_time;               ///< System sampling time in seconds.
    real_t _max_change_per_sample{1e6};  ///< The maximum allowed change in one sample.
    real_t _current_output{};            ///< The current output value of the limiter.
    bool _enabled{};                     ///< Enable/disable flag for rate limiting.
};

}  // namespace yeet::control
