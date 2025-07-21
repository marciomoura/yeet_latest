#ifndef YEET_CONTROL_RATE_OF_CHANGE_LIMITER_HPP
#define YEET_CONTROL_RATE_OF_CHANGE_LIMITER_HPP

#pragma once

#include "types.hpp"

namespace yeet::control {

/**
 * @brief Limits the rate of change of a signal.
 *
 * This class implements a slew rate limiter. It ensures that the output signal
 * ramps towards the input signal at a specified maximum rate, preventing
 * instantaneous jumps. The rate limit is symmetrical for both rising and
 * falling signals.
 */
class rate_of_change_limiter {
public:
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
     * configured rate limit.
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

private:
    real_t _sampling_time;          ///< System sampling time in seconds.
    real_t _max_change_per_sample;  ///< The maximum allowed change in one sample.
    real_t _current_output;         ///< The current output value of the limiter.
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_RATE_OF_CHANGE_LIMITER_HPP