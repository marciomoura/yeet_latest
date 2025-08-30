#pragma once

#include "yeet_control/types.hpp"

namespace yeet::control {

/**
 * @brief Implements a hysteresis limiter (Schmitt trigger) for analog signals.
 *
 * This component provides a boolean output that switches state based on an
 * input crossing upper and lower thresholds. This is useful for debouncing
 * signals or preventing chattering in state machines.
 *
 * - When the input rises above the high threshold, the output becomes true.
 * - The output remains true until the input falls below the low threshold.
 * - When the input falls below the low threshold, the output becomes false.
 * - The output remains false until the input rises above the high threshold.
 */
class hysteresis_limiter {
public:
    /**
     * @brief Constructs the hysteresis limiter with default thresholds.
     */
    hysteresis_limiter() = default;

    /**
     * @brief Configures the high and low switching thresholds.
     *
     * @param low_threshold The value the input must fall below to switch the output to false.
     * @param high_threshold The value the input must rise above to switch the output to true.
     */
    void configure_thresholds(real_t low_threshold, real_t high_threshold);

    /**
     * @brief Updates the limiter's state based on the current input.
     *
     * @param input The real-time input signal to evaluate.
     */
    void update(real_t input) noexcept;

    /**
     * @brief Resets the limiter to its initial (false) state.
     */
    void reset() noexcept;

    /**
     * @brief Gets the current boolean output of the limiter.
     *
     * @return True if the limiter is in the high state, false otherwise.
     */
    [[nodiscard]] bool get_output() const noexcept;

private:
    real_t _low_threshold{0.0F};
    real_t _high_threshold{0.0F};
    bool _output_state{false};
};

}  // namespace yeet::control