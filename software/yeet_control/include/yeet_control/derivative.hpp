#pragma once

#include "yeet_control/types.hpp"

namespace yeet::control {

/**
 * @brief Calculates the discrete-time derivative of a signal.
 *
 * This class computes the derivative of an input signal based on the
 * difference between the current and previous input values, divided by the
 * sampling time.
 *
 * The derivative is calculated as:
 *   y[k] = (u[k] - u[k-1]) / Ts
 * where:
 *   - y[k] is the output at the current step.
 *   - u[k] is the input at the current step.
 *   - u[k-1] is the input at the previous step.
 *   - Ts is the sampling time.
 */
class derivative {
public:
    /**
     * @brief Constructs the derivative calculator.
     * @param sampling_time The sampling time (Ts) in seconds. Must be positive.
     * @param initial_value The initial value to assume for the previous input (u[k-1]).
     */
    explicit derivative(double sampling_time, yeet::real_t initial_value = 0.0F);

    /**
     * @brief Configures the sampling time.
     * @param sampling_time The new sampling time in seconds. Must be positive.
     */
    void configure_sampling_time(double sampling_time);

    /**
     * @brief Updates the calculator with a new input value and computes the derivative.
     * @param input The current input value (u[k]).
     */
    void update(yeet::real_t input) noexcept;

    /**
     * @brief Resets the internal state of the calculator.
     * @param reset_value The value to which the previous input state is reset.
     */
    void reset(yeet::real_t reset_value = 0.0F);

    /**
     * @brief Gets the most recently calculated derivative value.
     * @return The derivative of the input signal [units/s].
     */
    yeet::real_t get_output() const noexcept;

private:
    double _sampling_time;
    yeet::real_t _previous_input;
    yeet::real_t _output{0.0F};
};

}  // namespace yeet::control
