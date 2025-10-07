#ifndef YEET_CONTROL_OFF_DELAY_HPP
#define YEET_CONTROL_OFF_DELAY_HPP

#pragma once

#include "types.hpp"

namespace yeet::control {

/**
 * @brief Creates a time delay for a falling edge signal (Off-Delay Timer).
 *
 * This class implements a classic off-delay timer. When the input signal goes
 * from false to true, the output becomes true immediately. When the input
 * signal goes from true to false, an internal timer begins counting. The output
 * will only become false if the input remains continuously false for the
 * specified duration. If the input goes true at any point, the timer resets.
 */
class off_delay {
public:
    /**
     * @brief Constructs the off-delay timer.
     *
     * @param delay_s The time in seconds the input must remain false before the output switches off.
     * @param sampling_time The execution interval (sampling time) in seconds.
     */
    off_delay(real_t delay_s, double sampling_time);

    /**
     * @brief Updates the timer logic with the current input value.
     *
     * This method should be called once per execution cycle. It evaluates the
     * input and updates the internal state of the timer.
     *
     * @param input The boolean input signal for the current time step.
     */
    void update(bool input);

    /**
     * @brief Gets the current output state of the timer.
     *
     * @return True if the input is true, or if the input has been false for less than the configured delay.
     */
    bool get_output() const;

    /**
     * @brief Forcibly resets the timer to its initial, off state.
     *
     * The internal counter is reset, and the output is set to false.
     */
    void reset();

    /**
     * @brief Configures or changes the delay duration of the timer.
     *
     * Calling this method also resets the timer to its initial state.
     *
     * @param delay_s The new delay time in seconds.
     */
    void configure(real_t delay_s);

private:
    double _sampling_time;      ///< System sampling time in seconds.
    int _samples_required;      ///< The number of consecutive samples required to meet the delay.
    int _sample_counter{0};     ///< The current count of consecutive false input samples.
    bool _output_state{false};  ///< The current boolean output of the timer.
};

}  // namespace yeet::control

#endif  // YEET_CONTROL_OFF_DELAY_HPP