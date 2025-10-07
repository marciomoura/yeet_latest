#pragma once

#include "yeet_control/off_delay.hpp"
#include "yeet_control/on_delay.hpp"

namespace yeet::control {

/**
 * @brief Creates a time delay for both rising and falling edge signals.
 *
 * This class implements a timer that delays both the activation (on-delay)
 * and deactivation (off-delay) of its output.
 * - When the input goes from false to true, the output becomes true only after
 *   the input has remained true for the configured on-delay duration.
 * - When the input goes from true to false, the output becomes false only after
 *   the input has remained false for the configured off-delay duration.
 */
class on_off_delay {
public:
    /**
     * @brief Constructs the on-off delay timer.
     *
     * @param timestep The execution interval (sampling time) in seconds.
     */
    on_off_delay(double sampling_time);

    /**
     * @brief Configures the on and off delay durations.
     *
     * @param on_delay_s The time in seconds for the on-delay.
     * @param off_delay_s The time in seconds for the off-delay.
     * @param timestep The execution interval (sampling time) in seconds.
     */
    void configure(real_t on_delay_s, real_t off_delay_s);

    /**
     * @brief Updates the timer logic with the current input value.
     *
     * @param input The boolean input signal for the current time step.
     */
    void update(bool input);

    /**
     * @brief Forcibly resets the timer to its initial, off state.
     */
    void reset();

    /**
     * @brief Gets the current output state of the timer.
     *
     * @return The current debounced boolean output.
     */
    [[nodiscard]] bool get_output() const;

private:
    double _sampling_time;
    on_delay _on_timer;
    off_delay _off_timer;
};

}  // namespace yeet::control