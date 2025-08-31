#pragma once

#include "yeet_control/off_delay.hpp"
#include "yeet_control/on_off_delay.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Detects zero-current and discontinuous conduction mode with configurable debouncing.
 *
 * This component provides two stable boolean outputs based on a single current input:
 * 1. `is_current_zero`: A signal with on-delay that immediately clears when current rises.
 * 2. `is_discontinuous_mode`: A slower, more stable signal for mode switching,
 *    featuring a long on-delay to confirm exit from discontinuous mode.
 */
class sfc_discontinuous_conduction_detector {
public:
    sfc_discontinuous_conduction_detector(duration_t sampling_time)
        : _is_zero_timer(duration_t{0.0f}, sampling_time), _is_discontinuous_mode_timer(sampling_time)
    {
    }

    /**
     * @brief Configures the detector's thresholds and timer delays.
     *
     * @param zero_current_threshold The value below which the current is considered zero.
     * @param is_zero_on_delay On-delay for the `is_current_zero` output.
     * @param discontinuous_on_delay On-delay for the `is_discontinuous_mode` output (typically long).
     * @param discontinuous_off_delay Off-delay for the `is_discontinuous_mode` output.
     */
    void configure(current_pu_t zero_current_threshold,
        duration_t is_zero_on_delay,
        duration_t discontinuous_on_delay,
        duration_t discontinuous_off_delay);

    /**
     * @brief Updates the detector's state based on the current input.
     *
     * @param current The input current value to process.
     */
    void update(current_pu_t current);

    /**
     * @brief Resets all internal timer states to their initial off state.
     */
    void reset();

    /**
     * @brief Gets the debounced zero-current status.
     *
     * @return True if the current is considered zero, false otherwise.
     */
    [[nodiscard]] bool get_is_current_zero() const;

    /**
     * @brief Gets the stable discontinuous conduction mode status.
     *
     * @return True if the system is in discontinuous mode, false otherwise.
     */
    [[nodiscard]] bool get_is_discontinuous_mode() const;

private:
    current_pu_t _zero_current_threshold{0.0f};
    control::on_delay _is_zero_timer;
    control::on_off_delay _is_discontinuous_mode_timer;
};

}  // namespace yeet::application::sfc