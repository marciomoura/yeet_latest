#pragma once

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::control {

/**
 * @class monotonic_angle_filter
 * @brief Filters an input angle to ensure it only moves in a forward (monotonic) direction.
 *
 * This component acts as a "digital ratchet" for angles. It tracks an incoming angle
 * (typically from a PLL or rotor position estimator) and ensures that the output
 * only ever progresses forward. If a "backward" jump is detected—defined as a
 * negative angular change larger than zero but smaller than PI—the filter ignores the
 * jump and holds the last known valid forward position.
 *
 * The detection logic leverages the properties of `angle_wrapped` arithmetic:
 * - A forward jump is a small positive difference (< PI).
 * - A backward jump appears as a large positive difference (> PI) due to underflow.
 *
 * This is particularly useful for thyristor pulse generation where observer noise
 * or PLL jitter right at a sector boundary could otherwise cause premature pulse
 * extinction (quenching).
 */
class monotonic_angle_filter {
public:
    /**
     * @brief Updates the filter with a new angle sample.
     *
     * @param current_angle The raw angle from the observer or PLL.
     * @return The filtered, monotonic angle.
     */
    void update(angle_wrapped current_angle) noexcept;

    /**
     * @brief Resets the filter's internal state.
     *
     * After calling reset, the next update will treat the input as the new
     * baseline position.
     */
    void reset() noexcept;

    /**
     * @brief Configures the maximum allowable backward step before the filter "unsticks".
     *
     * If a backward jump larger than this threshold is detected, the filter
     * assumes a legitimate angle step or system transient has occurred and
     * updates its baseline to the new angle.
     *
     * @param max_step_rad The threshold in radians (typically ~60 degrees).
     */
    void configure_max_backward_step(real_t max_step_rad) noexcept;

    /**
     * @brief Gets the current filtered output angle.
     * @return The monotonic angle.
     */
    angle_wrapped get_output() const noexcept;

private:
    angle_wrapped _last_valid_angle{0.0f};     ///< Last baseline angle that moved forward
    angle_wrapped _filtered_output{0.0f};      ///< Current output being held
    real_t _max_backward_step_rad{pi / 3.0f};  ///< Default threshold (~60 deg)
    bool _is_first_update{true};               ///< Initialization flag
};

}  // namespace yeet::control
