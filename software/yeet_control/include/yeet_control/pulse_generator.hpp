#pragma once

#include "yeet_control/angle.hpp"

namespace yeet::control {

/**
 * @brief Generates a single pulse signal based on angle range comparison.
 *
 * This class represents a single pulse generator that activates when a reference
 * angle falls within a specified angular range. It's designed to be used as a
 * building block for more complex pulse generation systems like 6-pulse thyristor
 * bridges.
 */
class pulse_generator {
public:
    using angle_t = yeet::control::angle<yeet::control::zero_to_two_pi>;

    /**
     * @brief Constructs a pulse generator.
     *
     * @param firing_offset The angular offset from the reference angle where the pulse starts.
     * @param pulse_width The angular width of the pulse.
     */
    explicit pulse_generator(angle_t firing_offset = angle_t(0.0),
                             angle_t pulse_width = angle_t(yeet::control::deg_to_rad(1.0)));

    /**
     * @brief Updates the pulse generator and returns the pulse state.
     *
     * The pulse is active when the total angle (reference_angle + firing_delay)
     * falls within the range [firing_offset, firing_offset + pulse_width).
     *
     * @param reference_angle The reference angle from the system (e.g., PLL output).
     * @param firing_delay Additional delay angle applied to the reference (e.g., alpha control).
     * @param enable Master enable signal. If false, pulse is always off.
     * @return true if the pulse should be active, false otherwise.
     */
    bool update(angle_t reference_angle, angle_t firing_delay, bool enable);

    /**
     * @brief Sets the firing offset angle.
     *
     * @param offset The new firing offset angle.
     */
    void set_firing_offset(angle_t offset);

    /**
     * @brief Sets the pulse width.
     *
     * @param width The new pulse width.
     */
    void set_pulse_width(angle_t width);

    /**
     * @brief Gets the current firing offset.
     *
     * @return The firing offset angle.
     */
    angle_t get_firing_offset() const;

    /**
     * @brief Gets the current pulse width.
     *
     * @return The pulse width.
     */
    angle_t get_pulse_width() const;

private:
    /**
     * @brief Checks if an angle is within a given range [start, start + width).
     *
     * This function correctly handles wrapping around 2π.
     *
     * @param angle_to_check The angle to check.
     * @param range_start The start of the range.
     * @param range_width The width of the range.
     * @return true if the angle is within the range, false otherwise.
     */
    static bool is_in_range(angle_t angle_to_check, angle_t range_start, angle_t range_width);

    angle_t _firing_offset;  ///< Angular offset where the pulse starts
    angle_t _pulse_width;    ///< Angular width of the pulse
};

}  // namespace yeet::control
