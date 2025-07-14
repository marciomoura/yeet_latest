#pragma once

#include <array>

#include "pulse_generator.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::control {

/**
 * @brief Generates firing pulses for a 6-pulse thyristor converter.
 *
 * This class takes a reference angle (typically from a PLL synchronized with the
 * grid voltage) and a firing angle command (alpha) to generate the six pulse
 * signals for a standard 6-pulse bridge.
 */
class six_pulse_generator {
public:
    using angle_t = yeet::control::angle<yeet::control::zero_to_two_pi>;

    /**
     * @brief Structure to hold the state of the six output pulses.
     */
    struct pulses {
        bool p1{false}, p2{false}, p3{false}, p4{false}, p5{false}, p6{false};
    };

    /**
     * @brief Constructs a six_pulse_generator.
     *
     * @param pulse_width The desired width of the firing pulses.
     */
    explicit six_pulse_generator(angle_t pulse_width = angle_t(yeet::control::deg_to_rad(30.0)));

    /**
     * @brief Updates the pulse generator and returns the new pulse states.
     *
     * The firing sequence is assumed to be 1-2-3-4-5-6, with each pulse
     * separated by 60 degrees. The firing angle `alpha` is measured from the
     * natural commutation point, which is assumed to be aligned with the
     * provided `reference_angle`.
     *
     * @param reference_angle The reference angle of the AC system.
     * @param firing_angle The desired firing angle (alpha).
     * @param enable Gating enable signal. If false, all pulses are off.
     * @return The generated pulses for the six thyristors.
     */
    pulses update(angle_t reference_angle, angle_t firing_angle, bool enable);

    /**
     * @brief Sets the pulse width for all six pulse generators.
     *
     * @param pulse_width The new pulse width.
     */
    void set_pulse_width(angle_t pulse_width);

    /**
     * @brief Gets the current pulse width.
     *
     * @return The pulse width.
     */
    angle_t get_pulse_width() const;

private:
    // Array of six individual pulse generators, one for each thyristor
    std::array<pulse_generator, 6> _pulse_generators;

    // Default firing offsets for the 6-pulse bridge topology
    // These are spaced 60 degrees apart following the standard firing sequence
    static constexpr std::array<double, 6> firing_offsets_deg = {
        0.0,    // P1: 0°
        60.0,   // P2: 60°
        120.0,  // P3: 120°
        180.0,  // P4: 180°
        240.0,  // P5: 240°
        300.0   // P6: 300°
    };
};

}  // namespace yeet::control
