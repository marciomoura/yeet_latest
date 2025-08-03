#pragma once

#include <array>

#include "pulse_generator.hpp"
#include "yeet_control/angle_wrapped.hpp"
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
    using angle_t = angle_wrapped;

    /**
     * @brief Defines the type of AC voltage reference used for synchronization.
     */
    enum class reference_type {
        line,  ///< Reference angle is from a line-to-line voltage (e.g., Vab). No offset needed.
        phase  ///< Reference angle is from a phase-to-neutral voltage (e.g., Van). A 30-degree offset is applied.
    };

    /**
     * @brief Structure to hold the state of the six output pulses.
     */
    struct pulses {
        bool p1{false}, p2{false}, p3{false}, p4{false}, p5{false}, p6{false};

        std::array<bool, 6> to_array() const { return {p1, p2, p3, p4, p5, p6}; }

        // Conversional operator
        operator std::array<bool, 6>() const { return to_array(); }

        constexpr size_t size() const { return 6; }

        auto begin() { return &p1; }
        auto end() { return &p6 + 1; }
    };

    /**
     * @brief Constructs a six_pulse_generator.
     *
     * @param ref_type Specifies whether the reference angle is from a line or phase voltage.
     * @param pulse_width The desired width of the firing pulses.
     */
    explicit six_pulse_generator(
        reference_type ref_type = reference_type::phase, angle_t pulse_width = angle_t(yeet::deg_to_rad(30.0)));

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

    /**
     * @brief Gets the current pulses.
     *
     * @return The current state of the six pulses.
     */
    pulses get_pulses() const { return _pulses; }

private:
    // The type of reference angle provided to the update method.
    reference_type _ref_type;

    // Array of six individual pulse generators, one for each thyristor
    std::array<pulse_generator, 6> _pulse_generators;

    pulses _pulses;

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
