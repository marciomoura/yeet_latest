#pragma once

#include "lookup_table_1d.hpp"
#include "reference_frame.hpp"
#include "units.hpp"

namespace yeet::control {

/**
 * @brief Compensates for a first-order hardware low-pass filter using lookup tables.
 *
 * This component uses two separate lookup tables: one for gain and one for phase-shift.
 * It provides a method to configure these tables based on a theoretical first-order LPF
 * response, and then applies the compensation to a three-phase input signal in the
 * stationary alpha-beta reference frame.
 *
 * The compensation formula is:
 *   V_comp = G * R(phi) * V
 * where G is the gain to be corrected, phi is the phase-shift to be corrected,
 * and R(phi) is a rotation matrix.
 */
class hardware_filter_compensator {
public:
    static constexpr size_t num_points = 21;

    hardware_filter_compensator() = default;

    /**
     * @brief Configures the lookup tables based on a first-order LPF characteristic.
     * @param cutoff_frequency The theoretical cutoff frequency (fc) of the hardware filter.
     * @param max_frequency The maximum frequency to populate the LUT for.
     */
    void configure_as_first_order_lpf(frequency_t cutoff_frequency, frequency_t max_frequency);

    /**
     * @brief Resets the compensation outputs.
     */
    void reset();

    /**
     * @brief Updates the compensation based on the current frequency.
     * @param input The three-phase input signal to be compensated.
     * @param frequency The current estimated frequency of the system.
     * @return The compensated three-phase signal.
     */
    template <typename T>
    abc<T> update(const abc<T>& input, frequency_t frequency)
    {
        if (_gain_lut.get_value(0.0f) == 0.0f) {
            return input;  // Not configured
        }

        const real_t f = frequency.value();
        const real_t gain = _gain_lut.get_value(f);
        const real_t phase = _phase_lut.get_value(f);

        // Convert to alpha-beta for rotation
        alphabeta<T> ab = input.to_alphabeta();

        // Apply gain and rotation
        // V_alpha_comp = G * (V_alpha * cos(phi) - V_beta * sin(phi))
        // V_beta_comp  = G * (V_alpha * sin(phi) + V_beta * cos(phi))
        const real_t cos_phi = std::cos(phase);
        const real_t sin_phi = std::sin(phase);

        alphabeta<T> ab_comp{
            gain * (ab.alpha() * cos_phi - ab.beta() * sin_phi), gain * (ab.alpha() * sin_phi + ab.beta() * cos_phi)};

        return ab_comp.to_abc();
    }

private:
    lookup_table_1d<real_t, num_points> _gain_lut;
    lookup_table_1d<real_t, num_points> _phase_lut;
};

}  // namespace yeet::control
