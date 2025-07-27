#include "sfc/sfc_commutation_mask.hpp"

#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"

namespace yeet {

abc<voltage_pu_t> sfc_commutation_mask::update(const abc<voltage_pu_t>& measured_voltages,
                                               const angle<zero_to_two_pi>& pll_angle, const frequency_t& pll_frequency,
                                               bool is_commuting, real_t dt) {
    if (is_commuting) {
        if (!_is_masking) {
            // Commutation has just started. Latch the current PLL angle.
            // The magnitude has already been captured in the previous non-commuting cycle.
            _is_masking = true;
            _internal_angle = pll_angle;
        }

        // While commuting, advance the internal angle using the estimated frequency.
        // Convert frequency to rad/s by multiplying by 2*pi
        const real_t angular_frequency_rad_per_s = pll_frequency.value() * two_pi;
        _internal_angle += angle<zero_to_two_pi>(angular_frequency_rad_per_s * dt);

        // Reconstruct the waveform from the latched magnitude and the propagated angle.
        // We create a vector in the dq frame and transform it back to abc.
        const dq<voltage_pu_t> synthetic_dq{{}, voltage_pu_t{-_last_good_magnitude}};
        return synthetic_dq.to_abc(_internal_angle);
        // // Reconstruct the waveform from the latched magnitude and the propagated angle.
        // // We assume a pure sinusoidal voltage, so the q-component is zero.
        // return make_alphabeta(_last_good_magnitude, pll_angle - angle<zero_to_two_pi>{pi / 3}).to_abc();
    } else {
        // Not commuting. Pass through the measured voltage and update state for the next potential notch.
        _is_masking = false;

        // Continuously update the last known good magnitude.
        // Convert the measured voltage to the dq frame using the PLL's angle.
        // const auto measured_alphabeta = measured_voltages.to_alphabeta();
        _last_good_magnitude = voltage_pu_t{1.0f};  // measured_alphabeta.magnitude();

        return measured_voltages;
    }
}

}  // namespace yeet