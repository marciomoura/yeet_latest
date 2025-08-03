#pragma once

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet {

/**
 * @brief Masks commutation notches in a measured three-phase voltage waveform.
 *
 * This class provides a clean, continuous three-phase sinusoidal signal to a
 * Phase-Locked Loop (PLL) by generating a synthetic waveform during commutation
 * events. When a commutation notch is detected, it reconstructs the waveform
 * based on the last known voltage magnitude and the estimated grid frequency,
 * effectively "riding through" the measurement disturbance.
 */
class sfc_commutation_mask {
public:
    sfc_commutation_mask(real_t sampling_time) : _sampling_time(sampling_time) {}
    /**
     * @brief Updates the commutation mask logic and returns the processed waveform.
     *
     * @param measured_voltages The instantaneous measured three-phase voltages (p.u.).
     * @param actual_angle The current angle output from the observer.
     * @param actual_frequency The current frequency output from the observer.
     * @param is_commuting A boolean flag indicating if a commutation event is active.
     * @return The processed three-phase voltage waveform (p.u.), which is either the
     *         original measurement or a synthetically generated one.
     */
    abc<voltage_pu_t> update(const abc<voltage_pu_t>& measured_voltages,
        const angle_wrapped& actual_angle,
        const frequency_t& actual_frequency,
        bool is_commuting);

    bool is_masking() const { return _is_masking; }

private:
    real_t _sampling_time{0.0};  ///< The sampling time in seconds

    /// @brief Stores the magnitude of the voltage vector (d-component) just before commutation.
    voltage_pu_t _last_good_magnitude{0.0};

    /// @brief The internal angle used for reconstructing the waveform during commutation.
    angle_wrapped _internal_angle{0.0};

    /// @brief A flag to track if the module is currently masking the input waveform.
    bool _is_masking{false};
};

}  // namespace yeet