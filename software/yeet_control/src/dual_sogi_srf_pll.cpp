#include "yeet_control/dual_sogi_srf_pll.hpp"

#include <cassert>

#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

dual_sogi_srf_pll::dual_sogi_srf_pll(duration_t sampling_time)
    : _sampling_time(sampling_time), _sequence_extractor(sampling_time), _internal_pll(sampling_time)
{
    assert(sampling_time.value() > real_t{0} && "Sampling time must be positive");

    // Apply default configuration
    configure_sequence_extractor(default_sogi_k, default_sogi_dc_rejection_time_constant);
    reset();
}

void dual_sogi_srf_pll::configure_sequence_extractor(
    real_t k, duration_t time_constant_dc_offset_rejection, bool scale_output)
{
    _sequence_extractor.configure(k, time_constant_dc_offset_rejection, scale_output);
}

void dual_sogi_srf_pll::configure_pi_controller(real_t kp, duration_t ti)
{
    _internal_pll.configure_pi_controller(kp, ti);
}

void dual_sogi_srf_pll::configure_frequency_limits(frequency_t min_freq, frequency_t max_freq)
{
    _internal_pll.configure_frequency_limits(min_freq, max_freq);
}

void dual_sogi_srf_pll::configure_nominal_frequency(frequency_t nominal_frequency)
{
    _internal_pll.configure_nominal_frequency(nominal_frequency);

    // Re-initialize the system with the new nominal frequency
    reset();
}

void dual_sogi_srf_pll::reset()
{
    _sequence_extractor.reset();
    _internal_pll.reset();
    _filtered_voltage_alphabeta = {};
}

void dual_sogi_srf_pll::update(const alphabeta<voltage_pu_t>& voltage_alphabeta)
{
    // Use the estimated frequency from the internal PLL to drive the SOGI
    angular_frequency_t const omega_estimated = two_pi * _internal_pll.get_estimated_frequency();

    // The sequence extractor filters the input voltage to extract the positive sequence
    _sequence_extractor.update(voltage_alphabeta, omega_estimated);

    // The internal PLL then locks onto this filtered positive sequence signal
    _filtered_voltage_alphabeta = _sequence_extractor.get_positive_sequence();

    // Update the internal PLL with the filtered positive sequence
    _internal_pll.update(_filtered_voltage_alphabeta);
}

frequency_t dual_sogi_srf_pll::get_estimated_frequency() const { return _internal_pll.get_estimated_frequency(); }

frequency_pu_t dual_sogi_srf_pll::get_estimated_frequency_pu() const
{
    return _internal_pll.get_estimated_frequency_pu();
}

angle_wrapped dual_sogi_srf_pll::get_estimated_angle() const { return _internal_pll.get_estimated_angle(); }

bool dual_sogi_srf_pll::is_locked() const { return _internal_pll.is_locked(); }

voltage_pu_t dual_sogi_srf_pll::get_estimation_error() const
{
    // The d-axis voltage in the PLL's dq frame represents the phase error
    return _internal_pll.get_voltage_dq().d();
}

voltage_pu_t dual_sogi_srf_pll::get_estimated_voltage_magnitude() const
{
    // The q-axis voltage in the PLL's dq frame represents the magnitude
    return _internal_pll.get_voltage_dq().q();
}

const alphabeta<voltage_pu_t>& dual_sogi_srf_pll::get_positive_sequence() const { return _filtered_voltage_alphabeta; }

}  // namespace yeet::control