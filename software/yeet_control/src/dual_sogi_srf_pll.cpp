#include "yeet_control/dual_sogi_srf_pll.hpp"

#include <cassert>

namespace yeet::control {

dual_sogi_srf_pll::dual_sogi_srf_pll(real_t sampling_time)
    : _sampling_time(sampling_time), _sequence_extractor(sampling_time), _internal_pll(sampling_time) {
    assert(sampling_time > real_t{0} && "Sampling time must be positive");
}

void dual_sogi_srf_pll::configure_sequence_extractor(real_t k, real_t time_constant_dc_offset_rejection,
                                                     bool scale_output) {
    _sequence_extractor.configure(k, time_constant_dc_offset_rejection, scale_output);
}

void dual_sogi_srf_pll::configure_pi_controller(real_t kp, real_t ti) { _internal_pll.configure_pi_controller(kp, ti); }

void dual_sogi_srf_pll::configure_frequency_limits(real_t min_freq, real_t max_freq) {
    _internal_pll.configure_frequency_limits(min_freq, max_freq);
}

void dual_sogi_srf_pll::configure_nominal_frequency(real_t nominal_frequency) {
    _internal_pll.configure_nominal_frequency(nominal_frequency);
    // Initialize the system with the nominal frequency to avoid startup issues
    // This ensures the SOGI filters start with a reasonable frequency estimate
    reset();
}

void dual_sogi_srf_pll::reset() {
    _sequence_extractor.reset();
    _internal_pll.reset();
    _filtered_voltage_alphabeta = alphabeta<real_t>{real_t{0}, real_t{0}};
    _filtered_voltage_abc = abc<real_t>{real_t{0}, real_t{0}, real_t{0}};
}

void dual_sogi_srf_pll::update(const alphabeta<real_t>& v_in) {
    real_t const omega_estimated = two_pi * _internal_pll.get_estimated_frequency();

    _sequence_extractor.update(v_in, omega_estimated);

    _filtered_voltage_alphabeta = _sequence_extractor.get_positive_sequence();

    _internal_pll.update(_filtered_voltage_alphabeta);
}

real_t dual_sogi_srf_pll::get_estimated_frequency() const { return _internal_pll.get_estimated_frequency(); }

real_t dual_sogi_srf_pll::get_estimated_angular_frequency() const {
    return two_pi * _internal_pll.get_estimated_frequency();
}

angle<zero_to_two_pi> dual_sogi_srf_pll::get_estimated_angle() const { return _internal_pll.get_estimated_angle(); }

bool dual_sogi_srf_pll::is_locked() const { return _internal_pll.is_locked(); }

real_t dual_sogi_srf_pll::get_q_error() const { return _internal_pll.get_voltage_dq().q(); }

const alphabeta<real_t>& dual_sogi_srf_pll::get_positive_sequence() const { return _filtered_voltage_alphabeta; }

alphabeta<real_t> dual_sogi_srf_pll::get_alpha_sogi_output() const {
    return _sequence_extractor.get_alpha_sogi_output();
}

alphabeta<real_t> dual_sogi_srf_pll::get_beta_sogi_output() const { return _sequence_extractor.get_beta_sogi_output(); }

// const three_phase_pll& dual_sogi_srf_pll::get_internal_pll() const { return _internal_pll; }

}  // namespace yeet::control