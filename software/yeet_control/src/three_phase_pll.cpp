#include "yeet_control/three_phase_pll.hpp"

#include <cassert>
#include <cmath>

#include "yeet_control/constants.hpp"

namespace yeet::control {

// Constructor
three_phase_pll::three_phase_pll(real_t sampling_time)
    : _sampling_time(sampling_time),
      _lock_q_error_threshold(real_t{0.05}),  // Default threshold
      _frequency_integrator(sampling_time),
      _frequency_pi_controller(sampling_time),
      _lock_timer(10.0f / 50.0f, sampling_time),  // Default lock time: 10 cycles @ 50Hz
      _filter_frequency_estimation(sampling_time),
      _omega_reject_filter(sampling_time) {
    assert(sampling_time > real_t{0} && "Sampling time must be positive");

    // Improve the default configuration of the filters
    _filter_frequency_estimation.configure(10.0f, 0.707);

    // Configure the adaptive filter
    const real_t initial_freq = 50.0;
    const real_t bandwidth = 10.0;      // The width of the notch
    const real_t freq_threshold = 2.0;  // Trigger swap if frequency moves by 2 Hz
    const real_t settling_time = 0.5;   // Settle for 100ms
    _omega_reject_filter.configure(initial_freq, bandwidth, freq_threshold, settling_time);
}

// Configuration Methods
void three_phase_pll::configure_pi_controller(real_t kp, real_t ti) {
    assert(kp > real_t{0} && "Proportional gain must be positive");
    assert(ti > real_t{0} && "Integral time constant must be positive");
    _frequency_pi_controller.configure_with_ti(kp, ti);
    _frequency_pi_controller.enable_integrator_clamping(true);
}

void three_phase_pll::configure_frequency_limits(real_t min_freq, real_t max_freq) {
    assert(min_freq > real_t{0} && "Minimum frequency must be positive");
    assert(max_freq > min_freq && "Maximum frequency must be greater than minimum");

    real_t freq_limit_min_pu = (min_freq - _nominal_frequency) / _nominal_frequency;
    real_t freq_limit_max_pu = (max_freq - _nominal_frequency) / _nominal_frequency;
    _frequency_pi_controller.set_output_limits(freq_limit_min_pu, freq_limit_max_pu);
}

void three_phase_pll::configure_lock_detection(real_t lock_duration_s, real_t q_error_threshold) {
    assert(lock_duration_s >= 0.0f && "Lock duration must be non-negative");
    assert(q_error_threshold > 0.0f && "Q-error threshold must be positive");
    _lock_q_error_threshold = q_error_threshold;
    _lock_timer.configure(lock_duration_s);
}

void three_phase_pll::configure_nominal_frequency(real_t nominal_frequency) {
    assert(nominal_frequency > real_t{0} && "Nominal frequency must be positive");
    _nominal_frequency = nominal_frequency;
    reset();
}

// State Management
void three_phase_pll::reset() {
    _frequency_pi_controller.reset();
    _frequency_integrator.reset();
    _omega_reject_filter.reset();
    _lock_timer.reset();  // Reset the timer

    // Initialize with nominal frequency
    _voltage_dq = {};
    _estimated_frequency = _nominal_frequency;

    // Set initial integrator value to nominal angular frequency
    _frequency_integrator.reset(real_t{2} * pi * _nominal_frequency);
}

// Core Update Logic
void three_phase_pll::update(const alphabeta<real_t>& voltage_alphabeta) {
    // Park transformation (αβ → dq) using current estimated angle
    // _voltage_dq = voltage_alphabeta.to_dq(_frequency_integrator.get_output());
    _voltage_dq = voltage_alphabeta.to_dq(_frequency_integrator.get_output());

    // PI controller processes the error to estimate frequency deviation
    // The q-axis component is used as the error signal
    real_t frequency_deviation_pu = _frequency_pi_controller.update(_voltage_dq.d());

    // Filter the frequency deviation to reject ripple, using the last known good frequency
    frequency_deviation_pu =
        _omega_reject_filter.update(frequency_deviation_pu, is_locked() ? _estimated_frequency : _nominal_frequency);

    // Update the lock detection logic using the on_delay timer
    const bool is_lock_condition_met = std::abs(_voltage_dq.d()) < _lock_q_error_threshold;
    _lock_timer.update(is_lock_condition_met);

    // Convert frequency deviation to absolute frequency
    real_t frequency_deviation_hz = frequency_deviation_pu * _nominal_frequency;
    _estimated_frequency = _nominal_frequency + frequency_deviation_hz;

    // Integrate angular frequency to get the new phase angle for the next cycle
    _frequency_integrator.update(real_t{2} * pi * _estimated_frequency);

    // Update the final output filter for a smooth frequency reading
    _filter_frequency_estimation.update(_estimated_frequency);
}

// Getter Methods
real_t three_phase_pll::get_estimated_frequency() const { return _filter_frequency_estimation.get_output(); }

angle<zero_to_two_pi> three_phase_pll::get_estimated_angle() const {
    // if is_locked, take the estimated frequency, otherwise use nominal frequency
    if (!is_locked()) {
        return _frequency_integrator.get_output() - angle<zero_to_two_pi>(2 * pi * _nominal_frequency * _sampling_time);
    }

    return _frequency_integrator.get_output() - angle<zero_to_two_pi>(2 * pi * _estimated_frequency * _sampling_time);
}

const dq<real_t>& three_phase_pll::get_voltage_dq() const { return _voltage_dq; }

bool three_phase_pll::is_locked() const { return _lock_timer.get_output(); }

const pi_controller<real_t>& three_phase_pll::get_frequency_controller() const { return _frequency_pi_controller; }

}  // namespace yeet::control