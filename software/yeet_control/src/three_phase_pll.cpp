#include "yeet_control/three_phase_pll.hpp"

#include <cassert>
#include <cmath>

#include "yeet_control/constants.hpp"
#include "yeet_control/precomputed_divisor.hpp"

namespace yeet::control {

// Constructor
three_phase_pll::three_phase_pll(duration_t sampling_time)
    : _sampling_time(sampling_time),
      _frequency_pi_controller(sampling_time.value()),
      _frequency_integrator(sampling_time.value()),
      _lock_timer(default_lock_duration.value(), sampling_time.value()),
      _filter_frequency_estimation(sampling_time.value()),
      _omega_reject_filter(sampling_time.value())
{
    assert(sampling_time.value() > real_t{0} && "Sampling time must be positive");

    // Apply default configuration
    configure_nominal_frequency(default_nominal_frequency);
    configure_pi_controller(default_pi_kp, default_pi_ti);
    configure_frequency_limits(default_min_frequency, default_max_frequency);
    configure_lock_detection(default_lock_duration, default_lock_q_error_threshold);

    // Improve the default configuration of the filters
    _filter_frequency_estimation.configure(
        default_estimated_frequency_filter_cutoff.value(), default_estimated_frequency_filter_damping);

    // Configure the adaptive filter
    _omega_reject_filter.configure(default_nominal_frequency.value(), default_notch_filter_bandwidth.value(),
        default_notch_filter_freq_threshold.value(), default_notch_filter_settling_time.value());

    reset();
}

// Configuration Methods
void three_phase_pll::configure_pi_controller(real_t kp, duration_t ti)
{
    assert(kp > real_t{0} && "Proportional gain must be positive");
    assert(ti > real_t{0} && "Integral time constant must be positive");
    _frequency_pi_controller.configure_with_ti(kp, ti);
    _frequency_pi_controller.enable_integrator_clamping(true);
}

void three_phase_pll::configure_frequency_limits(frequency_t min_freq, frequency_t max_freq)
{
    assert(min_freq > real_t{0} && "Minimum frequency must be positive");
    assert(max_freq > min_freq && "Maximum frequency must be greater than minimum");

    const real_t freq_limit_min_pu = (min_freq - _nominal_frequency.value()) / _nominal_frequency;
    const real_t freq_limit_max_pu = (max_freq - _nominal_frequency.value()) / _nominal_frequency;
    _frequency_pi_controller.set_output_limits(freq_limit_min_pu, freq_limit_max_pu);
}

void three_phase_pll::configure_lock_detection(duration_t lock_duration_s, voltage_pu_t q_error_threshold)
{
    assert(lock_duration_s.value() >= 0.0f && "Lock duration must be non-negative");
    assert(q_error_threshold.value() > 0.0f && "Q-error threshold must be positive");
    _lock_q_error_threshold = q_error_threshold;
    _lock_timer.configure(lock_duration_s.value());
}

void three_phase_pll::configure_nominal_frequency(frequency_t nominal_frequency)
{
    assert(nominal_frequency > real_t{0} && "Nominal frequency must be positive");
    _nominal_frequency = precomputed_divisor<frequency_t>{nominal_frequency};
    reset();
}

// State Management
void three_phase_pll::reset()
{
    _frequency_pi_controller.reset();
    _frequency_integrator.reset();
    _omega_reject_filter.reset();
    _lock_timer.reset();  // Reset the timer

    // Initialize with nominal frequency
    _voltage_dq = {};
    _estimated_frequency = _nominal_frequency.value();

    // Set initial integrator value to nominal angular frequency
    _frequency_integrator.reset(real_t{2} * pi * real_t(_nominal_frequency.value()));
}

// Core Update Logic
void three_phase_pll::update(const alphabeta<voltage_pu_t>& voltage_alphabeta)
{
    // Park transformation (αβ → dq) using current estimated angle
    _voltage_dq = voltage_alphabeta.to_dq(_frequency_integrator.get_output());

    // PI controller processes the error to estimate frequency deviation
    // The q-axis component is used as the error signal
    real_t const frequency_deviation_pu = _frequency_pi_controller.update(_voltage_dq.d().value());

    // Filter the frequency deviation to reject ripple, using the last known good frequency
    real_t const filtered_frequency_deviation_pu = _omega_reject_filter.update(
        frequency_deviation_pu, is_locked() ? _estimated_frequency.value() : _nominal_frequency.value());

    // Update the lock detection logic using the on_delay timer
    const bool is_lock_condition_met = std::abs(_voltage_dq.d().value()) < _lock_q_error_threshold.value();
    _lock_timer.update(is_lock_condition_met);

    // Convert frequency deviation to absolute frequency
    real_t const frequency_deviation_hz = filtered_frequency_deviation_pu * _nominal_frequency.value();
    _estimated_frequency = frequency_t{_nominal_frequency.value() + frequency_deviation_hz};

    // Integrate angular frequency to get the new phase angle for the next cycle
    _frequency_integrator.update(two_pi * _estimated_frequency.value());

    // Update the final output filter for a smooth frequency reading
    _filter_frequency_estimation.update(_estimated_frequency.value());
}

// Getter Methods
frequency_t three_phase_pll::get_estimated_frequency() const
{
    return frequency_t{_filter_frequency_estimation.get_output()};
}

frequency_pu_t three_phase_pll::get_estimated_frequency_pu() const
{
    return frequency_pu_t{get_estimated_frequency() / _nominal_frequency};
}

angle_wrapped three_phase_pll::get_estimated_angle() const
{
    // if is_locked, take the estimated frequency, otherwise use nominal frequency
    const auto actual_frequency = is_locked() ? _estimated_frequency : _nominal_frequency.value();

    return _frequency_integrator.get_output() - angle_wrapped(2 * pi * actual_frequency * _sampling_time);
}

const dq<voltage_pu_t>& three_phase_pll::get_voltage_dq() const { return _voltage_dq; }

bool three_phase_pll::is_locked() const { return _lock_timer.get_output(); }

alphabeta<voltage_pu_t> three_phase_pll::get_estimated_alphabeta() const
{
    angle_wrapped const estimated_angle = get_estimated_angle();

    return _voltage_dq.to_alphabeta(estimated_angle);
}

}  // namespace yeet::control