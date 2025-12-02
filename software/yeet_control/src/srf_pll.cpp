#include "yeet_control/srf_pll.hpp"

#include <cassert>
#include <cmath>

#include "yeet_control/constants.hpp"
#include "yeet_control/precomputed_divisor.hpp"
#include "yeet_control/reference_frame.hpp"

namespace yeet::control {

srf_pll::srf_pll(duration_t sampling_time)
    : _frequency_pi_controller(sampling_time.value()),
      _phase_integrator(sampling_time.value()),
      _frequency_estimation_filter(sampling_time.value())
{
    assert(sampling_time.value() > 0.0f && "Sampling time must be positive");

    configure_frequency_estimation_filter(default_estimation_filter_cutoff_frequency);

    configure_pi_controller(parameters_50_hz_crossover_frequency.kp, parameters_50_hz_crossover_frequency.ti);

    reset();
}

void srf_pll::configure_nominal_frequency(frequency_t nominal_frequency)
{
    assert(nominal_frequency.value() >= 0.0f && "Nominal frequency must be non-negative");
    _nominal_frequency = precomputed_divisor<frequency_t>{nominal_frequency};
}

void srf_pll::configure_pi_controller(real_t kp, duration_t ti) { _frequency_pi_controller.configure_with_ti(kp, ti); }

void srf_pll::configure_frequency_estimation_filter(frequency_t cutoff_frequency)
{
    _frequency_estimation_filter.configure(cutoff_frequency.value(), 0.707f);
}

void srf_pll::reset() { reset({}); }

void srf_pll::reset(frequency_pu_t initial_frequency)
{
    _dq_vector = {};
    _initial_frequency = initial_frequency;
    _estimated_frequency = to_si(_initial_frequency, _nominal_frequency);
    _frequency_pi_controller.reset();
    _phase_integrator.reset();
}

void srf_pll::preset(angle_wrapped preset_angle)
{
    // Directly set the phase integrator to the specified angle
    // This allows for bumpless transfer from external angle source
    _phase_integrator.reset(preset_angle);
}

void srf_pll::update(const alphabeta<voltage_pu_t>& input_alphabeta)
{
    // Park transform (αβ → dq) using the current estimated angle
    _dq_vector = input_alphabeta.to_dq(_phase_integrator.get_output());

    // The q-axis component is the phase error. A PI controller drives this
    // error to zero by adjusting the estimated angular frequency deviation.
    const real_t frequency_deviation = _frequency_pi_controller.update(_dq_vector.q());

    const real_t raw_frequency_estimation =
        _nominal_frequency.value() * (_initial_frequency.value() + frequency_deviation);

    // Integrate the angular frequency to get the new phase angle for the next cycle
    _phase_integrator.update(2 * pi * raw_frequency_estimation);

    _frequency_estimation_filter.update(raw_frequency_estimation);
    _estimated_frequency = frequency_t{_frequency_estimation_filter.get_output()};
}

angle_wrapped srf_pll::get_estimated_angle() const
{
    // Compensate for the one-sample computational delay inherent in the discrete implementation.
    // The angle from the integrator is for the beginning of the current sample (k).
    // We extrapolate it forward by one sample time (Ts) to get the estimated angle
    // for the beginning of the next sample (k+1), which is the best estimate for the present time.
    const real_t angle_compensation =
        0.25 * get_estimated_angular_frequency().value() * _frequency_pi_controller.get_sampling_time();

    return _phase_integrator.get_output() - angle_wrapped::from_radians(angle_t{angle_compensation});
}

angle_wrapped srf_pll::get_estimated_angle_aligned_phase_a() const
{
    // The primary estimated angle is aligned with a cosine. To align with a sine,
    // we add 90 degrees (pi/2 radians).
    return get_estimated_angle() + angle_wrapped::from_radians(angle_t{pi / 2.0});
}

frequency_t srf_pll::get_estimated_frequency() const { return _estimated_frequency; }

frequency_pu_t srf_pll::get_estimated_frequency_pu() const { return to_pu(_estimated_frequency, _nominal_frequency); }

angular_frequency_t srf_pll::get_estimated_angular_frequency() const
{
    return angular_frequency_t{2.0 * pi * _estimated_frequency};
}

const dq<voltage_pu_t>& srf_pll::get_dq_vector() const { return _dq_vector; }

alphabeta<voltage_pu_t> srf_pll::get_alphabeta() const { return _dq_vector.to_alphabeta(get_estimated_angle()); }

}  // namespace yeet::control