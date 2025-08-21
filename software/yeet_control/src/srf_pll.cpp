#include "yeet_control/srf_pll.hpp"

#include <cassert>
#include <cmath>

#include "yeet_control/constants.hpp"
#include "yeet_control/precomputed_divisor.hpp"

namespace yeet::control {

srf_pll::srf_pll(duration_t sampling_time)
    : _frequency_pi_controller(sampling_time.value()), _phase_integrator(sampling_time.value())
{
    assert(sampling_time.value() > 0.0f && "Sampling time must be positive");
    reset();
}

void srf_pll::configure_nominal_frequency(frequency_t nominal_frequency)
{
    assert(nominal_frequency.value() >= 0.0f && "Nominal frequency must be non-negative");
    _nominal_frequency = precomputed_divisor<frequency_t>{nominal_frequency};
}

void srf_pll::configure_pi_controller(real_t kp, duration_t ti) { _frequency_pi_controller.configure_with_ti(kp, ti); }

void srf_pll::reset() { reset({}); }

void srf_pll::reset(frequency_pu_t initial_frequency)
{
    _dq_vector = {};
    _initial_frequency = initial_frequency;
    _estimated_frequency = to_si(_initial_frequency, _nominal_frequency);
    _frequency_pi_controller.reset();
    _phase_integrator.reset();
}

void srf_pll::update(const alphabeta<voltage_pu_t>& input_alphabeta)
{
    // Park transform (αβ → dq) using the current estimated angle
    _dq_vector = input_alphabeta.to_dq(_phase_integrator.get_output());

    // The q-axis component is the phase error. A PI controller drives this
    // error to zero by adjusting the estimated angular frequency deviation.
    const real_t frequency_deviation = _frequency_pi_controller.update(_dq_vector.q());
    _estimated_frequency = _nominal_frequency.value() * (_initial_frequency.value() + frequency_deviation);

    // Integrate the angular frequency to get the new phase angle for the next cycle
    _phase_integrator.update(2 * pi * _estimated_frequency);
}

angle_wrapped srf_pll::get_estimated_angle() const
{
    return _phase_integrator.get_output() + angle_wrapped{pi / 2};
    // -       angle_wrapped(_estimated_angular_frequency * _frequency_pi_controller.get_sampling_time());
}

frequency_pu_t srf_pll::get_estimated_frequency_pu() const
{
    return frequency_pu_t{_estimated_frequency / _nominal_frequency};
}

angular_frequency_t srf_pll::get_estimated_angular_frequency() const
{
    return angular_frequency_t{2.0 * pi * _estimated_frequency};
}

const dq<voltage_pu_t>& srf_pll::get_dq_vector() const { return _dq_vector; }

}  // namespace yeet::control