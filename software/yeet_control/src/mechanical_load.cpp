#include "yeet_control/mechanical_load.hpp"

#include <cassert>

namespace yeet::control {

mechanical_load::mechanical_load(double sampling_time) : _speed_integrator(sampling_time)
{
    assert(sampling_time > 0.0f && "Sampling time must be positive.");
}

void mechanical_load::configure(inertia_constant_t inertia_constant_seconds,
    damping_coefficient_pu_t damping_factor_pu,
    real_t linear_load_coeff,
    real_t quadratic_load_coeff)
{
    assert(inertia_constant_seconds.value() > 0.0f && "Inertia constant H must be positive.");
    assert(damping_factor_pu.value() >= 0.0f && "Damping factor D must be non-negative.");
    assert(linear_load_coeff >= 0.0f && "Linear load coefficient k1 must be non-negative.");
    assert(quadratic_load_coeff >= 0.0f && "Quadratic load coefficient k2 must be non-negative.");

    _inertia_constant_seconds = inertia_constant_seconds;
    _damping_factor_pu = damping_factor_pu;
    _linear_load_coefficient = linear_load_coeff;
    _quadratic_load_coefficient = quadratic_load_coeff;
}

speed_pu_t mechanical_load::update(torque_pu_t electromagnetic_torque_pu, torque_pu_t load_torque)
{
    const real_t current_speed_pu = _speed_integrator.get_output();

    // Calculate damping torque: T_d_pu = D * omega_pu
    const real_t damping_torque_pu = _damping_factor_pu.value() * current_speed_pu;

    // Calculate speed-dependent load torque: T_load_speed_pu = k1 * omega_pu + k2 * omega_pu^2
    const real_t speed_dependent_load_torque_pu =
        _linear_load_coefficient * current_speed_pu + _quadratic_load_coefficient * current_speed_pu * current_speed_pu;

    // Calculate total load torque: T_l_total_pu = T_l_external_pu + T_load_speed_pu
    const real_t total_load_torque_pu = load_torque.value() + speed_dependent_load_torque_pu;

    // Calculate net accelerating torque: T_acc_pu = T_e_pu - T_l_total_pu - T_d_pu
    const real_t accelerating_torque_pu = electromagnetic_torque_pu.value() - total_load_torque_pu - damping_torque_pu;

    // Calculate angular acceleration: d(omega_pu)/dt = T_acc_pu / (2H)
    const real_t angular_acceleration_pu_per_s = accelerating_torque_pu / (2.0f * _inertia_constant_seconds.value());

    // Integrate acceleration to get speed
    const real_t new_speed_pu = _speed_integrator.update(angular_acceleration_pu_per_s);

    return speed_pu_t{new_speed_pu};
}

void mechanical_load::reset(speed_pu_t mechanical_speed_pu) { _speed_integrator.reset(mechanical_speed_pu.value()); }

speed_pu_t mechanical_load::get_speed_pu() const noexcept { return speed_pu_t{_speed_integrator.get_output()}; }

}  // namespace yeet::control