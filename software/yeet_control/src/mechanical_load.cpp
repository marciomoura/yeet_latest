#include "yeet_control/mechanical_load.hpp"

#include <cassert>

namespace yeet::control {

mechanical_load::mechanical_load(double ts) : _speed_integrator(ts) {}

void mechanical_load::configure(double inertia_constant_seconds, double damping_factor_pu) {
    assert(inertia_constant_seconds > 0.0 && "Inertia constant H must be positive.");
    assert(damping_factor_pu >= 0.0 && "Damping factor D must be non-negative.");
    _inertia_constant_seconds = inertia_constant_seconds;
    _damping_factor_pu = damping_factor_pu;
}

double mechanical_load::update(double electromagnetic_torque_pu, double load_torque_pu) {
    const double current_speed_pu = _speed_integrator.get_value();

    // Calculate damping torque: T_d_pu = D * omega_pu
    const double damping_torque_pu = _damping_factor_pu * current_speed_pu;

    // Calculate net accelerating torque: T_acc_pu = T_e_pu - T_l_pu - T_d_pu
    const double accelerating_torque_pu = electromagnetic_torque_pu - load_torque_pu - damping_torque_pu;

    // Calculate angular acceleration: d(omega_pu)/dt = T_acc_pu / (2H)
    const double angular_acceleration_pu_per_s = accelerating_torque_pu / (2.0 * _inertia_constant_seconds);

    // Integrate acceleration to get speed
    return _speed_integrator.update(angular_acceleration_pu_per_s);
}

void mechanical_load::reset(double mechanical_speed_pu) { _speed_integrator.set_value(mechanical_speed_pu); }

double mechanical_load::get_speed_pu() const { return _speed_integrator.get_value(); }

}  // namespace yeet::control