#include "yeet_control/field_current_regulator.hpp"

#include <algorithm>
#include <cassert>

namespace yeet::control {

field_current_regulator::field_current_regulator(duration_t sampling_time)
    : _sampling_time(sampling_time), _pi_controller(sampling_time.value())
{
    // Default PI gains for field current control (conservative)
    configure_pi_gains(5.0, 2.0);  // Moderate bandwidth for field control
}

void field_current_regulator::configure_field_circuit(resistance_pu_t field_resistance, duration_t field_time_constant)
{
    assert(field_resistance.value() > 0.0 && "Field resistance must be positive.");
    assert(field_time_constant.value() > 0.0 && "Field time constant must be positive.");

    _field_resistance = field_resistance;
    _field_time_constant = field_time_constant;
}

void field_current_regulator::configure_pi_gains(real_t kp, real_t ti) { _pi_controller.configure_with_ti(kp, ti); }

void field_current_regulator::configure_voltage_limits(voltage_pu_t min_voltage_pu, voltage_pu_t max_voltage_pu)
{
    assert(min_voltage_pu.value() >= 0.0 && "Minimum voltage must be non-negative.");
    assert(max_voltage_pu.value() > min_voltage_pu.value() && "Maximum voltage must exceed minimum voltage.");

    _min_voltage = min_voltage_pu;
    _max_voltage = max_voltage_pu;

    // Configure PI controller limits to match exciter limits
    _pi_controller.set_output_limits(min_voltage_pu.value(), max_voltage_pu.value());
    _pi_controller.enable_output_limits(true);
}

current_pu_t field_current_regulator::update(current_pu_t field_current_reference) noexcept
{
    // Calculate current error
    const real_t current_error = field_current_reference.value() - _field_current.value();

    // Run PI controller to generate exciter voltage command
    const real_t exciter_voltage_raw = _pi_controller.update(current_error);

    // Apply voltage limits (additional safety, PI controller should handle this)
    const real_t exciter_voltage_limited = std::clamp(exciter_voltage_raw, _min_voltage.value(), _max_voltage.value());
    _exciter_voltage = voltage_pu_t{exciter_voltage_limited};

    // Update field circuit dynamics
    _field_current = update_field_circuit_dynamics(_exciter_voltage);

    return _field_current;
}

void field_current_regulator::reset(current_pu_t initial_field_current)
{
    _field_current = initial_field_current;

    // Calculate steady-state exciter voltage for initial current
    // From: R_f * i_f = v_f (at steady state, di/dt = 0)
    const real_t steady_state_voltage = _field_resistance.value() * initial_field_current.value();
    _exciter_voltage = voltage_pu_t{steady_state_voltage};

    // Reset PI controller with this initial output
    _pi_controller.reset(steady_state_voltage);
}

current_pu_t field_current_regulator::get_field_current() const noexcept { return _field_current; }

voltage_pu_t field_current_regulator::get_exciter_voltage() const noexcept { return _exciter_voltage; }

current_pu_t field_current_regulator::update_field_circuit_dynamics(voltage_pu_t exciter_voltage) noexcept
{
    // First-order field winding dynamics: τ_f × di_f/dt + R_f × i_f = v_f
    // Discretized using backward Euler:
    // i_f[k] = (τ_f / (τ_f + R_f × Ts)) × i_f[k-1] + (Ts / (τ_f + R_f × Ts)) × (v_f[k] / R_f)
    //
    // Simplifying with α = Ts / τ_f:
    // i_f[k] = (1 - α × R_f) × i_f[k-1] + α × (v_f[k] / R_f)

    const real_t tau_f = _field_time_constant.value();
    const real_t r_f = _field_resistance.value();
    const real_t ts = _sampling_time.value();

    // Compute discrete-time coefficient
    const real_t denominator = tau_f + ts;
    const real_t a_prev = tau_f / denominator;        // Coefficient for previous current
    const real_t a_input = ts / (r_f * denominator);  // Coefficient for voltage input

    // Update field current
    const real_t new_field_current = a_prev * _field_current.value() + a_input * exciter_voltage.value();

    return current_pu_t{new_field_current};
}

}  // namespace yeet::control
