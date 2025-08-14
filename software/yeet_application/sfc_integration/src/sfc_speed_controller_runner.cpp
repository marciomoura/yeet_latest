#include "sfc_integration/sfc_speed_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_speed_controller_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    _sfc_speed_controller.configure_pi_gains(3.0, duration_t(0.5));  // Kp, Ti
    _sfc_speed_controller.configure_torque_limits_pu(torque_pu_t(-1.5), torque_pu_t(1.5));
    _sfc_speed_controller.configure_speed_reference_limits_pu(speed_pu_t(0.0), speed_pu_t(1.5));
    _sfc_speed_controller.configure_speed_reference_rate_limit(1.0);  // 1.0 pu/s rate limit
    _sfc_speed_controller.configure_feedforward_pu(0.0);              // Small feedforward gain
    _sfc_speed_controller.reset(speed_pu_t{1.0});
}

void sfc_speed_controller_runner::run()
{
    _sfc_speed_controller.update(_inputs.speed_reference_pu, _inputs.measured_speed_pu);

    // Convert torque output to current using the configured conversion factor
    torque_pu_t torque_ref = _sfc_speed_controller.get_torque_ref_pu();
    real_t current_value = torque_ref.value() * _parameters.torque_to_current_factor;
    _outputs.current_reference = current_pu_t(current_value);

    // Calculate speed error for monitoring
    _outputs.speed_error = _inputs.speed_reference_pu.read() - _inputs.measured_speed_pu.read();

    // Additional outputs using available methods
    _outputs.torque_reference = torque_ref;
    _outputs.speed_reference_limited = _sfc_speed_controller.get_processed_speed_ref_pu();
    _outputs.pi_output = _sfc_speed_controller.get_pi_output();
    _outputs.is_saturated = _sfc_speed_controller.is_saturated();
    _outputs.is_speed_control_active = _inputs.enable_speed_control;
}

void sfc_speed_controller_runner::stop() { _sfc_speed_controller.reset(); }

void sfc_speed_controller_runner::set_parameters()
{
    // Configure PI controller gains
    _sfc_speed_controller.configure_pi_gains(
        _parameters.controller_proportional_gain_kp, _parameters.controller_time_constant_ti);

    // Configure speed reference rate limiting for smooth acceleration/deceleration
    _sfc_speed_controller.configure_speed_reference_rate_limit(_parameters.speed_reference_rate_limit);

    // Configure feedforward control based on inertia constant
    // Access inertia constant parameter directly (no .value() needed for parameters)
    inertia_constant_pu_t feedforward_inertia = _parameters.feedforward_inertia;
    real_t feedforward_gain = feedforward_inertia.value();
    _sfc_speed_controller.configure_feedforward_pu(feedforward_gain);
}

void sfc_speed_controller_runner::set_parameter_callback()
{
    _parameters.controller_proportional_gain_kp.register_callback([&]() { set_parameters(); });
    _parameters.controller_time_constant_ti.register_callback([&]() { set_parameters(); });
    _parameters.speed_reference_rate_limit.register_callback([&]() { set_parameters(); });
    _parameters.torque_to_current_factor.register_callback([&]() { set_parameters(); });
    _parameters.feedforward_inertia.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration