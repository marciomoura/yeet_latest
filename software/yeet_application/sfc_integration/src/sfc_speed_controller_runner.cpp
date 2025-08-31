#include "sfc_integration/sfc_speed_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_speed_controller_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    // Initialize to initial speed
    _sfc_speed_controller.reset(_parameters.initial_speed);
}

void sfc_speed_controller_runner::run()
{
    if (!_inputs.enable_speed_control.read()) {
        _outputs.is_active.write(false);
        _sfc_speed_controller.reset(_inputs.measured_speed_pu);
    }
    else {
        _outputs.is_active.write(true);
        _sfc_speed_controller.freeze_integrator(_inputs.freeze_integrator.read() ||
                                                _inputs.current_controller_is_saturated.read() ||
                                                _inputs.modulator_freeze_integrator.read());
        _sfc_speed_controller.update(_inputs.speed_reference_pu, _inputs.measured_speed_pu);
    }

    _outputs.torque_reference = _sfc_speed_controller.get_torque_ref_pu();
    _outputs.current_reference = current_pu_t{_parameters.torque_to_current_factor * _outputs.torque_reference.read()};
    _outputs.speed_reference_limited = _sfc_speed_controller.get_processed_speed_ref_pu();
    _outputs.speed_error = _outputs.speed_reference_limited.read() - _inputs.measured_speed_pu.read();
    _outputs.pi_output = _sfc_speed_controller.get_pi_output();
    _outputs.is_saturated = _sfc_speed_controller.is_saturated();
}

void sfc_speed_controller_runner::stop() { _sfc_speed_controller.reset(); }

void sfc_speed_controller_runner::set_parameters()
{
    // Configure PI controller gains
    _sfc_speed_controller.configure_pi_gains(_parameters.kp, _parameters.ti);

    // Configure speed reference rate limiting for smooth acceleration/deceleration
    _sfc_speed_controller.configure_speed_reference_rate_limit(_parameters.speed_reference_rate_limit);

    // Configure feedforward control based on inertia constant
    _sfc_speed_controller.configure_feedforward_pu(_parameters.feedforward_inertia);

    // Configure torque limits
    _sfc_speed_controller.configure_torque_limits_pu(
        -_parameters.torque_reference_magnitude_limit.read(), _parameters.torque_reference_magnitude_limit.read());

    // Configure speed reference limits
    _sfc_speed_controller.configure_speed_reference_limits_pu(
        speed_pu_t(0.0f), _parameters.speed_reference_magnitude_limit);

    // Configure speed reference rate limit
    _sfc_speed_controller.configure_speed_reference_rate_limit(_parameters.speed_reference_rate_limit);
}

void sfc_speed_controller_runner::set_parameter_callback()
{
    _parameters.kp.register_callback([&]() { set_parameters(); });
    _parameters.ti.register_callback([&]() { set_parameters(); });
    _parameters.speed_reference_rate_limit.register_callback([&]() { set_parameters(); });
    _parameters.speed_reference_magnitude_limit.register_callback([&]() { set_parameters(); });
    _parameters.torque_to_current_factor.register_callback([&]() { set_parameters(); });
    _parameters.torque_reference_magnitude_limit.register_callback([&]() { set_parameters(); });
    // _parameters.initial_speed.register_callback([&]() { set_parameters(); });
    _parameters.feedforward_inertia.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration