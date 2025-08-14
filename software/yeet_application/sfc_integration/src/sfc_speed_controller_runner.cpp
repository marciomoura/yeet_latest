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
    const speed_pu_t measured_speed = speed_pu_t{_inputs.measured_speed_pu.read().value() / 50.0};

    _sfc_speed_controller.update(_inputs.speed_reference_pu, measured_speed);
    // Convert torque output to current (extract numerical value)
    torque_pu_t torque_ref = _sfc_speed_controller.get_torque_ref_pu();
    _outputs.current_reference = current_pu_t(torque_ref.value());
    _outputs.speed_error = speed_pu_t(_inputs.speed_reference_pu.read().value() - measured_speed);
}

void sfc_speed_controller_runner::stop() { _sfc_speed_controller.reset(); }

void sfc_speed_controller_runner::set_parameters()
{
    // // Convert integral gain to integral time constant
    // duration_t integral_time = duration_t(1.0 / _parameters.pi_integral_gain);

    // _sfc_speed_controller.configure_pi_gains(_parameters.pi_proportional_gain, integral_time);

    // // Convert current limits to torque limits (extract numerical value for per-unit conversion)
    // current_pu_t min_current = _parameters.min_current_reference;
    // current_pu_t max_current = _parameters.max_current_reference;
    // torque_pu_t min_torque = torque_pu_t(min_current.value());
    // torque_pu_t max_torque = torque_pu_t(max_current.value());
    // _sfc_speed_controller.configure_torque_limits_pu(min_torque, max_torque);

    // speed_pu_t speed_limit = _parameters.speed_reference_magnitude_limit;
    // _sfc_speed_controller.configure_speed_reference_limits_pu(speed_pu_t(-speed_limit.value()), speed_limit);
    // _sfc_speed_controller.configure_speed_reference_rate_limit(_parameters.speed_reference_rate_limit);
}

void sfc_speed_controller_runner::set_parameter_callback()
{
    // _parameters.pi_proportional_gain.register_callback([&]() { set_parameters(); });
    // _parameters.pi_integral_gain.register_callback([&]() { set_parameters(); });
    // _parameters.speed_reference_rate_limit.register_callback([&]() { set_parameters(); });
    // _parameters.speed_reference_magnitude_limit.register_callback([&]() { set_parameters(); });
    // _parameters.max_current_reference.register_callback([&]() { set_parameters(); });
    // _parameters.min_current_reference.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration