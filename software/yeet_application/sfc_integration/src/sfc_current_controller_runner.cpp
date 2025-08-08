#include "sfc_integration/sfc_current_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_current_controller_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    _sfc_current_controller.enable_current_reference_slew_rate(false);
    _sfc_current_controller.reset();
    _sfc_current_controller.configure_pi_gains(0.5, duration_t(ms(100.0)));
}

void sfc_current_controller_runner::run()
{
    _sfc_current_controller.update(_inputs.current_reference_pu, _inputs.dc_link_current_measured_pu,
        _inputs.ideal_dc_voltage_pu, _inputs.inverter_back_emf_machine_pu);

    // Convert angle_t to angle_wrapped
    angle_t firing_angle_rad = _sfc_current_controller.get_firing_angle_reference_radians();
    _outputs.firing_angle_reference = angle_wrapped::from_radians(angle_t{firing_angle_rad});

    // Calculate current error
    current_pu_t current_ref = _inputs.current_reference_pu;
    current_pu_t current_meas = _inputs.dc_link_current_measured_pu;
    _outputs.current_error = current_pu_t(current_ref.value() - current_meas.value());

    _outputs.pi_output = angle_wrapped::from_radians(angle_t{firing_angle_rad});
    _outputs.is_saturated = false;  // This should be implemented in the controller
    _outputs.is_active = true;      // Assume always active for now
}

void sfc_current_controller_runner::stop() { _sfc_current_controller.reset(); }

void sfc_current_controller_runner::set_parameters()
{
    // // Convert integral gain to integral time constant
    // duration_t integral_time = duration_t(1.0 / _parameters.pi_integral_gain);

    // _sfc_current_controller.configure_pi_gains(_parameters.pi_proportional_gain, integral_time);

    // // Convert angle_wrapped to angle_t (radians)
    // angle_wrapped min_firing_angle_wrapped = _parameters.min_firing_angle;
    // angle_wrapped max_firing_angle_wrapped = _parameters.max_firing_angle;
    // angle_t min_angle = angle_t(min_firing_angle_wrapped.get_radians());
    // angle_t max_angle = angle_t(max_firing_angle_wrapped.get_radians());
    // _sfc_current_controller.configure_firing_angle_limits_radians(min_angle, max_angle);

    // current_pu_t current_limit = _parameters.current_reference_magnitude_limit;
    // _sfc_current_controller.configure_current_limits_pu(current_pu_t(-current_limit.value()), current_limit);
    // _sfc_current_controller.configure_current_reference_slew_rate(_parameters.current_reference_rate_limit);
}

void sfc_current_controller_runner::set_parameter_callback()
{
    _parameters.pi_proportional_gain.register_callback([&]() { set_parameters(); });
    _parameters.pi_integral_gain.register_callback([&]() { set_parameters(); });
    _parameters.min_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.max_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.current_reference_rate_limit.register_callback([&]() { set_parameters(); });
    _parameters.current_reference_magnitude_limit.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration