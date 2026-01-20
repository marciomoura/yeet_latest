#include "sfc_integration/sfc_current_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_current_controller_runner::register_datamodel(yeet::telemetry::parameter_handler& handler)
{
    // Delegate to the datamodel's registration method
    sfc_current_controller_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(handler);
}

void sfc_current_controller_runner::initialize()
{
    _sfc_current_controller.reset();

    set_controller_parameters();
    set_limit_parameters();
    set_pi_output_limits();
    set_parameter_callback();
}

void sfc_current_controller_runner::run()
{
    const bool freeze_integrator = _inputs.freeze_integrator.read();

    _sfc_current_controller.freeze_integrator(freeze_integrator);

    const voltage_pu_t reflected_machine_voltage_magnitude =
        _per_unit_system->reflect_machine_to_grid_voltage_pu(_inputs.machine_voltage_magnitude.read());

    current_pu_t dc_link_current_reference_pu;
    if (_parameters.override_current_reference.read()) {
        dc_link_current_reference_pu = _parameters.override_current_reference_value.read();
    }
    else {
        dc_link_current_reference_pu = _inputs.current_reference.read();
    }

    _sfc_current_controller.update(dc_link_current_reference_pu, _inputs.current_measured.read(),
        _inputs.grid_voltage_magnitude.read(), reflected_machine_voltage_magnitude);

    _outputs.firing_angle.write(
        angle_wrapped::from_radians(_sfc_current_controller.get_firing_angle_reference_radians()));

    _outputs.current_error.write(dc_link_current_reference_pu - _inputs.current_measured.read());
    _outputs.is_saturated.write(_sfc_current_controller.is_saturated());
    _outputs.pi_output.write(_sfc_current_controller.get_pi_output());
    _outputs.is_reference_current_limited.write(_sfc_current_controller.is_reference_current_limited());
    _outputs.current_reference.write(dc_link_current_reference_pu);
    _outputs.is_current_measured_above_magnitude_limit.write(
        _sfc_current_controller.is_current_measured_above_magnitude_limit());

    _outputs.is_active.write(true);  // TODO: To be implemented
}

void sfc_current_controller_runner::stop() { _sfc_current_controller.reset(); }

void sfc_current_controller_runner::set_limit_parameters()
{
    const angle_t min_firing_angle = _parameters.min_firing_angle.read().get_radians();
    const angle_t max_firing_angle = _parameters.max_firing_angle.read().get_radians();
    const current_pu_t current_reference_magnitude_limit = _parameters.current_reference_magnitude_limit.read();

    _sfc_current_controller.configure_firing_angle_limits_radians(min_firing_angle, max_firing_angle);
    _sfc_current_controller.configure_current_limits_pu(
        -current_reference_magnitude_limit, current_reference_magnitude_limit);
}

void sfc_current_controller_runner::set_pi_output_limits()
{
    const real_t pi_output_min = _parameters.pi_output_min.read();
    const real_t pi_output_max = _parameters.pi_output_max.read();

    _sfc_current_controller.configure_pi_output_limits(pi_output_min, pi_output_max);
}

void sfc_current_controller_runner::set_controller_parameters()
{
    const real_t kp = _parameters.kp.read();
    const duration_t ti = _parameters.ti.read();

    _sfc_current_controller.configure_pi_gains(kp, ti);
}

void sfc_current_controller_runner::set_parameter_callback()
{
    _parameters.kp.register_callback(
        [&]() { _sfc_current_controller.configure_pi_gains(_parameters.kp, _parameters.ti); });
    _parameters.ti.register_callback(
        [&]() { _sfc_current_controller.configure_pi_gains(_parameters.kp, _parameters.ti); });
    _parameters.min_firing_angle.register_callback([&]() {
        _sfc_current_controller.configure_firing_angle_limits_radians(
            _parameters.min_firing_angle.read().get_radians(), _parameters.max_firing_angle.read().get_radians());
    });
    _parameters.max_firing_angle.register_callback([&]() {
        _sfc_current_controller.configure_firing_angle_limits_radians(
            _parameters.min_firing_angle.read().get_radians(), _parameters.max_firing_angle.read().get_radians());
    });
    _parameters.current_reference_magnitude_limit.register_callback([&]() {
        const current_pu_t limit = _parameters.current_reference_magnitude_limit.read();
        _sfc_current_controller.configure_current_limits_pu(-limit, limit);
    });
    _parameters.pi_output_min.register_callback([&]() {
        _sfc_current_controller.configure_pi_output_limits(_parameters.pi_output_min, _parameters.pi_output_max);
    });
    _parameters.pi_output_max.register_callback([&]() {
        _sfc_current_controller.configure_pi_output_limits(_parameters.pi_output_min, _parameters.pi_output_max);
    });
    _parameters.enable_feedforward.register_callback(
        [&]() { _sfc_current_controller.enable_feedforward(_parameters.enable_feedforward.read()); });
}

}  // namespace yeet::application::sfc::integration