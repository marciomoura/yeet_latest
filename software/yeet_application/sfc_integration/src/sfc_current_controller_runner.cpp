#include "sfc_integration/sfc_current_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_current_controller_runner::initialize()
{
    _sfc_current_controller.enable_current_reference_slew_rate(false);
    _sfc_current_controller.reset();

    set_controller_parameters();
    set_limit_parameters();
    set_parameter_callback();
}

void sfc_current_controller_runner::run()
{
    _sfc_current_controller.freeze_integrator(
        _inputs.freeze_integrator.read() || _inputs.modulator_freeze_integrator.read());

    const voltage_pu_t reflected_machine_side_voltage_magnitude =
        _per_unit_system->reflect_machine_to_grid_voltage_pu(_inputs.machine_side_voltage_magnitude.read());

    _sfc_current_controller.update(_inputs.dc_link_current_reference.read(), _inputs.dc_link_current_measured.read(),
        _inputs.grid_side_voltage_magnitude.read(), reflected_machine_side_voltage_magnitude);

    _outputs.firing_angle.write(
        angle_wrapped::from_radians(_sfc_current_controller.get_firing_angle_reference_radians()));

    _outputs.current_error.write(_inputs.dc_link_current_reference.read() - _inputs.dc_link_current_measured.read());
    _outputs.current_controller_is_saturated.write(_sfc_current_controller.is_saturated());

    _outputs.is_active.write(true);  // TODO: Implement
}

void sfc_current_controller_runner::stop() { _sfc_current_controller.reset(); }

void sfc_current_controller_runner::set_limit_parameters()
{
    const angle_t min_firing_angle = _parameters.min_firing_angle.read().get_radians();
    const angle_t max_firing_angle = _parameters.max_firing_angle.read().get_radians();
    const real_t current_reference_rate_limit = _parameters.current_reference_rate_limit.read();
    const current_pu_t current_reference_magnitude_limit = _parameters.current_reference_magnitude_limit.read();

    _sfc_current_controller.configure_firing_angle_limits_radians(min_firing_angle, max_firing_angle);
    _sfc_current_controller.configure_current_limits_pu(
        -current_reference_magnitude_limit, current_reference_magnitude_limit);
    _sfc_current_controller.configure_current_reference_slew_rate(current_reference_rate_limit);
}

void sfc_current_controller_runner::set_controller_parameters()
{
    const real_t kp = _parameters.kp.read();
    const duration_t ti = _parameters.ti.read();

    _sfc_current_controller.configure_pi_gains(kp, ti);
}

void sfc_current_controller_runner::set_parameter_callback()
{
    _parameters.kp.register_callback([&]() { set_controller_parameters(); });
    _parameters.ti.register_callback([&]() { set_controller_parameters(); });
    _parameters.min_firing_angle.register_callback([&]() { set_limit_parameters(); });
    _parameters.max_firing_angle.register_callback([&]() { set_limit_parameters(); });
    _parameters.current_reference_rate_limit.register_callback([&]() { set_limit_parameters(); });
    _parameters.current_reference_magnitude_limit.register_callback([&]() { set_limit_parameters(); });
}

}  // namespace yeet::application::sfc::integration