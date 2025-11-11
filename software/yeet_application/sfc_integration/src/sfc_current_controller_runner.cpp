#include "sfc_integration/sfc_current_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_current_controller_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_current_controller_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_current_controller_runner::initialize()
{
    _sfc_current_controller.reset();

    set_controller_parameters();
    set_limit_parameters();
    set_parameter_callback();
}

void sfc_current_controller_runner::run()
{
    // Disable integrator when current reference is at or near zero
    // The LCI cannot actually achieve zero current (requires minimum current for commutation).
    // Integrating this persistent error would cause the integrator to wind down to negative values.
    // This is a physical constraint - we cannot impress zero current in a thyristor-based system.
    // Threshold set conservatively low to only prevent windup at true zero reference.
    constexpr current_pu_t minimum_controllable_current{0.001};  // 0.1% of base current
    bool is_integrator_frozen_for_zero_reference = _inputs.current_reference.read() <= minimum_controllable_current;

    const bool freeze_integrator = _inputs.freeze_integrator.read() || is_integrator_frozen_for_zero_reference;
    _sfc_current_controller.freeze_integrator(freeze_integrator);

    const voltage_pu_t reflected_machine_voltage_magnitude =
        _per_unit_system->reflect_machine_to_grid_voltage_pu(_inputs.machine_voltage_magnitude.read());

    const current_pu_t dc_link_current_reference_pu =
        _inputs.current_reference.read() + _parameters.current_reference_rectifier_operation.read();

    _sfc_current_controller.update(dc_link_current_reference_pu, _inputs.current_measured.read(),
        _inputs.grid_voltage_magnitude.read(), reflected_machine_voltage_magnitude);

    _outputs.firing_angle.write(
        angle_wrapped::from_radians(_sfc_current_controller.get_firing_angle_reference_radians()));

    _outputs.current_error.write(_inputs.current_reference.read() - _inputs.current_measured.read());
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
    _parameters.current_reference_magnitude_limit.register_callback([&]() { set_limit_parameters(); });
}

}  // namespace yeet::application::sfc::integration