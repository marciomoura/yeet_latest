#include "sfc_integration/sfc_speed_controller_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_speed_controller_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_speed_controller_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_speed_controller_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    // Initialize to initial speed
    _sfc_speed_controller.reset(_parameters.initial_speed);
}

void sfc_speed_controller_runner::run()
{
    // CRITICAL: Use composite saturation from torque controller, which includes
    // both torque-level and current-level saturation
    // const bool downstream_saturated = _inputs.is_saturated.read();

    // // CRITICAL: Also freeze during external events from supervisor
    // const bool external_freeze_requested = _inputs.freeze_integrator.read();

    // Combine: freeze if EITHER downstream is saturated OR external freeze requested
    const bool freeze_integrator = _inputs.is_saturated;

    if (!_inputs.release_pulses.read()) {
        // When disabled, controller is inactive (no output updates)
        // When re-enabling, could reset to current measured speed to prevent bump
        // _sfc_speed_controller.reset(_inputs.measured_speed.read());
        return;  // Skip update when disabled
    }

    // Execute controller update
    _sfc_speed_controller.freeze_integrator(freeze_integrator);
    _sfc_speed_controller.update(_inputs.speed_reference, _inputs.measured_speed);

    // Write output signals
    _outputs.torque_reference = _sfc_speed_controller.get_torque_ref_pu();
    _outputs.pi_output = _sfc_speed_controller.get_pi_output();
    _outputs.is_saturated = _sfc_speed_controller.is_saturated();
}

void sfc_speed_controller_runner::stop() { _sfc_speed_controller.reset(); }

void sfc_speed_controller_runner::set_parameters()
{
    // Configure PI controller gains
    _sfc_speed_controller.configure_pi_gains(_parameters.kp, _parameters.ti);

    // Configure feedforward control based on inertia constant
    _sfc_speed_controller.configure_feedforward_pu(_parameters.feedforward_inertia);

    // Configure torque limits
    _sfc_speed_controller.configure_torque_limits_pu(
        -_parameters.torque_reference_magnitude_limit.read(), _parameters.torque_reference_magnitude_limit.read());
}

void sfc_speed_controller_runner::set_parameter_callback()
{
    _parameters.kp.register_callback([&]() { set_parameters(); });
    _parameters.ti.register_callback([&]() { set_parameters(); });
    _parameters.torque_reference_magnitude_limit.register_callback([&]() { set_parameters(); });
    _parameters.feedforward_inertia.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration