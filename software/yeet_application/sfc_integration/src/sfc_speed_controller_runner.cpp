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
    _sfc_speed_controller.configure_torque_limits_pu(
        torque_pu_t{0.0}, _parameters.torque_reference_magnitude_limit.read());
    _sfc_speed_controller.reset(_parameters.initial_speed);
}

void sfc_speed_controller_runner::run()
{
    // Track previous release state for edge detection
    static bool previous_release_state = false;
    const bool current_release_state = _inputs.release_pulses.read();

    if (!current_release_state) {
        // Not enabled - reset controller state
        _sfc_speed_controller.preset_with_torque({});
    }

    // Detect rising edge (false → true transition) for bumpless transfer
    if (!previous_release_state && current_release_state) {
        // First release: initialize integrator with torque reference for smooth transition
        _sfc_speed_controller.preset_with_torque(_inputs.applied_torque_reference);
    }

    previous_release_state = current_release_state;

    // Execute controller update
    if (current_release_state) {
        // Combine: freeze if downstream is saturated OR supervisor requests freeze
        const bool freeze_integrator = _inputs.is_saturated || _inputs.freeze_integrator;

        _sfc_speed_controller.freeze_integrator(freeze_integrator);
        _sfc_speed_controller.update(_inputs.speed_reference, _inputs.measured_speed);

        // Write output signals
        _outputs.torque_reference = _sfc_speed_controller.get_torque_ref_pu();
        _outputs.pi_output = _sfc_speed_controller.get_pi_output();
        _outputs.is_saturated = _sfc_speed_controller.is_saturated();
    }
    else {
        // Write zero outputs when not enabled
        _outputs.torque_reference = torque_pu_t{0.0};
        _outputs.pi_output = real_t{0.0};
        _outputs.is_saturated = false;
    }
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