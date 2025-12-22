#include "sfc_integration/sfc_machine_side_integrator_supervisor_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_integrator_supervisor_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_integrator_supervisor_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_machine_side_integrator_supervisor_runner::initialize()
{
    _outputs.freeze_integrator = false;
    _outputs.preset_request = false;
    _outputs.release_pulses = false;

    _previous_release_speed_controller = false;
}

void sfc_machine_side_integrator_supervisor_runner::run()
{
    const bool release_speed_controller = _inputs.release_speed_controller.read();

    // Detect rising edge: transition from false to true
    const bool handover_edge = !_previous_release_speed_controller && release_speed_controller;

    // Update state for next cycle
    _previous_release_speed_controller = release_speed_controller;

    if (_parameters.release_pulses_override) {
        _outputs.release_pulses = _parameters.release_pulses_override_value;
    }
    else {
        _outputs.release_pulses = _inputs.grid_side_release_pulses;
    }

    if (_parameters.freeze_override) {
        _outputs.freeze_integrator = _parameters.freeze_override_value;
    }
    else {
        // Freeze speed controller integrator when:
        // 1. Grid-side releases pulses = false (disabled or forced commutation)
        // 2. Torque controller is saturated (anti-windup)
        // 3. Braking requested but clamped to minimum motoring torque
        _outputs.freeze_integrator =
            !_inputs.grid_side_release_pulses || _inputs.torque_saturated || _inputs.is_braking_requested;
    }

    // One-shot preset request at handover. Controllers are responsible for their own
    // internal bumpless initialization when this pulse is asserted.
    _outputs.preset_request = handover_edge;
}

void sfc_machine_side_integrator_supervisor_runner::stop()
{
    _outputs.freeze_integrator = false;
    _outputs.preset_request = false;
    _outputs.release_pulses = false;

    // Reset edge detector state to ensure proper detection on restart
    _previous_release_speed_controller = false;
}

}  // namespace yeet::application::sfc::integration
