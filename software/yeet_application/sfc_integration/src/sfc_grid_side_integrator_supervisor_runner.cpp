#include "sfc_integration/sfc_grid_side_integrator_supervisor_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_integrator_supervisor_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_integrator_supervisor_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_grid_side_integrator_supervisor_runner::initialize()
{
    _outputs.preset_controller = false;
    _outputs.release_pulses = false;
    _outputs.shutdown_request = false;
    _outputs.freeze_integrator = false;
}

void sfc_grid_side_integrator_supervisor_runner::run()
{
    // If state machine requests pulse release, pass to output
    if (_parameters.release_pulses_override) {
        _outputs.release_pulses = _parameters.release_pulses_override_value;
    }
    else {
        _outputs.release_pulses = _inputs.state_machine_release_pulses;
    }

    // Handle trip latching on machine_side_shutdown_request input
    if (_parameters.is_tripped_override) {
        _outputs.shutdown_request = _parameters.is_tripped_override_value;
    }
    else {
        // Use grid-side protection shutdown request
        _outputs.shutdown_request = _inputs.grid_protection_shutdown_request;
    }

    // Override freeze if parameter is set
    if (_parameters.freeze_override) {
        _outputs.freeze_integrator = _parameters.freeze_override_value;
    }
    else {
        // Freeze current controller integrator when:
        // 1. State machine releases pulses = false (system not modulating)
        // 2. Shutdown request active (protection trip or machine-side shutdown)
        // 3. Forced commutation requests freeze (during forced commutation transitions)
        _outputs.freeze_integrator = !_inputs.state_machine_release_pulses || _outputs.shutdown_request ||
                                     _inputs.forced_commutation_freeze_integrator;
    }

    // Logic should be decided on preset
    _outputs.preset_controller = false;
}

void sfc_grid_side_integrator_supervisor_runner::stop()
{
    _outputs.preset_controller = false;
    _outputs.release_pulses = false;
    _outputs.shutdown_request = false;
    _outputs.freeze_integrator = false;
}

}  // namespace yeet::application::sfc::integration
