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
    _outputs.request_shutdown = false;
}

void sfc_machine_side_integrator_supervisor_runner::run()
{
    if (_parameters.override_release_pulses) {
        _outputs.release_pulses = _parameters.release_pulses_value;
    }
    else {
        _outputs.release_pulses = _inputs.grid_side_release_pulses;
    }

    if (_parameters.override_is_tripped) {
        _outputs.request_shutdown = _parameters.is_tripped_value;
    }
    else {
        // Propagate machine-side protection trips to grid-side via fiber optics
        _outputs.request_shutdown = _inputs.protection_shutdown_request;
    }

    if (_parameters.override_freeze) {
        _outputs.freeze_integrator = _parameters.freeze_value;
    }
    else {
        // Freeze speed controller integrator when:
        // 1. Grid-side releases pulses = false (disabled or forced commutation)
        // 2. Torque controller is saturated (anti-windup)
        // 3. Braking requested but clamped to minimum motoring torque
        _outputs.freeze_integrator =
            !_inputs.grid_side_release_pulses || _inputs.torque_saturated || _inputs.is_braking_requested;
    }

    // TODO
    _outputs.preset_request = false;
}

void sfc_machine_side_integrator_supervisor_runner::stop()
{
    _outputs.freeze_integrator = false;
    _outputs.preset_request = false;
    _outputs.release_pulses = false;
    _outputs.request_shutdown = true;
}

}  // namespace yeet::application::sfc::integration
