#include "sfc_integration/sfc_machine_side_integrator_supervisor_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_integrator_supervisor_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_integrator_supervisor_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_machine_side_integrator_supervisor_runner::initialize() { _sfc_machine_side_integrator_supervisor.reset(); }

void sfc_machine_side_integrator_supervisor_runner::run()
{
    // Read all inputs from datamodel
    const bool grid_side_freeze = _inputs.grid_side_freeze.read();
    const bool emergency_stop = _inputs.emergency_stop.read();
    const bool torque_saturated = _inputs.torque_saturated.read();
    const bool enable_speed_controller = _inputs.enable_speed_controller.read();
    const bool speed_preset_complete = _inputs.speed_preset_complete.read();
    const bool enable_torque_controller = _inputs.enable_torque_controller.read();
    const bool torque_preset_complete = _inputs.torque_preset_complete.read();
    const bool override_freeze = _parameters.override_freeze.read();

    // Update supervisor core logic
    _sfc_machine_side_integrator_supervisor.update(grid_side_freeze, emergency_stop, torque_saturated,
        enable_speed_controller, speed_preset_complete, enable_torque_controller, torque_preset_complete,
        override_freeze);

    // Write outputs to datamodel
    _outputs.freeze_integrator.write(_sfc_machine_side_integrator_supervisor.get_freeze_integrator());
    _outputs.preset_request.write(_sfc_machine_side_integrator_supervisor.get_preset_request());
    _outputs.release_pulses.write(_sfc_machine_side_integrator_supervisor.get_release_pulses());
    _outputs.is_tripped.write(_sfc_machine_side_integrator_supervisor.get_is_tripped());
}

void sfc_machine_side_integrator_supervisor_runner::stop() { _sfc_machine_side_integrator_supervisor.reset(); }

}  // namespace yeet::application::sfc::integration
