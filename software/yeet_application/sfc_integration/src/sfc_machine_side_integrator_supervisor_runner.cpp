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
    const bool grid_side_freeze_integrator = _inputs.grid_side_freeze_integrator.read();
    const bool grid_side_is_tripped = _inputs.grid_side_is_tripped.read();
    const bool grid_side_release_pulses = _inputs.grid_side_release_pulses.read();
    const bool current_is_zero = _inputs.current_is_zero.read();
    const bool speed_saturated = _inputs.speed_saturated.read();
    const bool torque_saturated = _inputs.torque_saturated.read();
    const bool override_freeze = _parameters.override_freeze.read();

    // Update supervisor core logic
    _sfc_machine_side_integrator_supervisor.update(grid_side_freeze_integrator, grid_side_is_tripped,
        grid_side_release_pulses, current_is_zero, speed_saturated, torque_saturated, override_freeze);

    // Write outputs to datamodel
    _outputs.freeze_integrator.write(torque_saturated);
    _outputs.preset_request.write(_sfc_machine_side_integrator_supervisor.get_preset_request());
    _outputs.release_pulses.write(_sfc_machine_side_integrator_supervisor.get_release_pulses());
    _outputs.is_tripped.write(_sfc_machine_side_integrator_supervisor.get_is_tripped());
}

void sfc_machine_side_integrator_supervisor_runner::stop() { _sfc_machine_side_integrator_supervisor.reset(); }

}  // namespace yeet::application::sfc::integration
