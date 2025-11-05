#include "sfc_integration/sfc_grid_side_integrator_supervisor_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_integrator_supervisor_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_grid_side_integrator_supervisor_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_grid_side_integrator_supervisor_runner::initialize() { _sfc_grid_side_integrator_supervisor.reset(); }

void sfc_grid_side_integrator_supervisor_runner::run()
{
    // Read all inputs from datamodel
    const bool forced_commutation_freeze = _inputs.forced_commutation_freeze_integrator.read();
    const bool shutdown_request = _inputs.shutdown_request.read();
    const bool preset_complete = _inputs.preset_complete.read();
    const bool override_freeze = _parameters.override_freeze.read();

    // Update supervisor core logic
    // Note: enable_current_controller parameter is placeholder for future use
    _sfc_grid_side_integrator_supervisor.update(
        forced_commutation_freeze, shutdown_request, true, preset_complete, override_freeze);

    // Write outputs to datamodel
    _outputs.freeze_integrator.write(_sfc_grid_side_integrator_supervisor.get_freeze_integrator());
    _outputs.preset_request.write(false);  //_sfc_grid_side_integrator_supervisor.get_preset_request());
    // _outputs.release_pulses.write(_sfc_grid_side_integrator_supervisor.get_release_pulses());
    // Directly pass through some signals
    _outputs.release_pulses = _inputs.state_machine_release_pulses;
    _outputs.is_tripped.write(_sfc_grid_side_integrator_supervisor.get_is_tripped());
}

void sfc_grid_side_integrator_supervisor_runner::stop() { _sfc_grid_side_integrator_supervisor.reset(); }

}  // namespace yeet::application::sfc::integration
