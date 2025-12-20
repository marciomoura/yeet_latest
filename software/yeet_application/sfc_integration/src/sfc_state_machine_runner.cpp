#include "sfc_integration/sfc_state_machine_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_state_machine_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_state_machine_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_state_machine_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    // Reset state machine to initial IDLE state
    _sfc_state_machine.reset();
}

void sfc_state_machine_runner::run()
{
    // First, update input echo outputs so state machine events see current values
    // These are read by the state machine event lambdas during update()
    _outputs.auxiliaries_are_on = _parameters.auxiliaries_are_on_override.read()
                                      ? _parameters.auxiliaries_are_on_override_value.read()
                                      : _inputs.auxiliaries_are_on.read();

    _outputs.mv_breaker_is_closed = _parameters.mv_breaker_is_closed_override.read()
                                        ? _parameters.mv_breaker_is_closed_override_value.read()
                                        : _inputs.mv_breaker_is_closed.read();

    // Echo start/stop commands from inputs (which come from external_ios, sourced from grid_io_mock)
    // Parameters can override for direct testing without IO mock
    _outputs.start_command = _parameters.start_command.read() || _inputs.start_command.read();
    _outputs.stop_command = _parameters.stop_command.read() || _inputs.stop_command.read();

    _outputs.shutdown_request = _parameters.shutdown_request_override.read()
                                    ? _parameters.shutdown_request_override_value.read()
                                    : _inputs.shutdown_request.read();

    _outputs.is_current_extinguished = _parameters.is_current_extinguished_override.read()
                                           ? _parameters.is_current_extinguished_override_value.read()
                                           : _inputs.is_current_extinguished.read();

    // Execute state machine update with pre-configured event lambdas
    _sfc_state_machine.update(_state_machine_inputs);

    // Get state machine outputs and apply safety interlocks
    sfc::sfc_state_outputs state_outputs = _sfc_state_machine.get_outputs();
    sfc::sfc_state_machine::apply_safety_interlocks(state_outputs, _state_machine_inputs);
    const sfc::sfc_state_id current_state = _sfc_state_machine.get_current_state();

    // Map state machine outputs directly to runner outputs
    _outputs.release_pulses = state_outputs.pulse_releases;
    _outputs.current_state_id = current_state;
    _outputs.has_transitioned = _sfc_state_machine.has_transitioned();
    _outputs.preset_request = state_outputs.freeze_integrator;
    _outputs.request_extinguish_dc_current = state_outputs.request_extinguish_dc_current;

    // Physical output commands (already include safety interlocks from state machine)
    _outputs.auxiliaries_on_command = state_outputs.auxiliaries_on_command;
    _outputs.mv_breaker_close = state_outputs.mv_breaker_close;
    _outputs.mv_breaker_open = state_outputs.mv_breaker_open;

    // Protection coordination - separate grid and machine side release signals
    _outputs.release_grid_protections = state_outputs.release_grid_protections;
    _outputs.release_machine_protections = state_outputs.release_machine_protections;
}

void sfc_state_machine_runner::stop()
{
    // Reset state machine to safe initial state
    _sfc_state_machine.reset();
}

void sfc_state_machine_runner::set_parameters()
{
    // Currently no configurable parameters for the state machine
    // This method is reserved for future parameter configurations
}

void sfc_state_machine_runner::set_parameter_callback()
{
    // Currently no parameters to register callbacks for
    // This method is reserved for future parameter callback registrations
}

}  // namespace yeet::application::sfc::integration
