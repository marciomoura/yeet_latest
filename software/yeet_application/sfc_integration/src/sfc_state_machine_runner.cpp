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
    // First, populate signal outputs with combined input + override logic
    // These signal outputs provide visibility into what values are actually being used
    _outputs.auxiliaries_are_on = _parameters.override_enable_auxiliaries_are_on.read()
                                      ? _parameters.override_value_auxiliaries_are_on.read()
                                      : _inputs.auxiliaries_are_on.read();

    _outputs.breakers_are_on = _parameters.override_enable_breakers_are_on.read()
                                   ? _parameters.override_value_breakers_are_on.read()
                                   : _inputs.breakers_are_on.read();

    _outputs.start_command = _parameters.override_enable_start_command.read()
                                 ? _parameters.override_value_start_command.read()
                                 : _inputs.start_command.read();

    _outputs.stop_command = _parameters.override_enable_stop_command.read()
                                ? _parameters.override_value_stop_command.read()
                                : _inputs.stop_command.read();

    _outputs.shutdown_request = _parameters.override_enable_shutdown_request.read()
                                    ? _parameters.override_value_shutdown_request.read()
                                    : _inputs.shutdown_request.read();

    _outputs.is_current_extinguished = _parameters.override_enable_is_current_extinguished.read()
                                           ? _parameters.override_value_is_current_extinguished.read()
                                           : _inputs.is_current_extinguished.read();

    // Execute state machine update with pre-configured event lambdas
    _sfc_state_machine.update(_state_machine_inputs);

    // Map state machine outputs to runner outputs (inlined)
    const sfc::sfc_state_outputs state_outputs = _sfc_state_machine.get_outputs();
    const sfc::sfc_state_id current_state = _sfc_state_machine.get_current_state();

    // Primary output signals
    _outputs.release_pulses = state_outputs.pulse_releases;
    _outputs.current_state_id = current_state;
    _outputs.has_transitioned = _sfc_state_machine.has_transitioned();
    _outputs.preset_request = state_outputs.freeze_integrator;
    _outputs.request_extinguish_dc_current = state_outputs.request_extinguish_dc_current;

    // Release protections whenever state machine is not in IDLE
    _outputs.release_protections =
        (current_state == sfc::sfc_state_id::OPERATION_OFF || current_state == sfc::sfc_state_id::OPERATION_ON ||
            current_state == sfc::sfc_state_id::SHUTDOWN_REQUESTED);

    // Trip acknowledgment: Signal to protection that state machine has handled the trip
    // Conditions: In SHUTDOWN_REQUESTED state + current extinguished + no active faults
    // This allows protection to clear its latched trip signal and state machine to proceed to OPERATION_OFF
    _outputs.trip_acknowledged = (current_state == sfc::sfc_state_id::SHUTDOWN_REQUESTED) &&
                                 _outputs.is_current_extinguished && !_inputs.any_protection_fault_active;

    // Main circuit breaker control with door interlock
    // MCB close command: Only when state machine requests it AND door is NOT open
    _outputs.mcb_close_command = state_outputs.mcb_close_command && !_inputs.door_is_open.read();
    
    // MCB open command: When state machine requests it (shutdown/trip scenarios)
    _outputs.mcb_open_command = state_outputs.mcb_open_command;
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