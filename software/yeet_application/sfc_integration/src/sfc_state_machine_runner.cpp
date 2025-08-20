#include "sfc_integration/sfc_state_machine_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_state_machine_runner::initialize()
{
    set_parameters();
    set_parameter_callback();

    // Reset state machine to initial IDLE state
    _sfc_state_machine.reset();
}

void sfc_state_machine_runner::run()
{
    // Execute state machine update with pre-configured event lambdas
    _sfc_state_machine.update(_state_machine_inputs);

    // Map state machine outputs to runner outputs (inlined)
    sfc::sfc_state_outputs state_outputs = _sfc_state_machine.get_outputs();
    sfc::sfc_state_id current_state = _sfc_state_machine.get_current_state();

    // Primary output signals
    _outputs.pulse_releases = state_outputs.pulse_releases;
    _outputs.current_state_id = current_state;
    _outputs.has_transitioned = _sfc_state_machine.has_transitioned();
    _outputs.freeze_integrator = state_outputs.freeze_integrator;

    // Individual state status flags for detailed monitoring
    _outputs.is_idle = (current_state == sfc::sfc_state_id::IDLE);
    _outputs.is_auxiliaries_on = (current_state == sfc::sfc_state_id::AUXILIARIES_ON);
    _outputs.is_breakers_off = (current_state == sfc::sfc_state_id::BREAKERS_OFF);
    _outputs.is_breakers_on = (current_state == sfc::sfc_state_id::BREAKERS_ON);
    _outputs.is_operation_off = (current_state == sfc::sfc_state_id::OPERATION_OFF);
    _outputs.is_operation_on = (current_state == sfc::sfc_state_id::OPERATION_ON);
    _outputs.is_shutdown_requested = (current_state == sfc::sfc_state_id::SHUTDOWN_REQUESTED);
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