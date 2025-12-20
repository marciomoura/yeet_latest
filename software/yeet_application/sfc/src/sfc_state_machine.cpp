#include "sfc/sfc_state_machine.hpp"

namespace yeet::application::sfc {

sfc_state_machine::sfc_state_machine() noexcept
    : _idle_state(&_auxiliaries_on_state),
      _auxiliaries_on_state(&_breakers_off_state, &_idle_state),
      _breakers_off_state(&_idle_state, &_breakers_on_state, &_auxiliaries_on_state),
      _breakers_on_state(&_operation_off_state, &_breakers_off_state),
      _operation_off_state(&_operation_on_state, &_breakers_on_state),
      _operation_on_state(&_shutdown_requested_state),
      _shutdown_requested_state(&_operation_off_state, &_idle_state),  // Added idle_state for trip path
      _state_machine(&_idle_state)                                     // Initialize state machine LAST
{
    initialize_transitions();
}

void sfc_state_machine::update(const sfc_state_inputs& inputs) noexcept { _state_machine.update(inputs); }

void sfc_state_machine::reset() noexcept
{
    _state_machine.reset(&_idle_state);
    _shutdown_requested_state.reset();  // Clear acknowledgment flag
}

sfc_state_outputs sfc_state_machine::get_outputs() const noexcept { return _state_machine.get_outputs(); }

void sfc_state_machine::apply_safety_interlocks(sfc_state_outputs& outputs, const sfc_state_inputs& inputs) noexcept
{
    // Safety interlock: Block MV breaker close if:
    // - Any fault is active (emergency off, breaker tripping, protection faults)
    // - Inhibited from starting (door open, protection inhibit)
    // - Protection trip is active
    // - Shutdown is requested
    if (inputs.any_fault_active_event.is_true() || inputs.is_inhibited_from_starting_event.is_true() ||
        inputs.protection_trip_active_event.is_true() || inputs.shutdown_request.is_true()) {
        outputs.mv_breaker_close = false;
    }

    // Safety interlock: Force MV breaker open on critical faults
    if (inputs.any_fault_active_event.is_true() || inputs.protection_trip_active_event.is_true()) {
        outputs.mv_breaker_open = true;
    }

    // Safety interlock: De-energize auxiliaries on any trip or critical shutdown
    if (inputs.protection_trip_active_event.is_true() || inputs.any_fault_active_event.is_true() ||
        inputs.is_inhibited_from_starting_event.is_true() || inputs.shutdown_request.is_true()) {
        outputs.auxiliaries_on_command = false;
    }
}

sfc_state_id sfc_state_machine::get_current_state() const noexcept { return _state_machine.get_current_state(); }

bool sfc_state_machine::has_transitioned() const noexcept { return _state_machine.has_transitioned(); }

void sfc_state_machine::initialize_transitions() noexcept
{
    // Initialize state transition graph if needed
    // This method can be used for any additional setup
}

// IDLE State

sfc_state_type* idle_state::update(const sfc_state_inputs& inputs) noexcept
{
    // If trip latch is active, require turn_on to be released first
    // This ensures operator acknowledges the trip before restart
    if (_trip_latch) {
        if (!inputs.turn_on_command.is_true()) {
            _trip_latch = false;  // Clear latch when turn_on is released
        }
        return this;  // Stay in IDLE until latch is cleared
    }

    // Normal transition to AUXILIARIES_ON when operator requests system energization
    if (inputs.turn_on_command.is_true()) {
        return _auxiliaries_on_state;
    }

    return this;
}

sfc_state_outputs idle_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};
    // IDLE: All systems de-energized, explicitly set ALL outputs
    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;
    outputs.request_extinguish_dc_current = false;
    outputs.reset_controllers = false;
    outputs.auxiliaries_on_command = false;
    outputs.mv_breaker_close = false;
    outputs.mv_breaker_open = true;
    outputs.release_grid_protections = false;
    outputs.release_machine_protections = false;
    return outputs;
}

// AUXILIARIES_ON State

sfc_state_type* auxiliaries_on_state::update(const sfc_state_inputs& inputs) noexcept
{
    // If turn_on_command goes false, return to IDLE
    if (!inputs.turn_on_command.is_true()) {
        return _idle_state;
    }

    // Wait for auxiliaries feedback confirmation before advancing to BREAKERS_OFF
    if (inputs.auxiliaries_are_on_event.is_true()) {
        return _breakers_off_state;
    }

    // Stay in AUXILIARIES_ON while waiting for aux feedback
    return this;
}

sfc_state_outputs auxiliaries_on_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;
    outputs.request_extinguish_dc_current = false;
    outputs.reset_controllers = false;
    outputs.auxiliaries_on_command = true;
    outputs.mv_breaker_close = false;
    outputs.mv_breaker_open = true;
    outputs.release_grid_protections = false;
    outputs.release_machine_protections = false;

    return outputs;
}

// BREAKERS_OFF State

sfc_state_type* breakers_off_state::update(const sfc_state_inputs& inputs) noexcept
{
    // If turn_on_command goes false, return to IDLE
    if (!inputs.turn_on_command.is_true()) {
        return _idle_state;
    }

    // If auxiliaries feedback drops, go back to AUXILIARIES_ON to re-energize
    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _auxiliaries_on_state;
    }

    // If fault is active or inhibited, stay in BREAKERS_OFF (don't close breaker)
    // The mv_breaker_close output will be blocked by safety interlocks
    // Wait here until fault/inhibit clears
    if (inputs.any_fault_active_event.is_true() || inputs.is_inhibited_from_starting_event.is_true()) {
        return this;
    }

    // Transition to BREAKERS_ON once breakers are confirmed closed
    if (inputs.mcb_is_closed_event.is_true()) {
        return _breakers_on_state;
    }

    // Stay in BREAKERS_OFF while waiting for breaker closure
    return this;
}

sfc_state_outputs breakers_off_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;
    outputs.request_extinguish_dc_current = false;
    outputs.reset_controllers = false;
    outputs.auxiliaries_on_command = true;
    outputs.mv_breaker_close = true;  // Request breaker close
    outputs.mv_breaker_open = false;
    outputs.release_grid_protections = false;
    outputs.release_machine_protections = false;

    return outputs;
}

// BREAKERS_ON State

sfc_state_type* breakers_on_state::update(const sfc_state_inputs& inputs) noexcept
{
    // If turn_on_command goes false, open breaker and return to BREAKERS_OFF
    if (!inputs.turn_on_command.is_true()) {
        return _breakers_off_state;
    }

    // If auxiliaries feedback drops, open breaker and return to BREAKERS_OFF
    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _breakers_off_state;
    }

    // If breaker opens unexpectedly, return to BREAKERS_OFF
    if (!inputs.mcb_is_closed_event.is_true()) {
        return _breakers_off_state;
    }

    // If fault is active or inhibited, open breaker and return to BREAKERS_OFF
    // This ensures we don't stay with closed breaker during unsafe conditions
    if (inputs.any_fault_active_event.is_true() || inputs.is_inhibited_from_starting_event.is_true()) {
        return _breakers_off_state;
    }

    // All conditions OK - advance to OPERATION_OFF (standby, waiting for start command)
    return _operation_off_state;
}

sfc_state_outputs breakers_on_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;
    outputs.request_extinguish_dc_current = false;
    outputs.reset_controllers = false;
    outputs.auxiliaries_on_command = true;
    outputs.mv_breaker_close = true;  // Keep breaker closed
    outputs.mv_breaker_open = false;
    outputs.release_grid_protections = true;
    outputs.release_machine_protections = false;

    return outputs;
}

// OPERATION_OFF State

sfc_state_type* operation_off_state::update(const sfc_state_inputs& inputs) noexcept
{
    // If turn_on_command goes false, regress to BREAKERS_ON (will then go to BREAKERS_OFF)
    if (!inputs.turn_on_command.is_true()) {
        return _breakers_on_state;
    }

    // If breaker opens, regress to BREAKERS_ON
    if (!inputs.mcb_is_closed_event.is_true()) {
        return _breakers_on_state;
    }

    // If auxiliaries feedback drops, regress to BREAKERS_ON
    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _breakers_on_state;
    }

    // If fault is active, stay in OPERATION_OFF (block start, but don't regress yet)
    // The safety interlocks will handle breaker opening if needed
    if (inputs.any_fault_active_event.is_true()) {
        return this;
    }

    // If stop command is received, remain in OPERATION_OFF
    if (inputs.stop_command.is_true()) {
        return this;
    }

    // Cannot transition to OPERATION_ON if inhibited from starting (door open, protection inhibit, etc.)
    if (inputs.is_inhibited_from_starting_event.is_true()) {
        return this;
    }

    // Transition to OPERATION_ON once start command is active
    if (inputs.start_command.is_true()) {
        return _operation_on_state;
    }

    // Stay in OPERATION_OFF waiting for start command
    return this;
}

sfc_state_outputs operation_off_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;
    outputs.request_extinguish_dc_current = false;
    outputs.reset_controllers = false;
    outputs.auxiliaries_on_command = true;
    outputs.mv_breaker_close = true;  // Keep breaker closed
    outputs.mv_breaker_open = false;
    outputs.release_grid_protections = true;
    outputs.release_machine_protections = false;

    return outputs;
}

// OPERATION_ON State

sfc_state_type* operation_on_state::update(const sfc_state_inputs& inputs) noexcept
{
    // Clear first cycle flag after first update
    if (_is_first_cycle) {
        _is_first_cycle = false;
    }

    // If turn_on_command goes false, trigger shutdown
    if (!inputs.turn_on_command.is_true()) {
        _is_first_cycle = true;  // Reset for next entry
        return _shutdown_requested_state;
    }

    // Any fault (emergency off, breaker tripping, etc.) triggers immediate shutdown
    if (inputs.any_fault_active_event.is_true()) {
        _is_first_cycle = true;  // Reset for next entry
        return _shutdown_requested_state;
    }

    if (!inputs.mcb_is_closed_event.is_true()) {
        _is_first_cycle = true;  // Reset for next entry
        return _shutdown_requested_state;
    }

    if (!inputs.auxiliaries_are_on_event.is_true()) {
        _is_first_cycle = true;  // Reset for next entry
        return _shutdown_requested_state;
    }

    if (inputs.stop_command.is_true()) {
        _is_first_cycle = true;  // Reset for next entry
        return _shutdown_requested_state;
    }

    if (inputs.shutdown_request.is_true()) {
        _is_first_cycle = true;  // Reset for next entry
        return _shutdown_requested_state;
    }

    if (!inputs.start_command.is_true()) {
        _is_first_cycle = true;  // Reset for next entry
        return _shutdown_requested_state;
    }

    return this;
}

sfc_state_outputs operation_on_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = true;  // Enable pulse generation in OPERATION_ON
    outputs.freeze_integrator = false;
    outputs.request_extinguish_dc_current = false;
    outputs.reset_controllers = _is_first_cycle;  // Pulse reset on state entry
    outputs.auxiliaries_on_command = true;
    outputs.mv_breaker_close = true;  // Keep breaker closed during operation
    outputs.mv_breaker_open = false;
    outputs.release_grid_protections = true;
    outputs.release_machine_protections = true;

    return outputs;
}

// SHUTDOWN_REQUESTED State

sfc_state_type* shutdown_requested_state::update(const sfc_state_inputs& inputs) noexcept
{
    // Wait for current to be extinguished before any transition
    if (!inputs.is_current_extinguished.is_true()) {
        return this;
    }

    // TRIP PATH: If protection trip is active, go directly to IDLE
    // This handles fault-initiated shutdowns (emergency off, protection trips, etc.)
    // The trip latch ensures operator must toggle turn_on before restart
    if (inputs.protection_trip_active_event.is_true() || inputs.any_fault_active_event.is_true()) {
        _acknowledgment_sent = false;       // Reset for next entry
        _idle_state->set_trip_latch(true);  // Require turn_on toggle before restart
        return _idle_state;
    }

    // NORMAL PATH: User-initiated shutdown (stop command or start inactive)
    // Go to OPERATION_OFF with acknowledgment handshake
    if (_acknowledgment_sent) {
        // Conditions were met last cycle and acknowledgment was sent - now safe to transition
        _acknowledgment_sent = false;  // Reset for next entry into SHUTDOWN_REQUESTED
        return _operation_off_state;
    }
    else {
        // First cycle with conditions met - set flag and stay in SHUTDOWN_REQUESTED
        _acknowledgment_sent = true;
    }

    return this;
}

sfc_state_outputs shutdown_requested_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = true;                 // Keep pulses active for controlled shutdown
    outputs.freeze_integrator = false;             // Do not freeze integrator during shutdown
    outputs.request_extinguish_dc_current = true;  // Request extinguish current mode
    outputs.reset_controllers = false;
    outputs.auxiliaries_on_command = false;
    outputs.mv_breaker_close = false;
    outputs.mv_breaker_open = true;
    outputs.release_grid_protections = true;
    outputs.release_machine_protections = false;

    return outputs;
}

}  // namespace yeet::application::sfc
