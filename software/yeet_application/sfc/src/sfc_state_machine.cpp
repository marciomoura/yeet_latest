#include "sfc/sfc_state_machine.hpp"

namespace yeet::application::sfc {

sfc_state_machine::sfc_state_machine() noexcept
    : _idle_state(&_auxiliaries_on_state),
      _auxiliaries_on_state(&_breakers_off_state, &_idle_state),
      _breakers_off_state(&_idle_state, &_breakers_on_state, &_auxiliaries_on_state),
      _breakers_on_state(&_operation_off_state, &_breakers_off_state),
      _operation_off_state(&_operation_on_state, &_breakers_on_state),
      _operation_on_state(&_shutdown_requested_state),
      _shutdown_requested_state(&_operation_off_state),
      _state_machine(&_idle_state)  // ✅ Initialize state machine LAST
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
    if (inputs.auxiliaries_are_on_event.is_true()) {
        return _auxiliaries_on_state;
    }

    return this;
}

sfc_state_outputs idle_state::get_outputs() const noexcept
{
    return sfc_state_outputs{};  // All outputs false/zero in IDLE
}

// AUXILIARIES_ON State

sfc_state_type* auxiliaries_on_state::update(const sfc_state_inputs& inputs) noexcept
{
    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _idle_state;
    }

    // Transition to BREAKERS_OFF once auxiliaries are stable
    return _breakers_off_state;
}

sfc_state_outputs auxiliaries_on_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;

    return outputs;
}

// BREAKERS_OFF State

sfc_state_type* breakers_off_state::update(const sfc_state_inputs& inputs) noexcept
{
    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _idle_state;
    }

    // Transition to BREAKERS_ON once breakers are closed
    if (inputs.mcb_is_closed_event.is_true()) {
        return _breakers_on_state;
    }

    return this;
}

sfc_state_outputs breakers_off_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;
    outputs.mcb_close = true;  // Request MCB close when in BREAKERS_OFF state

    return outputs;
}

// BREAKERS_ON State

sfc_state_type* breakers_on_state::update(const sfc_state_inputs& inputs) noexcept
{
    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _breakers_off_state;
    }

    if (!inputs.mcb_is_closed_event.is_true()) {
        return _breakers_off_state;
    }

    // Automatically transition to OPERATION_OFF once breakers are confirmed stable
    return _operation_off_state;
}

sfc_state_outputs breakers_on_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;

    return outputs;
}

// OPERATION_OFF State

sfc_state_type* operation_off_state::update(const sfc_state_inputs& inputs) noexcept
{
    if (!inputs.mcb_is_closed_event.is_true()) {
        return _breakers_on_state;
    }

    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _breakers_on_state;
    }

    // If stop command is received, remain in OPERATION_OFF
    if (inputs.stop_command.is_true()) {
        return this;
    }

    // Cannot transition to OPERATION_ON if protection system is inhibited (pending alarm confirmation)
    if (!inputs.protection_not_inhibited_event.is_true()) {
        return this;
    }

    // Transition to OPERATION_ON once start command is maintained
    if (inputs.start_command.is_true()) {
        return _operation_on_state;
    }

    return this;
}

sfc_state_outputs operation_off_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;

    return outputs;
}

// OPERATION_ON State

sfc_state_type* operation_on_state::update(const sfc_state_inputs& inputs) noexcept
{
    // Clear first cycle flag after first update
    if (_is_first_cycle) {
        _is_first_cycle = false;
    }

    // Emergency off or breaker tripping triggers immediate shutdown
    if (inputs.emergency_off_event.is_true() || inputs.breaker_is_tripping_event.is_true()) {
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
    outputs.reset_controllers = _is_first_cycle;  // Pulse reset on state entry

    return outputs;
}

// SHUTDOWN_REQUESTED State

sfc_state_type* shutdown_requested_state::update(const sfc_state_inputs& inputs) noexcept
{
    // Conditions for transition to OPERATION_OFF:
    // 1. Current is extinguished
    // 2. No active protection faults
    // 3. Acknowledgment has been sent (ensures protection module sees the acknowledgment for at least one cycle)

    bool conditions_met =
        inputs.is_current_extinguished.is_true() && !inputs.any_protection_fault_active_event.is_true();

    if (conditions_met) {
        if (_acknowledgment_sent) {
            // Conditions were met last cycle and acknowledgment was sent - now safe to transition
            _acknowledgment_sent = false;  // Reset for next entry into SHUTDOWN_REQUESTED
            return _operation_off_state;
        }
        else {
            // First cycle with conditions met - set flag and stay in SHUTDOWN_REQUESTED
            // This holds trip_acknowledged=true for one cycle, allowing protection to see it
            _acknowledgment_sent = true;
        }
    }
    else {
        // Conditions not met - reset flag
        _acknowledgment_sent = false;
    }

    return this;
}

sfc_state_outputs shutdown_requested_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = true;                 // Keep pulses active for controlled shutdown
    outputs.freeze_integrator = false;             // Do not freeze integrator during shutdown
    outputs.request_extinguish_dc_current = true;  // Request extinguish current mode
    outputs.mcb_open = true;                       // Always open MCB during shutdown/trip for safety

    return outputs;
}

}  // namespace yeet::application::sfc
