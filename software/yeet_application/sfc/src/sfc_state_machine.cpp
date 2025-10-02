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

void sfc_state_machine::reset() noexcept { _state_machine.reset(&_idle_state); }

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

    if (inputs.breakers_are_on_event.is_true()) {
        return _breakers_off_state;
    }

    return this;
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

    if (inputs.breakers_are_on_event.is_true()) {
        return _breakers_on_state;
    }

    return this;
}

sfc_state_outputs breakers_off_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = false;
    outputs.freeze_integrator = true;

    return outputs;
}

// BREAKERS_ON State

sfc_state_type* breakers_on_state::update(const sfc_state_inputs& inputs) noexcept
{
    if (!inputs.breakers_are_on_event.is_true()) {
        return _breakers_off_state;
    }

    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _breakers_off_state;
    }

    if (inputs.start_command.is_true()) {
        return _operation_off_state;
    }

    return this;
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
    if (!inputs.breakers_are_on_event.is_true()) {
        return _breakers_on_state;
    }

    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _breakers_on_state;
    }

    if (inputs.stop_command.is_true()) {
        return this;
    }

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
    if (!inputs.breakers_are_on_event.is_true()) {
        return _shutdown_requested_state;
    }

    if (!inputs.auxiliaries_are_on_event.is_true()) {
        return _shutdown_requested_state;
    }

    if (inputs.stop_command.is_true()) {
        return _shutdown_requested_state;
    }

    if (inputs.shutdown_request.is_true()) {
        return _shutdown_requested_state;
    }

    if (!inputs.start_command.is_true()) {
        return _shutdown_requested_state;
    }

    return this;
}

sfc_state_outputs operation_on_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = true;  // Enable pulse generation in OPERATION_ON
    outputs.freeze_integrator = false;

    return outputs;
}

// SHUTDOWN_REQUESTED State

sfc_state_type* shutdown_requested_state::update(const sfc_state_inputs& inputs) noexcept
{
    if (inputs.is_current_estinguished.is_true()) {
        return _operation_off_state;
    }

    return this;
}

sfc_state_outputs shutdown_requested_state::get_outputs() const noexcept
{
    sfc_state_outputs outputs{};

    outputs.pulse_releases = true;      // Keep pulses active for controlled shutdown
    outputs.freeze_integrator = false;  // Do not freeze integrator during shutdown

    return outputs;
}

}  // namespace yeet::application::sfc