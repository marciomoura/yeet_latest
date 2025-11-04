#include "sfc/sfc_machine_side_integrator_supervisor.hpp"

namespace yeet::application::sfc {

sfc_machine_side_integrator_supervisor::sfc_machine_side_integrator_supervisor(duration_t sampling_time)
    : _sampling_time(sampling_time)
{
}

void sfc_machine_side_integrator_supervisor::reset()
{
    _is_frozen = false;
    _is_tripped = false;
    _preset_request = false;
    _preset_done = false;
    _previous_freeze_requested = false;
}

void sfc_machine_side_integrator_supervisor::update(bool grid_side_freeze,
    bool emergency_stop,
    bool torque_saturated,
    bool enable_speed_controller,
    bool speed_preset_complete,
    bool enable_torque_controller,
    bool torque_preset_complete,
    bool override_freeze)
{
    // Determine if freeze is being requested (OR all sources)
    // Note: Anti-windup freeze from torque_saturated is always enabled in the logic
    bool freeze_requested = grid_side_freeze || emergency_stop || torque_saturated;

    // Apply override for testing
    if (override_freeze) {
        freeze_requested = false;
    }

    // Handle trip latching on emergency_stop
    if (emergency_stop) {
        _is_tripped = true;
    }

    // State machine: manage freeze state transitions
    if (freeze_requested && !_is_frozen) {
        // Entering freeze state
        _is_frozen = true;
    }
    else if (!freeze_requested && _is_frozen) {
        // Exiting freeze state - initiate preset sequence
        _is_frozen = false;
        _preset_request = true;
        _preset_done = false;
    }

    // Preset handshake: either speed or torque controller signals completion
    // We preset both controllers together, so we check if either has completed
    bool any_preset_complete = speed_preset_complete || torque_preset_complete;

    if (_preset_request && any_preset_complete) {
        _preset_request = false;
        _preset_done = true;
    }

    // Reset preset_done flag when preset_request is not active (for next cycle)
    if (!_preset_request && _preset_done) {
        _preset_done = false;
    }

    _previous_freeze_requested = freeze_requested;
}

bool sfc_machine_side_integrator_supervisor::get_freeze_integrator() const noexcept { return _is_frozen; }

bool sfc_machine_side_integrator_supervisor::get_preset_request() const noexcept { return _preset_request; }

bool sfc_machine_side_integrator_supervisor::get_release_pulses() const noexcept
{
    return !_is_frozen && !_is_tripped && !_preset_request;
}

bool sfc_machine_side_integrator_supervisor::get_is_tripped() const noexcept { return _is_tripped; }

}  // namespace yeet::application::sfc
