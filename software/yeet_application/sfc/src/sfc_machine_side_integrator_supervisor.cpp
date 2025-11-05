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
    _grid_side_release_pulses = false;
    _current_is_zero = false;
    _previous_freeze_requested = false;
}

void sfc_machine_side_integrator_supervisor::update(bool grid_side_freeze_integrator,
    bool grid_side_is_tripped,
    bool grid_side_release_pulses,
    bool current_is_zero,
    bool speed_saturated,
    bool torque_saturated,
    bool override_freeze)
{
    // Determine if freeze is being requested (OR all sources)
    // Anti-windup freeze from speed/torque saturation is always enabled
    bool freeze_requested = grid_side_freeze_integrator || speed_saturated;  // || torque_saturated;

    // Apply override for testing
    if (override_freeze) {
        freeze_requested = false;
    }

    // Handle trip latching from grid-side
    if (grid_side_is_tripped) {
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

    // Preset handshake: controllers automatically complete preset internally
    // Keep preset_request active for one cycle, then auto-complete
    if (_preset_request && _preset_done) {
        // Preset was requested last cycle and marked done - clear now
        _preset_request = false;
        _preset_done = false;
    }
    else if (_preset_request && !_preset_done) {
        // First cycle with preset request - mark as done but keep request active
        _preset_done = true;
    }

    // Store grid-side commands for use in output logic
    _grid_side_release_pulses = grid_side_release_pulses;
    _current_is_zero = current_is_zero;

    _previous_freeze_requested = freeze_requested;
}

bool sfc_machine_side_integrator_supervisor::get_freeze_integrator() const noexcept { return _is_frozen; }

bool sfc_machine_side_integrator_supervisor::get_preset_request() const noexcept { return _preset_request; }

bool sfc_machine_side_integrator_supervisor::get_release_pulses() const noexcept { return _grid_side_release_pulses; }

bool sfc_machine_side_integrator_supervisor::get_is_tripped() const noexcept { return _is_tripped; }

}  // namespace yeet::application::sfc
