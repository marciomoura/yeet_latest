#include "sfc/sfc_grid_side_integrator_supervisor.hpp"

namespace yeet::application::sfc {

sfc_grid_side_integrator_supervisor::sfc_grid_side_integrator_supervisor(duration_t sampling_time)
    : _sampling_time(sampling_time)
{
}

void sfc_grid_side_integrator_supervisor::reset()
{
    _is_frozen = false;
    _is_tripped = false;
    _preset_request = false;
    _preset_done = false;
    _previous_freeze_requested = false;
}

void sfc_grid_side_integrator_supervisor::update(bool modulator_reset_request,
    bool shutdown_request,
    bool enable_current_controller,
    bool preset_complete,
    bool override_freeze)
{
    // Determine if freeze is being requested (OR all sources)
    bool freeze_requested = modulator_reset_request || shutdown_request;

    // Apply override for testing
    if (override_freeze) {
        freeze_requested = false;
    }

    // Handle trip latching on shutdown_request
    if (shutdown_request) {
        _is_tripped = true;
    }

    // State machine: manage freeze state transitions
    if (freeze_requested && !_is_frozen) {
        // Entering freeze state
        _is_frozen = true;
        // Request preset sequence when unfreezing later
    }
    else if (!freeze_requested && _is_frozen) {
        // Exiting freeze state - initiate preset sequence
        _is_frozen = false;
        _preset_request = true;
        _preset_done = false;
    }

    // Preset handshake: controller signals completion
    if (_preset_request && preset_complete) {
        _preset_request = false;
        _preset_done = true;
    }

    // Reset preset_done flag when preset_request is not active (for next cycle)
    if (!_preset_request && _preset_done) {
        _preset_done = false;
    }

    _previous_freeze_requested = freeze_requested;
}

bool sfc_grid_side_integrator_supervisor::get_freeze_integrator() const noexcept { return _is_frozen; }

bool sfc_grid_side_integrator_supervisor::get_preset_request() const noexcept { return _preset_request; }

bool sfc_grid_side_integrator_supervisor::get_release_pulses() const noexcept
{
    return !_is_frozen && !_is_tripped && !_preset_request;
}

bool sfc_grid_side_integrator_supervisor::get_is_tripped() const noexcept { return _is_tripped; }

}  // namespace yeet::application::sfc
