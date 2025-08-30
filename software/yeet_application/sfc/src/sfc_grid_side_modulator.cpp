#include "sfc/sfc_grid_side_modulator.hpp"

#include <cassert>

namespace yeet::application::sfc {

sfc_grid_side_modulator::sfc_grid_side_modulator()
{
    // Initialize to safe default state
    reset();
}

void sfc_grid_side_modulator::update(bool enable,
    sfc_operating_mode_t operating_mode,
    bool request_force_commutation,
    bool dc_link_current_is_zero,
    angle_t firing_angle) noexcept
{
    if (!enable) {
        _current_state = grid_side_modulator_state_t::running;
        _current_firing_angle_rad = get_safe_default_angle();
        _force_commutation_acknowledged = false;
        _main_pi_reset_requested = false;
        _release_pulses = false;
        return;
    }

    // Grid-side always has its pulses released if the module is enabled
    _release_pulses = true;

    // Operating mode determines behavior - similar to machine-side modulator pattern
    switch (operating_mode) {
        case sfc_operating_mode_t::load_commutated:
            // Natural commutation mode: ignore force commutation requests, always pass through PI angle
            _current_state = grid_side_modulator_state_t::running;
            _current_firing_angle_rad = firing_angle;
            _force_commutation_acknowledged = false;
            _main_pi_reset_requested = false;
            break;

        case sfc_operating_mode_t::pulse_mode: {
            // Forced commutation mode: respond to force commutation requests
            const auto state_update =
                compute_pulse_mode_state_update(request_force_commutation, dc_link_current_is_zero, firing_angle);

            _current_state = state_update.new_state;
            _current_firing_angle_rad = state_update.new_firing_angle;
            _force_commutation_acknowledged = state_update.new_force_commutation_acknowledged;
            _main_pi_reset_requested = state_update.new_main_pi_reset_requested;
            break;
        }
    }
}
sfc_grid_side_modulator::pulse_mode_state_update sfc_grid_side_modulator::compute_pulse_mode_state_update(
    bool request_force_commutation, bool dc_link_current_is_zero, angle_t firing_angle) const noexcept
{
    sfc_grid_side_modulator::pulse_mode_state_update state_update{};

    switch (_current_state) {
        case grid_side_modulator_state_t::running:
            // If a force commutation is requested, switch to forced commutation state
            // which should bring the dc-link current down
            if (request_force_commutation) {
                // Applying the forced commutation angle immediately
                state_update.new_state = grid_side_modulator_state_t::forced_commutation;
                state_update.new_firing_angle = _force_commutation_angle_rad;
                state_update.new_force_commutation_acknowledged = false;
                state_update.new_main_pi_reset_requested = true;
            }
            else {
                state_update.new_state = grid_side_modulator_state_t::running;
                state_update.new_firing_angle = firing_angle;
                state_update.new_force_commutation_acknowledged = false;
                state_update.new_main_pi_reset_requested = false;
            }
            break;

        case grid_side_modulator_state_t::forced_commutation:
            // The condition to move back to running state is
            // the signal `dc_link_current_is_zero`
            if (dc_link_current_is_zero) {
                // In case dc-link is zero, the machine-side will clear the force-commutation request.
                // Hence, the grid-side will move back to running state and will clear the acknowledgment flag.
                if (!request_force_commutation) {
                    // Commutation is complete, return to normal operation
                    state_update.new_state = grid_side_modulator_state_t::running;
                    state_update.new_firing_angle = firing_angle;
                    state_update.new_force_commutation_acknowledged = false;
                    state_update.new_main_pi_reset_requested = false;
                }
                else {
                    state_update.new_state =
                        grid_side_modulator_state_t::forced_commutation;           ///< Forced commutation state
                    state_update.new_firing_angle = _force_commutation_angle_rad;  ///< Force commutation angle
                    state_update.new_force_commutation_acknowledged = true;        ///< Force commutation acknowledged
                    state_update.new_main_pi_reset_requested = true;               ///< Main PI reset requested
                }
            }
            else {
                // Continue applying the forced commutation angle
                state_update.new_state = grid_side_modulator_state_t::forced_commutation;  ///< Forced commutation state
                state_update.new_firing_angle = _force_commutation_angle_rad;              ///< Force commutation angle
                state_update.new_force_commutation_acknowledged = false;  ///< Force commutation acknowledged
                state_update.new_main_pi_reset_requested = true;          ///< Main PI reset requested
            }
            break;

        default:
            assert(false && "Unhandled state in sfc_grid_side_modulator");
            // Return safe default to satisfy compiler
            state_update.new_state = grid_side_modulator_state_t::running;
            state_update.new_firing_angle = get_safe_default_angle();
            state_update.new_force_commutation_acknowledged = false;
            state_update.new_main_pi_reset_requested = false;
            break;
    }

    return state_update;
}

void sfc_grid_side_modulator::reset()
{
    _current_state = grid_side_modulator_state_t::running;
    _current_firing_angle_rad = get_safe_default_angle();
    _force_commutation_acknowledged = false;
    _main_pi_reset_requested = false;
}

angle_t sfc_grid_side_modulator::get_firing_angle_rad() const noexcept { return _current_firing_angle_rad; }

grid_side_modulator_state_t sfc_grid_side_modulator::get_current_state() const noexcept { return _current_state; }

bool sfc_grid_side_modulator::get_force_commutation_acknowledged() const noexcept
{
    return _force_commutation_acknowledged;
}

bool sfc_grid_side_modulator::get_is_requesting_main_pi_reset() const noexcept { return _main_pi_reset_requested; }

void sfc_grid_side_modulator::acknowledge_main_pi_reset() { _main_pi_reset_requested = false; }

angle_t sfc_grid_side_modulator::get_safe_default_angle() const noexcept
{
    // Return 0° (natural commutation point) as the safe default
    return angle_t(0.0f);
}

bool sfc_grid_side_modulator::get_pulse_release() const noexcept { return _release_pulses; }

}  // namespace yeet::application::sfc
