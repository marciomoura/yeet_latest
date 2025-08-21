#include "sfc/sfc_grid_side_modulator.hpp"

#include <cassert>

namespace yeet::application::sfc {

sfc_grid_side_modulator::sfc_grid_side_modulator()
{
    // Initialize to safe default state
    reset();
}

void sfc_grid_side_modulator::configure_force_commutation_angle_rad(angle_t angle)
{
    _force_commutation_angle_rad = angle;
}

void sfc_grid_side_modulator::update(
    bool request_force_commutation, bool dc_link_current_is_zero, angle_t firing_angle_from_main_pi) noexcept
{
    // Clear the one-cycle acknowledgment flag
    _force_commutation_acknowledged = false;

    switch (_current_state) {
        case grid_side_modulator_state_t::natural_commutation:
            // In normal operation, pass through the firing angle from main PI controller
            _current_firing_angle_rad = firing_angle_from_main_pi;

            // Check if forced commutation is requested
            if (request_force_commutation) {
                if (dc_link_current_is_zero) {
                    // Edge case: request and zero current arrive simultaneously
                    // Immediately complete the handshake without transitioning
                    _force_commutation_acknowledged = true;
                    // Stay in normal operation, no PI reset needed
                }
                else {
                    // Normal case: transition to force inversion and request PI reset
                    _main_pi_reset_requested = true;
                    _current_state = grid_side_modulator_state_t::forced_commutation;
                    _current_firing_angle_rad = _force_commutation_angle_rad;
                }
            }
            break;

        case grid_side_modulator_state_t::forced_commutation:
            // Continue applying the forced commutation angle
            _current_firing_angle_rad = _force_commutation_angle_rad;

            // Check if DC link current has reached zero
            if (dc_link_current_is_zero) {
                // Commutation is complete, return to normal operation
                _current_state = grid_side_modulator_state_t::natural_commutation;
                _current_firing_angle_rad = firing_angle_from_main_pi;

                // Signal acknowledgment for one cycle and clear PI reset
                _force_commutation_acknowledged = true;
                _main_pi_reset_requested = false;
            }
            break;
    }
}

void sfc_grid_side_modulator::reset()
{
    _current_state = grid_side_modulator_state_t::natural_commutation;
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

}  // namespace yeet::application::sfc
