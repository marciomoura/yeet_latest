#include "sfc/sfc_machine_side_modulator.hpp"

#include <cassert>
#include <cmath>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

sfc_machine_side_modulator::sfc_machine_side_modulator(duration_t sampling_time)
{
    // Validate sampling time (should be positive)
    assert(sampling_time.value() > 0.0 && "Sampling time must be positive");

    // Initialize to safe default state
    reset();
}

void sfc_machine_side_modulator::update(bool enable,
    sfc_operating_mode_t operating_mode,
    bool force_commutation_acknowledged,
    angle_t rotor_position_rad,
    angle_t firing_angle_rad) noexcept
{
    // Update rotor sector tracking - this must happen regardless of enable state
    // to maintain continuity when system is re-enabled
    const int current_sector = calculate_rotor_sector(rotor_position_rad);

    // If system is disabled, output safe defaults regardless of mode
    if (!enable) {
        _current_firing_angle_rad = get_safe_default_angle();
        _commutation_state = machine_side_commutation_state_t::running;
        _release_pulses = false;
        _request_force_commutation = false;
        // Update previous sector for next iteration
        _previous_rotor_sector = current_sector;
        return;
    }

    // Select firing angle based on operating mode
    switch (operating_mode) {
        case sfc_operating_mode_t::natural_commutation:
            // Natural commutation mode: pass through the machine firing angle
            _current_firing_angle_rad = firing_angle_rad;
            _release_pulses = true;              // Always release pulses in natural commutation
            _request_force_commutation = false;  // Never request commutation
            _commutation_state = machine_side_commutation_state_t::running;
            break;

        case sfc_operating_mode_t::forced_commutation:
            // Forced commutation mode: calculate position-based firing angle and manage handshake
            _current_firing_angle_rad = _forced_commutation_firing_angle_rad;

            // Compute the forced commutation outputs
            const forced_commutation_outputs outputs =
                compute_forced_commutation_outputs(force_commutation_acknowledged, rotor_position_rad);

            // Apply the computed outputs to the internal state
            _release_pulses = outputs.release_pulses;
            _request_force_commutation = outputs.request_force_commutation;
            _commutation_state = outputs.next_state;
            break;
    }

    // Update previous sector for next iteration
    _previous_rotor_sector = current_sector;
}

void sfc_machine_side_modulator::reset()
{
    // Reset to safe initial state
    _current_firing_angle_rad = get_safe_default_angle();

    // Reset forced commutation handshake state
    _commutation_state = machine_side_commutation_state_t::running;
    _release_pulses = true;
    _request_force_commutation = false;

    // Reset sector tracking - remove _rotor_sector_changed initialization
    _previous_rotor_sector = -1;
}

angle_t sfc_machine_side_modulator::get_firing_angle_rad() const noexcept { return _current_firing_angle_rad; }

bool sfc_machine_side_modulator::get_release_pulses() const noexcept { return _release_pulses; }

bool sfc_machine_side_modulator::get_request_force_commutation() const noexcept { return _request_force_commutation; }

machine_side_commutation_state_t sfc_machine_side_modulator::get_commutation_state() const noexcept
{
    return _commutation_state;
}

bool sfc_machine_side_modulator::has_rotor_sector_changed(angle_t rotor_position_rad) const noexcept
{
    // Calculate current sector
    const int current_sector = calculate_rotor_sector(rotor_position_rad);

    // Detect sector changes by comparing with previous sector
    // If previous sector is -1 (initial state), no change is detected
    if (_previous_rotor_sector != -1 && current_sector != _previous_rotor_sector) {
        return true;  // Sector change detected
    }

    return false;  // No sector change
}

int sfc_machine_side_modulator::calculate_rotor_sector(angle_t rotor_position_rad) const noexcept
{
    // Normalize rotor position to [0, 2π] range
    angle_wrapped normalized_position = angle_wrapped(rotor_position_rad.value());

    // Divide the electrical rotation into 6 sectors (60° each)
    constexpr real_t sector_width_rad = yeet::two_pi / 6.0;

    // Determine which sector the rotor is in (0-5)
    const int sector = static_cast<int>(std::floor(normalized_position.get_radians() / sector_width_rad));

    // Ensure sector is in valid range [0, 5] due to floating point precision
    return std::min(5, std::max(0, sector));
}

angle_t sfc_machine_side_modulator::get_safe_default_angle() const noexcept
{
    // Return a safe default firing angle when system is disabled
    // Zero degrees is typically safe as it corresponds to natural commutation point
    return angle_t(0.0);
}

forced_commutation_outputs sfc_machine_side_modulator::compute_forced_commutation_outputs(
    bool force_commutation_acknowledged, angle_t rotor_position_rad) const noexcept
{
    forced_commutation_outputs outputs{};

    switch (_commutation_state) {
        case machine_side_commutation_state_t::running:
            // Normal operation: releasing pulses for current sector
            outputs.release_pulses = true;
            outputs.request_force_commutation = false;
            outputs.next_state = machine_side_commutation_state_t::running;

            // Check for sector change to initiate commutation
            if (has_rotor_sector_changed(rotor_position_rad)) {
                // Rotor moved to next sector - initiate forced commutation
                outputs.request_force_commutation = true;  // Assert request to grid-side
                outputs.release_pulses = false;            // Block own pulses immediately
                outputs.next_state = machine_side_commutation_state_t::waiting_for_ack;
            }
            break;

        case machine_side_commutation_state_t::waiting_for_ack:
            // Waiting for grid-side to acknowledge commutation request
            outputs.release_pulses = false;            // Keep pulses blocked
            outputs.request_force_commutation = true;  // Keep force-commutation true until acknowledged
            outputs.next_state = machine_side_commutation_state_t::waiting_for_ack;

            // Check for acknowledgment from grid-side
            if (force_commutation_acknowledged) {
                // Grid-side has acknowledged - complete the handshake
                outputs.release_pulses = true;              // Release pulses for new sector
                outputs.request_force_commutation = false;  // Clear request signal
                outputs.next_state = machine_side_commutation_state_t::running;
            }
            break;
    }

    return outputs;
}

}  // namespace yeet::application::sfc