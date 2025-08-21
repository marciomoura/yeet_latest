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

void sfc_machine_side_modulator::configure_pulse_mode_firing_angle_rad(angle_t fixed_angle)
{
    // Accept any angle value - it will be normalized internally when used
    _pulse_mode_firing_angle_rad = fixed_angle;
}

void sfc_machine_side_modulator::update(bool enable,
    sfc_operating_mode_t operating_mode,
    bool force_commutation_acknowledged,
    angle_t absolute_rotor_position_rad,
    angle_t firing_angle_from_lut) noexcept
{
    // Store system enable state and operating mode
    _system_enabled = enable;
    _current_mode = operating_mode;

    // Update rotor sector tracking - this must happen regardless of enable state
    // to maintain continuity when system is re-enabled
    update_rotor_sector_tracking(absolute_rotor_position_rad);

    // If system is disabled, output safe defaults regardless of mode
    if (!enable) {
        _current_firing_angle_rad = get_safe_default_angle();
        _release_pulses = false;  // Block pulses when disabled
        _request_force_commutation = false;
        _commutation_state = machine_side_commutation_state_t::running;
        return;
    }

    // Select firing angle based on operating mode
    switch (_current_mode) {
        case sfc_operating_mode_t::load_commutated:
            // Natural commutation mode: pass through the firing angle from lookup table
            _current_firing_angle_rad = firing_angle_from_lut;
            _release_pulses = true;              // Always release pulses in natural commutation
            _request_force_commutation = false;  // Never request commutation
            _commutation_state = machine_side_commutation_state_t::running;
            break;

        case sfc_operating_mode_t::pulse_mode:
            // Forced commutation mode: calculate position-based firing angle and manage handshake
            _current_firing_angle_rad = calculate_pulse_mode_firing_angle(absolute_rotor_position_rad);
            update_forced_commutation_state_machine(force_commutation_acknowledged);
            break;
    }
}

void sfc_machine_side_modulator::reset()
{
    // Reset to safe initial state
    _current_mode = sfc_operating_mode_t::pulse_mode;
    _current_firing_angle_rad = get_safe_default_angle();
    _system_enabled = false;

    // Reset forced commutation handshake state
    _commutation_state = machine_side_commutation_state_t::running;
    _release_pulses = true;
    _request_force_commutation = false;

    // Reset sector tracking
    _previous_rotor_sector = -1;
    _rotor_sector_changed = false;
}

angle_t sfc_machine_side_modulator::get_firing_angle_rad() const noexcept { return _current_firing_angle_rad; }

sfc_operating_mode_t sfc_machine_side_modulator::get_current_operating_mode() const noexcept { return _current_mode; }

bool sfc_machine_side_modulator::get_release_pulses() const noexcept { return _release_pulses; }

bool sfc_machine_side_modulator::get_request_force_commutation() const noexcept { return _request_force_commutation; }

machine_side_commutation_state_t sfc_machine_side_modulator::get_commutation_state() const noexcept
{
    return _commutation_state;
}

angle_t sfc_machine_side_modulator::calculate_pulse_mode_firing_angle(angle_t rotor_position_rad) const noexcept
{
    // In pulse mode, we need to determine which thyristor pair should be fired
    // based on the rotor position to generate maximum torque

    // Normalize rotor position to [0, 2π] range
    angle_wrapped normalized_position = angle_wrapped(rotor_position_rad.value());

    // Divide the electrical rotation into 6 sectors (60° each)
    // Each sector corresponds to a different thyristor pair
    constexpr real_t sector_width_rad = yeet::two_pi / 6.0;

    // Determine which sector the rotor is in (0-5)
    const int current_sector = static_cast<int>(std::floor(normalized_position.get_radians() / sector_width_rad));

    // Calculate the base firing angle for this sector
    // The sectors are aligned with the natural commutation points
    const real_t sector_base_angle_rad = current_sector * sector_width_rad;

    // Add the configured fixed angle offset for pulse mode
    // This offset determines the timing relative to the optimal torque position
    const real_t total_firing_angle_rad = sector_base_angle_rad + _pulse_mode_firing_angle_rad.value();

    // Do NOT wrap the firing angle - it may legitimately exceed 2π
    // This is important for thyristor control systems where absolute timing matters
    return angle_t(total_firing_angle_rad);
}

void sfc_machine_side_modulator::update_rotor_sector_tracking(angle_t rotor_position_rad) noexcept
{
    // Calculate current sector
    const int current_sector = calculate_rotor_sector(rotor_position_rad);

    // Detect sector changes
    if (_previous_rotor_sector != -1 && current_sector != _previous_rotor_sector) {
        // Sector change detected - set the flag
        _rotor_sector_changed = true;
    }
    else {
        // No sector change - clear the flag
        _rotor_sector_changed = false;
    }

    // Update previous sector for next iteration
    _previous_rotor_sector = current_sector;
}

void sfc_machine_side_modulator::update_forced_commutation_state_machine(bool force_commutation_acknowledged) noexcept
{
    switch (_commutation_state) {
        case machine_side_commutation_state_t::running:
            // Normal operation: releasing pulses for current sector
            _release_pulses = true;
            _request_force_commutation = false;

            // Check for sector change to initiate commutation
            if (_rotor_sector_changed) {
                // Rotor moved to next sector - initiate forced commutation
                _request_force_commutation = true;  // Assert request to grid-side
                _release_pulses = false;            // Block own pulses immediately
                _commutation_state = machine_side_commutation_state_t::waiting_for_ack;
            }
            break;

        case machine_side_commutation_state_t::waiting_for_ack:
            // Waiting for grid-side to acknowledge commutation request
            _release_pulses = false;  // Keep pulses blocked
            // _request_force_commutation remains true until acknowledged

            // Check for acknowledgment from grid-side
            if (force_commutation_acknowledged) {
                // Grid-side has acknowledged - complete the handshake
                _release_pulses = true;              // Release pulses for new sector
                _request_force_commutation = false;  // Clear request signal
                _commutation_state = machine_side_commutation_state_t::running;
            }
            break;
    }
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

}  // namespace yeet::application::sfc