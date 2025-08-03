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
    angle_t absolute_rotor_position_rad,
    angle_t firing_angle_from_lut) noexcept
{
    // Store system enable state and operating mode
    _system_enabled = enable;
    _current_mode = operating_mode;

    // Update rotor sector tracking - this must happen regardless of enable state
    // to maintain continuity when system is re-enabled
    update_rotor_sector_tracking(absolute_rotor_position_rad);

    // If system is disabled, output safe default angle regardless of mode
    if (!enable) {
        _current_firing_angle_rad = get_safe_default_angle();
        return;
    }

    // Select firing angle based on operating mode
    switch (_current_mode) {
        case sfc_operating_mode_t::load_commutated:
            // Pass through the firing angle from lookup table
            _current_firing_angle_rad = firing_angle_from_lut;
            break;

        case sfc_operating_mode_t::pulse_mode:
            // Calculate position-based firing angle for maximum torque
            _current_firing_angle_rad = calculate_pulse_mode_firing_angle(absolute_rotor_position_rad);
            break;
    }
}

void sfc_machine_side_modulator::reset()
{
    // Reset to safe initial state
    _current_mode = sfc_operating_mode_t::pulse_mode;
    _current_firing_angle_rad = get_safe_default_angle();
    _system_enabled = false;

    // Reset sector tracking
    _previous_rotor_sector = -1;
    _rotor_sector_change_request = false;
}

angle_t sfc_machine_side_modulator::get_firing_angle_rad() const noexcept { return _current_firing_angle_rad; }

sfc_operating_mode_t sfc_machine_side_modulator::get_current_operating_mode() const noexcept { return _current_mode; }

bool sfc_machine_side_modulator::get_rotor_sector_change_request() const noexcept
{
    return _rotor_sector_change_request;
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

    // Wrap the result to [0, 2π] and return as generic angle_t
    const angle_wrapped wrapped_angle(total_firing_angle_rad);
    return angle_t(wrapped_angle.get_radians());
}

void sfc_machine_side_modulator::update_rotor_sector_tracking(angle_t rotor_position_rad) noexcept
{
    // Calculate current sector
    const int current_sector = calculate_rotor_sector(rotor_position_rad);

    // Detect sector changes
    if (_previous_rotor_sector != -1 && current_sector != _previous_rotor_sector) {
        // Sector change detected - set the request flag
        _rotor_sector_change_request = true;
    }
    else {
        // No sector change - clear the request flag
        _rotor_sector_change_request = false;
    }

    // Update previous sector for next iteration
    _previous_rotor_sector = current_sector;
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