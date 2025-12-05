#include "yeet_control/six_pulse_generator.hpp"

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::control {

six_pulse_generator::six_pulse_generator(reference_type ref_type, angle_t pulse_width) : _ref_type(ref_type)
{
    // Initialize all six pulse generators with their respective firing offsets
    for (size_t i = 0; i < 6; ++i) {
        angle_t offset = angle_t(yeet::deg_to_rad(firing_offsets_deg[i]));
        _pulse_generators[i] = pulse_generator(offset, pulse_width);
    }
}

six_pulse_generator::pulses six_pulse_generator::update(angle_t reference_angle, angle_t firing_angle, bool enable)
{
    update_sector(reference_angle);
    return generate_pulses(reference_angle, firing_angle, enable);
}

six_pulse_generator::pulses six_pulse_generator::update(angle_t firing_angle, bool enable)
{
    if (!_virtual_reference_enabled) {
        // Return all-off pulses if virtual reference is not enabled
        _pulses = pulses{};
        return _pulses;
    }

    // Update the virtual reference angle
    update_virtual_reference();

    // Use the internal virtual reference angle
    update_sector(_virtual_reference_angle);
    return generate_pulses(_virtual_reference_angle, firing_angle, enable);
}

six_pulse_generator::pulses six_pulse_generator::generate_pulses(
    angle_t reference_angle, angle_t firing_angle, bool enable)
{
    angle_t final_firing_angle = firing_angle;

    // If the reference angle is from a phase voltage, a 30-degree phase shift
    // is required to align the firing angle with the line-to-line commutation point.
    if (_ref_type == reference_type::phase) {
        static const angle_wrapped phase_reference_offset(deg_to_rad(30.0));
        final_firing_angle += phase_reference_offset;
    }

    // Update each pulse generator and collect the results
    _pulses.p1 = _pulse_generators[0].update(reference_angle, final_firing_angle, enable);
    _pulses.p2 = _pulse_generators[1].update(reference_angle, final_firing_angle, enable);
    _pulses.p3 = _pulse_generators[2].update(reference_angle, final_firing_angle, enable);
    _pulses.p4 = _pulse_generators[3].update(reference_angle, final_firing_angle, enable);
    _pulses.p5 = _pulse_generators[4].update(reference_angle, final_firing_angle, enable);
    _pulses.p6 = _pulse_generators[5].update(reference_angle, final_firing_angle, enable);

    // Apply pulse mask if enabled
    if (_pulse_mask_enabled) {
        _pulses.p1 = _pulses.p1 && _pulse_mask.p1;
        _pulses.p2 = _pulses.p2 && _pulse_mask.p2;
        _pulses.p3 = _pulses.p3 && _pulse_mask.p3;
        _pulses.p4 = _pulses.p4 && _pulse_mask.p4;
        _pulses.p5 = _pulses.p5 && _pulse_mask.p5;
        _pulses.p6 = _pulses.p6 && _pulse_mask.p6;
    }

    return _pulses;
}

void six_pulse_generator::update_sector(angle_t reference_angle)
{
    // Store the previous sector to detect a change
    const pulse_sector_t previous_sector = _current_sector;

    // Calculate the current sector based on the reference angle.
    const int sector_int = static_cast<int>(reference_angle.get_degrees() / 60.0) + 1;
    _current_sector = static_cast<pulse_sector_t>(sector_int);

    // Set the flag if the sector has changed in this update cycle
    _sector_has_changed = (_current_sector != previous_sector);
}

void six_pulse_generator::update_virtual_reference()
{
    // Integrate angle: angle += 2π × frequency × dt
    // angle_wrapped automatically handles wrapping at 2π
    const angle_t angular_increment = angle_t(two_pi * _virtual_reference_frequency * _virtual_reference_sampling_time);
    _virtual_reference_angle += angular_increment;
}

void six_pulse_generator::configure_virtual_reference(frequency_t frequency, duration_t sampling_time)
{
    _virtual_reference_frequency = frequency;
    _virtual_reference_sampling_time = sampling_time;
}

void six_pulse_generator::enable_virtual_reference(bool enable) { _virtual_reference_enabled = enable; }

void six_pulse_generator::reset_virtual_reference() { _virtual_reference_angle = angle_t{0.0}; }

void six_pulse_generator::reset_virtual_reference(angle_t initial_angle) { _virtual_reference_angle = initial_angle; }

void six_pulse_generator::configure_pulse_mask(const pulse_mask& mask) { _pulse_mask = mask; }

void six_pulse_generator::enable_pulse_mask(bool enable) { _pulse_mask_enabled = enable; }

void six_pulse_generator::set_pulse_width(angle_t pulse_width)
{
    // Update pulse width for all six generators
    for (auto& generator : _pulse_generators) {
        generator.set_pulse_width(pulse_width);
    }
}

six_pulse_generator::angle_t six_pulse_generator::get_pulse_width() const
{
    // Return the pulse width from the first generator (they should all be the same)
    return _pulse_generators[0].get_pulse_width();
}

six_pulse_generator::pulses six_pulse_generator::get_pulses() const { return _pulses; }

six_pulse_generator::pulse_sector_t six_pulse_generator::get_sector() const noexcept { return _current_sector; }

bool six_pulse_generator::get_sector_changed() const noexcept { return _sector_has_changed; }

six_pulse_generator::angle_t six_pulse_generator::get_virtual_reference_angle() const noexcept
{
    return _virtual_reference_angle;
}

bool six_pulse_generator::is_virtual_reference_enabled() const noexcept { return _virtual_reference_enabled; }

frequency_t six_pulse_generator::get_virtual_reference_frequency() const noexcept
{
    return _virtual_reference_frequency;
}

six_pulse_generator::pulse_mask six_pulse_generator::get_pulse_mask() const noexcept { return _pulse_mask; }

bool six_pulse_generator::is_pulse_mask_enabled() const noexcept { return _pulse_mask_enabled; }

}  // namespace yeet::control
