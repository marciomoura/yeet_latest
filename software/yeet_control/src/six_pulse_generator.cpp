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
    // Store the previous sector to detect a change
    const pulse_sector_t previous_sector = _current_sector;

    // Calculate the current sector based on the reference angle.
    const int sector_int = static_cast<int>(reference_angle.get_degrees() / 60.0) + 1;
    _current_sector = static_cast<pulse_sector_t>(sector_int);

    // Set the flag if the sector has changed in this update cycle
    _sector_has_changed = (_current_sector != previous_sector);

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

    return _pulses;
}

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

}  // namespace yeet::control
