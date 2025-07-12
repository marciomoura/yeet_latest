#include "yeet_control/six_pulse_generator.hpp"

namespace yeet::control {

six_pulse_generator::six_pulse_generator(angle_t pulse_width) {
    // Initialize all six pulse generators with their respective firing offsets
    for (size_t i = 0; i < 6; ++i) {
        angle_t offset = angle_t(yeet::control::deg_to_rad(firing_offsets_deg[i]));
        _pulse_generators[i] = pulse_generator(offset, pulse_width);
    }
}

six_pulse_generator::pulses six_pulse_generator::update(angle_t reference_angle, angle_t firing_angle, bool enable) {
    pulses p;

    // Update each pulse generator and collect the results
    p.p1 = _pulse_generators[0].update(reference_angle, firing_angle, enable);
    p.p2 = _pulse_generators[1].update(reference_angle, firing_angle, enable);
    p.p3 = _pulse_generators[2].update(reference_angle, firing_angle, enable);
    p.p4 = _pulse_generators[3].update(reference_angle, firing_angle, enable);
    p.p5 = _pulse_generators[4].update(reference_angle, firing_angle, enable);
    p.p6 = _pulse_generators[5].update(reference_angle, firing_angle, enable);

    return p;
}

void six_pulse_generator::set_pulse_width(angle_t pulse_width) {
    // Update pulse width for all six generators
    for (auto& generator : _pulse_generators) {
        generator.set_pulse_width(pulse_width);
    }
}

six_pulse_generator::angle_t six_pulse_generator::get_pulse_width() const {
    // Return the pulse width from the first generator (they should all be the same)
    return _pulse_generators[0].get_pulse_width();
}

}  // namespace yeet::control
