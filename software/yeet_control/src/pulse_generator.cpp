#include "yeet_control/pulse_generator.hpp"

namespace yeet::control {

pulse_generator::pulse_generator(angle_t firing_offset, angle_t pulse_width)
    : _firing_offset(firing_offset), _pulse_width(pulse_width) {}

bool pulse_generator::update(angle_t reference_angle, angle_t firing_delay, bool enable) {
    if (!enable) {
        return false;  // Pulse is off when disabled
    }

    // The pulse should be active when the reference_angle falls within the pulse's
    // angular window, accounting for the firing delay.
    //
    // Two equivalent ways to check this:
    // 1. Shift the reference: (reference_angle - firing_delay) ∈ [firing_offset, firing_offset + pulse_width)
    // 2. Shift the window: reference_angle ∈ [firing_offset + firing_delay, firing_offset + firing_delay + pulse_width)
    //
    // Note: Positive firing_delay delays the pulse pattern (pulses occur later in the cycle)
    angle_t adjusted_firing_start = _firing_offset + firing_delay;

    return is_in_range(reference_angle, adjusted_firing_start, _pulse_width);
}

void pulse_generator::set_firing_offset(angle_t offset) { _firing_offset = offset; }

void pulse_generator::set_pulse_width(angle_t width) { _pulse_width = width; }

pulse_generator::angle_t pulse_generator::get_firing_offset() const { return _firing_offset; }

pulse_generator::angle_t pulse_generator::get_pulse_width() const { return _pulse_width; }

bool pulse_generator::is_in_range(angle_t angle_to_check, angle_t range_start, angle_t range_width) {
    // Calculate the relative angle by subtracting the range start from the angle to check
    // The angle class automatically handles wrapping to [0, 2π)
    angle_t relative_angle = angle_to_check - range_start;

    return relative_angle < range_width;
}

}  // namespace yeet::control
