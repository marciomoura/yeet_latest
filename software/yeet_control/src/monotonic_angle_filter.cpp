#include "yeet_control/monotonic_angle_filter.hpp"

namespace yeet::control {

void monotonic_angle_filter::update(angle_wrapped current_angle) noexcept
{
    if (_is_first_update) {
        _last_valid_angle = current_angle;
        _filtered_output = current_angle;
        _is_first_update = false;
        return;
    }

    // Calculate the shortest angular distance from last valid position to current.
    // In angle_wrapped math:
    // (current - last) < PI  => Forward movement
    // (current - last) > PI  => Backward movement (due to underflow represented as a large positive)
    angle_wrapped delta = current_angle - _last_valid_angle;

    if (delta.get_radians() <= pi) {
        // Normal forward movement (including small zero-crossings)
        _last_valid_angle = current_angle;
        _filtered_output = current_angle;
    }
    else {
        // Potential backward jump detected.
        // Check if the backward distance exceeds the "unstick" threshold.
        const real_t backward_dist = two_pi - delta.get_radians();

        if (backward_dist > _max_backward_step_rad) {
            // The step is too large to be considered noise/jitter (e.g., > 60 deg).
            // Unstick the filter and accept the new position.
            _last_valid_angle = current_angle;
            _filtered_output = current_angle;
        }
        else {
            // Small backward jitter: Keep the last known forward position (ratchet)
            _filtered_output = _last_valid_angle;
        }
    }
}

void monotonic_angle_filter::configure_max_backward_step(real_t max_step_rad) noexcept
{
    _max_backward_step_rad = max_step_rad;
}

void monotonic_angle_filter::reset() noexcept
{
    _is_first_update = true;
    _last_valid_angle = angle_wrapped(0.0f);
    _filtered_output = angle_wrapped(0.0f);
}

angle_wrapped monotonic_angle_filter::get_output() const noexcept { return _filtered_output; }

}  // namespace yeet::control
