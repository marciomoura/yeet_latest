#include "yeet_control/boolean_debouncer.hpp"

#include <algorithm>

namespace yeet::control {

void boolean_debouncer::configure_sampling_time(duration_t sampling_time) noexcept
{
    _sampling_time = sampling_time.value();
    update_increments();
}

void boolean_debouncer::configure_delay(duration_t on_delay_s, duration_t off_delay_s) noexcept
{
    _on_delay_s = on_delay_s.value();
    _off_delay_s = off_delay_s.value();
    update_increments();
}

void boolean_debouncer::update_increments() noexcept
{
    // If delay is zero, we want it to be effectively immediate.
    // Using 1.1f ensures it hits the saturation limit (1.0 or 0.0) in one step.
    _on_increment = (_on_delay_s > 0.0f) ? (_sampling_time / _on_delay_s) : 1.1f;
    _off_decrement = (_off_delay_s > 0.0f) ? (_sampling_time / _off_delay_s) : 1.1f;
}

void boolean_debouncer::update(bool input) noexcept
{
    if (input) {
        _integrator += _on_increment;
    }
    else {
        _integrator -= _off_decrement;
    }

    // Saturate the integrator to [0.0, 1.0]
    _integrator = std::clamp(_integrator, 0.0f, 1.0f);

    // Hysteresis logic: Output only changes at the boundaries
    if (_integrator >= 1.0f) {
        _output = true;
    }
    else if (_integrator <= 0.0f) {
        _output = false;
    }
}

void boolean_debouncer::reset() noexcept
{
    _integrator = 0.0f;
    _output = false;
}

void boolean_debouncer::reset(bool initial_state) noexcept
{
    _output = initial_state;
    _integrator = initial_state ? 1.0f : 0.0f;
}

bool boolean_debouncer::get_output() const noexcept { return _output; }

real_t boolean_debouncer::get_integrator() const noexcept { return _integrator; }

}  // namespace yeet::control
