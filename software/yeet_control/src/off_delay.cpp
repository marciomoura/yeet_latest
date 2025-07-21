#include "yeet_control/off_delay.hpp"

#include <cassert>

namespace yeet::control {

off_delay::off_delay(real_t delay_s, real_t sampling_time) : _sampling_time(sampling_time) {
    assert(sampling_time > 0.0f && "Sampling time must be positive.");
    configure(delay_s);
}

void off_delay::configure(real_t delay_s) {
    assert(delay_s >= 0.0f && "Delay must be non-negative.");
    _samples_required = static_cast<int>(delay_s / _sampling_time);
    reset();
}

void off_delay::reset() {
    _sample_counter = 0;
    _output_state = false;
}

void off_delay::update(bool input) {
    if (input) {
        // Input is true: Reset the timer and force the output to true.
        _sample_counter = 0;
        _output_state = true;
    } else {
        // Input is false: Increment the counter if the delay is not yet met.
        if (_sample_counter < _samples_required) {
            _sample_counter++;
        }
        // The output remains true until the counter reaches the required value.
        _output_state = (_sample_counter < _samples_required);
    }
}

bool off_delay::get_output() const { return _output_state; }

}  // namespace yeet::control