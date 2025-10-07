#include "yeet_control/on_delay.hpp"

#include <cassert>

namespace yeet::control {

on_delay::on_delay(real_t delay_s, double sampling_time) : _sampling_time(sampling_time)
{
    assert(sampling_time > 0.0f && "Sampling time must be positive.");
    configure(delay_s);
}

void on_delay::configure(real_t delay_s)
{
    assert(delay_s >= 0.0f && "Delay must be non-negative.");
    _samples_required = static_cast<int>(delay_s / _sampling_time);
    reset();
}

void on_delay::reset()
{
    _sample_counter = 0;
    _output_state = false;
}

void on_delay::update(bool input)
{
    if (input) {
        // Input is true: Increment the counter if the delay is not yet met.
        if (_sample_counter < _samples_required) {
            _sample_counter++;
        }
        // The output becomes true only once the counter reaches the required value.
        _output_state = (_sample_counter >= _samples_required);
    }
    else {
        // Input is false: Reset the counter and force the output to false.
        _sample_counter = 0;
        _output_state = false;
    }
}

bool on_delay::get_output() const { return _output_state; }

}  // namespace yeet::control