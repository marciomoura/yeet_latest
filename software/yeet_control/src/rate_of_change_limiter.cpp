#include "yeet_control/rate_of_change_limiter.hpp"

#include <algorithm>  // For std::clamp
#include <cassert>

namespace yeet::control {

rate_of_change_limiter::rate_of_change_limiter(real_t rate_limit_per_second, real_t sampling_time, real_t initial_value)
    : _sampling_time(sampling_time), _current_output(initial_value) {
    assert(sampling_time > 0.0f && "Sampling time must be positive.");
    configure(rate_limit_per_second);
}

void rate_of_change_limiter::configure(real_t rate_limit_per_second) {
    assert(rate_limit_per_second > 0.0f && "Rate limit must be positive.");
    _max_change_per_sample = rate_limit_per_second * _sampling_time;
}

void rate_of_change_limiter::reset(real_t reset_value) { _current_output = reset_value; }

void rate_of_change_limiter::update(real_t input) {
    // Calculate the difference between the target and the current output
    const real_t error = input - _current_output;

    // Clamp the change to the maximum allowed per sample
    const real_t change = std::clamp(error, -_max_change_per_sample, _max_change_per_sample);

    // Apply the limited change to the output
    _current_output += change;
}

real_t rate_of_change_limiter::get_output() const { return _current_output; }

}  // namespace yeet::control