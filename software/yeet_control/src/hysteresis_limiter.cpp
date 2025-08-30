#include "yeet_control/hysteresis_limiter.hpp"

#include <cassert>

namespace yeet::control {

void hysteresis_limiter::configure_thresholds(real_t low_threshold, real_t high_threshold)
{
    assert(low_threshold <= high_threshold && "Low threshold cannot be greater than high threshold.");
    _low_threshold = low_threshold;
    _high_threshold = high_threshold;
}

void hysteresis_limiter::update(real_t input) noexcept
{
    if (_output_state) {
        // Currently in the high state, check for transition to low.
        if (input < _low_threshold) {
            _output_state = false;
        }
    }
    else {
        // Currently in the low state, check for transition to high.
        if (input > _high_threshold) {
            _output_state = true;
        }
    }
}

void hysteresis_limiter::reset() noexcept { _output_state = false; }

bool hysteresis_limiter::get_output() const noexcept { return _output_state; }

}  // namespace yeet::control