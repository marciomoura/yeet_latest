#include "yeet_control/elapsed_timer.hpp"

#include <algorithm>
#include <cassert>


namespace yeet::control {

elapsed_timer::elapsed_timer(duration_t sampling_time)
    : _sampling_time(sampling_time), _elapsed_time(0.0), _enabled(false) {
    // Validate sampling time
    assert(sampling_time.value() > 0.0 && "Sampling time must be positive");
}

void elapsed_timer::configure_sampling_time(duration_t sampling_time) {
    assert(sampling_time.value() > 0.0 && "Sampling time must be positive");
    _sampling_time = sampling_time;
}

void elapsed_timer::update(bool enable) noexcept {
    _enabled = enable;

    // Only increment elapsed time when enabled
    if (_enabled) {
        _elapsed_time = duration_t(_elapsed_time.value() + _sampling_time.value());
    }
}

void elapsed_timer::reset() noexcept { _elapsed_time = duration_t(0.0); }

void elapsed_timer::reset(duration_t initial_time) noexcept { _elapsed_time = initial_time; }

duration_t elapsed_timer::get_elapsed_time() const noexcept { return _elapsed_time; }

bool elapsed_timer::has_elapsed(duration_t threshold) const noexcept {
    return _elapsed_time.value() >= threshold.value();
}

duration_t elapsed_timer::get_remaining_time(duration_t threshold) const noexcept {
    if (_elapsed_time.value() >= threshold.value()) {
        return duration_t(0.0);
    }
    return duration_t(threshold.value() - _elapsed_time.value());
}

bool elapsed_timer::is_enabled() const noexcept { return _enabled; }

}  // namespace yeet::control