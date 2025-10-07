#include "yeet_control/interval_timer.hpp"

#include <cassert>

namespace yeet::control {

// Conversion factor from seconds to microseconds
constexpr real_t US_PER_SECOND = 1e6;

interval_timer::interval_timer(double sampling_time) : _sampling_time(sampling_time)
{
    assert(sampling_time > 0.0 && "Sampling time must be positive.");
}

void interval_timer::start()
{
    _elapsed_time_us = 0.0;
    _is_running = true;
}

real_t interval_timer::stop()
{
    _is_running = false;
    return _elapsed_time_us;
}

void interval_timer::update()
{
    if (_is_running) {
        _elapsed_time_us += _sampling_time * US_PER_SECOND;
    }
}

bool interval_timer::is_running() const { return _is_running; }

real_t interval_timer::get_elapsed_time() const { return _elapsed_time_us; }

void interval_timer::reset()
{
    _elapsed_time_us = 0.0;
    _is_running = false;
}

void interval_timer::configure(double sampling_time)
{
    assert(sampling_time > 0.0 && "Sampling time must be positive.");
    _sampling_time = sampling_time;
    reset();
}

}  // namespace yeet::control