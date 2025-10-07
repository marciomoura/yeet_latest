#include "yeet_control/derivative.hpp"

#include <cassert>

namespace yeet::control {

derivative::derivative(double sampling_time, yeet::real_t initial_value)
    : _sampling_time(sampling_time), _previous_input(initial_value)
{
    assert(sampling_time > 0.0F && "Sampling time must be positive.");
}

void derivative::configure_sampling_time(double sampling_time)
{
    assert(sampling_time > 0.0F && "Sampling time must be positive.");
    _sampling_time = sampling_time;
}

void derivative::update(yeet::real_t input) noexcept
{
    assert(_sampling_time > 0.0F);
    _output = (input - _previous_input) / _sampling_time;
    _previous_input = input;
}

void derivative::reset(yeet::real_t reset_value)
{
    _previous_input = reset_value;
    _output = 0.0F;
}

yeet::real_t derivative::get_output() const noexcept { return _output; }

}  // namespace yeet::control
