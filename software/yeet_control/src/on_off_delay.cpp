#include "yeet_control/on_off_delay.hpp"

#include <cassert>

namespace yeet::control {

on_off_delay::on_off_delay(double sampling_time)
    : _on_timer(real_t{0.0}, sampling_time), _off_timer(real_t{0.0}, sampling_time)
{
}

void on_off_delay::configure(real_t on_delay_s, real_t off_delay_s)
{
    _on_timer.configure(on_delay_s);
    _off_timer.configure(off_delay_s);
}

void on_off_delay::reset()
{
    _on_timer.reset();
    _off_timer.reset();
}

void on_off_delay::update(bool input)
{
    // delays false -> true detection
    // true -> false are immediate
    _on_timer.update(input);

    const bool on_timer_output = _on_timer.get_output();
    // delays true -> false detection
    // false -> true are immediate
    _off_timer.update(on_timer_output);
}

bool on_off_delay::get_output() const { return _off_timer.get_output(); }

}  // namespace yeet::control