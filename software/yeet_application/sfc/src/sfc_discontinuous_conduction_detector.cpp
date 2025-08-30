#include "sfc/sfc_discontinuous_conduction_detector.hpp"

#include <cmath>

namespace yeet::application::sfc {

void sfc_discontinuous_conduction_detector::configure(current_pu_t zero_current_threshold,
    duration_t is_zero_on_delay,
    duration_t discontinuous_on_delay,
    duration_t discontinuous_off_delay)
{
    _zero_current_threshold = zero_current_threshold;
    _is_zero_timer.configure(is_zero_on_delay);
    _is_discontinuous_mode_timer.configure(discontinuous_on_delay, discontinuous_off_delay);
}

void sfc_discontinuous_conduction_detector::update(current_pu_t current)
{
    const bool is_below_threshold = (std::abs(current.value()) < _zero_current_threshold.value());

    // off_delay timer: immediate true when input is true, delayed false when input goes false
    // We want immediate false when current rises, delayed true when current drops
    // So we need to invert the logic: feed !is_below_threshold to get the desired behavior
    _is_zero_timer.update(is_below_threshold);
    _is_discontinuous_mode_timer.update(is_below_threshold);
}

void sfc_discontinuous_conduction_detector::reset()
{
    _is_zero_timer.reset();
    _is_discontinuous_mode_timer.reset();
}

bool sfc_discontinuous_conduction_detector::get_is_current_zero() const { return _is_zero_timer.get_output(); }

bool sfc_discontinuous_conduction_detector::get_is_discontinuous_mode() const
{
    return _is_discontinuous_mode_timer.get_output();
}

}  // namespace yeet::application::sfc