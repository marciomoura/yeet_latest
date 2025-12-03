#include "sfc/sfc_mode_detector.hpp"

#include <cassert>

namespace yeet::application::sfc {

sfc_mode_detector::sfc_mode_detector() { reset(); }

void sfc_mode_detector::configure_mode_handover_speed_pu(speed_pu_t handover_speed)
{
    assert(handover_speed.value() >= 0.0 && "Handover speed must be non-negative");
    _handover_speed_pu = handover_speed;
    update_limiter_thresholds();
}

void sfc_mode_detector::configure_hysteresis_factor(real_t hysteresis_factor)
{
    assert(hysteresis_factor >= 0.0 && hysteresis_factor < 1.0 && "Hysteresis must be between 0 and 1");
    _hysteresis_factor = hysteresis_factor;
    update_limiter_thresholds();
}

void sfc_mode_detector::configure_force_natural_commutation(bool force_natural_commutation)
{
    _force_natural_commutation = force_natural_commutation;
}

void sfc_mode_detector::update(bool enable, speed_pu_t measured_speed) noexcept
{
    _previous_mode = _current_mode;

    if (!enable) {
        _current_mode = sfc_operating_mode_t::forced_commutation;
        _speed_limiter.reset();
    }
    else if (_force_natural_commutation) {
        // When forced natural commutation is enabled, always operate in natural commutation mode
        _current_mode = sfc_operating_mode_t::natural_commutation;
    }
    else {
        _speed_limiter.update(measured_speed.value());
        if (_speed_limiter.get_output()) {
            _current_mode = sfc_operating_mode_t::natural_commutation;
        }
        else {
            _current_mode = sfc_operating_mode_t::forced_commutation;
        }
    }
}

void sfc_mode_detector::reset()
{
    _current_mode = sfc_operating_mode_t::forced_commutation;
    _previous_mode = sfc_operating_mode_t::forced_commutation;
    _speed_limiter.reset();
    update_limiter_thresholds();
}

sfc_operating_mode_t sfc_mode_detector::get_operating_mode() const noexcept { return _current_mode; }

bool sfc_mode_detector::has_mode_changed() const noexcept { return _current_mode != _previous_mode; }

void sfc_mode_detector::update_limiter_thresholds()
{
    const real_t base_speed = _handover_speed_pu.value();
    const real_t low_threshold = base_speed * (1.0F - _hysteresis_factor);
    const real_t high_threshold = base_speed * (1.0F + _hysteresis_factor);
    _speed_limiter.configure_thresholds(low_threshold, high_threshold);
}

}  // namespace yeet::application::sfc