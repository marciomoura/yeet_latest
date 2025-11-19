#include "sfc/sfc_stall_protection.hpp"

namespace yeet::application::sfc {

sfc_stall_protection::sfc_stall_protection(duration_t sampling_period)
    : _trip_timer(real_t{0.0}, sampling_period.value()), _warning_timer(real_t{0.0}, sampling_period.value())
{
    // Configure with default LCI stall protection thresholds
    configure_current_threshold(default_stall_current_threshold, default_current_hysteresis);
    configure_speed_threshold(default_stall_speed_threshold, default_speed_hysteresis);
    configure_trip_delay(default_stall_trip_delay);
    configure_warning_delay(default_stall_warning_delay);
}

void sfc_stall_protection::configure_current_threshold(current_pu_t current_threshold, real_t hysteresis_factor)
{
    _current_threshold = current_threshold;
    // Configure hysteresis limiter: output true when current > upper threshold, false when current < lower threshold
    current_pu_t lower_threshold = current_threshold * (real_t{1.0} - hysteresis_factor);
    _current_limiter.configure_thresholds(lower_threshold.value(), current_threshold.value());
}

void sfc_stall_protection::configure_speed_threshold(speed_pu_t speed_threshold, real_t hysteresis_factor)
{
    _speed_threshold = speed_threshold;
    // Configure hysteresis limiter: For low-speed detection, we need inverted logic
    // Output should be TRUE when speed < lower threshold (stall condition)
    // Since hysteresis_limiter outputs TRUE when input > high_threshold,
    // we store the negated threshold configuration and will invert the output
    speed_pu_t upper_threshold = speed_threshold * (real_t{1.0} + hysteresis_factor);
    // We configure with the same thresholds but will use inverted output (!get_output())
    // This means: output FALSE when speed > upper, output TRUE when speed < lower
    _speed_limiter.configure_thresholds(speed_threshold.value(), upper_threshold.value());
}

void sfc_stall_protection::configure_trip_delay(duration_t trip_delay) { _trip_timer.configure(trip_delay.value()); }

void sfc_stall_protection::configure_warning_delay(duration_t warning_delay)
{
    _warning_timer.configure(warning_delay.value());
}

void sfc_stall_protection::update(current_pu_t measured_current, speed_pu_t measured_speed, bool startup_active)
{
    _current_value = measured_current;
    _speed_value = measured_speed;

    // Update hysteresis limiters
    _current_limiter.update(measured_current.value());
    _speed_limiter.update(measured_speed.value());

    // Check stall condition: High current AND low speed AND startup active
    bool stall_condition = check_stall_condition(startup_active);
    _stall_condition_active = stall_condition;

    // Update timers (on-delay behavior)
    _trip_timer.update(stall_condition);
    _warning_timer.update(stall_condition);

    // Latch trip when timer expires
    if (_trip_timer.get_output() && !_is_tripped) {
        _is_tripped = true;
    }
}

void sfc_stall_protection::reset()
{
    if (can_reset()) {
        _is_tripped = false;
        _trip_timer.reset();
        _warning_timer.reset();
    }
}

bool sfc_stall_protection::is_tripped() const noexcept { return _is_tripped; }

bool sfc_stall_protection::is_warning() const noexcept { return _warning_timer.get_output(); }

bool sfc_stall_protection::can_reset() const noexcept
{
    // Can reset when either current drops below threshold OR speed rises above threshold
    // For current: hysteresis_limiter output is FALSE when current < low_threshold
    // For speed: inverted output is FALSE when speed > high_threshold (speed cleared)
    bool current_cleared = !_current_limiter.get_output();
    bool speed_cleared = _speed_limiter.get_output();  // TRUE when speed > high_threshold

    return current_cleared || speed_cleared;
}
bool sfc_stall_protection::is_stall_condition_active() const noexcept { return _stall_condition_active; }

current_pu_t sfc_stall_protection::get_current_value() const noexcept { return _current_value; }

speed_pu_t sfc_stall_protection::get_speed_value() const noexcept { return _speed_value; }

current_pu_t sfc_stall_protection::get_current_threshold() const noexcept { return _current_threshold; }

speed_pu_t sfc_stall_protection::get_speed_threshold() const noexcept { return _speed_threshold; }

bool sfc_stall_protection::check_stall_condition(bool startup_active) const noexcept
{
    // Stall logic: All three conditions must be true
    // 1. Current exceeds threshold (hysteresis limiter output is true when current > high_threshold)
    // 2. Speed below threshold (invert hysteresis limiter because we configured it normally)
    //    Output is TRUE when speed < low_threshold (inverted logic for low-speed detection)
    // 3. Startup active (state-dependent, not during normal operation)
    bool high_current = _current_limiter.get_output();
    bool low_speed = !_speed_limiter.get_output();  // Inverted: TRUE when speed is LOW

    return startup_active && high_current && low_speed;
}

}  // namespace yeet::application::sfc
