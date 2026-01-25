#include "sfc/sfc_undervoltage_protection.hpp"

namespace yeet::application::sfc {

sfc_undervoltage_protection::sfc_undervoltage_protection(duration_t sampling_period)
    : _trip_timer(default_trip_delay.value(), (double)sampling_period.value()),
      _recovery_timer(default_recovery_delay.value(), (double)sampling_period.value()),
      _warning_timer(default_warning_delay.value(), (double)sampling_period.value())
{
}

void sfc_undervoltage_protection::configure_detection_threshold(voltage_pu_t detection_threshold)
{
    _detection_threshold = detection_threshold;
}

void sfc_undervoltage_protection::configure_recovery_threshold(voltage_pu_t recovery_threshold)
{
    _recovery_threshold = recovery_threshold;
}

void sfc_undervoltage_protection::configure_trip_threshold(voltage_pu_t trip_threshold, real_t hysteresis_factor)
{
    // Legacy mapping: trip threshold sets detection, recovery sets detection + hysteresis
    _detection_threshold = trip_threshold;
    _recovery_threshold = trip_threshold * (real_t{1.0} + hysteresis_factor);
}

void sfc_undervoltage_protection::configure_warning_threshold(voltage_pu_t warning_threshold)
{
    _warning_threshold = warning_threshold;
}

void sfc_undervoltage_protection::configure_trip_delay(duration_t trip_delay)
{
    _trip_timer.configure(trip_delay.value());
}

void sfc_undervoltage_protection::configure_recovery_delay(duration_t recovery_delay)
{
    _recovery_timer.configure(recovery_delay.value());
}

void sfc_undervoltage_protection::configure_warning_delay(duration_t warning_delay)
{
    _warning_timer.configure(warning_delay.value());
}

void sfc_undervoltage_protection::update(voltage_pu_t grid_voltage_magnitude_pu, bool enabled)
{
    _current_value = grid_voltage_magnitude_pu;

    if (!enabled) {
        _dip_raw_input = false;
        _trip_timer.reset();
        _recovery_timer.reset();
        _warning_timer.reset();
        return;
    }

    // 1. Raw dip detection with hysteresis
    if (grid_voltage_magnitude_pu < _detection_threshold) {
        _dip_raw_input = true;
    }
    else if (grid_voltage_magnitude_pu > _recovery_threshold) {
        _dip_raw_input = false;
    }

    // 2. State "Detected" (Ride-through / End-position mode)
    // OFF-delay: goes HIGH immediately, goes LOW only after recovery_delay
    _recovery_timer.update(_dip_raw_input);

    // 3. Trip Logic
    // Trip only if voltage stays continuously below detection threshold for trip_delay
    _trip_timer.update(grid_voltage_magnitude_pu < _detection_threshold);
    if (_trip_timer.get_output()) {
        _is_tripped = true;
    }

    // 4. Warning Logic
    _warning_timer.update(grid_voltage_magnitude_pu < _warning_threshold);
}

void sfc_undervoltage_protection::reset()
{
    if (can_reset()) {
        _is_tripped = false;
        _trip_timer.reset();
    }
}

bool sfc_undervoltage_protection::is_tripped() const noexcept { return _is_tripped; }

bool sfc_undervoltage_protection::is_detected() const noexcept { return _recovery_timer.get_output(); }

bool sfc_undervoltage_protection::is_warning() const noexcept { return _warning_timer.get_output(); }

bool sfc_undervoltage_protection::can_reset() const noexcept { return _current_value > _recovery_threshold; }

voltage_pu_t sfc_undervoltage_protection::get_current_value() const noexcept { return _current_value; }

voltage_pu_t sfc_undervoltage_protection::get_detection_threshold() const noexcept { return _detection_threshold; }

voltage_pu_t sfc_undervoltage_protection::get_recovery_threshold() const noexcept { return _recovery_threshold; }

voltage_pu_t sfc_undervoltage_protection::get_warning_threshold() const noexcept { return _warning_threshold; }

}  // namespace yeet::application::sfc
