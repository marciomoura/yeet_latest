#include "sfc/sfc_undervoltage_protection.hpp"

namespace yeet::application::sfc {

sfc_undervoltage_protection::sfc_undervoltage_protection(duration_t sampling_period) : _protection(sampling_period)
{
    // Configure with default LCI grid-side undervoltage protection thresholds
    configure_trip_threshold(default_trip_threshold, default_hysteresis_factor);
    configure_warning_threshold(default_warning_threshold);
    configure_trip_delay(default_trip_delay);
    configure_warning_delay(default_warning_delay);
}

void sfc_undervoltage_protection::configure_trip_threshold(voltage_pu_t trip_threshold, real_t hysteresis_factor)
{
    _protection.configure_trip_threshold(trip_threshold, hysteresis_factor);
}

void sfc_undervoltage_protection::configure_warning_threshold(voltage_pu_t warning_threshold)
{
    _protection.configure_warning_threshold(warning_threshold);
}

void sfc_undervoltage_protection::configure_trip_delay(duration_t trip_delay)
{
    _protection.configure_trip_delay(trip_delay);
}

void sfc_undervoltage_protection::configure_warning_delay(duration_t warning_delay)
{
    _protection.configure_warning_delay(warning_delay);
}

void sfc_undervoltage_protection::update(voltage_pu_t grid_voltage_magnitude_pu, bool enabled)
{
    if (enabled) {
        _protection.update(grid_voltage_magnitude_pu);
    }
    else {
        // When disabled, we feed a healthy 1.0 pu voltage to clear warnings
        // and prevent trip integrations.
        _protection.update(voltage_pu_t{1.0f});
    }
}

void sfc_undervoltage_protection::reset() { _protection.reset(); }

bool sfc_undervoltage_protection::is_tripped() const noexcept { return _protection.is_tripped(); }

bool sfc_undervoltage_protection::is_warning() const noexcept { return _protection.is_warning(); }

bool sfc_undervoltage_protection::can_reset() const noexcept { return _protection.can_reset(); }

voltage_pu_t sfc_undervoltage_protection::get_current_value() const noexcept { return _protection.get_current_value(); }

voltage_pu_t sfc_undervoltage_protection::get_trip_threshold() const noexcept
{
    return _protection.get_trip_threshold();
}

voltage_pu_t sfc_undervoltage_protection::get_warning_threshold() const noexcept
{
    return _protection.get_warning_threshold();
}

}  // namespace yeet::application::sfc
