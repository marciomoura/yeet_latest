#include "sfc/sfc_overvoltage_protection.hpp"

namespace yeet::application::sfc {

sfc_overvoltage_protection::sfc_overvoltage_protection(duration_t sampling_period) : _protection(sampling_period)
{
    // Configure with default LCI grid-side overvoltage protection thresholds
    configure_trip_threshold(default_trip_threshold, default_hysteresis_factor);
    configure_warning_threshold(default_warning_threshold);
    configure_trip_delay(default_trip_delay);
    configure_warning_delay(default_warning_delay);
}

void sfc_overvoltage_protection::configure_trip_threshold(voltage_pu_t trip_threshold, real_t hysteresis_factor)
{
    _protection.configure_trip_threshold(trip_threshold, hysteresis_factor);
}

void sfc_overvoltage_protection::configure_warning_threshold(voltage_pu_t warning_threshold)
{
    _protection.configure_warning_threshold(warning_threshold);
}

void sfc_overvoltage_protection::configure_trip_delay(duration_t trip_delay)
{
    _protection.configure_trip_delay(trip_delay);
}

void sfc_overvoltage_protection::configure_warning_delay(duration_t warning_delay)
{
    _protection.configure_warning_delay(warning_delay);
}

void sfc_overvoltage_protection::update(voltage_pu_t grid_voltage_magnitude_pu)
{
    _protection.update(grid_voltage_magnitude_pu);
}

void sfc_overvoltage_protection::reset() { _protection.reset(); }

bool sfc_overvoltage_protection::is_tripped() const noexcept { return _protection.is_tripped(); }

bool sfc_overvoltage_protection::is_warning() const noexcept { return _protection.is_warning(); }

bool sfc_overvoltage_protection::can_reset() const noexcept { return _protection.can_reset(); }

voltage_pu_t sfc_overvoltage_protection::get_current_value() const noexcept { return _protection.get_current_value(); }

voltage_pu_t sfc_overvoltage_protection::get_trip_threshold() const noexcept
{
    return _protection.get_trip_threshold();
}

voltage_pu_t sfc_overvoltage_protection::get_warning_threshold() const noexcept
{
    return _protection.get_warning_threshold();
}

}  // namespace yeet::application::sfc
