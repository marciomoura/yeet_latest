#include "sfc/sfc_current_protection.hpp"

namespace yeet::application::sfc {

sfc_current_protection::sfc_current_protection(duration_t sampling_period) : _protection(sampling_period)
{
    // Configure with default LCI protection thresholds
    configure_trip_threshold(default_trip_threshold, default_hysteresis_factor);
    configure_warning_threshold(default_warning_threshold);
    configure_trip_delay(default_trip_delay);
    configure_warning_delay(default_warning_delay);
}

void sfc_current_protection::configure_trip_threshold(current_pu_t trip_threshold, real_t hysteresis_factor)
{
    _protection.configure_trip_threshold(trip_threshold, hysteresis_factor);
}

void sfc_current_protection::configure_warning_threshold(current_pu_t warning_threshold)
{
    _protection.configure_warning_threshold(warning_threshold);
}

void sfc_current_protection::configure_trip_delay(duration_t trip_delay)
{
    _protection.configure_trip_delay(trip_delay);
}

void sfc_current_protection::configure_warning_delay(duration_t warning_delay)
{
    _protection.configure_warning_delay(warning_delay);
}

void sfc_current_protection::update(current_pu_t dc_link_current_pu) { _protection.update(dc_link_current_pu); }

void sfc_current_protection::reset() { _protection.reset(); }

bool sfc_current_protection::is_tripped() const noexcept { return _protection.is_tripped(); }

bool sfc_current_protection::is_active() const noexcept { return _protection.is_active(); }

bool sfc_current_protection::is_warning() const noexcept { return _protection.is_warning(); }

bool sfc_current_protection::can_reset() const noexcept { return _protection.can_reset(); }

current_pu_t sfc_current_protection::get_current_value() const noexcept { return _protection.get_current_value(); }

current_pu_t sfc_current_protection::get_trip_threshold() const noexcept { return _protection.get_trip_threshold(); }

current_pu_t sfc_current_protection::get_warning_threshold() const noexcept
{
    return _protection.get_warning_threshold();
}

}  // namespace yeet::application::sfc
