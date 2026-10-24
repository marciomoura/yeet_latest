#include "sfc/sfc_vhz_protection.hpp"

namespace yeet::application::sfc {

sfc_vhz_protection::sfc_vhz_protection(duration_t sampling_period)
    : _over_flux_protection(sampling_period), _under_flux_protection(sampling_period)
{
    // Configure with default LCI V/Hz protection thresholds
    configure_over_flux_trip_threshold(default_over_flux_trip_threshold, default_over_flux_hysteresis);
    configure_over_flux_warning_threshold(default_over_flux_warning_threshold);
    configure_over_flux_trip_delay(default_over_flux_trip_delay);
    configure_over_flux_warning_delay(default_over_flux_warning_delay);

    configure_under_flux_trip_threshold(default_under_flux_trip_threshold, default_under_flux_hysteresis);
    configure_under_flux_warning_threshold(default_under_flux_warning_threshold);
    configure_under_flux_trip_delay(default_under_flux_trip_delay);
    configure_under_flux_warning_delay(default_under_flux_warning_delay);
}

void sfc_vhz_protection::configure_over_flux_trip_threshold(flux_pu_t trip_threshold, real_t hysteresis_factor)
{
    _over_flux_protection.configure_trip_threshold(trip_threshold, hysteresis_factor);
}

void sfc_vhz_protection::configure_over_flux_warning_threshold(flux_pu_t warning_threshold)
{
    _over_flux_protection.configure_warning_threshold(warning_threshold);
}

void sfc_vhz_protection::configure_over_flux_trip_delay(duration_t trip_delay)
{
    _over_flux_protection.configure_trip_delay(trip_delay);
}

void sfc_vhz_protection::configure_over_flux_warning_delay(duration_t warning_delay)
{
    _over_flux_protection.configure_warning_delay(warning_delay);
}

void sfc_vhz_protection::configure_under_flux_trip_threshold(flux_pu_t trip_threshold, real_t hysteresis_factor)
{
    _under_flux_protection.configure_trip_threshold(trip_threshold, hysteresis_factor);
}

void sfc_vhz_protection::configure_under_flux_warning_threshold(flux_pu_t warning_threshold)
{
    _under_flux_protection.configure_warning_threshold(warning_threshold);
}

void sfc_vhz_protection::configure_under_flux_trip_delay(duration_t trip_delay)
{
    _under_flux_protection.configure_trip_delay(trip_delay);
}

void sfc_vhz_protection::configure_under_flux_warning_delay(duration_t warning_delay)
{
    _under_flux_protection.configure_warning_delay(warning_delay);
}

void sfc_vhz_protection::update(voltage_pu_t voltage_magnitude, frequency_pu_t frequency_pu)
{
    // Low-frequency guard: bypass V/Hz protection at very low frequency.
    if (abs(frequency_pu) < minimum_frequency_threshold) {
        _current_vhz_pu = flux_pu_t{0.0};
        return;
    }

    // Compute the flux per-unit ratio: V/Hz = V / f
    _current_vhz_pu = voltage_magnitude / frequency_pu;

    // Update both over-flux and under-flux protections
    _over_flux_protection.update(_current_vhz_pu);
    _under_flux_protection.update(_current_vhz_pu);
}

void sfc_vhz_protection::reset()
{
    _over_flux_protection.reset();
    _under_flux_protection.reset();
}

bool sfc_vhz_protection::is_over_flux_tripped() const noexcept { return _over_flux_protection.is_tripped(); }

bool sfc_vhz_protection::is_under_flux_tripped() const noexcept { return _under_flux_protection.is_tripped(); }

bool sfc_vhz_protection::is_over_flux_active() const noexcept { return _over_flux_protection.is_active(); }

bool sfc_vhz_protection::is_under_flux_active() const noexcept { return _under_flux_protection.is_active(); }

bool sfc_vhz_protection::is_any_tripped() const noexcept { return is_over_flux_tripped() || is_under_flux_tripped(); }

bool sfc_vhz_protection::is_over_flux_warning() const noexcept { return _over_flux_protection.is_warning(); }

bool sfc_vhz_protection::is_under_flux_warning() const noexcept { return _under_flux_protection.is_warning(); }

bool sfc_vhz_protection::can_reset() const noexcept
{
    return _over_flux_protection.can_reset() && _under_flux_protection.can_reset();
}

flux_pu_t sfc_vhz_protection::get_vhz_ratio_pu() const noexcept { return _current_vhz_pu; }

flux_pu_t sfc_vhz_protection::get_over_flux_trip_threshold() const noexcept
{
    return _over_flux_protection.get_trip_threshold();
}

flux_pu_t sfc_vhz_protection::get_under_flux_trip_threshold() const noexcept
{
    return _under_flux_protection.get_trip_threshold();
}

}  // namespace yeet::application::sfc
