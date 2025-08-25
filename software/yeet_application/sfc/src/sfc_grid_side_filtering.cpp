#include "sfc/sfc_grid_side_filtering.hpp"

namespace yeet::application::sfc {

sfc_grid_side_filtering::sfc_grid_side_filtering(duration_t sampling_period)
    : _voltage_filter(sampling_period.value()),
      _current_filter(sampling_period.value()),
      _voltage_magnitude_filter(sampling_period.value())
{
    configure_voltage_filter(default_cutoff_frequency, default_damping_ratio);
    configure_current_filter(default_cutoff_frequency, default_damping_ratio);
    configure_voltage_magnitude_filter(default_voltage_magnitude_cutoff_frequency);
}

void sfc_grid_side_filtering::configure_voltage_filter(frequency_t cutoff_freq, real_t damping_ratio)
{
    _voltage_filter.configure(cutoff_freq.value(), damping_ratio);
}

void sfc_grid_side_filtering::configure_current_filter(frequency_t cutoff_freq, real_t damping_ratio)
{
    _current_filter.configure(cutoff_freq.value(), damping_ratio);
}

void sfc_grid_side_filtering::configure_voltage_magnitude_filter(frequency_t cutoff_freq, real_t damping_ratio)
{
    _voltage_magnitude_filter.configure(cutoff_freq.value(), damping_ratio);
}

void sfc_grid_side_filtering::update(const abc<voltage_pu_t>& voltage_pu, const abc<current_pu_t>& current_pu)
{
    // Filter voltages
    auto const voltage_pu_dimensionless = to_dimensionless_abc(voltage_pu);
    auto const filtered_voltage = _voltage_filter.update(voltage_pu_dimensionless);
    _filtered_voltages_pu = to_dimension_abc<voltage_pu_t>(filtered_voltage);

    // Filter currents
    auto const current_pu_dimensionless = to_dimensionless_abc(current_pu);
    auto const filtered_current = _current_filter.update(current_pu_dimensionless);
    _filtered_currents_pu = to_dimension_abc<current_pu_t>(filtered_current);

    // Transform filtered voltage to αβ frame and calculate magnitude
    _filtered_voltage_alphabeta_pu = _filtered_voltages_pu.to_alphabeta();
    const auto unfiltered_magnitude = voltage_pu.to_alphabeta().magnitude();
    _filtered_voltage_magnitude_pu = voltage_pu_t{_voltage_magnitude_filter.update(unfiltered_magnitude.value())};
}
}  // namespace yeet::application::sfc