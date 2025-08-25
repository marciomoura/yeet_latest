#include "sfc/sfc_dc_link_filtering.hpp"

namespace yeet::application::sfc {

sfc_dc_link_filtering::sfc_dc_link_filtering(duration_t sampling_period)
    : _voltage_filter(sampling_period.value()),
      _current_lpf(sampling_period.value()),
      _current_lpf_heavy(sampling_period.value())
{
    configure_current_filter(default_cutoff_frequency);
    configure_voltage_filter(default_cutoff_frequency);
    configure_heavy_current_filter(default_heavy_filter_cutoff_frequency);
}

void sfc_dc_link_filtering::configure_voltage_filter(frequency_t voltage_filter_cutoff_freq)
{
    _voltage_filter.configure(voltage_filter_cutoff_freq.value(), default_damping_ratio);
}

void sfc_dc_link_filtering::configure_current_filter(frequency_t current_filter_cutoff_freq)
{
    _current_lpf.configure(current_filter_cutoff_freq.value(), default_damping_ratio);
}

void sfc_dc_link_filtering::configure_heavy_current_filter(frequency_t current_filter_cutoff_freq)
{
    _current_lpf_heavy.configure(current_filter_cutoff_freq.value(), default_damping_ratio);
}

void sfc_dc_link_filtering::update(voltage_pu_t voltage_pu, current_pu_t current_pu)
{
    _filtered_voltage_pu = voltage_pu_t{_voltage_filter.update(voltage_pu.value())};
    _filtered_current_pu = current_pu_t{_current_lpf.update(current_pu.value())};
    _heavy_filtered_current_pu = current_pu_t{_current_lpf_heavy.update(current_pu.value())};
}

}  // namespace yeet::application::sfc
