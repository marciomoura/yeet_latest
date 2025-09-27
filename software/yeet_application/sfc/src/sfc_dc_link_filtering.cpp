#include "sfc/sfc_dc_link_filtering.hpp"

namespace yeet::application::sfc {

sfc_dc_link_filtering::sfc_dc_link_filtering(duration_t sampling_period)
    : _current_lpf(sampling_period.value()), _current_lpf_heavy(sampling_period.value())
{
    configure_current_filter(default_cutoff_frequency);
    configure_heavy_current_filter(default_heavy_filter_cutoff_frequency);
}

void sfc_dc_link_filtering::configure_current_filter(frequency_t current_filter_cutoff_freq)
{
    _current_lpf.configure(current_filter_cutoff_freq.value(), default_damping_ratio);
}

void sfc_dc_link_filtering::configure_heavy_current_filter(frequency_t current_filter_cutoff_freq)
{
    _current_lpf_heavy.configure(current_filter_cutoff_freq.value(), default_damping_ratio);
}

void sfc_dc_link_filtering::update(current_pu_t current_pu)
{
    _filtered_current_pu = current_pu_t{_current_lpf.update(current_pu.value())};
    _heavy_filtered_current_pu = current_pu_t{_current_lpf_heavy.update(current_pu.value())};
}

}  // namespace yeet::application::sfc
