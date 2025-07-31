#pragma once

#include <cmath>

#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/second_order_filter.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles filtering for the DC-link side of the SFC.
 */
class sfc_dc_link_filtering {
public:
    static constexpr real_t default_damping_ratio = 0.707;
    static constexpr auto default_cutoff_frequency = frequency_t{100.0};
    static constexpr auto default_heavy_filter_cutoff_frequency =
        frequency_t{50.0 * 6.0 / 10.0};  // 5 decades lower to provide damping of 40dB

    explicit sfc_dc_link_filtering(duration_t sampling_period);

    void configure_voltage_filter(frequency_t voltage_filter_cutoff_freq);

    void configure_current_filter(frequency_t current_filter_cutoff_freq);

    void update(voltage_pu_t voltage_pu, current_pu_t current_pu);

    voltage_pu_t get_filtered_voltage_pu() const { return _filtered_voltage_pu; }

    current_pu_t get_filtered_current_pu() const { return _filtered_current_pu; }
    current_pu_t get_heavy_filtered_current_pu() const { return _heavy_filtered_current_pu; }

private:
    voltage_pu_t _filtered_voltage_pu{};
    current_pu_t _filtered_current_pu{};
    current_pu_t _heavy_filtered_current_pu{};

    control::second_order_low_pass_filter<real_t> _voltage_filter;
    control::second_order_low_pass_filter<real_t> _current_lpf;
    control::second_order_low_pass_filter<real_t> _current_lpf_heavy;
};

}  // namespace yeet::application::sfc