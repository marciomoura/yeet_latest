#pragma once

#include "yeet_control/reference_frame.hpp"
#include "yeet_control/second_order_filter.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles filtering for the machine side of the SFC.
 */
class sfc_machine_side_filtering {
public:
    static constexpr auto default_cutoff_frequency = frequency_t{100.0};
    static constexpr auto default_voltage_magnitude_cutoff_frequency = frequency_t{10.0};
    static constexpr real_t default_damping_ratio = 0.707;

    explicit sfc_machine_side_filtering(duration_t sampling_period);

    void update(const abc<voltage_pu_t>& voltage_pu, const abc<current_pu_t>& current_pu);

    void configure_voltage_filter(frequency_t cutoff_freq, real_t damping_ratio = default_damping_ratio);

    void configure_current_filter(frequency_t cutoff_freq, real_t damping_ratio = default_damping_ratio);

    void configure_voltage_magnitude_filter(frequency_t cutoff_freq, real_t damping_ratio = default_damping_ratio);

    const abc<voltage_pu_t>& get_filtered_voltages_pu() const { return _filtered_voltages_pu; }

    const abc<current_pu_t>& get_filtered_currents_pu() const { return _filtered_currents_pu; }

    const alphabeta<voltage_pu_t>& get_filtered_voltage_alphabeta_pu() const { return _filtered_voltage_alphabeta_pu; }

    voltage_pu_t get_filtered_voltage_magnitude_pu() const { return _filtered_voltage_magnitude_pu; }

private:
    abc<voltage_pu_t> _filtered_voltages_pu;
    abc<current_pu_t> _filtered_currents_pu;
    alphabeta<voltage_pu_t> _filtered_voltage_alphabeta_pu;
    voltage_pu_t _filtered_voltage_magnitude_pu{};
    control::second_order_low_pass_filter<abc<real_t>> _voltage_filter;
    control::second_order_low_pass_filter<abc<real_t>> _current_filter;
    control::second_order_low_pass_filter<real_t> _voltage_magnitude_filter;
};

}  // namespace yeet::application::sfc