#pragma once

#include <cmath>

#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/second_order_filter.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles filtering for the grid side of the SFC.
 */
class sfc_grid_side_filtering {
public:
    static constexpr auto default_cutoff_frequency = frequency_t{100.0};
    static constexpr auto default_voltage_magnitude_cutoff_frequency = frequency_t{10.0};
    static constexpr real_t default_damping_ratio = 0.707;

    explicit sfc_grid_side_filtering(duration_t sampling_period);

    void configure(frequency_t voltage_filter_cutoff_freq, frequency_t current_filter_cutoff_freq,
                   real_t damping_ratio = default_damping_ratio);

    void configure_voltage_magnitude_filter(frequency_t cutoff_freq, real_t damping_ratio = default_damping_ratio);

    void update(const abc<voltage_pu_t>& voltage_pu, const abc<current_pu_t>& current_pu);

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

/**
 * @brief Handles filtering for the DC-link side of the SFC.
 */
class sfc_dc_link_filtering {
public:
    static constexpr auto default_cutoff_frequency = frequency_t{100.0};
    static constexpr real_t default_damping_ratio = 0.707;

    explicit sfc_dc_link_filtering(duration_t sampling_period);

    void configure_voltage_filter(frequency_t voltage_filter_cutoff_freq);

    void configure_current_filter(frequency_t current_filter_cutoff_freq);

    void update(voltage_pu_t voltage_pu, current_pu_t current_pu);

    voltage_pu_t get_filtered_voltage_pu() const { return _filtered_voltage_pu; }

    current_pu_t get_filtered_current_pu() const { return _filtered_current_pu; }

private:
    voltage_pu_t _filtered_voltage_pu{};
    current_pu_t _filtered_current_pu{};

    control::second_order_low_pass_filter<real_t> _voltage_filter;
    control::second_order_low_pass_filter<real_t> _current_lpf;
};

/**
 * @brief Handles filtering for the machine side of the SFC.
 */
class sfc_machine_side_filtering {
public:
    static constexpr auto default_cutoff_frequency = frequency_t{100.0};
    static constexpr auto default_voltage_magnitude_cutoff_frequency = frequency_t{10.0};
    static constexpr real_t default_damping_ratio = 0.707;

    explicit sfc_machine_side_filtering(duration_t sampling_period);

    void update(const abc<voltage_pu_t>& voltage_pu, const abc<current_pu_t>& current_pu,
                frequency_t machine_frequency);

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