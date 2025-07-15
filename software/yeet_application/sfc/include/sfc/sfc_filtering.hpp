#pragma once

#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/second_order_filter.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles filtering for the grid side of the SFC.
 */
class sfc_grid_side_filtering {
public:
    explicit sfc_grid_side_filtering(double sampling_period);

    void configure(double voltage_filter_cutoff_freq_hz, double current_filter_cutoff_freq_hz,
                   double damping_ratio = 0.707);

    void update(const control::abc<double>& raw_voltages, const control::abc<double>& raw_currents);

    const control::abc<double>& get_filtered_voltages() const { return _filtered_voltages; }
    const control::abc<double>& get_filtered_currents() const { return _filtered_currents; }

private:
    control::abc<double> _filtered_voltages;
    control::abc<double> _filtered_currents;
    control::second_order_low_pass_filter<control::abc<double>> _voltage_filter;
    control::second_order_low_pass_filter<control::abc<double>> _current_filter;
};

/**
 * @brief Handles filtering for the DC-link side of the SFC.
 */
class sfc_dc_link_filtering {
public:
    explicit sfc_dc_link_filtering(double sampling_period);

    void configure(double voltage_filter_cutoff_freq_hz, double current_ripple_freq_hz);

    void update(double raw_voltage, double raw_current);

    double get_filtered_voltage() const { return _filtered_voltage; }
    double get_filtered_current() const { return _filtered_current; }

private:
    double _filtered_voltage{};
    double _filtered_current{};
    control::second_order_low_pass_filter<double> _voltage_filter;
    control::second_order_band_reject_filter<double> _current_notch_filter;
    control::first_order_low_pass_filter<double> _current_lpf;
};

/**
 * @brief Handles filtering for the machine side of the SFC.
 */
class sfc_machine_side_filtering {
public:
    explicit sfc_machine_side_filtering(double sampling_period);

    void configure_voltage_filter(double cutoff_freq_hz, double damping_ratio = 0.707);
    void configure_current_filter(double cutoff_freq_hz, double damping_ratio = 0.707);

    void update(const control::abc<double>& raw_voltages, const control::abc<double>& raw_currents,
                double machine_frequency_hz);

    const control::abc<double>& get_filtered_voltages() const { return _filtered_voltages; }
    const control::abc<double>& get_filtered_currents() const { return _filtered_currents; }

private:
    control::abc<double> _filtered_voltages;
    control::abc<double> _filtered_currents;
    control::second_order_low_pass_filter<control::abc<double>> _voltage_filter;
    control::second_order_low_pass_filter<control::abc<double>> _current_filter;
};

}  // namespace yeet::application::sfc