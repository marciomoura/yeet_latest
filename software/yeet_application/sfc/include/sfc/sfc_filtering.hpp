#pragma once

#include <cmath>

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
    void configure_voltage_magnitude_filter(double cutoff_freq_hz, double damping_ratio = 0.707);

    void update(const control::abc<double>& voltage_pu, const control::abc<double>& current_pu);

    const control::abc<double>& get_filtered_voltages_pu() const { return _filtered_voltages_pu; }
    const control::abc<double>& get_filtered_currents_pu() const { return _filtered_currents_pu; }
    const control::alphabeta<double>& get_filtered_voltage_alphabeta_pu() const {
        return _filtered_voltage_alphabeta_pu;
    }
    double get_filtered_voltage_magnitude_pu() const { return _filtered_voltage_magnitude_pu; }

private:
    control::abc<double> _filtered_voltages_pu;
    control::abc<double> _filtered_currents_pu;
    control::alphabeta<double> _filtered_voltage_alphabeta_pu;
    double _filtered_voltage_magnitude_pu{};
    control::second_order_low_pass_filter<control::abc<double>> _voltage_filter;
    control::second_order_low_pass_filter<control::abc<double>> _current_filter;
    control::second_order_low_pass_filter<double> _voltage_magnitude_filter;
};

/**
 * @brief Handles filtering for the DC-link side of the SFC.
 */
class sfc_dc_link_filtering {
public:
    explicit sfc_dc_link_filtering(double sampling_period);

    void configure(double voltage_filter_cutoff_freq_hz, double current_ripple_freq_hz);

    void update(double voltage_pu, double current_pu);

    double get_filtered_voltage_pu() const { return _filtered_voltage_pu; }
    double get_filtered_current_pu() const { return _filtered_current_pu; }
    double get_filtered_current_magnitude_pu() const { return std::abs(_filtered_current_pu); }

private:
    double _filtered_voltage_pu{};
    double _filtered_current_pu{};
    control::second_order_low_pass_filter<double> _voltage_filter;
    control::second_order_band_reject_filter<double> _current_notch_filter;
    control::second_order_low_pass_filter<double> _current_lpf;
};

/**
 * @brief Handles filtering for the machine side of the SFC.
 */
class sfc_machine_side_filtering {
public:
    explicit sfc_machine_side_filtering(double sampling_period);

    void configure_voltage_filter(double cutoff_freq_hz, double damping_ratio = 0.707);
    void configure_current_filter(double cutoff_freq_hz, double damping_ratio = 0.707);
    void configure_voltage_magnitude_filter(double cutoff_freq_hz, double damping_ratio = 0.707);

    void update(const control::abc<double>& voltage_pu, const control::abc<double>& current_pu,
                double machine_frequency_hz);

    const control::abc<double>& get_filtered_voltages_pu() const { return _filtered_voltages_pu; }
    const control::abc<double>& get_filtered_currents_pu() const { return _filtered_currents_pu; }
    const control::alphabeta<double>& get_filtered_voltage_alphabeta_pu() const {
        return _filtered_voltage_alphabeta_pu;
    }
    double get_filtered_voltage_magnitude_pu() const { return _filtered_voltage_magnitude_pu; }

private:
    control::abc<double> _filtered_voltages_pu;
    control::abc<double> _filtered_currents_pu;
    control::alphabeta<double> _filtered_voltage_alphabeta_pu;
    double _filtered_voltage_magnitude_pu{};
    control::second_order_low_pass_filter<control::abc<double>> _voltage_filter;
    control::second_order_low_pass_filter<control::abc<double>> _current_filter;
    control::second_order_low_pass_filter<double> _voltage_magnitude_filter;
};

}  // namespace yeet::application::sfc