#pragma once

#include "sfc_per_unit_system.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/second_order_filter.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles measurements for the grid side of the SFC.
 */
class sfc_grid_side_measurement {
public:
    explicit sfc_grid_side_measurement(double sampling_period);

    void configure_filters(double voltage_filter_cutoff_freq_hz, double current_filter_cutoff_freq_hz,
                           double damping_ratio = 0.707);

    void update(const control::abc<double>& raw_voltages, const control::abc<double>& raw_currents,
                const control::angle<control::zero_to_two_pi>& grid_angle, const sfc_per_unit_system& pu_system);

    const control::dq<double>& get_voltages_pu() const { return _voltages_pu; }
    const control::dq<double>& get_currents_pu() const { return _currents_pu; }

    const control::second_order_low_pass_filter<control::abc<double>>& get_voltage_filter() const {
        return _voltage_filter;
    }
    const control::second_order_low_pass_filter<control::abc<double>>& get_current_filter() const {
        return _current_filter;
    }

private:
    control::dq<double> _voltages_pu;
    control::dq<double> _currents_pu;
    control::second_order_low_pass_filter<control::abc<double>> _voltage_filter;
    control::second_order_low_pass_filter<control::abc<double>> _current_filter;
};

/**
 * @brief Handles measurements for the DC-link side of the SFC.
 */
class sfc_dc_link_measurement {
public:
    explicit sfc_dc_link_measurement(double sampling_period);

    void configure_filters(double voltage_filter_cutoff_freq_hz, double current_ripple_freq_hz);

    void update(double raw_voltage, double raw_current, const sfc_per_unit_system& pu_system);

    double get_voltage_pu() const { return _voltage_pu; }
    double get_current_pu() const { return _current_pu; }

    const control::second_order_low_pass_filter<double>& get_voltage_filter() const { return _voltage_filter; }
    const control::second_order_band_reject_filter<double>& get_current_notch_filter() const {
        return _current_notch_filter;
    }
    const control::first_order_low_pass_filter<double>& get_current_lpf() const { return _current_lpf; }

private:
    double _voltage_pu;
    double _current_pu;
    control::second_order_low_pass_filter<double> _voltage_filter;
    control::second_order_band_reject_filter<double> _current_notch_filter;
    control::first_order_low_pass_filter<double> _current_lpf;
};

/**
 * @brief Handles measurements for the machine side of the SFC.
 */
class sfc_machine_side_measurement {
public:
    explicit sfc_machine_side_measurement(double sampling_period);

    void update(const control::abc<double>& raw_voltages, const control::abc<double>& raw_currents,
                double machine_frequency_hz, const control::angle<control::zero_to_two_pi>& machine_angle,
                const sfc_per_unit_system& pu_system);

    void configure_voltage_filter(double cutoff_freq_hz, double damping_ratio = 0.707);
    void configure_current_filter(double cutoff_freq_hz, double damping_ratio = 0.707);

    const control::dq<double>& get_voltages_pu() const { return _voltages_pu; }
    const control::dq<double>& get_currents_pu() const { return _currents_pu; }

private:
    control::dq<double> _voltages_pu;
    control::dq<double> _currents_pu;
    control::second_order_low_pass_filter<control::abc<double>> _voltage_filter;
    control::second_order_low_pass_filter<control::abc<double>> _current_filter;
};

}  // namespace yeet::application::sfc
