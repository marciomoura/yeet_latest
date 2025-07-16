#include "sfc/sfc_filtering.hpp"

namespace yeet::application::sfc {

sfc_grid_side_filtering::sfc_grid_side_filtering(double sampling_period)
    : _voltage_filter(sampling_period), _current_filter(sampling_period), _voltage_magnitude_filter(sampling_period) {}

void sfc_grid_side_filtering::configure(double voltage_filter_cutoff_freq_hz, double current_filter_cutoff_freq_hz,
                                        double damping_ratio) {
    _voltage_filter.configure(voltage_filter_cutoff_freq_hz, damping_ratio);
    _current_filter.configure(current_filter_cutoff_freq_hz, damping_ratio);
}

void sfc_grid_side_filtering::configure_voltage_magnitude_filter(double cutoff_freq_hz, double damping_ratio) {
    _voltage_magnitude_filter.configure(cutoff_freq_hz, damping_ratio);
}

void sfc_grid_side_filtering::update(const control::abc<double>& voltage_pu, const control::abc<double>& current_pu) {
    _filtered_voltages_pu = _voltage_filter.update(voltage_pu);
    _filtered_currents_pu = _current_filter.update(current_pu);

    // Transform filtered voltage to αβ frame and calculate magnitude
    _filtered_voltage_alphabeta_pu = _filtered_voltages_pu.to_alphabeta();
    const double unfiltered_magnitude = voltage_pu.to_alphabeta().magnitude();
    _filtered_voltage_magnitude_pu = _voltage_magnitude_filter.update(unfiltered_magnitude);
}

sfc_dc_link_filtering::sfc_dc_link_filtering(double sampling_period)
    : _voltage_filter(sampling_period), _current_notch_filter(sampling_period), _current_lpf(sampling_period) {}

void sfc_dc_link_filtering::configure(double voltage_filter_cutoff_freq_hz, double current_ripple_freq_hz) {
    _voltage_filter.configure(voltage_filter_cutoff_freq_hz, 0.707);
    double notch_bandwidth_hz = current_ripple_freq_hz / 10.0;
    _current_notch_filter.configure(current_ripple_freq_hz, notch_bandwidth_hz);
    _current_lpf.configure(current_ripple_freq_hz / 10.0, 0.707);
}

void sfc_dc_link_filtering::update(double voltage_pu, double current_pu) {
    _filtered_voltage_pu = _voltage_filter.update(voltage_pu);
    double notched_current = _current_notch_filter.update(current_pu);
    _filtered_current_pu = _current_lpf.update(notched_current);
}

sfc_machine_side_filtering::sfc_machine_side_filtering(double sampling_period)
    : _voltage_filter(sampling_period), _current_filter(sampling_period), _voltage_magnitude_filter(sampling_period) {}

void sfc_machine_side_filtering::update(const control::abc<double>& voltage_pu, const control::abc<double>& current_pu,
                                        double machine_frequency_hz) {
    if (machine_frequency_hz > 1.0) {
        double voltage_cutoff = machine_frequency_hz * 10.0;
        double current_cutoff = machine_frequency_hz * 5.0;
        configure_voltage_filter(voltage_cutoff);
        configure_current_filter(current_cutoff);
    }
    _filtered_voltages_pu = _voltage_filter.update(voltage_pu);
    _filtered_currents_pu = _current_filter.update(current_pu);

    // Transform filtered voltage to αβ frame and calculate magnitude
    _filtered_voltage_alphabeta_pu = _filtered_voltages_pu.to_alphabeta();
    const double unfiltered_magnitude = voltage_pu.to_alphabeta().magnitude();
    _filtered_voltage_magnitude_pu = _voltage_magnitude_filter.update(unfiltered_magnitude);
}

void sfc_machine_side_filtering::configure_voltage_filter(double cutoff_freq_hz, double damping_ratio) {
    _voltage_filter.configure(cutoff_freq_hz, damping_ratio);
}

void sfc_machine_side_filtering::configure_current_filter(double cutoff_freq_hz, double damping_ratio) {
    _current_filter.configure(cutoff_freq_hz, damping_ratio);
}

void sfc_machine_side_filtering::configure_voltage_magnitude_filter(double cutoff_freq_hz, double damping_ratio) {
    _voltage_magnitude_filter.configure(cutoff_freq_hz, damping_ratio);
}

}  // namespace yeet::application::sfc