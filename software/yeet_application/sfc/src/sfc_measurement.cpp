#include "sfc/sfc_measurement.hpp"

#include "yeet_control/per_unit.hpp"

namespace yeet::application::sfc {

sfc_grid_side_measurement::sfc_grid_side_measurement(double sampling_period)
    : _voltage_filter(sampling_period), _current_filter(sampling_period) {}

void sfc_grid_side_measurement::configure_filters(double voltage_filter_cutoff_freq_hz,
                                                  double current_filter_cutoff_freq_hz, double damping_ratio) {
    _voltage_filter.configure(voltage_filter_cutoff_freq_hz, damping_ratio);
    _current_filter.configure(current_filter_cutoff_freq_hz, damping_ratio);
}

void sfc_grid_side_measurement::update(const control::abc<double>& raw_voltages,
                                       const control::abc<double>& raw_currents,
                                       const control::angle<control::zero_to_two_pi>& grid_angle,
                                       const sfc_per_unit_system& pu_system) {
    // Filter raw signals
    auto filtered_voltages = _voltage_filter.update(raw_voltages);
    auto filtered_currents = _current_filter.update(raw_currents);

    // Convert to per-unit

    auto voltages_pu_abc = control::convert_si_to_pu(filtered_voltages, pu_system.get_base_voltage_peak());
    auto currents_pu_abc = control::convert_si_to_pu(filtered_currents, pu_system.get_base_current_peak());

    // Convert to dq frame
    _voltages_pu = voltages_pu_abc.to_dq(grid_angle);
    _currents_pu = currents_pu_abc.to_dq(grid_angle);
}

sfc_dc_link_measurement::sfc_dc_link_measurement(double sampling_period)
    : _voltage_filter(sampling_period), _current_notch_filter(sampling_period), _current_lpf(sampling_period) {}

void sfc_dc_link_measurement::configure_filters(double voltage_filter_cutoff_freq_hz, double current_ripple_freq_hz) {
    _voltage_filter.configure(voltage_filter_cutoff_freq_hz, 0.707);

    // Configure the notch filter to remove the dominant ripple frequency.
    // The bandwidth of the notch is set to a fraction of the ripple frequency.
    double notch_bandwidth_hz = current_ripple_freq_hz / 10.0;
    _current_notch_filter.configure(current_ripple_freq_hz, notch_bandwidth_hz);

    // Configure a gentle LPF to remove any remaining high-frequency noise.
    // The cutoff is set well above the ripple frequency.
    _current_lpf.configure(current_ripple_freq_hz * 2.0);
}

void sfc_dc_link_measurement::update(double raw_voltage, double raw_current, const sfc_per_unit_system& pu_system) {
    // Filter raw signals
    double filtered_voltage = _voltage_filter.update(raw_voltage);

    // The current signal is first passed through a notch filter, then an LPF.
    double notched_current = _current_notch_filter.update(raw_current);
    double filtered_current = _current_lpf.update(notched_current);

    // Convert to per-unit
    _voltage_pu = control::convert_si_to_pu(filtered_voltage, pu_system.get_base_dc_voltage());
    _current_pu = control::convert_si_to_pu(filtered_current, pu_system.get_base_dc_current());
}

sfc_machine_side_measurement::sfc_machine_side_measurement(double sampling_period)
    : _voltage_filter(sampling_period), _current_filter(sampling_period) {}

void sfc_machine_side_measurement::update(const control::abc<double>& raw_voltages,
                                          const control::abc<double>& raw_currents, double machine_frequency_hz,
                                          const control::angle<control::zero_to_two_pi>& machine_angle,
                                          const sfc_per_unit_system& pu_system) {
    // For adaptive filtering, the filter parameters must be updated based on machine speed.
    // This is handled by the user of this class by calling the configure methods.
    // A simple adaptive strategy is to scale the cutoff frequency with the machine frequency.
    if (machine_frequency_hz > 1.0) {  // Avoid configuring with 0 Hz

        // TODO: Use a more sophisticated adaptive strategy based on the application requirements.
        //       Maybe use a lookup table or a more complex model.
        double voltage_cutoff = machine_frequency_hz * 10.0;
        double current_cutoff = machine_frequency_hz * 5.0;
        configure_voltage_filter(voltage_cutoff);
        configure_current_filter(current_cutoff);
    }

    // Filter raw signals
    auto filtered_voltages = _voltage_filter.update(raw_voltages);
    auto filtered_currents = _current_filter.update(raw_currents);

    // Convert to per-unit
    auto voltages_pu_abc = control::convert_si_to_pu(filtered_voltages, pu_system.get_base_voltage_peak());
    auto currents_pu_abc = control::convert_si_to_pu(filtered_currents, pu_system.get_base_current_peak());

    // Convert to dq frame
    _voltages_pu = voltages_pu_abc.to_dq(machine_angle);
    _currents_pu = currents_pu_abc.to_dq(machine_angle);
}

void sfc_machine_side_measurement::configure_voltage_filter(double cutoff_freq_hz, double damping_ratio) {
    _voltage_filter.configure(cutoff_freq_hz, damping_ratio);
}

void sfc_machine_side_measurement::configure_current_filter(double cutoff_freq_hz, double damping_ratio) {
    _current_filter.configure(cutoff_freq_hz, damping_ratio);
}

}  // namespace yeet::application::sfc
