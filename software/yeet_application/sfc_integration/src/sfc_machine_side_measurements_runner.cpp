#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_measurements_runner::initialize() {
    set_parameters();
    set_parameter_callback();
}

void sfc_machine_side_measurements_runner::run() {
    _sfc_machine_side_measurements.update(_per_unit_system->get_machine_bases(), _inputs.machine_voltages_raw,
                                          _inputs.machine_currents_raw);

    const auto voltage_pu = _sfc_machine_side_measurements.get_voltage_pu();
    const auto current_pu = _sfc_machine_side_measurements.get_current_pu();

    _sfc_machine_side_filtering.update(voltage_pu, current_pu);

    _outputs.machine_voltage_alphabeta_pu = _sfc_machine_side_filtering.get_filtered_voltage_alphabeta_pu();
    _outputs.machine_voltage_magnitude_pu = _sfc_machine_side_filtering.get_filtered_voltage_magnitude_pu();

    _outputs.is_valid = _inputs.sensor_health_ok;
}

void sfc_machine_side_measurements_runner::stop() {}

void sfc_machine_side_measurements_runner::set_parameters() {
    // Configure filtering parameters (non-per-unit related)
    _sfc_machine_side_filtering.configure_voltage_filter(_parameters.voltage_lpf_cutoff);
    _sfc_machine_side_filtering.configure_current_filter(_parameters.current_lpf_cutoff);
    _sfc_machine_side_filtering.configure_voltage_magnitude_filter(_parameters.voltage_lpf_cutoff);
}

void sfc_machine_side_measurements_runner::set_parameter_callback() {
    // Register callbacks for non-per-unit parameters only
    _parameters.voltage_lpf_cutoff.register_callback([&]() { set_parameters(); });
    _parameters.current_lpf_cutoff.register_callback([&]() { set_parameters(); });
    _parameters.speed_lpf_cutoff.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration