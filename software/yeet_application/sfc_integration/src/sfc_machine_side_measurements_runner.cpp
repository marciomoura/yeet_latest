#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_measurements_runner::initialize() {
    set_parameters();
    set_parameter_callback();
}

void sfc_machine_side_measurements_runner::run() {
    _sfc_machine_side_measurements.update(_machine_side_bases, _inputs.machine_voltages, _inputs.machine_currents);

    const auto voltage_pu = _sfc_machine_side_measurements.get_voltage_pu();
    const auto current_pu = _sfc_machine_side_measurements.get_current_pu();

    _sfc_machine_side_filtering.update(voltage_pu, current_pu);

    _outputs.machine_voltage_alphabeta_pu = _sfc_machine_side_filtering.get_filtered_voltage_alphabeta_pu();
    _outputs.machine_voltage_magnitude_pu = _sfc_machine_side_filtering.get_filtered_voltage_magnitude_pu();

    _outputs.is_valid = _inputs.sensor_health_ok;
}

void sfc_machine_side_measurements_runner::stop() {}

void sfc_machine_side_measurements_runner::set_parameters() {
    _machine_side_bases.voltage_base = _parameters.voltage_base;
    _machine_side_bases.current_base = _parameters.current_base;
    _machine_side_bases.speed_base = _parameters.speed_base;
    _sfc_machine_side_filtering.configure_voltage_filter(_parameters.filter_cutoff_frequency);
    _sfc_machine_side_filtering.configure_current_filter(_parameters.filter_cutoff_frequency);
    _sfc_machine_side_filtering.configure_voltage_magnitude_filter(_parameters.filter_cutoff_frequency);
}

void sfc_machine_side_measurements_runner::set_parameter_callback() {
    _parameters.voltage_base.register_callback([&]() { set_parameters(); });
    _parameters.current_base.register_callback([&]() { set_parameters(); });
    _parameters.speed_base.register_callback([&]() { set_parameters(); });
    _parameters.filter_cutoff_frequency.register_callback([&]() { set_parameters(); });
}

}