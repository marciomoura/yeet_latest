#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_measurements_runner::initialize() {
    set_parameters();
    set_parameter_callback();
}

void sfc_grid_side_measurements_runner::run() {
    _sfc_grid_side_measurements.update(_grid_side_bases, _inputs.line_voltages, _inputs.line_currents);

    _outputs.grid_voltage_alphabeta_pu = _sfc_grid_side_measurements.get_voltage_alphabeta_pu();
    _outputs.grid_voltage_magnitude_pu = _sfc_grid_side_measurements.get_voltage_magnitude_pu();
    _outputs.is_valid = _inputs.sensor_health_ok;
}

void sfc_grid_side_measurements_runner::stop() {}

void sfc_grid_side_measurements_runner::set_parameters() {
    _grid_side_bases.voltage_base = _parameters.voltage_base;
    _grid_side_bases.current_base = _parameters.current_base;
}

void sfc_grid_side_measurements_runner::set_parameter_callback() {
    _parameters.voltage_base.register_callback([&]() { set_parameters(); });
    _parameters.current_base.register_callback([&]() { set_parameters(); });
}

}