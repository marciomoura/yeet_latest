#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_measurements_runner::initialize() {
    set_parameters();
    set_parameter_callback();
}

void sfc_grid_side_measurements_runner::run() {
    _sfc_grid_side_measurements.update(_per_unit_system->get_grid_bases(), _inputs.line_voltages_raw,
                                       _inputs.line_currents_raw);

    _outputs.grid_voltage_alphabeta_pu = _sfc_grid_side_measurements.get_voltage_alphabeta_pu();
    _outputs.grid_voltage_magnitude_pu = _sfc_grid_side_measurements.get_voltage_magnitude_pu();
    _outputs.is_valid = _inputs.sensor_health_ok;
}

void sfc_grid_side_measurements_runner::stop() {}

void sfc_grid_side_measurements_runner::set_parameters() {
    // No parameters to set - per-unit system is managed externally
}

void sfc_grid_side_measurements_runner::set_parameter_callback() {
    // Register callbacks for non-per-unit parameters only
    // (No base value parameters to register callbacks for)
}

}  // namespace yeet::application::sfc::integration