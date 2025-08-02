#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

void sfc_dc_link_measurements_runner::initialize() {
    set_parameters();
    set_parameter_callback();
}

void sfc_dc_link_measurements_runner::run() {
    _sfc_dc_link_measurements.update(_per_unit_system->get_dc_link_bases(), _inputs.dc_voltage, _inputs.dc_current);

    _outputs.dc_voltage_pu = _sfc_dc_link_measurements.get_voltage_pu();
    _outputs.dc_current_pu = _sfc_dc_link_measurements.get_current_pu();
    _outputs.is_valid = _inputs.sensor_health_ok;
}

void sfc_dc_link_measurements_runner::stop() {}

void sfc_dc_link_measurements_runner::set_parameters() {
    // No parameters to set - per-unit system is managed externally
}

void sfc_dc_link_measurements_runner::set_parameter_callback() {
    // Register callbacks for non-per-unit parameters only
    // (No base value parameters to register callbacks for)
}

}  // namespace yeet::application::sfc::integration