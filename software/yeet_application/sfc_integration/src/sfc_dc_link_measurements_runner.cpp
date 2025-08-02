#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"

#include "sfc/sfc_per_unit_system.hpp"

namespace yeet::application::sfc::integration {

void sfc_dc_link_measurements_runner::initialize() {
    set_parameters();
    set_parameter_callback();
}

void sfc_dc_link_measurements_runner::run() {
    _sfc_dc_link_measurements.update(_dc_link_bases, _inputs.dc_voltage, _inputs.dc_current);

    _outputs.dc_voltage_pu = _sfc_dc_link_measurements.get_voltage_pu();
    _outputs.dc_current_pu = _sfc_dc_link_measurements.get_current_pu();
    _outputs.is_valid = _inputs.sensor_health_ok;
}

void sfc_dc_link_measurements_runner::stop() {}

void sfc_dc_link_measurements_runner::set_parameters() {
    _dc_link_bases.voltage_base = _parameters.voltage_base;
    _dc_link_bases.current_base = _parameters.current_base;
    _dc_link_bases.power_base = _parameters.power_base;
}

void sfc_dc_link_measurements_runner::set_parameter_callback() {
    _parameters.voltage_base.register_callback([&]() { set_parameters(); });
    _parameters.current_base.register_callback([&]() { set_parameters(); });
    _parameters.power_base.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration