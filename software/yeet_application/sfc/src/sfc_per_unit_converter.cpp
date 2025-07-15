#include "sfc/sfc_per_unit_converter.hpp"

#include "yeet_control/per_unit.hpp"

namespace yeet::application::sfc {

sfc_per_unit_converter::sfc_per_unit_converter(const sfc_per_unit_system& pu_system) : _pu_system(pu_system) {}

void sfc_per_unit_converter::update(const control::abc<double>& grid_voltages_si,
                                    const control::abc<double>& grid_currents_si,
                                    const control::abc<double>& machine_voltages_si,
                                    const control::abc<double>& machine_currents_si, double dc_link_current_si) {
    // Convert grid-side measurements to per-unit abc
    _grid_voltages_pu = control::convert_si_to_pu(grid_voltages_si, _pu_system.get_base_voltage_peak());
    _grid_currents_pu = control::convert_si_to_pu(grid_currents_si, _pu_system.get_base_current_peak());

    // Convert machine-side measurements to per-unit abc
    _machine_voltages_pu = control::convert_si_to_pu(machine_voltages_si, _pu_system.get_base_voltage_peak());
    _machine_currents_pu = control::convert_si_to_pu(machine_currents_si, _pu_system.get_base_current_peak());

    // Convert DC-link measurements
    _dc_link_current_pu = control::convert_si_to_pu(dc_link_current_si, _pu_system.get_base_dc_current());
}

}  // namespace yeet::application::sfc