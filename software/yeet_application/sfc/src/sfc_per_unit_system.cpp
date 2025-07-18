#include "sfc/sfc_per_unit_system.hpp"

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

sfc_per_unit_system::grid_side_bases sfc_per_unit_system::calculate_grid_side_base_values() const {
    grid_side_bases bases{};
    bases.base_voltage_peak_phase = _grid_voltage_ll_rms / sqrt_3 * sqrt_2;
    bases.base_current_peak_phase = (real_t(2.0) * sqrt_3 / pi) * _dc_link_current_rated;
    bases.base_impedance = bases.base_voltage_peak_phase / bases.base_current_peak_phase;
    bases.base_frequency = _grid_frequency;
    bases.base_angular_frequency = two_pi * _grid_frequency;
    bases.base_inductance = bases.base_impedance / bases.base_angular_frequency;
    return bases;
}

sfc_per_unit_system::dc_link_bases sfc_per_unit_system::calculate_dc_link_base_values() const {
    dc_link_bases bases{};
    bases.base_current = _dc_link_current_rated;
    bases.base_voltage = (3.0 * sqrt_2 / pi) * _grid_voltage_ll_rms;
    bases.base_impedance = bases.base_voltage / bases.base_current;
    // Base dc_link inductance is typically referenced to the line frequency.
    bases.base_inductance = bases.base_impedance / (two_pi * _grid_frequency);
    return bases;
}

sfc_per_unit_system::machine_side_bases sfc_per_unit_system::calculate_machine_side_base_values() const {
    machine_side_bases bases{};
    bases.base_power = _machine_power;
    bases.base_voltage_peak_phase = _machine_voltage_ll_rms / sqrt_3 * sqrt_2;
    bases.base_current_peak_phase = (2.0 / 3.0) * _machine_power / bases.base_voltage_peak_phase;
    bases.base_impedance =
        (3.0 / 2.0) * (bases.base_voltage_peak_phase * bases.base_voltage_peak_phase) / _machine_power;
    bases.base_frequency = _machine_frequency;
    bases.base_angular_frequency = two_pi * _machine_frequency;
    bases.base_inductance = bases.base_impedance / bases.base_angular_frequency;

    // Torque and Flux calculations
    const real_t pole_pairs = static_cast<real_t>(_machine_num_poles) / 2.0;
    const auto base_mechanical_speed = bases.base_angular_frequency / pole_pairs;
    bases.base_torque = _machine_power / base_mechanical_speed;
    bases.base_flux_linkage = bases.base_voltage_peak_phase / bases.base_angular_frequency;
    return bases;
}

void sfc_per_unit_system::calculate_base_values() {
    _grid_side = calculate_grid_side_base_values();
    _dc_link = calculate_dc_link_base_values();
    _machine_side = calculate_machine_side_base_values();
}

}  // namespace yeet::application::sfc