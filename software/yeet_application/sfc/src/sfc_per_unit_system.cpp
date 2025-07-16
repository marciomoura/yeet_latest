#include "sfc/sfc_per_unit_system.hpp"

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

sfc_per_unit_system::grid_side_bases sfc_per_unit_system::calculate_grid_side_base_values() const {
    grid_side_bases bases{};
    bases.base_voltage_peak_phase_v = _grid_voltage_ll_rms_v / yeet::control::sqrt_3 * yeet::control::sqrt_2;
    bases.base_current_peak_phase_a =
        (2.0 * yeet::control::sqrt_3 / yeet::control::pi<double>)*_dc_link_current_rated_a;
    bases.base_impedance_ohm = bases.base_voltage_peak_phase_v / bases.base_current_peak_phase_a;
    bases.base_frequency_hz = _grid_frequency_hz;
    bases.base_angular_frequency_rad_s = yeet::control::two_pi<double> * _grid_frequency_hz;
    bases.base_inductance_h = bases.base_impedance_ohm / bases.base_angular_frequency_rad_s;
    return bases;
}

sfc_per_unit_system::dc_link_bases sfc_per_unit_system::calculate_dc_link_base_values() const {
    dc_link_bases bases{};
    bases.base_current_a = _dc_link_current_rated_a;
    bases.base_voltage_v = (3.0 * yeet::control::sqrt_2 / yeet::control::pi<double>)*_grid_voltage_ll_rms_v;
    bases.base_impedance_ohm = bases.base_voltage_v / bases.base_current_a;
    // Base dc_link inductance is typically referenced to the line frequency.
    bases.base_inductance_h = bases.base_impedance_ohm / (yeet::control::two_pi<double> * _grid_frequency_hz);
    return bases;
}

sfc_per_unit_system::machine_side_bases sfc_per_unit_system::calculate_machine_side_base_values() const {
    machine_side_bases bases{};
    bases.base_power_va = _machine_power_va;
    bases.base_voltage_peak_phase_v = _machine_voltage_ll_rms_v / yeet::control::sqrt_3 * yeet::control::sqrt_2;
    bases.base_current_peak_phase_a = (2.0 / 3.0) * _machine_power_va / bases.base_voltage_peak_phase_v;
    bases.base_impedance_ohm =
        (3.0 / 2.0) * (bases.base_voltage_peak_phase_v * bases.base_voltage_peak_phase_v) / _machine_power_va;
    bases.base_frequency_hz = _machine_frequency_hz;
    bases.base_angular_frequency_rad_s = yeet::control::two_pi<double> * _machine_frequency_hz;
    bases.base_inductance_h = bases.base_impedance_ohm / bases.base_angular_frequency_rad_s;

    // Torque and Flux calculations
    const double pole_pairs = static_cast<double>(_machine_num_poles) / 2.0;
    const double base_mechanical_speed_rad_s = bases.base_angular_frequency_rad_s / pole_pairs;
    bases.base_torque_nm = _machine_power_va / base_mechanical_speed_rad_s;
    bases.base_flux_linkage_vs_rad = bases.base_voltage_peak_phase_v / bases.base_angular_frequency_rad_s;
    return bases;
}

void sfc_per_unit_system::calculate_base_values() {
    _grid_side = calculate_grid_side_base_values();
    _dc_link = calculate_dc_link_base_values();
    _machine_side = calculate_machine_side_base_values();
}

}  // namespace yeet::application::sfc