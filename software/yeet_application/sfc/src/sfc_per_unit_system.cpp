#include "sfc/sfc_per_unit_system.hpp"

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

sfc_per_unit_system::grid_side_bases sfc_per_unit_system::calculate_grid_side_base_values() const
{
    const voltage_t voltage_peak_phase = _grid_voltage_ll_rms / sqrt_3 * sqrt_2;
    const current_t current_peak_phase = (real_t(2.0) * sqrt_3 / pi) * _dc_link_current_rated;
    const impedance_t impedance = voltage_peak_phase / current_peak_phase;
    const angular_frequency_t angular_frequency = two_pi * _grid_frequency;
    const inductance_t inductance = impedance / angular_frequency;

    return grid_side_bases{precomputed_divisor<voltage_t>(voltage_peak_phase),
        precomputed_divisor<current_t>(current_peak_phase), precomputed_divisor<impedance_t>(impedance),
        precomputed_divisor<frequency_t>(_grid_frequency), precomputed_divisor<angular_frequency_t>(angular_frequency),
        precomputed_divisor<inductance_t>(inductance)};
}

sfc_per_unit_system::dc_link_bases sfc_per_unit_system::calculate_dc_link_base_values() const
{
    const voltage_t voltage = (3.0 * sqrt_2 / pi) * _grid_voltage_ll_rms;
    const impedance_t impedance = voltage / _dc_link_current_rated;
    const inductance_t inductance = impedance / (two_pi * _grid_frequency);

    return dc_link_bases{
        precomputed_divisor<voltage_t>(voltage),                 // base_voltage
        precomputed_divisor<current_t>(_dc_link_current_rated),  // base_current
        precomputed_divisor<impedance_t>(impedance),             // base_impedance
        precomputed_divisor<inductance_t>(inductance)            // base_inductance
    };
}

sfc_per_unit_system::machine_side_bases sfc_per_unit_system::calculate_machine_side_base_values() const
{
    const voltage_t voltage_peak_phase = _machine_voltage_ll_rms / sqrt_3 * sqrt_2;
    const current_t current_peak_phase = (2.0 / 3.0) * _machine_power / voltage_peak_phase;
    const impedance_t impedance = (3.0 / 2.0) * (voltage_peak_phase * voltage_peak_phase) / _machine_power;
    const angular_frequency_t angular_frequency = two_pi * _machine_frequency;
    const inductance_t inductance = impedance / angular_frequency;

    // Torque and Flux calculations
    const real_t pole_pairs = static_cast<real_t>(_machine_num_poles) / 2.0;
    const angular_frequency_t base_mechanical_speed = angular_frequency / pole_pairs;
    const torque_t torque = _machine_power / base_mechanical_speed;
    const flux_t flux_linkage = voltage_peak_phase / angular_frequency;

    return machine_side_bases{precomputed_divisor<power_t>(_machine_power),
        precomputed_divisor<voltage_t>(voltage_peak_phase), precomputed_divisor<current_t>(current_peak_phase),
        precomputed_divisor<impedance_t>(impedance), precomputed_divisor<frequency_t>(_machine_frequency),
        precomputed_divisor<angular_frequency_t>(angular_frequency), precomputed_divisor<inductance_t>(inductance),
        precomputed_divisor<torque_t>(torque), precomputed_divisor<flux_t>(flux_linkage)};
}

void sfc_per_unit_system::calculate_base_values()
{
    _grid_side = calculate_grid_side_base_values();
    _dc_link = calculate_dc_link_base_values();
    _machine_side = calculate_machine_side_base_values();
}

}  // namespace yeet::application::sfc