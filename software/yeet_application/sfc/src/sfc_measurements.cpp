#include "sfc/sfc_measurements.hpp"

namespace yeet::application::sfc {

// --- Grid-Side Measurements ---

void sfc_grid_side_measurements::update(sfc_per_unit_system::grid_side_bases const& bases,
                                        const abc<voltage_t>& raw_voltages, const abc<current_t>& raw_currents) {
    // Convert to per-unit using peak phase base values
    _voltage_pu = to_pu(raw_voltages, bases.base_voltage_peak_phase);
    _current_pu = to_pu(raw_currents, bases.base_current_peak_phase);

    // Transform voltage to αβ frame and calculate magnitude
    _voltage_alphabeta_pu = _voltage_pu.to_alphabeta();
    // _voltage_magnitude_pu = _voltage_alphabeta_pu.magnitude();
}

// --- DC-Link Measurements ---

void sfc_dc_link_measurements::update(sfc_per_unit_system::dc_link_bases const& bases, voltage_t raw_voltage,
                                      current_t raw_current) {
    // Convert to per-unit
    _voltage_pu = to_pu(raw_voltage, bases.base_voltage);
    _current_pu = to_pu(raw_current, bases.base_current);
}

// --- Machine-Side Measurements ---

void sfc_machine_side_measurements::update(sfc_per_unit_system::machine_side_bases const& bases,
                                           const abc<voltage_t>& raw_voltages, const abc<current_t>& raw_currents) {
    // Convert to per-unit using peak phase base values
    _voltage_pu = to_pu(raw_voltages, bases.base_voltage_peak_phase);
    _current_pu = to_pu(raw_currents, bases.base_current_peak_phase);

    // Transform voltage to αβ frame and calculate magnitude
    _voltage_alphabeta_pu = _voltage_pu.to_alphabeta();
    // _voltage_magnitude_pu = _voltage_alphabeta_pu.magnitude();
}

}  // namespace yeet::application::sfc