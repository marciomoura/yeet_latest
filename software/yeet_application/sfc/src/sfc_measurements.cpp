#include "sfc/sfc_measurements.hpp"

namespace yeet::application::sfc {

// --- Grid-Side Measurements ---

void sfc_grid_side_measurements::update(sfc_per_unit_system::grid_side_bases const& bases,
                                        const control::abc<double>& raw_voltages,
                                        const control::abc<double>& raw_currents) {
    // Convert to per-unit using peak phase base values
    _voltage_pu = raw_voltages / bases.base_voltage_peak_phase_v;

    _current_pu = raw_currents / bases.base_current_peak_phase_a;
}

// --- DC-Link Measurements ---

void sfc_dc_link_measurements::update(sfc_per_unit_system::dc_link_bases const& bases, double raw_voltage,
                                      double raw_current) {
    // Convert to per-unit
    _voltage_pu = raw_voltage / bases.base_voltage_v;
    _current_pu = raw_current / bases.base_current_a;
}

// --- Machine-Side Measurements ---

void sfc_machine_side_measurements::update(sfc_per_unit_system::machine_side_bases const& bases,
                                           const control::abc<double>& raw_voltages,
                                           const control::abc<double>& raw_currents) {
    // Convert to per-unit using peak phase base values
    _voltage_pu = raw_voltages / bases.base_voltage_peak_phase_v;

    _current_pu = raw_currents / bases.base_current_peak_phase_a;
}

}  // namespace yeet::application::sfc