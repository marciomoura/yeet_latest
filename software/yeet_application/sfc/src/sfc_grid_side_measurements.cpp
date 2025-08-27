#include "sfc/sfc_grid_side_measurements.hpp"

namespace yeet::application::sfc {

void sfc_grid_side_measurements::update(sfc_per_unit_system::grid_side_bases const& bases,
    const abc<voltage_t>& raw_voltages,
    const abc<current_t>& raw_currents)
{
    // Convert to per-unit using peak phase base values
    _voltage_pu = to_pu(raw_voltages, bases.base_voltage_peak_phase);
    _current_pu = to_pu(raw_currents, bases.base_current_peak_phase);
}

}  // namespace yeet::application::sfc