#include "sfc/sfc_dc_link_measurements.hpp"

namespace yeet::application::sfc {

void sfc_dc_link_measurements::update(sfc_per_unit_system::dc_link_bases const& bases, voltage_t raw_voltage,
                                      current_t raw_current) {
    // Convert to per-unit
    _voltage_pu = to_pu(raw_voltage, bases.base_voltage);
    _current_pu = to_pu(raw_current, bases.base_current);
}

}  // namespace yeet::application::sfc