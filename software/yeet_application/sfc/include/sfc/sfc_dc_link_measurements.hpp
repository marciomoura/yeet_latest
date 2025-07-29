#pragma once

#include "sfc_per_unit_system.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles measurement conversion for the DC link of the SFC.
 *
 * This class takes raw DC link voltage and current measurements and
 * converts them into the per-unit system defined for the DC link.
 * It does not perform any filtering.
 */
class sfc_dc_link_measurements {
public:
    /**
     * @brief Updates the per-unit values from new raw measurements.
     * @param bases The per-unit base values for the DC link.
     * @param raw_voltage The raw DC link voltage measurement [V].
     * @param raw_current The raw DC link current measurement [A].
     */
    void update(sfc_per_unit_system::dc_link_bases const& bases, voltage_t raw_voltage, current_t raw_current);

    voltage_pu_t get_voltage_pu() const { return _voltage_pu; }
    current_pu_t get_current_pu() const { return _current_pu; }

private:
    voltage_pu_t _voltage_pu{};
    current_pu_t _current_pu{};
};

}  // namespace yeet::application::sfc