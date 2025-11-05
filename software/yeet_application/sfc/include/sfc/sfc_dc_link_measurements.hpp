#pragma once

#include "sfc_per_unit_system.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles measurement conversion for the DC link of the SFC.
 *
 * This class utilizes the grid-side converter current and calculates the
 * DC link current in the per-unit system.
 */
class sfc_dc_link_measurements {
public:
    /**
     * @brief Updates the per-unit values from new raw measurements.
     * @param bases The per-unit base values.
     * @param grid_current_abc The per-unit grid-side current [pu]
     */
    void update(sfc_per_unit_system const& bases, abc<current_pu_t> const& grid_current_abc);

    current_pu_t get_current_pu() const { return _current_pu; }

private:
    current_pu_t _current_pu{};
};

}  // namespace yeet::application::sfc