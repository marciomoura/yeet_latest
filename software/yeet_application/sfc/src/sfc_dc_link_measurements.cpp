#include "sfc/sfc_dc_link_measurements.hpp"

namespace yeet::application::sfc {

void sfc_dc_link_measurements::update(sfc_per_unit_system const& bases, abc<current_pu_t> const& grid_current_abc)
{
    auto const dc_link_current_grid_side_base = real_t{0.5} * sum(grid_current_abc.absolute());

    _current_pu = max(bases.reflect_grid_to_dc_link_current_pu(dc_link_current_grid_side_base) - current_pu_t{0.05},
        current_pu_t{0.0});
}

}  // namespace yeet::application::sfc