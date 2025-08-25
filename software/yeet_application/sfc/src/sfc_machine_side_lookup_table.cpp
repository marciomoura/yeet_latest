#include "sfc/sfc_machine_side_lookup_table.hpp"

#include <algorithm>
#include <cassert>

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

sfc_machine_side_firing_angle_lookup_table::sfc_machine_side_firing_angle_lookup_table()
{
    configure_table(kDefaultDcCurrentAxisPu, kDefaultFiringAngleDeg);
    reset();
}

void sfc_machine_side_firing_angle_lookup_table::configure_table(
    const std::array<current_pu_t, NUM_POINTS>& dc_current_axis_pu,
    const std::array<real_t, NUM_POINTS>& firing_angle_deg)
{
    // Runtime check for sorted axis (in debug builds only)
    assert(std::is_sorted(dc_current_axis_pu.begin(), dc_current_axis_pu.end()) && "DC current axis must be sorted.");

    std::array<real_t, NUM_POINTS> converted_dc_current_axis{};
    std::transform(dc_current_axis_pu.begin(), dc_current_axis_pu.end(), converted_dc_current_axis.begin(),
        [](current_pu_t val) { return val.value(); });

    _lookup_table.configure(converted_dc_current_axis, firing_angle_deg);
}

void sfc_machine_side_firing_angle_lookup_table::reset() { _firing_angle_deg = kDefaultFiringAngleDeg[0]; }

void sfc_machine_side_firing_angle_lookup_table::update(current_pu_t dc_link_current_pu)
{
    _firing_angle_deg = _lookup_table.get_value(dc_link_current_pu.value());
}

real_t sfc_machine_side_firing_angle_lookup_table::get_firing_angle_deg() const { return _firing_angle_deg; }

angle_t sfc_machine_side_firing_angle_lookup_table::get_firing_angle_rad() const
{
    return deg_to_rad(_firing_angle_deg);
}

}  // namespace yeet::application::sfc