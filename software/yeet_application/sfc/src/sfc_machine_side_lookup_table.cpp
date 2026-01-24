#include "sfc/sfc_machine_side_lookup_table.hpp"

#include <algorithm>
#include <cassert>

#include "yeet_control/constants.hpp"

namespace yeet::application::sfc {

sfc_machine_side_firing_angle_lookup_table::sfc_machine_side_firing_angle_lookup_table()
{
    configure_table(kDefaultDcCurrentAxisPu, kDefaultFiringAngle);
    configure_speed_delta_table(kDefaultSpeedAxisPu, kDefaultSpeedDelta);
    reset();
}

void sfc_machine_side_firing_angle_lookup_table::configure_table(
    const std::array<current_pu_t, NUM_POINTS>& dc_current_axis_pu,
    const std::array<angle_wrapped, NUM_POINTS>& firing_angle)
{
    // Runtime check for sorted axis (in debug builds only)
    assert(std::is_sorted(dc_current_axis_pu.begin(), dc_current_axis_pu.end()) && "DC current axis must be sorted.");

    std::array<real_t, NUM_POINTS> converted_dc_current_axis{};
    std::transform(dc_current_axis_pu.begin(), dc_current_axis_pu.end(), converted_dc_current_axis.begin(),
        [](current_pu_t val) { return val.value(); });

    std::array<real_t, NUM_POINTS> converted_firing_angle{};
    std::transform(firing_angle.begin(), firing_angle.end(), converted_firing_angle.begin(),
        [](angle_wrapped val) { return static_cast<angle_t>(val).value(); });

    _lookup_table.configure(converted_dc_current_axis, converted_firing_angle);
}

void sfc_machine_side_firing_angle_lookup_table::configure_speed_delta_table(
    const std::array<speed_pu_t, SPEED_DELTA_NUM_POINTS>& speed_axis_pu,
    const std::array<angle_wrapped, SPEED_DELTA_NUM_POINTS>& delta_angle)
{
    // Runtime check for sorted axis (in debug builds only)
    assert(std::is_sorted(speed_axis_pu.begin(), speed_axis_pu.end()) && "Speed axis must be sorted.");

    std::array<real_t, SPEED_DELTA_NUM_POINTS> converted_speed_axis{};
    std::transform(speed_axis_pu.begin(), speed_axis_pu.end(), converted_speed_axis.begin(),
        [](speed_pu_t val) { return val.value(); });

    std::array<real_t, SPEED_DELTA_NUM_POINTS> converted_delta_angle{};
    for (size_t i = 0; i < SPEED_DELTA_NUM_POINTS; ++i) {
        real_t rad = static_cast<real_t>(static_cast<angle_t>(delta_angle[i]).value());
        // Important, angle_wrapped wraps around [0, 2pi], but for deltas we need [-pi, pi] because the delta_angle is
        // Hence, mapping [0, 2pi] to [-pi, pi] is needed to correctly interpolante the angles
        if (rad > yeet::pi) {
            rad -= yeet::two_pi;
        }
        converted_delta_angle[i] = rad;
    }

    _speed_delta_lookup_table.configure(converted_speed_axis, converted_delta_angle);
}

void sfc_machine_side_firing_angle_lookup_table::reset() { _firing_angle = kDefaultFiringAngle[0]; }

void sfc_machine_side_firing_angle_lookup_table::update(current_pu_t dc_link_current, speed_pu_t machine_speed)
{
    const angle_t base_angle_rad = angle_t{_lookup_table.get_value(dc_link_current.value())};
    const angle_t delta_angle_rad = angle_t{_speed_delta_lookup_table.get_value(machine_speed.value())};

    _firing_angle = angle_wrapped::from_radians(base_angle_rad) + angle_wrapped::from_radians(delta_angle_rad);
}

angle_wrapped sfc_machine_side_firing_angle_lookup_table::get_firing_angle() const { return _firing_angle; }

}  // namespace yeet::application::sfc