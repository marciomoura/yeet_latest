#pragma once

#include <array>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/lookup_table_1d.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Provides the machine-side inverter firing angle using a configurable 1D lookup table.
 *
 * This class implements a mapping from DC link current to the required inverter
 * firing angle (alpha_inv). It uses a fixed-size, no-allocation 1D lookup table.
 * The table can be configured at runtime to adapt the control response.
 *
 */
class sfc_machine_side_firing_angle_lookup_table {
public:
    // Define the dimensions for the lookup table at compile time
    static constexpr size_t NUM_POINTS = 6;
    static constexpr size_t SPEED_DELTA_NUM_POINTS = 3;

    // Default x-axis (DC link current in per-unit)
    static constexpr std::array<current_pu_t, NUM_POINTS> kDefaultDcCurrentAxisPu = {current_pu_t{0.0},
        current_pu_t{0.3}, current_pu_t{0.5}, current_pu_t{1.0}, current_pu_t{1.8}, current_pu_t{2.0}};

    // Default y-axis (inverter firing angle)
    static constexpr std::array<angle_wrapped, NUM_POINTS> kDefaultFiringAngle = {angle_wrapped::from_degrees(150.0),
        angle_wrapped::from_degrees(145.0), angle_wrapped::from_degrees(140.0), angle_wrapped::from_degrees(140.0),
        angle_wrapped::from_degrees(135.0), angle_wrapped::from_degrees(135.0)};

    // Default speed-based delta lookup table values
    // X-Axis (Machine speed in per-unit, 1.0 pu = 3000 rpm)
    // 0 rpm, 600 rpm (0.2 pu), 3150 rpm (1.05 pu)
    static constexpr std::array<speed_pu_t, SPEED_DELTA_NUM_POINTS> kDefaultSpeedAxisPu = {
        speed_pu_t{0.0}, speed_pu_t{0.2}, speed_pu_t{1.05}};

    // Y-Axis (Firing angle delta)
    static constexpr std::array<angle_wrapped, SPEED_DELTA_NUM_POINTS> kDefaultSpeedDelta = {
        angle_wrapped::from_degrees(0.0), angle_wrapped::from_degrees(0.0), angle_wrapped::from_degrees(-7.1)};

    /**
     * @brief Constructs the firing angle lookup table, initializing with default values.
     */
    sfc_machine_side_firing_angle_lookup_table();

    /**
     * @brief Configures the lookup table with new current and firing angle points.
     *
     * @param dc_current_axis_pu The array of DC link current breakpoints [p.u.]. Must be sorted.
     * @param firing_angle The corresponding array of firing angles.
     */
    void configure_table(const std::array<current_pu_t, NUM_POINTS>& dc_current_axis_pu,
        const std::array<angle_wrapped, NUM_POINTS>& firing_angle);

    /**
     * @brief Configures the speed-based delta lookup table.
     *
     * @param speed_axis_pu The array of machine speed breakpoints [p.u.]. Must be sorted.
     * @param delta_angle The corresponding array of firing angle deltas.
     */
    void configure_speed_delta_table(const std::array<speed_pu_t, SPEED_DELTA_NUM_POINTS>& speed_axis_pu,
        const std::array<angle_wrapped, SPEED_DELTA_NUM_POINTS>& delta_angle);

    /**
     * @brief Resets the component to a safe initial state.
     *
     * The firing angle output is reset to the first value in the default table,
     * which corresponds to the zero-current condition.
     */
    void reset();

    /**
     * @brief Updates the firing angle based on the DC link current and machine speed.
     *
     * @param dc_link_current The measured DC link current [p.u.].
     * @param machine_speed The measured machine speed [p.u.].
     */
    void update(current_pu_t dc_link_current, speed_pu_t machine_speed);

    /**
     * @brief Gets the last calculated inverter firing angle.
     *
     * @return The firing angle in degrees [90-180].
     */
    /**
     * @brief Gets the last calculated inverter firing angle.
     *
     * @return The firing angle.
     */
    [[nodiscard]] angle_wrapped get_firing_angle() const;

private:
    control::lookup_table_1d<real_t, NUM_POINTS> _lookup_table;
    control::lookup_table_1d<real_t, SPEED_DELTA_NUM_POINTS> _speed_delta_lookup_table;

    angle_wrapped _firing_angle{angle_wrapped::from_degrees(150.0)};
};

}  // namespace yeet::application::sfc