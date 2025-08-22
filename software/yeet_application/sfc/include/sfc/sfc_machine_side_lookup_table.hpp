#pragma once

#include <array>
#include <cassert>

#include "yeet_control/constants.hpp"
#include "yeet_control/lookup_table_1d.hpp"

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

    // Default x-axis (DC link current in per-unit)
    static constexpr std::array<real_t, NUM_POINTS> kDefaultDcCurrentAxisPu = {0.0, 0.3, 0.5, 1.0, 1.88, 2.0};

    // Default y-axis (inverter firing angle in degrees)
    static constexpr std::array<real_t, NUM_POINTS> kDefaultFiringAngleDeg = {150.0, 150.0, 150.0, 140.0, 130.0, 120.0};

    /**
     * @brief Constructs the firing angle lookup table, initializing with default values.
     */
    sfc_machine_side_firing_angle_lookup_table()
    {
        configure_table(kDefaultDcCurrentAxisPu, kDefaultFiringAngleDeg);
        reset();
    }

    /**
     * @brief Configures the lookup table with new current and firing angle points.
     *
     * @param dc_current_axis_pu The array of DC link current breakpoints [p.u.]. Must be sorted.
     * @param firing_angle_deg The corresponding array of firing angles [degrees].
     */
    void configure_table(const std::array<real_t, NUM_POINTS>& dc_current_axis_pu,
        const std::array<real_t, NUM_POINTS>& firing_angle_deg)
    {
        // Runtime check for sorted axis (in debug builds only)
        assert(
            std::is_sorted(dc_current_axis_pu.begin(), dc_current_axis_pu.end()) && "DC current axis must be sorted.");
        _lookup_table.configure(dc_current_axis_pu, firing_angle_deg);
    }

    /**
     * @brief Resets the component to a safe initial state.
     *
     * The firing angle output is reset to the first value in the default table,
     * which corresponds to the zero-current condition.
     */
    void reset() { _firing_angle_deg = kDefaultFiringAngleDeg[0]; }

    /**
     * @brief Updates the firing angle based on the current DC link current.
     *
     * @param dc_link_current_pu The measured DC link current [p.u.].
     */
    void update(current_pu_t dc_link_current_pu)
    {
        _firing_angle_deg = _lookup_table.get_value(dc_link_current_pu.value());
    }

    /**
     * @brief Gets the last calculated inverter firing angle.
     *
     * @return The firing angle in degrees [90-180].
     */
    [[nodiscard]] real_t get_firing_angle_deg() const { return _firing_angle_deg; }

    /**
     * @brief Gets the last calculated inverter firing angle in radians.
     *
     * @return The firing angle in radians.
     */
    [[nodiscard]] real_t get_firing_angle_rad() const { return _firing_angle_deg * pi / 180.0; }

private:
    control::lookup_table_1d<real_t, NUM_POINTS> _lookup_table;
    real_t _firing_angle_deg{0.0};
};

}  // namespace yeet::application::sfc