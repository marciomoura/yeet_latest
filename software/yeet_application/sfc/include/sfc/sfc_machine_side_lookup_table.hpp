#pragma once

#include <algorithm>  // For std::clamp
#include <array>
#include <cmath>

#include "sfc_per_unit_system.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/lookup_table_2d.hpp"

namespace yeet::application::sfc {

/**
 * @brief Implements the Constant Gamma Control strategy for the LCI machine-side converter.
 *
 * This class calculates and manages a 2D lookup table to determine the inverter
 * firing angle (beta) based on machine speed and DC link current. It is designed
 * for real-time embedded systems, using a fixed-size, no-allocation table.
 *
 * The core strategy is to maintain a constant, safe extinction angle (gamma) for the
 * inverter thyristors. The lookup table is calculated online via the `recalculate_table`
 * method, which allows for easy commissioning, tuning, and adaptation of the control
 * policy (i.e., changing the gamma setpoint) without requiring a recompile.
 */
class sfc_machine_side_firing_angle_lookup_table {
public:
    // Define the dimensions for our test table at compile time
    static constexpr size_t NUM_CURRENT_POINTS = 6;
    static constexpr size_t NUM_SPEED_POINTS = 6;

    // Define default axes for the lookup table
    static constexpr std::array<real_t, NUM_CURRENT_POINTS> kDefaultCurrentAxisPu = {0.0, 0.25, 0.5, 0.75, 1.0, 1.2};
    static constexpr std::array<real_t, NUM_SPEED_POINTS> kDefaultSpeedAxisPu = {0.0, 0.2, 0.5, 0.75, 1.0, 1.25};

    /**
     * @brief Constructs the firing angle lookup table controller.
     *
     * @param initial_gamma_setpoint_deg The initial target extinction angle in degrees.
     */
    sfc_machine_side_firing_angle_lookup_table(real_t initial_gamma_setpoint_deg)
        : _gamma_setpoint_deg(initial_gamma_setpoint_deg)
    {
    }

    /**
     * @brief Recalculates the table using default axes.
     *
     * This overload uses the pre-defined default current and speed axes for convenience.
     *
     * @param gamma_setpoint_deg The new target extinction angle in degrees.
     * @param machine_side_commutation_reactance_pu The machine-side commutation reactance at base frequency [p.u.].
     */
    void recalculate_table(real_t gamma_setpoint_deg, real_t machine_side_commutation_reactance_pu)
    {
        recalculate_table(
            gamma_setpoint_deg, kDefaultCurrentAxisPu, kDefaultSpeedAxisPu, machine_side_commutation_reactance_pu);
    }

    /**
     * @brief Calculates and populates the beta firing angle lookup table with custom axes.
     *
     * This method can be called online to recommission the table with a new
     * control policy (i.e., a new gamma setpoint). It performs no dynamic allocation.
     *
     * @param gamma_setpoint_deg The new target extinction angle in degrees.
     * @param dc_current_axis_pu The axis definition for DC link current [p.u.].
     * @param speed_axis_pu The axis definition for machine electrical frequency [p.u.].
     * @param machine_side_commutation_reactance_pu The machine-side commutation reactance at base frequency [p.u.].
     */
    void recalculate_table(real_t gamma_setpoint_deg,
        const std::array<real_t, NUM_CURRENT_POINTS>& dc_current_axis_pu,
        const std::array<real_t, NUM_SPEED_POINTS>& speed_axis_pu,
        real_t machine_side_commutation_reactance_pu)
    {
        _gamma_setpoint_deg = gamma_setpoint_deg;
        std::array<std::array<real_t, NUM_CURRENT_POINTS>, NUM_SPEED_POINTS> firing_angle_table_deg;

        const real_t gamma_rad = gamma_setpoint_deg * pi / 180.0;

        // Populate the firing angle table matrix
        for (size_t i = 0; i < NUM_SPEED_POINTS; ++i) {
            for (size_t j = 0; j < NUM_CURRENT_POINTS; ++j) {
                const real_t machine_frequency_pu = speed_axis_pu[i];
                const real_t dc_current_pu = dc_current_axis_pu[j];

                real_t beta_advance_deg = calculate_beta_advance_angle_deg(
                    gamma_rad, machine_frequency_pu, dc_current_pu, machine_side_commutation_reactance_pu);

                // Convert from the advance angle (beta) to the textbook inverter firing angle (alpha_inv)
                // where alpha_inv = 180 - beta.
                firing_angle_table_deg[i][j] = 180.0 - beta_advance_deg;
            }
        }

        // Configure the 2D lookup table with the newly calculated values
        _lookup_table.configure(dc_current_axis_pu, speed_axis_pu, firing_angle_table_deg);
    }

    /**
     * @brief Gets the required inverter firing angle for a given operating point.
     *
     * @param machine_speed_hz The current measured machine electrical frequency [Hz].
     * @param dc_link_current_a The current measured DC link current [A].
     * @return The calculated beta firing angle in degrees [90-180].
     */
    [[nodiscard]] real_t get_firing_angle_deg(real_t machine_speed_hz, real_t dc_link_current_a) const
    {
        return _lookup_table.get_value(dc_link_current_a, machine_speed_hz);
    }

    /**
     * @brief Gets the required inverter firing angle for a given operating point in radians.
     *
     * This is a convenience method that converts the angle from degrees to radians.
     *
     * @param machine_speed_hz The current measured machine electrical frequency [Hz].
     * @param dc_link_current_a The current measured DC link current [A].
     * @return The calculated beta firing angle in radians.
     */
    [[nodiscard]] real_t get_firing_angle_rad(real_t machine_speed_hz, real_t dc_link_current_a) const
    {
        return get_firing_angle_deg(machine_speed_hz, dc_link_current_a) * pi / 180.0;
    }

    /**
     * @brief Gets the current gamma setpoint being used by the table.
     * @return The target extinction angle in degrees.
     */
    [[nodiscard]] real_t get_current_gamma_setpoint() const { return _gamma_setpoint_deg; }

private:
    /**
     * @brief Calculates the required firing advance angle (beta) for constant gamma control.
     *
     * This function implements the core control equation for a load-commutated inverter (LCI)
     * operating with a constant extinction angle (gamma) policy. The fundamental relationship is:
     *
     * cos(β) = cos(γ) - dX
     *
     * Where:
     * - β (beta) is the firing advance angle relative to the machine's voltage zero-crossing.
     * - γ (gamma) is the desired extinction angle, which is the time from current zero until the
     *   thyristor is reverse-biased by the machine's line-to-line voltage.
     * - dX is the per-unit voltage drop across the commutation reactance, which is dependent on
     *   the DC link current and the machine's electrical frequency.
     *
     * The term dX is calculated as: dX = (3/π) * Xc_pu * Id_pu, where Xc_pu is the per-unit
     * commutation reactance at the given frequency (machine_frequency_pu * base_reactance) and Id_pu is the
     * per-unit DC link current.
     *
     * The result is clamped to the valid range for acos [-1, 1] to ensure numerical stability
     * and handle operating points where full advance (180 degrees) is required.
     *
     * @param gamma_rad The desired extinction angle [radians].
     * @param machine_frequency_pu The machine electrical frequency [p.u.].
     * @param dc_current_pu The DC link current [p.u.].
     * @param commutation_reactance_pu The machine-side commutation reactance at base frequency [p.u.].
     * @return The calculated beta advance angle [degrees].
     */
    static real_t calculate_beta_advance_angle_deg(
        real_t gamma_rad, real_t machine_frequency_pu, real_t dc_current_pu, real_t commutation_reactance_pu)
    {
        const real_t inductive_voltage_drop_pu =
            (3.0 / pi) * commutation_reactance_pu * machine_frequency_pu * dc_current_pu;
        real_t cos_beta_advance = std::cos(gamma_rad) - inductive_voltage_drop_pu;
        cos_beta_advance = std::clamp(cos_beta_advance, real_t{-1.0}, real_t{1.0});
        return std::acos(cos_beta_advance) * 180.0 / pi;
    }

    real_t _gamma_setpoint_deg;

    control::lookup_table_2d<real_t, NUM_CURRENT_POINTS, NUM_SPEED_POINTS> _lookup_table;
};

}  // namespace yeet::application::sfc