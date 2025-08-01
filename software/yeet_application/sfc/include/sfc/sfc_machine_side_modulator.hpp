#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @class sfc_machine_side_modulator
 * @brief Manages firing angle selection for the machine-side converter (inverter).
 *
 * This component acts as a mode-dependent switch that selects the appropriate firing
 * angle strategy based on the provided operating mode. It switches between a lookup-table-based
 * approach for high-speed operation and a fixed, position-based approach for startup.
 *
 * The modulator follows the yeet_application_sfc architectural patterns:
 * - Configuration methods with `configure_` prefix
 * - Main execution via `update()` method with all inputs as arguments
 * - State access via `get_` methods
 * - Real-time safety with no dynamic allocation
 */
class sfc_machine_side_modulator {
public:
    /**
     * @brief Constructs the machine-side modulator.
     * @param sampling_time The control loop sampling time.
     */
    explicit sfc_machine_side_modulator(duration_t sampling_time);

    /**
     * @brief Configures the fixed firing angle used in Pulse Mode.
     *
     * This angle is applied relative to the selected thyristor pair to generate
     * maximum torque. Typical values are 0° or 90° depending on the reference frame.
     *
     * @param fixed_angle The fixed firing angle in radians.
     */
    void configure_pulse_mode_firing_angle_rad(angle_t fixed_angle);

    /**
     * @brief Updates the modulator with current system state and operating mode.
     *
     * This method selects the appropriate firing angle source based on the provided
     * operating mode. All real-time inputs must be provided as arguments following
     * the yeet_application_sfc pattern.
     *
     * @param enable System enable signal. When false, modulator outputs safe default angle.
     * @param operating_mode The current operating mode (from external mode detector).
     * @param absolute_rotor_position_rad Absolute rotor position for Pulse Mode calculations.
     * @param firing_angle_from_lut Firing angle from lookup table for Load-Commutated Mode.
     */
    void update(bool enable, sfc_operating_mode_t operating_mode, angle_t absolute_rotor_position_rad,
                angle_t firing_angle_from_lut) noexcept;

    /**
     * @brief Resets the modulator to initial state.
     *
     * Clears internal state and forces the modulator back to a safe initial condition.
     * Should be called when the system is disabled or during fault conditions.
     */
    void reset();

    /**
     * @brief Gets the current firing angle output.
     * @return The firing angle in radians to be applied to the machine-side converter.
     */
    angle_t get_firing_angle_rad() const noexcept;

    /**
     * @brief Gets the current operating mode.
     * @return The current operating mode (load_commutated or pulse_mode).
     */
    sfc_operating_mode_t get_current_operating_mode() const noexcept;

    /**
     * @brief Gets the rotor sector change request signal.
     *
     * This signal indicates when the rotor has moved from one electrical sector
     * to the next (60° electrical transition). It is used by the grid-side modulator
     * to coordinate pulse timing and commutation events.
     *
     * @return True when a sector change has been detected, false otherwise.
     */
    bool get_rotor_sector_change_request() const noexcept;

private:
    // Configuration parameters
    angle_t _pulse_mode_firing_angle_rad{0.0f};  ///< Fixed angle for Pulse Mode

    // Internal state
    sfc_operating_mode_t _current_mode{sfc_operating_mode_t::pulse_mode};  ///< Current operating mode
    angle_t _current_firing_angle_rad{0.0f};                               ///< Current output firing angle
    bool _system_enabled{false};                                           ///< System enable state

    // Rotor sector tracking
    int _previous_rotor_sector{-1};            ///< Previous rotor sector (0-5)
    bool _rotor_sector_change_request{false};  ///< Sector change detection flag

    /**
     * @brief Calculates firing angle for Pulse Mode based on rotor position.
     * @param rotor_position_rad Absolute rotor position.
     * @return The calculated firing angle for maximum torque.
     */
    angle_t calculate_pulse_mode_firing_angle(angle_t rotor_position_rad) const noexcept;

    /**
     * @brief Updates rotor sector tracking and generates change request signals.
     * @param rotor_position_rad Current absolute rotor position.
     */
    void update_rotor_sector_tracking(angle_t rotor_position_rad) noexcept;

    /**
     * @brief Determines the current rotor sector from position.
     * @param rotor_position_rad Absolute rotor position.
     * @return Current sector number (0-5).
     */
    int calculate_rotor_sector(angle_t rotor_position_rad) const noexcept;

    angle_t get_safe_default_angle() const noexcept;
};

}  // namespace yeet::application::sfc
