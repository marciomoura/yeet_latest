#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Forced commutation states for the machine-side modulator.
 */
enum class machine_side_commutation_state_t {
    running,         ///< Releasing pulses for current sector
    waiting_for_ack  ///< Waiting for grid-side acknowledgment
};

/**
 * @class sfc_machine_side_modulator
 * @brief Manages firing angle selection and commutation handshake for the machine-side converter (inverter).
 *
 * This component acts as the initiator of the forced commutation sequence and follows
 * commands from the grid-side modulator. It translates rotor position into precisely
 * timed requests and firing commands for both natural and forced commutation modes.
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
     * operating mode and manages the forced commutation handshake protocol.
     * All real-time inputs must be provided as arguments following the yeet_application_sfc pattern.
     *
     * @param enable System enable signal. When false, modulator outputs safe default angle.
     * @param operating_mode The current operating mode (from external mode detector).
     * @param force_commutation_acknowledged Acknowledgment signal from grid-side modulator.
     * @param absolute_rotor_position_rad Absolute rotor position for sector tracking.
     * @param firing_angle_from_lut Firing angle from lookup table for Natural Commutation Mode.
     */
    void update(bool enable,
        sfc_operating_mode_t operating_mode,
        bool force_commutation_acknowledged,
        angle_t absolute_rotor_position_rad,
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
     * @brief Gets the pulse release signal for the six-pulse generator.
     *
     * This signal controls whether the machine-side converter should fire its
     * thyristors. It is false during forced commutation to allow current extinction.
     *
     * @return True when pulses should be released, false when blocked.
     */
    bool get_release_pulses() const noexcept;

    /**
     * @brief Gets the force commutation request signal.
     *
     * This signal is sent to the grid-side modulator to initiate forced commutation
     * when a rotor sector change is detected in pulse mode. It remains active until
     * the grid-side acknowledges the request.
     *
     * @return True when requesting force commutation, false otherwise.
     */
    bool get_request_force_commutation() const noexcept;

    /**
     * @brief Gets the current commutation state.
     * @return The current state of the forced commutation state machine.
     */
    machine_side_commutation_state_t get_commutation_state() const noexcept;

private:
    // Configuration parameters
    angle_t _pulse_mode_firing_angle_rad{deg_to_rad(90.0)};  ///< Fixed angle for Pulse Mode (90° for max torque)

    // Internal state
    sfc_operating_mode_t _current_mode{sfc_operating_mode_t::pulse_mode};  ///< Current operating mode
    angle_t _current_firing_angle_rad{0.0f};                               ///< Current output firing angle
    bool _system_enabled{false};                                           ///< System enable state

    // Forced commutation handshake state
    machine_side_commutation_state_t _commutation_state{machine_side_commutation_state_t::running};
    bool _release_pulses{true};              ///< Pulse release control
    bool _request_force_commutation{false};  ///< Request signal to grid-side

    // Rotor sector tracking
    int _previous_rotor_sector{-1};     ///< Previous rotor sector (0-5)
    bool _rotor_sector_changed{false};  ///< Sector change detection flag

    /**
     * @brief Calculates firing angle for Pulse Mode based on rotor position.
     * @param rotor_position_rad Absolute rotor position.
     * @return The calculated firing angle for maximum torque.
     */
    angle_t calculate_pulse_mode_firing_angle(angle_t rotor_position_rad) const noexcept;

    /**
     * @brief Updates rotor sector tracking and detects sector changes.
     * @param rotor_position_rad Current absolute rotor position.
     */
    void update_rotor_sector_tracking(angle_t rotor_position_rad) noexcept;

    /**
     * @brief Updates the forced commutation state machine.
     * @param force_commutation_acknowledged Acknowledgment from grid-side modulator.
     */
    void update_forced_commutation_state_machine(bool force_commutation_acknowledged) noexcept;

    /**
     * @brief Determines the current rotor sector from position.
     * @param rotor_position_rad Absolute rotor position.
     * @return Current sector number (0-5).
     */
    int calculate_rotor_sector(angle_t rotor_position_rad) const noexcept;

    /**
     * @brief Gets the safe default firing angle when system is disabled.
     * @return A safe firing angle (typically 0°).
     */
    angle_t get_safe_default_angle() const noexcept;
};

}  // namespace yeet::application::sfc
