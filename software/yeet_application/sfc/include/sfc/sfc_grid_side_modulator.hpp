#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/elapsed_timer.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Pulse mode states for the grid-side modulator.
 */
enum class pulse_mode_state_t {
    idle,              ///< Waiting for enable signal
    setup_pulse,       ///< Preparing for current pulse
    pulse_on,          ///< Building DC link current
    force_commutation  ///< Forcing current to zero for commutation
};

/**
 * @class sfc_grid_side_modulator
 * @brief Manages firing angle selection for the grid-side converter (rectifier).
 *
 * This component acts as a mode-dependent switch that selects the appropriate firing
 * angle strategy based on the provided operating mode. It switches between a
 * pass-through from the external current controller for normal operation and an
 * internal state machine for pulse mode operation.
 *
 * The modulator follows the yeet_application_sfc architectural patterns:
 * - Configuration methods with `configure_` prefix
 * - Main execution via `update()` method with all inputs as arguments
 * - State access via `get_` methods
 * - Real-time safety with no dynamic allocation
 */
class sfc_grid_side_modulator {
public:
    /**
     * @brief Constructs the grid-side modulator.
     * @param sampling_time The control loop sampling time.
     */
    explicit sfc_grid_side_modulator(duration_t sampling_time);

    /**
     * @brief Configures the thresholds for pulse mode operation.
     *
     * These thresholds determine when to transition between pulse mode states
     * based on the measured DC link current.
     *
     * @param zero_threshold Current below this value is considered zero.
     * @param setpoint_reached_threshold Threshold for considering setpoint reached.
     */
    void configure_pulse_mode_current_thresholds(current_pu_t zero_threshold, current_pu_t setpoint_reached_threshold);

    /**
     * @brief Configures the fixed firing angle used for forced commutation.
     *
     * This angle is applied during the force_commutation state to rapidly
     * drive the DC link current to zero. Typical values are around 150°.
     *
     * @param angle The commutation angle in radians.
     */
    void configure_force_commutation_angle_rad(angle_t angle);

    /**
     * @brief Configures timing parameters for pulse mode operation.
     *
     * @param min_pulse_duration_s Minimum time to maintain pulse_on state.
     * @param max_pulse_duration_s Maximum time allowed in pulse_on state.
     * @param commutation_timeout_s Maximum time allowed for forced commutation.
     */
    void configure_pulse_mode_timing(duration_t min_pulse_duration_s, duration_t max_pulse_duration_s,
                                     duration_t commutation_timeout_s);

    /**
     * @brief Updates the modulator with current system state and operating mode.
     *
     * This method selects the appropriate firing angle source based on the provided
     * operating mode and manages the pulse mode state machine. All real-time inputs
     * must be provided as arguments following the yeet_application_sfc pattern.
     *
     * @param enable System enable signal. When false, modulator outputs safe default angle.
     * @param operating_mode The current operating mode (from external mode detector).
     * @param current_setpoint_pu Desired DC link current for pulse mode.
     * @param measured_dc_current_pu Current DC link current measurement.
     * @param firing_angle_from_main_controller Firing angle from external current controller.
     * @param rotor_sector_change_request Signal indicating rotor has moved to next sector.
     */
    void update(bool enable, sfc_operating_mode_t operating_mode, current_pu_t current_setpoint_pu,
                current_pu_t measured_dc_current_pu, angle_t firing_angle_from_main_controller,
                bool rotor_sector_change_request) noexcept;

    /**
     * @brief Resets the modulator to initial state.
     *
     * Clears internal state and forces the modulator back to a safe initial condition.
     * Should be called when the system is disabled or during fault conditions.
     */
    void reset();

    /**
     * @brief Gets the current firing angle output.
     * @return The firing angle in radians to be applied to the grid-side converter.
     */
    angle_t get_firing_angle_rad() const noexcept;

    /**
     * @brief Gets the current operating mode.
     * @return The current operating mode (load_commutated or pulse_mode).
     */
    sfc_operating_mode_t get_current_operating_mode() const noexcept;

    /**
     * @brief Gets the current pulse mode state.
     * @return The current state of the pulse mode state machine.
     */
    pulse_mode_state_t get_pulse_mode_state() const noexcept;

    /**
     * @brief Checks if a reset of the main PI controller is requested.
     *
     * This flag is set when transitioning from Pulse Mode back to Load-Commutated Mode
     * to signal that the external current controller should be reset to prevent
     * integral windup issues.
     *
     * @return True if main PI controller reset is requested.
     */
    bool is_requesting_main_pi_reset() const noexcept;

    /**
     * @brief Acknowledges the main PI controller reset request.
     *
     * This method should be called by the higher-level control task after
     * it has reset the external current controller to clear the reset request flag.
     */
    void acknowledge_main_pi_reset();

private:
    // Configuration parameters
    current_pu_t _zero_threshold_pu{0.01f};                   ///< Current threshold for zero detection
    current_pu_t _setpoint_reached_threshold_pu{0.05f};       ///< Threshold for setpoint reached detection
    angle_t _force_commutation_angle_rad{deg_to_rad(150.0)};  ///< 150° in radians for forced commutation
    duration_t _min_pulse_duration_s{0.001f};                 ///< Minimum pulse duration (1ms)
    duration_t _max_pulse_duration_s{0.010f};                 ///< Maximum pulse duration (10ms)
    duration_t _commutation_timeout_s{0.005f};                ///< Commutation timeout (5ms)

    // Internal state
    sfc_operating_mode_t _current_mode{sfc_operating_mode_t::pulse_mode};  ///< Current operating mode
    pulse_mode_state_t _pulse_state{pulse_mode_state_t::idle};             ///< Current pulse mode state
    angle_t _current_firing_angle_rad{0.0f};                               ///< Current output firing angle
    bool _system_enabled{false};                                           ///< System enable state
    bool _main_pi_reset_requested{false};                                  ///< Flag for PI reset request

    // Timing state using yeet_control elapsed_timer
    yeet::control::elapsed_timer _pulse_state_timer;  ///< Timer for current pulse state

    // Previous state tracking
    sfc_operating_mode_t _previous_mode{sfc_operating_mode_t::pulse_mode};  ///< Previous operating mode
    current_pu_t _previous_setpoint_pu{0.0f};                               ///< Previous current setpoint

    /**
     * @brief Updates the pulse mode state machine.
     * @param current_setpoint_pu Desired DC link current.
     * @param measured_dc_current_pu Current DC link current measurement.
     * @param rotor_sector_change_request Signal for rotor sector change.
     */
    void update_pulse_mode_state_machine(current_pu_t current_setpoint_pu, current_pu_t measured_dc_current_pu,
                                         bool rotor_sector_change_request) noexcept;

    /**
     * @brief Determines the firing angle for pulse mode operation.
     * @param firing_angle_from_main_controller Angle from external controller.
     * @return The firing angle to use in current pulse state.
     */
    angle_t calculate_pulse_mode_firing_angle(angle_t firing_angle_from_main_controller) const noexcept;

    /**
     * @brief Gets the safe default firing angle when system is disabled.
     * @return A safe firing angle (typically 0°).
     */
    angle_t get_safe_default_angle() const noexcept;

    /**
     * @brief Checks if current is considered zero based on threshold.
     * @param current_pu Current value to check.
     * @return True if current is below zero threshold.
     */
    bool is_current_zero(current_pu_t current_pu) const noexcept;

    /**
     * @brief Checks if current setpoint has been reached.
     * @param setpoint_pu Desired current value.
     * @param measured_pu Actual current value.
     * @return True if current is within threshold of setpoint.
     */
    bool is_setpoint_reached(current_pu_t setpoint_pu, current_pu_t measured_pu) const noexcept;
};

}  // namespace yeet::application::sfc