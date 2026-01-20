#pragma once

#include "sfc_types.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Operating states for the grid-side modulator.
 */
enum class grid_side_modulator_state_t {
    running,               ///< Normal operation - pass through PI controller angle and firing angle
    forced_commutation,    ///< Force commutation - apply deep inverting angle to bring current to zero
    extinguishing_current  ///< Current extinction mode - apply inverting angle for shutdown/trip
};

/**
 * @class sfc_grid_side_modulator
 * @brief Master controller for the forced commutation sequence in the LCI system.
 *
 * This component acts as the central coordinator for the commutation process at low speeds.
 * During normal operation, it passes through the firing angle from the main DC link current
 * controller. However, when a commutation is requested by the machine-side modulator, it
 * takes full command of the system, forcing the grid-side converter into inversion to
 * extinguish the DC link current.
 *
 */
class sfc_grid_side_modulator {
public:
    /**
     * @brief Constructs the grid-side modulator.
     * @param sampling_time The control loop sampling time.
     */
    explicit sfc_grid_side_modulator(duration_t sampling_time);

    /**
     * @brief Updates the modulator with current system state.
     *
     * This method implements the handshake protocol for forced commutation.
     * All real-time inputs must be provided as arguments following the
     * yeet_application_sfc pattern.
     *
     * @param enable System enable signal.
     * @param operating_mode Current operating mode (natural or forced commutation).
     * @param reference_angle Reference angle from PLL (grid synchronization).
     * @param request_force_commutation Signal from machine-side modulator requesting commutation.
     * @param is_current_zero Signal indicating DC link current has reached zero.
     * @param firing_angle_from_main_pi Firing angle from external PI current controller.
     * @param request_extinguish_current Priority request to extinguish current (shutdown/trip).
     */
    void update(bool enable,
        sfc_operating_mode_t operating_mode,
        angle_t reference_angle,
        bool request_force_commutation,
        bool is_current_zero,
        angle_t firing_angle_from_main_pi,
        bool request_extinguish_current) noexcept;

    /**
     * @brief Resets the modulator to initial state.
     *
     * Clears internal state and forces the modulator back to normal operation.
     * Should be called when the system is disabled or during fault conditions.
     */
    void reset();

    /**
     * @brief Configures the forced commutation angle.
     * @param angle The angle to apply during forced commutation.
     */
    void configure_force_commutation_angle(angle_t angle) noexcept;

    /**
     * @brief Configures the commutation timeout.
     * @param timeout The maximum duration for a commutation event.
     */
    void configure_commutation_timeout(duration_t timeout) noexcept;

    /**
     * @brief Configures the pulse width for the thyristors.
     * @param width The pulse width in radians.
     */
    void configure_pulse_width(angle_t width) noexcept;

    /**
     * @brief Configures the reference voltage type.
     * @param type The reference type (phase or line).
     */
    void configure_reference_type(control::six_pulse_generator::reference_type type) noexcept;

    /**
     * @brief Configures the virtual reference frequency for the pulse generator.
     * @param frequency The virtual reference frequency in Hertz.
     */
    void configure_virtual_reference(frequency_t frequency) noexcept;

    /**
     * @brief Configures the pulse mask for the thyristors.
     * @param p1 Pulse mask for thyristor 1.
     * @param p2 Pulse mask for thyristor 2.
     * @param p3 Pulse mask for thyristor 3.
     * @param p4 Pulse mask for thyristor 4.
     * @param p5 Pulse mask for thyristor 5.
     * @param p6 Pulse mask for thyristor 6.
     */
    void configure_pulse_mask(bool p1, bool p2, bool p3, bool p4, bool p5, bool p6) noexcept;

    /**
     * @brief Gets the current firing angle output.
     * @return The firing angle in radians to be applied to the grid-side converter.
     */
    angle_t get_firing_angle_rad() const noexcept;

    /**
     * @brief Gets the current firing pulses for the thyristors.
     * @return A structure containing the gating signals for p1 through p6.
     */
    sfc_thyristor_pulses_t get_firing_pulses() const noexcept;

    /**
     * @brief Indicates if any thyristor has been fired in the current cycle.
     * @return True if any gating signal transition from 0 to 1 occurred.
     */
    bool get_any_thyristor_fired() const noexcept;

    /**
     * @brief Gets the current operating state.
     * @return The current state (natural_commutation or forced_commutation).
     */
    grid_side_modulator_state_t get_current_state() const noexcept;

    /**
     * @brief Gets the force commutation acknowledgment signal.
     *
     * This signal is asserted for one cycle when commutation is complete
     * and should be sent to the machine-side modulator.
     *
     * @return True if force commutation has been acknowledged this cycle.
     */
    bool get_force_commutation_acknowledged() const noexcept;

    /**
     * @brief Checks if a reset of the main PI controller is requested.
     *
     * This flag is set when entering force commutation mode to signal that
     * the external current controller should be reset to prevent integral windup.
     *
     * @return True if main PI controller reset is requested.
     */
    bool get_is_requesting_main_pi_reset() const noexcept;

    /**
     * @brief Acknowledges the main PI controller reset request.
     *
     * This method should be called by the higher-level control task after
     * it has reset the external current controller to clear the reset request flag.
     */
    void acknowledge_main_pi_reset();

    /**
     * @brief Gets the pulse release status.
     *
     * This signal indicates whether the firing pulses for the grid-side
     * converter should be enabled.
     *
     * @return True if pulses should be released, false otherwise.
     */
    bool get_pulse_release() const noexcept;

    /**
     * @brief Checks if the modulator is currently extinguishing current.
     *
     * This flag indicates that the modulator is in current extinction mode,
     * typically triggered by a shutdown request or trip condition.
     *
     * @return True if currently extinguishing current, false otherwise.
     */
    bool get_is_extinguishing_current() const noexcept;

    /**
     * @brief Gets the current virtual reference angle.
     * @return The internally integrated virtual reference angle.
     */
    angle_t get_virtual_reference_angle() const noexcept;

    /**
     * @brief Checks if virtual reference mode is active.
     */
    bool is_virtual_reference_active() const noexcept;

    /**
     * @brief Checks if individual pulse override is active.
     */
    bool is_individual_pulse_override_active() const noexcept;

private:
    /**
     * @brief Internal state update structure for pulse mode operation.
     */
    struct forced_commutation_state_update {
        grid_side_modulator_state_t new_state;
        angle_t new_firing_angle;
        bool new_force_commutation_acknowledged;
        bool new_main_pi_reset_requested;
    };

    forced_commutation_state_update compute_forced_commutation_state_update(
        bool request_force_commutation, bool is_current_zero, angle_t firing_angle) const noexcept;

    // Configuration parameters
    angle_t _force_commutation_angle_rad{deg_to_rad(150.0)};  ///< 150° for forced commutation
    duration_t _commutation_timeout{0.1f};                    ///< Default 100ms timeout

    // Internal state
    grid_side_modulator_state_t _current_state{grid_side_modulator_state_t::running};
    angle_t _current_firing_angle_rad{0.0f};      ///< Current output firing angle
    bool _force_commutation_acknowledged{false};  ///< Acknowledgment signal for one cycle
    bool _main_pi_reset_requested{false};         ///< Flag for PI reset request
    bool _release_pulses{true};                   ///< Pulse release control

    control::six_pulse_generator _pulse_generator;  ///< Thyristor pulse generator
    bool _any_thyristor_fired{false};               ///< Signal to detect gating transitions
    duration_t _sampling_time;                      ///< Control loop sampling time

    /**
     * @brief Gets the safe default firing angle when system is disabled.
     * @return A safe firing angle (0° - natural commutation point).
     */
    angle_t get_safe_default_angle() const noexcept;
};

}  // namespace yeet::application::sfc