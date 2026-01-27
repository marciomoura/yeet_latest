#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/monotonic_angle_filter.hpp"
#include "yeet_control/six_pulse_generator.hpp"
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
 */
class sfc_machine_side_modulator {
public:
    /**
     * @brief Constructs the machine-side modulator.
     * @param sampling_time The control loop sampling time.
     */
    explicit sfc_machine_side_modulator(duration_t sampling_time);

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
     * @param firing_angle_rad Firing angle from lookup table for Natural Commutation Mode.
     */
    void update(bool enable,
        sfc_operating_mode_t operating_mode,
        bool force_commutation_acknowledged,
        angle_wrapped absolute_rotor_position_rad,
        angle_wrapped firing_angle_rad) noexcept;

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
     * @brief Configures the firing angle for pulse mode operation.
     * @param angle The fixed firing angle in radians.
     */
    void configure_forced_commutation_firing_angle(angle_t angle) noexcept;

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
     * @brief Gets the current commutation state.
     * @return The current state of the forced commutation state machine.
     */
    machine_side_commutation_state_t get_commutation_state() const noexcept;

    /**
     * @brief Gets the current rotor sector (0-5).
     *
     * Returns the electrical sector of the rotor based on the last update.
     * Each sector is 60 degrees (π/3 radians) wide. Sector 0 corresponds to
     * [0°, 60°), sector 1 to [60°, 120°), and so on.
     *
     * @return Current sector number (0-5), or -1 if not yet initialized.
     */
    int get_current_rotor_sector() const noexcept;

    /**
     * @brief Gets the filtered, monotonic rotor position used for pulsing.
     * @return The monotonic rotor position in radians.
     */
    // angle_t get_monotonic_rotor_position() const noexcept;

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
    // Configuration parameters
    angle_wrapped _forced_commutation_firing_angle_rad{
        angle_wrapped::from_degrees(180.0)};  ///< Fixed angle for Pulse Mode (180° for max torque)

    // Internal state
    angle_wrapped _current_firing_angle_rad{0.0f};           ///< Current output firing angle
    control::monotonic_angle_filter _rotor_position_filter;  ///< Filters rotor position for monotonicity
    control::six_pulse_generator _pulse_generator;           ///< Thyristor pulse generator
    bool _any_thyristor_fired{false};                        ///< Signal to trigger forced commutation handshake
    duration_t _sampling_time;                               ///< Control loop sampling time

    // Forced commutation handshake state
    machine_side_commutation_state_t _commutation_state{machine_side_commutation_state_t::running};
    bool _release_pulses{true};              ///< Pulse release control
    bool _request_force_commutation{false};  ///< Request signal to grid-side

    /**
     * @brief Gets the safe default firing angle when system is disabled.
     * @return A safe firing angle (typically 0°).
     */
    angle_t get_safe_default_angle() const noexcept;
};

}  // namespace yeet::application::sfc
