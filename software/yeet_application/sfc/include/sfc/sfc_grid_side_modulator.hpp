#pragma once

#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Operating states for the grid-side modulator.
 */
enum class grid_side_modulator_state_t {
    natural_commutation,  ///< Normal operation - pass through PI controller angle
    forced_commutation    ///< Force commutation - apply deep inverting angle
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
     */
    explicit sfc_grid_side_modulator();

    /**
     * @brief Configures the fixed firing angle used for forced commutation.
     *
     * This angle is applied during force commutation to rapidly drive the DC link
     * current to zero. Typical values are around 150° (deep inversion).
     *
     * @param angle The commutation angle in radians.
     */
    void configure_force_commutation_angle_rad(angle_t angle);

    /**
     * @brief Updates the modulator with current system state.
     *
     * This method implements the handshake protocol for forced commutation.
     * All real-time inputs must be provided as arguments following the
     * yeet_application_sfc pattern.
     *
     * @param request_force_commutation Signal from machine-side modulator requesting commutation.
     * @param dc_link_current_is_zero Signal indicating DC link current has reached zero.
     * @param firing_angle_from_main_pi Firing angle from external PI current controller.
     */
    void update(
        bool request_force_commutation, bool dc_link_current_is_zero, angle_t firing_angle_from_main_pi) noexcept;

    /**
     * @brief Resets the modulator to initial state.
     *
     * Clears internal state and forces the modulator back to normal operation.
     * Should be called when the system is disabled or during fault conditions.
     */
    void reset();

    /**
     * @brief Gets the current firing angle output.
     * @return The firing angle in radians to be applied to the grid-side converter.
     */
    angle_t get_firing_angle_rad() const noexcept;

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

private:
    // Configuration parameters
    angle_t _force_commutation_angle_rad{deg_to_rad(150.0)};  ///< 150° for forced commutation

    // Internal state
    grid_side_modulator_state_t _current_state{grid_side_modulator_state_t::natural_commutation};
    angle_t _current_firing_angle_rad{0.0f};      ///< Current output firing angle
    bool _force_commutation_acknowledged{false};  ///< Acknowledgment signal for one cycle
    bool _main_pi_reset_requested{false};         ///< Flag for PI reset request

    /**
     * @brief Gets the safe default firing angle when system is disabled.
     * @return A safe firing angle (0° - natural commutation point).
     */
    angle_t get_safe_default_angle() const noexcept;
};

}  // namespace yeet::application::sfc