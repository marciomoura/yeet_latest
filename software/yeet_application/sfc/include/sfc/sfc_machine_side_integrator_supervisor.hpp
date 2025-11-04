#pragma once

#include "yeet_control/units.hpp"

/**
 * @file sfc_machine_side_integrator_supervisor.hpp
 * @brief Machine-side integrator supervisor core component
 *
 * Centralizes freeze/preset/trip logic for both speed and torque controllers.
 * Manages requests from:
 * - Grid-side supervisor (via fiber optics)
 * - Torque controller saturation (anti-windup)
 * - Local emergency stop
 */

namespace yeet::application::sfc {

class sfc_machine_side_integrator_supervisor {
public:
    /**
     * @brief Constructor
     * @param sampling_time Component sampling period
     */
    explicit sfc_machine_side_integrator_supervisor(duration_t sampling_time);

    /**
     * @brief Initialize the supervisor to default state
     */
    void reset();

    /**
     * @brief Main update cycle - process all inputs and update state
     * @param grid_side_freeze Unified freeze from grid-side supervisor
     * @param emergency_stop Local emergency stop/trip request
     * @param torque_saturated Anti-windup freeze from torque controller saturation
     * @param enable_speed_controller Speed controller enable command
     * @param speed_preset_complete Speed controller signals preset is done
     * @param enable_torque_controller Torque controller enable command
     * @param torque_preset_complete Torque controller signals preset is done
     * @param override_freeze Testing override to disable freezes
     */
    void update(bool grid_side_freeze,
        bool emergency_stop,
        bool torque_saturated,
        bool enable_speed_controller,
        bool speed_preset_complete,
        bool enable_torque_controller,
        bool torque_preset_complete,
        bool override_freeze);

    /**
     * @brief Get unified freeze command for both controllers
     */
    bool get_freeze_integrator() const noexcept;

    /**
     * @brief Get preset request to both controllers
     */
    bool get_preset_request() const noexcept;

    /**
     * @brief Get enable signal for both controllers to release pulses
     */
    bool get_release_pulses() const noexcept;

    /**
     * @brief Get trip status (fault condition active)
     */
    bool get_is_tripped() const noexcept;

private:
    duration_t _sampling_time;

    // State variables
    bool _is_frozen{false};
    bool _is_tripped{false};
    bool _preset_request{false};
    bool _preset_done{false};

    // Track previous freeze state
    bool _previous_freeze_requested{false};
};

}  // namespace yeet::application::sfc
