#pragma once

#include "yeet_control/units.hpp"

/**
 * @file sfc_grid_side_integrator_supervisor.hpp
 * @brief Grid-side integrator supervisor core component
 *
 * Centralizes freeze/preset/trip logic for the current controller.
 * Manages requests from the modulator (forced commutation) and external
 * shutdown signals, coordinating a unified freeze output.
 */

namespace yeet::application::sfc {

class sfc_grid_side_integrator_supervisor {
public:
    /**
     * @brief Constructor
     * @param sampling_time Component sampling period
     */
    explicit sfc_grid_side_integrator_supervisor(duration_t sampling_time);

    /**
     * @brief Initialize the supervisor to default state
     */
    void reset();

    /**
     * @brief Main update cycle - process all inputs and update state
     * @param modulator_reset_request Freeze from modulator forced commutation
     * @param shutdown_request Trip request from external IOS
     * @param enable_current_controller Controller enable command
     * @param preset_complete Controller signals preset is done
     * @param override_freeze Testing override to disable freezes
     */
    void update(bool modulator_reset_request,
        bool shutdown_request,
        bool enable_current_controller,
        bool preset_complete,
        bool override_freeze);

    /**
     * @brief Get unified freeze command for current controller integrator
     */
    bool get_freeze_integrator() const noexcept;

    /**
     * @brief Get preset request to current controller
     */
    bool get_preset_request() const noexcept;

    /**
     * @brief Get enable signal for current controller to release pulses
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

    // Track previous input to detect state changes
    bool _previous_freeze_requested{false};
};

}  // namespace yeet::application::sfc
