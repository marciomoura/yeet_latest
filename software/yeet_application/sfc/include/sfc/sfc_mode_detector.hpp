#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet_control/hysteresis_limiter.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @class sfc_mode_detector
 * @brief Detects the operating mode based on machine speed with hysteresis.
 *
 * This component determines whether the system should operate in Load-Commutated Mode
 * or Pulse Mode based exclusively on machine speed. It uses a hysteresis limiter
 * to prevent mode chattering around the handover speed threshold.
 */
class sfc_mode_detector {
public:
    /**
     * @brief Constructs the mode detector.
     */
    sfc_mode_detector();

    /**
     * @brief Configures the base speed threshold for mode handover.
     * @param handover_speed The base speed threshold in per-unit.
     */
    void configure_mode_handover_speed_pu(speed_pu_t handover_speed);

    /**
     * @brief Configures the hysteresis factor for mode switching.
     *
     * The actual switching thresholds become:
     * - Switch Up (Pulse -> Load): handover_speed * (1 + hysteresis_factor)
     * - Switch Down (Load -> Pulse): handover_speed * (1 - hysteresis_factor)
     *
     * @param hysteresis_factor Hysteresis as a fraction of handover speed.
     */
    void configure_hysteresis_factor(real_t hysteresis_factor);

    /**
     * @brief Updates the mode detector with the current machine speed.
     *
     * @param enable System enable signal. When false, defaults to forced_commutation.
     * @param measured_speed Current machine speed in per-unit.
     */
    void update(bool enable, speed_pu_t measured_speed) noexcept;

    /**
     * @brief Resets the mode detector to its initial state (forced_commutation).
     */
    void reset();

    /**
     * @brief Gets the current detected operating mode.
     * @return The current operating mode (natural_commutation or forced_commutation).
     */
    [[nodiscard]] sfc_operating_mode_t get_operating_mode() const noexcept;

    /**
     * @brief Checks if a mode transition occurred in the last update.
     * @return True if the mode changed between the previous and current update.
     */
    [[nodiscard]] bool has_mode_changed() const noexcept;

private:
    // Configuration parameters
    speed_pu_t _handover_speed_pu{0.1f};
    real_t _hysteresis_factor{0.05f};

    // Internal state
    sfc_operating_mode_t _current_mode{sfc_operating_mode_t::forced_commutation};
    sfc_operating_mode_t _previous_mode{sfc_operating_mode_t::forced_commutation};
    control::hysteresis_limiter _speed_limiter;

    /**
     * @brief Updates the thresholds of the internal hysteresis limiter based on
     *        the configured handover speed and hysteresis factor.
     */
    void update_limiter_thresholds();
};

}  // namespace yeet::application::sfc