#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @class sfc_mode_detector
 * @brief Detects and manages the operating mode of the SFC system.
 *
 * This component determines whether the system should operate in Load-Commutated Mode
 * or Pulse Mode based on machine speed and other system conditions. It implements
 * hysteresis to prevent mode chattering and provides smooth transitions between modes.
 *
 * The detector follows the yeet_application_sfc architectural patterns:
 * - Configuration methods with `configure_` prefix
 * - Main execution via `update()` method with all inputs as arguments
 * - State access via `get_` methods
 * - Real-time safety with no dynamic allocation
 */
class sfc_mode_detector {
public:
    /**
     * @brief Constructs the mode detector.
     * @param sampling_time The control loop sampling time.
     */
    explicit sfc_mode_detector(duration_t sampling_time);

    /**
     * @brief Configures the speed threshold for mode handover.
     *
     * When measured speed exceeds this threshold, the detector switches to
     * Load-Commutated Mode. When speed falls below this threshold, it switches
     * to Pulse Mode.
     *
     * @param handover_speed The speed threshold in per-unit. Typical value: 0.1 pu (10%).
     */
    void configure_mode_handover_speed_pu(speed_pu_t handover_speed);

    /**
     * @brief Configures the hysteresis factor for mode switching.
     *
     * Hysteresis prevents mode chattering around the handover speed threshold.
     * The actual switching thresholds become:
     * - Pulse → Load-Commutated: handover_speed * (1 + hysteresis_factor)
     * - Load-Commutated → Pulse: handover_speed * (1 - hysteresis_factor)
     *
     * @param hysteresis_factor Hysteresis as a fraction of handover speed. Typical: 0.05 (5%).
     */
    void configure_hysteresis_factor(real_t hysteresis_factor);

    /**
     * @brief Configures additional conditions for mode detection.
     *
     * Additional safety conditions can prevent mode switching under certain circumstances.
     *
     * @param min_enable_time_s Minimum time system must be enabled before allowing mode switches.
     * @param max_current_for_switch_pu Maximum DC current allowed during mode transitions.
     */
    void configure_safety_conditions(duration_t min_enable_time_s, current_pu_t max_current_for_switch_pu);

    /**
     * @brief Updates the mode detector with current system state.
     *
     * This method analyzes the system conditions and determines the appropriate
     * operating mode. All real-time inputs must be provided as arguments following
     * the yeet_application_sfc pattern.
     *
     * @param enable System enable signal. When false, defaults to pulse_mode.
     * @param measured_speed_pu Current machine speed in per-unit.
     * @param measured_dc_current_pu Current DC link current in per-unit.
     * @param back_emf_magnitude_pu Magnitude of the machine's back-EMF in per-unit.
     */
    void update(bool enable, speed_pu_t measured_speed_pu, current_pu_t measured_dc_current_pu,
                voltage_pu_t back_emf_magnitude_pu) noexcept;

    /**
     * @brief Resets the mode detector to initial state.
     *
     * Clears internal state and forces the detector back to pulse_mode.
     * Should be called when the system is disabled or during fault conditions.
     */
    void reset();

    /**
     * @brief Gets the current detected operating mode.
     * @return The current operating mode (load_commutated or pulse_mode).
     */
    sfc_operating_mode_t get_operating_mode() const noexcept;

    /**
     * @brief Gets the previous operating mode.
     * @return The operating mode from the previous update cycle.
     */
    sfc_operating_mode_t get_previous_operating_mode() const noexcept;

    /**
     * @brief Checks if a mode transition occurred in the last update.
     * @return True if the mode changed between the previous and current update.
     */
    bool has_mode_changed() const noexcept;

    /**
     * @brief Gets the time elapsed since the last mode change.
     * @return Time in seconds since the last mode transition.
     */
    duration_t get_time_since_mode_change() const noexcept;

    /**
     * @brief Checks if the system is ready for Load-Commutated Mode.
     *
     * This considers all conditions including speed, back-EMF, and safety constraints.
     *
     * @return True if conditions are suitable for Load-Commutated Mode.
     */
    bool is_ready_for_load_commutated_mode() const noexcept;

private:
    // Configuration parameters
    speed_pu_t _handover_speed_pu{0.1f};            ///< Speed threshold for mode switching
    real_t _hysteresis_factor{0.05f};               ///< Hysteresis factor (5% default)
    duration_t _min_enable_time_s{0.1f};            ///< Minimum enable time before mode switches
    current_pu_t _max_current_for_switch_pu{0.5f};  ///< Maximum current for safe mode switching

    // Internal state
    sfc_operating_mode_t _current_mode{sfc_operating_mode_t::pulse_mode};   ///< Current operating mode
    sfc_operating_mode_t _previous_mode{sfc_operating_mode_t::pulse_mode};  ///< Previous operating mode
    duration_t _time_since_mode_change{0.0f};                               ///< Time since last mode change
    duration_t _time_since_enable{0.0f};                                    ///< Time since system was enabled
    bool _system_enabled{false};                                            ///< System enable state
    duration_t _sampling_time;                                              ///< Control loop sampling time

    // Current measurements (stored for diagnostics)
    speed_pu_t _current_speed_pu{0.0f};         ///< Current speed measurement
    current_pu_t _current_dc_current_pu{0.0f};  ///< Current DC link current
    voltage_pu_t _current_back_emf_pu{0.0f};    ///< Current back-EMF magnitude

    /**
     * @brief Determines if speed conditions favor Load-Commutated Mode.
     * @param measured_speed_pu Current machine speed.
     * @return True if speed supports Load-Commutated Mode.
     */
    bool is_speed_sufficient_for_load_commutated(speed_pu_t measured_speed_pu) const noexcept;

    /**
     * @brief Determines if back-EMF conditions favor Load-Commutated Mode.
     * @param back_emf_magnitude_pu Current back-EMF magnitude.
     * @return True if back-EMF supports natural commutation.
     */
    bool is_back_emf_sufficient_for_commutation(voltage_pu_t back_emf_magnitude_pu) const noexcept;

    /**
     * @brief Checks if safety conditions permit mode switching.
     * @param measured_dc_current_pu Current DC link current.
     * @return True if it's safe to switch modes.
     */
    bool are_safety_conditions_met(current_pu_t measured_dc_current_pu) const noexcept;

    /**
     * @brief Updates internal timing counters.
     * @param enable Current system enable state.
     */
    void update_timing_counters(bool enable) noexcept;
};

}  // namespace yeet::application::sfc