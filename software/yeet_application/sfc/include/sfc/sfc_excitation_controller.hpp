#pragma once

#include <array>
#include <cstddef>

#include "yeet_control/elapsed_timer.hpp"
#include "yeet_control/lookup_table_1d.hpp"
#include "yeet_control/rate_of_change_limiter.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Manages excitation current control for LCI-driven synchronous machines.
 *
 * This component generates a constant field current reference for the excitation system.
 * The controller maintains a fixed flux level and provides a timer-based flux build-up
 * detection mechanism.
 *
 * The controller follows the standard yeet_application_sfc architectural pattern:
 * - Explicit enable/disable via start_excitation() and stop_excitation()
 * - All real-time inputs passed to update()
 * - Configuration via configure_*() methods
 * - Outputs retrieved via get_*() methods
 *
 * Key Features:
 * - Constant field current reference (simplified soft-starter mode)
 * - Timer-based field establishment detection
 */
class sfc_excitation_controller {
public:
    // // Default lookup table dimensions
    // static constexpr size_t NUM_LOOKUP_POINTS = 6;

    // // Default speed-to-field-current mapping (conservative safe values)
    // // Speed axis in per-unit (0.0 = standstill, 1.0 = rated speed)
    // static constexpr std::array<real_t, NUM_LOOKUP_POINTS> kDefaultSpeedAxisPu = {0.00, 0.05, 0.80,
    // 0.95, 1.00, 1.05};

    // // Field current reference axis in per-unit (typical soft-starter uses constant value)
    // static constexpr std::array<real_t, NUM_LOOKUP_POINTS> kDefaultFieldCurrentReferencePu = {
    //     1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    /**
     * @brief Constructs the excitation controller.
     * @param sampling_time The control loop sampling time [s].
     */
    explicit sfc_excitation_controller(duration_t sampling_time);

    /**
     * @brief Configures the constant field current setpoint.
     *
     * This is the constant field current reference maintained throughout operation.
     *
     * @param field_current_setpoint Constant field current reference [pu].
     */
    void configure_field_current_setpoint(current_pu_t field_current_setpoint);

    // /**
    //  * @brief Configures the constant flux region speed threshold.
    //  *
    //  * Below this speed threshold, the field current reference is held constant at the
    //  * setpoint. This prevents excessive field current variations during low-speed
    //  * operation when rotor speed may be oscillatory.
    //  *
    //  * Typical value: 0.15-0.25 pu for LCI soft-starters.
    //  *
    //  * @param speed_threshold_pu Speed threshold below which flux remains constant [pu].
    //  */
    // void configure_constant_flux_speed_threshold(speed_pu_t speed_threshold_pu);

    /**
     * @brief Configures the flux build-up duration.
     *
     * This is the time to wait after enabling excitation for the field winding to
     * fully energize before considering the flux established.
     * Typical value: 2-5 seconds for LCI machines.
     *
     * @param flux_buildup_duration Duration for flux to build up [s].
     */
    void configure_flux_buildup_duration(duration_t flux_buildup_duration);

    // /**
    //  * @brief Configures the maximum timeout for field establishment.
    //  *
    //  * If no field current feedback is available, the controller assumes
    //  * the field is established after this timeout expires (assuming no faults).
    //  *
    //  * @param timeout Maximum time to wait for field establishment [s].
    //  */
    // void configure_field_established_timeout(duration_t timeout);

    // /**
    //  * @brief Configures the field current reference ramp-down time on shutdown.
    //  *
    //  * Controls how quickly field current reference is reduced to zero when stop_excitation()
    //  * is called. A gradual ramp prevents voltage transients and allows for controlled
    //  * de-fluxing of the machine.
    //  *
    //  * @param ramp_down_time Time to ramp from current value to zero [s].
    //  */
    // void configure_field_current_ramp_down_time(duration_t ramp_down_time);

    // /**
    //  * @brief Configures the field current reference lookup table.
    //  *
    //  * This table defines the speed-to-field-current mapping. For soft-starter applications,
    //  * this is typically a constant value across all speeds. The table must be commissioned
    //  * based on the specific machine's rated field current and desired flux level.
    //  *
    //  * @param speed_axis_pu Array of speed breakpoints [pu], must be sorted ascending.
    //  * @param field_current_reference Array of corresponding field current references [pu].
    //  */
    // void configure_field_current_lookup_table(const std::array<real_t, NUM_LOOKUP_POINTS>& speed_axis_pu,
    //     const std::array<real_t, NUM_LOOKUP_POINTS>& field_current_reference);

    /**
     * @brief Starts the excitation sequence.
     *
     * Preconditions:
     * - exciter_ready must be true
     *
     * Actions:
     * - Enables excitation output
     * - Sets field current reference to configured setpoint
     * - Starts flux build-up timer
     *
     * @return True if started successfully, false if preconditions not met.
     */
    bool start_excitation();

    /**
     * @brief Stops the excitation sequence.
     *
     * Actions:
     * - Disables excitation output
     * - Resets field current reference to zero
     * - Resets all internal timers
     */
    void stop_excitation();

    /**
     * @brief Executes one update cycle of the excitation controller.
     *
     * This method must be called every control cycle.
     *
     * Behavior:
     * - When enable_excitation_command = false: outputs disabled, states reset
     * - When enable_excitation_command = true: outputs field current setpoint
     * - Monitors flux build-up timer to set is_field_established flag
     *
     * @param enable_excitation_command Command to enable/disable excitation output (true = enable).
     */
    void update(bool enable_excitation_command);

    /**
     * @brief Resets all internal states.
     *
     * Returns the controller to initial idle state. Should be called
     * during system initialization or after fault recovery.
     */
    void reset();

    /**
     * @brief Gets the current field current reference.
     * @return Field current reference to send to excitation current regulator [pu].
     */
    current_pu_t get_field_current_reference_pu() const noexcept;

    /**
     * @brief Gets the excitation enable command.
     * @return True = enable exciter, false = disable exciter.
     */
    bool get_enable_excitation() const noexcept;

    /**
     * @brief Checks if the flux build-up is complete.
     * @return True if flux build-up timer has elapsed.
     */
    bool get_is_field_established() const noexcept;

    /**
     * @brief Checks if the controller is currently running.
     * @return True if excitation sequence is active.
     */
    bool get_is_running() const noexcept;

    /**
     * @brief Gets the current elapsed time since excitation was enabled.
     * @return Flux build-up timer value [s], zero if not running.
     */
    duration_t get_flux_buildup_elapsed_time() const noexcept;

private:
    // Sampling time
    const duration_t _sampling_time;

    // Control components
    // control::lookup_table_1d<real_t, NUM_LOOKUP_POINTS> _field_current_lookup_table;
    // control::rate_of_change_limiter _field_current_rate_limiter;
    control::elapsed_timer _flux_buildup_timer;

    // Configuration parameters
    current_pu_t _field_current_setpoint{0.8};  // Constant field current reference
    duration_t _flux_buildup_duration{2.0};     // Time for flux to build up
    // duration_t _field_established_timeout{5.0};
    // speed_pu_t _constant_flux_speed_threshold{0.2};  // Speed below which field current is constant

    // Internal state flags
    bool _is_running{false};
    bool _is_field_established{false};
    // bool _is_ramping_down{false};

    // Outputs
    current_pu_t _field_current_reference_pu{0.0};
    bool _enable_excitation{false};

    // Last known good values for fault handling
    // speed_pu_t _last_valid_speed{0.0};
};

}  // namespace yeet::application::sfc
