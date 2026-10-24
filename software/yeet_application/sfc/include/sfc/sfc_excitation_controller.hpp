#pragma once

#include <array>
#include <cstddef>

#include "yeet_control/elapsed_timer.hpp"
#include "yeet_control/lookup_table_1d.hpp"
#include "yeet_control/pi_controller.hpp"
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
    /// Number of points used for the speed -> voltage reference lookup table
    static constexpr size_t NUM_VOLTAGE_REFERENCE_POINTS = 7;

    /// Default speed axis for the voltage reference lookup table [pu]
    static constexpr std::array<speed_pu_t, NUM_VOLTAGE_REFERENCE_POINTS> kDefaultVoltageReferenceSpeedAxisPu = {
        speed_pu_t{0.0}, speed_pu_t{0.2}, speed_pu_t{0.4}, speed_pu_t{0.6}, speed_pu_t{0.8}, speed_pu_t{1.0},
        speed_pu_t{1.1}};

    /// Default voltage reference values for the lookup table [pu]
    /// Linear V/Hz ramp: V = 0.2 + 0.8*speed (0.2 pu at standstill, 1.0 pu at rated, continues to 1.1 pu)
    /// Above rated speed, voltage continues to scale proportionally to maintain constant flux
    static constexpr std::array<voltage_pu_t, NUM_VOLTAGE_REFERENCE_POINTS> kDefaultVoltageReferencePu = {
        voltage_pu_t{0.0}, voltage_pu_t{0.2}, voltage_pu_t{0.4}, voltage_pu_t{0.6}, voltage_pu_t{0.8},
        voltage_pu_t{1.0}, voltage_pu_t{1.0}};

    /**
     * @brief Constructs the excitation controller.
     * @param sampling_time The control loop sampling time [s].
     */
    explicit sfc_excitation_controller(duration_t sampling_time);

    /**
     * @brief Configures the field current reference used in forced mode.
     *
     * This value is applied when the machine is in the low-speed/low-voltage region
     * and the PI voltage controller is not yet released.
     *
     * @param forced_field_current_pu Forced field current reference [pu].
     */
    void configure_forced_field_current(current_pu_t forced_field_current_pu);

    /**
     * @brief Configures the voltage threshold for releasing to automatic control.
     *
     * When measured machine voltage magnitude exceeds this threshold OR the speed
     * exceeds the speed threshold, the controller switches from forced current
     * to closed-loop voltage regulation.
     *
     * @param threshold_pu Release voltage threshold [pu].
     */
    void configure_release_voltage_threshold(voltage_pu_t threshold_pu);

    /**
     * @brief Configures the speed threshold for releasing to automatic control.
     *
     * When machine speed exceeds this threshold OR the voltage exceeds the
     * voltage threshold, the controller switches from forced current
     * to closed-loop voltage regulation.
     *
     * @param threshold_pu Release speed threshold [pu].
     */
    void configure_release_speed_threshold(speed_pu_t threshold_pu);

    /**
     * @brief Configures the voltage reference lookup table (speed -> voltage reference).
     *
     * @param speed_axis_pu Speed breakpoints [pu]. Must be sorted ascending.
     * @param voltage_reference_pu Voltage reference values [pu].
     */
    void configure_voltage_reference_lookup_table(
        const std::array<speed_pu_t, NUM_VOLTAGE_REFERENCE_POINTS>& speed_axis_pu,
        const std::array<voltage_pu_t, NUM_VOLTAGE_REFERENCE_POINTS>& voltage_reference_pu);

    /**
     * @brief Configures the machine-voltage PI controller.
     *
     * The PI controller processes the voltage error (Vref - Vmeas) and generates a field current reference.
     *
     * @param kp Proportional gain.
     * @param ti Integral time constant [s].
     */
    void configure_voltage_pi_gains(real_t kp, duration_t ti);

    /**
     * @brief Configures output limits for the PI-generated field current reference.
     *
     * @param min_field_current_pu Minimum field current reference [pu].
     * @param max_field_current_pu Maximum field current reference [pu].
     */
    void configure_voltage_pi_output_limits(current_pu_t min_field_current_pu, current_pu_t max_field_current_pu);

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
    void update(speed_pu_t machine_speed_pu, voltage_pu_t machine_voltage_magnitude_pu, bool enable_excitation_command);

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

    /**
     * @brief Gets the current voltage reference used by the controller.
     * @return Voltage reference [pu].
     */
    voltage_pu_t get_machine_voltage_reference_pu() const noexcept;

    /**
     * @brief Gets the current voltage control error (Vref - Vmeas).
     * @return Voltage error [pu].
     */
    voltage_pu_t get_machine_voltage_error_pu() const noexcept;

private:
    // Sampling time
    const duration_t _sampling_time;

    // Control components
    // control::lookup_table_1d<real_t, NUM_LOOKUP_POINTS> _field_current_lookup_table;
    // control::rate_of_change_limiter _field_current_rate_limiter;
    control::elapsed_timer _flux_buildup_timer;

    control::lookup_table_1d<real_t, NUM_VOLTAGE_REFERENCE_POINTS> _voltage_reference_lookup_table;
    control::pi_controller<real_t> _voltage_pi_controller;

    // Configuration parameters
    current_pu_t _forced_field_current_pu{0.8};       // Forced current reference before release
    voltage_pu_t _release_voltage_threshold_pu{0.2};  // Voltage threshold for releasing PI control
    speed_pu_t _release_speed_threshold_pu{0.2};      // Speed threshold for releasing PI control
    duration_t _flux_buildup_duration{2.0};           // Time for flux to build up

    // Internal state flags
    bool _is_running{false};
    bool _is_field_established{false};
    // bool _is_ramping_down{false};

    // Outputs
    current_pu_t _field_current_reference_pu{0.0};
    bool _enable_excitation{false};

    voltage_pu_t _machine_voltage_reference_pu{0.0};
    voltage_pu_t _machine_voltage_error_pu{0.0};

    // PI mode tracking for bumpless transfer
    bool _is_voltage_pi_active{false};
    current_pu_t _last_forced_field_current_reference_pu{0.0};

    // PI output limits cache (for integrator preset clamping)
    current_pu_t _voltage_pi_output_min_pu{0.0};
    current_pu_t _voltage_pi_output_max_pu{2.0};

    // Last known good values for fault handling
    // speed_pu_t _last_valid_speed{0.0};
};

}  // namespace yeet::application::sfc
