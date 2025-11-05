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
 * @brief Manages excitation voltage control for LCI-driven synchronous machines.
 *
 * This component generates a time-varying excitation voltage setpoint that maintains
 * a constant Volts-per-Hertz (V/Hz) ratio during machine acceleration. The algorithm
 * uses a pre-commissioned lookup table that maps rotor speed to the required excitation
 * voltage setpoint, derived from the machine's open-circuit saturation curve.
 *
 * The controller follows the standard yeet_application_sfc architectural pattern:
 * - Explicit enable/disable via start_excitation() and stop_excitation()
 * - All real-time inputs passed to update()
 * - Configuration via configure_*() methods
 * - Outputs retrieved via get_*() methods
 * - Timer-based field establishment detection when feedback unavailable
 *
 * Key Features:
 * - Pre-fluxing sequence with configurable duration
 * - Speed-dependent excitation voltage via lookup table
 * - Controlled voltage ramp-down on shutdown
 * - Automatic fault handling (exciter not ready)
 */
class sfc_excitation_controller {
public:
    // Default lookup table dimensions
    static constexpr size_t NUM_LOOKUP_POINTS = 6;

    // Default speed-to-excitation-voltage mapping (conservative safe values)
    // Speed axis in per-unit (0.0 = standstill, 1.0 = rated speed)
    static constexpr std::array<real_t, NUM_LOOKUP_POINTS> kDefaultSpeedAxisPu = {0.00, 0.05, 0.80, 0.95, 1.00, 1.05};

    // Excitation voltage setpoint axis in per-unit (illustrative values for documentation)
    static constexpr std::array<real_t, NUM_LOOKUP_POINTS> kDefaultExcitationVoltageSetpointPu = {
        0.3, 0.3, 1.0, 0.8, 0.73, 0.7};

    /**
     * @brief Constructs the excitation controller.
     * @param sampling_time The control loop sampling time [s].
     */
    explicit sfc_excitation_controller(duration_t sampling_time);

    /**
     * @brief Configures the excitation voltage setpoint lookup table.
     *
     * This table defines the speed-to-excitation-voltage mapping that achieves
     * the desired V/Hz profile. The table must be commissioned based on
     * the specific machine's saturation characteristics.
     *
     * @param speed_axis_pu Array of speed breakpoints [pu], must be sorted ascending.
     * @param excitation_voltage_setpoint Array of corresponding excitation voltage setpoints [pu].
     */
    void configure_excitation_voltage_lookup_table(const std::array<real_t, NUM_LOOKUP_POINTS>& speed_axis_pu,
        const std::array<real_t, NUM_LOOKUP_POINTS>& excitation_voltage_setpoint);

    /**
     * @brief Configures the pre-fluxing duration.
     *
     * This is the time to wait at standstill with initial excitation voltage applied,
     * allowing the field winding to fully energize before torque production.
     * Typical value: 3-5 times the field winding L/R time constant.
     *
     * @param prefluxing_duration Duration to maintain initial flux [s].
     */
    void configure_prefluxing_duration(duration_t prefluxing_duration);

    /**
     * @brief Configures the maximum timeout for field establishment.
     *
     * If no field current feedback is available, the controller assumes
     * the field is established after this timeout expires (assuming no faults).
     *
     * @param timeout Maximum time to wait for field establishment [s].
     */
    void configure_field_established_timeout(duration_t timeout);

    /**
     * @brief Configures the excitation voltage setpoint ramp-down time on shutdown.
     *
     * Controls how quickly excitation voltage is reduced to zero when stop_excitation()
     * is called. A gradual ramp prevents voltage transients.
     *
     * @param ramp_down_time Time to ramp from current value to zero [s].
     */
    void configure_excitation_ramp_down_time(duration_t ramp_down_time);

    /**
     * @brief Starts the excitation sequence.
     *
     * Preconditions:
     * - exciter_ready must be true
     * - Controller must not already be running
     *
     * Actions:
     * - Enables excitation output
     * - Sets excitation voltage setpoint to initial value (0% speed point)
     * - Starts pre-fluxing timer
     *
     * @return True if started successfully, false if preconditions not met.
     */
    bool start_excitation();

    /**
     * @brief Stops the excitation sequence.
     *
     * Actions:
     * - Initiates controlled ramp-down of excitation voltage setpoint
     * - Disables excitation output after ramp completes
     * - Resets all internal timers
     */
    void stop_excitation();

    /**
     * @brief Executes one update cycle of the excitation controller.
     *
     * This method must be called every control cycle with current real-time data.
     *
     * Behavior:
     * - If not running: outputs remain disabled
     * - If pre-fluxing: maintains initial excitation voltage setpoint, monitors timer
     * - If field established: updates excitation voltage setpoint from lookup table
     * - Monitors exciter health, stops automatically on fault
     *
     * @param machine_rotor_speed_pu Current rotor speed [pu].
     * @param exciter_ready Health status from external exciter (true = healthy).
     */
    void update(speed_pu_t machine_rotor_speed_pu, bool exciter_ready);

    /**
     * @brief Resets all internal states.
     *
     * Returns the controller to initial idle state. Should be called
     * during system initialization or after fault recovery.
     */
    void reset();

    /**
     * @brief Gets the current excitation voltage setpoint.
     * @return Excitation voltage setpoint to send to exciter [pu].
     */
    voltage_pu_t get_excitation_voltage_setpoint_pu() const noexcept;

    /**
     * @brief Gets the excitation enable command.
     * @return True = enable exciter, false = disable exciter.
     */
    bool get_enable_excitation() const noexcept;

    /**
     * @brief Checks if the pre-fluxing sequence is complete.
     * @return True if field is established and ready for torque production.
     */
    bool get_is_field_established() const noexcept;

    /**
     * @brief Checks if the controller is currently running.
     * @return True if excitation sequence is active.
     */
    bool get_is_running() const noexcept;

    /**
     * @brief Gets the current elapsed time in pre-fluxing phase.
     * @return Pre-fluxing timer value [s], zero if not in pre-fluxing.
     */
    duration_t get_prefluxing_elapsed_time() const noexcept;

private:
    // Sampling time
    const duration_t _sampling_time;

    // Control components
    control::lookup_table_1d<real_t, NUM_LOOKUP_POINTS> _excitation_voltage_lookup_table;
    control::rate_of_change_limiter _excitation_voltage_rate_limiter;
    control::elapsed_timer _prefluxing_timer;

    // Configuration parameters
    duration_t _prefluxing_duration{2.0};
    duration_t _field_established_timeout{5.0};

    // Internal state flags
    bool _is_running{false};
    bool _is_field_established{false};
    bool _is_ramping_down{false};

    // Outputs
    voltage_pu_t _excitation_voltage_setpoint_pu{0.0};
    bool _enable_excitation{false};

    // Last known good values for fault handling
    speed_pu_t _last_valid_speed{0.0};
};

}  // namespace yeet::application::sfc
