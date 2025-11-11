#pragma once

#include "sfc/sfc_state_machine.hpp"
#include "sfc/sfc_torque_controller.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/elapsed_timer.hpp"
#include "yeet_control/integrator.hpp"
#include "yeet_control/linear_ramp.hpp"
#include "yeet_control/rate_of_change_limiter.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Startup sequence states for machine-side converter
 *
 * The sequencer progresses through these states to safely bring the
 * synchronous machine from standstill to closed-loop speed control.
 */
enum class startup_sequencer_state_t {
    IDLE,                  ///< System not running, all outputs safe
    FLUX_BUILDUP,          ///< Field excitation establishment phase
    STARTING_PROCEDURE,    ///< Initial rotation with virtual reference frame
    CLOSED_LOOP_OPERATION  ///< Normal speed-controlled operation
};

/**
 * @brief Reference frame source selection
 *
 * Controls which angle source is used for the machine-side modulator.
 */
enum class reference_frame_source_t {
    VIRTUAL,   ///< Use generated virtual reference frame (open-loop V/Hz)
    ESTIMATED  ///< Use observer-estimated reference frame (closed-loop)
};

/**
 * @brief Virtual frequency profile configuration
 */
struct virtual_frequency_config {
    frequency_pu_t initial{0.02};  ///< Initial virtual frequency [pu] (e.g., 0.02 = 1Hz at 50Hz base)
    frequency_pu_t final{0.20};    ///< Final virtual frequency [pu] (e.g., 0.20 = 10Hz at 50Hz base)
    duration_t duration{10.0};     ///< Duration of frequency ramp [s], 0 = skip virtual flux phase
};

/**
 * @brief Virtual torque reference profile configuration
 */
struct virtual_torque_config {
    torque_pu_t initial{0.3};  ///< Initial virtual torque reference [pu]
    torque_pu_t final{0.3};    ///< Final virtual torque reference [pu]
    duration_t duration{0.0};  ///< Duration of torque ramp [s], 0 = immediate to final value
};

/**
 * @brief Speed reference rate limit configuration
 */
struct speed_reference_rate_limit_config {
    real_t seconds_per_pu{10.0};  ///< Rate limit in seconds per per-unit speed change
    bool enabled{true};           ///< Enable/disable rate limiting
};

/**
 * @brief Flying restart configuration
 */
struct flying_restart_config {
    speed_pu_t speed_threshold{0.05};    ///< Minimum speed for flying restart [pu] (5% rated)
    bool auto_start_stop_enabled{true};  ///< Enable automatic start/stop based on state machine
};

/**
 * @brief Flux build-up outputs
 */
struct flux_buildup_outputs {
    // Flux buildup is timer-based only, no outputs needed
    // Excitation controller manages its own setpoint
};

/**
 * @brief Virtual reference frame outputs
 */
struct virtual_reference_outputs {
    angle_wrapped rotor_angle{0.0};       ///< Virtual rotor angle [rad]
    frequency_pu_t rotor_frequency{0.0};  ///< Virtual rotor frequency [pu]
    torque_pu_t torque_reference{0.0};    ///< Virtual torque reference [pu]
};

/**
 * @brief Speed control outputs
 */
struct speed_control_outputs {
    speed_pu_t speed_reference{0.0};  ///< Rate-limited speed reference [pu]
};

/**
 * @brief Controller command outputs
 */
struct controller_command_outputs {
    reference_frame_source_t reference_frame_source{reference_frame_source_t::ESTIMATED};
    sfc_torque_controller::control_mode torque_control_mode{sfc_torque_controller::control_mode::SPEED_CONTROL};
    bool release_machine_side_pulses{false};
    bool release_excitation{false};
    bool release_machine_observer{false};
    bool release_speed_controller{false};
    bool release_torque_controller{false};
    startup_sequencer_state_t current_state{startup_sequencer_state_t::IDLE};
};

/**
 * @brief Complete outputs from the startup sequencer
 *
 * These outputs control the behavior of subordinate controllers and
 * the modulator during the startup sequence.
 */
struct startup_sequencer_outputs {
    flux_buildup_outputs flux_buildup;
    virtual_reference_outputs virtual_reference;
    speed_control_outputs speed_control;
    controller_command_outputs controller_commands;
};

/**
 * @brief Manages the machine-side startup sequence for LCI drives
 *
 * This component coordinates the complete startup procedure from standstill
 * to normal closed-loop operation. It manages:
 * - Flux build-up (pre-fluxing) to establish magnetic field
 * - Starting with virtual reference frame (open-loop V/Hz control)
 * - Transition to estimated reference frame (observer-based control)
 * - Speed controller enable/disable
 * - Torque control mode switching
 *
 * The sequencer operates as a sub-state machine within the main OPERATION_ON state
 * and ensures safe, repeatable startup behavior.
 *
 * @note This component follows the yeet_application_sfc architectural pattern:
 *       - All real-time inputs passed to update()
 *       - Configuration via configure() method
 *       - Outputs retrieved via get_outputs()
 *       - Explicit start/stop control
 */
class sfc_startup_sequencer {
public:
    /**
     * @brief Constructs the startup sequencer
     * @param sampling_time Control loop sampling time [s]
     */
    explicit sfc_startup_sequencer(duration_t sampling_time);

    /**
     * @brief Configures the virtual frequency profile
     *
     * @param config Virtual frequency configuration
     */
    void configure_virtual_frequency(const virtual_frequency_config& config);

    /**
     * @brief Configures the virtual torque reference profile
     *
     * @param config Virtual torque configuration
     */
    void configure_virtual_torque(const virtual_torque_config& config);

    /**
     * @brief Configures the speed reference rate limiter
     *
     * The rate limit is specified as seconds per per-unit of speed change.
     * For example, a value of 10.0 means it takes 10 seconds to change the
     * speed reference by 1.0 pu (0.1 pu/s rate).
     *
     * @param config Speed reference rate limit configuration
     */
    void configure_speed_reference_rate_limit(const speed_reference_rate_limit_config& config);

    /**
     * @brief Configures the flying restart parameters
     *
     * Flying restart allows the sequencer to skip the virtual reference phase
     * and jump directly to closed-loop operation if the machine is already spinning.
     *
     * @param config Flying restart configuration
     */
    void configure_flying_restart(const flying_restart_config& config);

    /**
     * @brief Starts the startup sequence
     *
     * Called when the main state machine enters OPERATION_ON or by manual command.
     * Transitions from IDLE to FLUX_BUILDUP or directly to CLOSED_LOOP if flying restart.
     *
     * Preconditions:
     * - System must be in safe state
     * - Excitation system must be ready (for normal start)
     *
     * @return True if sequence started successfully, false if preconditions not met
     */
    bool start_sequence();

    /**
     * @brief Stops the startup sequence
     *
     * Initiates controlled shutdown, returns sequencer to IDLE state.
     * Called when the main state machine exits OPERATION_ON or on fault.
     */
    void stop_sequence();

    /**
     * @brief Updates the sequencer state machine
     *
     * Must be called every control cycle with current measurements.
     * Advances through states based on conditions and timers.
     * Implements automatic start/stop based on state machine state if enabled.
     *
     * @param is_field_established Field excitation established flag from excitation controller
     * @param estimated_rotor_angle Angle from observer [rad]
     * @param measured_speed Speed from observer [pu] for flying restart detection
     * @param current_state_id Grid-side state machine state for auto-start/stop
     * @param external_speed_reference External speed reference input [pu]
     * @param base_frequency Nominal machine base frequency [Hz] for angle integration
     */
    void update(bool is_field_established,
        angle_wrapped estimated_rotor_angle,
        speed_pu_t measured_speed,
        sfc_state_id current_state_id,
        speed_pu_t external_speed_reference,
        frequency_t base_frequency);

    /**
     * @brief Resets the sequencer to initial state
     *
     * Clears all internal states, timers, and returns to IDLE.
     * Should be called during system initialization or after fault recovery.
     */
    void reset();

    /**
     * @brief Gets the current sequencer outputs
     *
     * Returns the complete output structure containing all control signals
     * for subordinate controllers and the modulator.
     *
     * @return Output structure with all control signals
     */
    const startup_sequencer_outputs& get_outputs() const noexcept;

    /**
     * @brief Gets the current state
     * @return Current sequencer state
     */
    startup_sequencer_state_t get_current_state() const noexcept;

    /**
     * @brief Checks if sequencer has completed startup
     *
     * Returns true when the sequencer has successfully reached
     * CLOSED_LOOP_OPERATION state and is ready for normal operation.
     *
     * @return True when in CLOSED_LOOP_OPERATION state
     */
    bool is_startup_complete() const noexcept;

    /**
     * @brief Gets the current virtual frequency
     *
     * Useful for monitoring and diagnostics during starting procedure.
     *
     * @return Virtual frequency [pu]
     */
    frequency_pu_t get_virtual_frequency() const noexcept;

    /**
     * @brief Gets elapsed time in current state
     *
     * Useful for monitoring state progression and debugging.
     *
     * @return Time elapsed in current state [s]
     */
    duration_t get_time_in_current_state() const noexcept;

private:
    // Timing
    const duration_t _sampling_time;

    // Control components
    linear_ramp<frequency_pu_t> _virtual_frequency_ramp;
    linear_ramp<torque_pu_t> _virtual_torque_ramp;
    control::elapsed_timer _state_elapsed_timer;
    control::integrator<angle_wrapped> _virtual_angle_integrator;
    control::rate_of_change_limiter _speed_reference_limiter;

    // Configuration
    virtual_frequency_config _virtual_frequency_config;
    virtual_torque_config _virtual_torque_config;
    speed_reference_rate_limit_config _speed_reference_rate_limit_config;
    flying_restart_config _flying_restart_config;

    // State
    startup_sequencer_state_t _current_state{startup_sequencer_state_t::IDLE};
    startup_sequencer_state_t _previous_state{startup_sequencer_state_t::IDLE};
    sfc_state_id _previous_state_machine_state{sfc_state_id::IDLE};
    bool _shutdown_speed_override_active{false};
    startup_sequencer_outputs _outputs;

    // Virtual reference frame generation
    frequency_pu_t _virtual_frequency{0.0};
    angle_wrapped _virtual_rotor_angle{0.0};
    torque_pu_t _virtual_torque{0.0};

    // State transition methods
    void transition_to_flux_buildup();
    void transition_to_starting_procedure();
    void transition_to_closed_loop_operation();
    void transition_to_idle();

    // State update methods
    void update_idle(speed_pu_t external_speed_reference);
    void update_flux_buildup(bool is_field_established);
    void update_starting_procedure(frequency_t base_frequency);
    void update_closed_loop_operation(speed_pu_t external_speed_reference);

    // Automatic start/stop coordination
    void update_state_machine_coordination(
        speed_pu_t measured_speed, sfc_state_id current_state_id, speed_pu_t& external_speed_reference);

    // State transition detection helpers
    bool transitioned_from(sfc_state_id from_state, sfc_state_id current_state) const noexcept;
    bool transitioned_to(sfc_state_id to_state, sfc_state_id current_state) const noexcept;
    bool transition(sfc_state_id from_state, sfc_state_id to_state, sfc_state_id current_state) const noexcept;

    // Virtual reference frame generation
    bool update_virtual_reference_frame(frequency_t base_frequency);
    bool update_virtual_torque_reference();
    void synchronize_virtual_to_estimated(angle_wrapped estimated_angle);

    // Helper methods
    void update_outputs_for_current_state();
    void reset_timers();
};

}  // namespace yeet::application::sfc
