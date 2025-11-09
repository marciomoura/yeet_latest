#pragma once

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
    frequency_t initial{1.0};   ///< Initial virtual frequency [Hz]
    frequency_t final{10.0};    ///< Final virtual frequency [Hz]
    duration_t duration{10.0};  ///< Duration of frequency ramp [s], 0 = skip virtual flux phase
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
    angle_t rotor_angle{0.0};           ///< Virtual rotor angle [rad]
    frequency_t rotor_frequency{0.0};   ///< Virtual rotor frequency [Hz]
    torque_pu_t torque_reference{0.0};  ///< Virtual torque reference [pu]
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
     * @brief Starts the startup sequence
     *
     * Called when the main state machine enters OPERATION_ON.
     * Transitions from IDLE to FLUX_BUILDUP state.
     *
     * Preconditions:
     * - System must be in safe state (standstill)
     * - Excitation system must be ready
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
     *
     * @param is_field_established Field excitation established flag from excitation controller
     * @param estimated_rotor_angle Angle from observer [rad]
     */
    void update(bool is_field_established, angle_t estimated_rotor_angle);

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
     * @return Virtual frequency [Hz]
     */
    frequency_t get_virtual_frequency() const noexcept;

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
    linear_ramp<frequency_t> _virtual_frequency_ramp;
    linear_ramp<torque_pu_t> _virtual_torque_ramp;
    control::elapsed_timer _state_elapsed_timer;
    control::integrator<angle_wrapped> _virtual_angle_integrator;

    // Configuration
    virtual_frequency_config _virtual_frequency_config;
    virtual_torque_config _virtual_torque_config;

    // State
    startup_sequencer_state_t _current_state{startup_sequencer_state_t::IDLE};
    startup_sequencer_state_t _previous_state{startup_sequencer_state_t::IDLE};
    startup_sequencer_outputs _outputs;

    // Virtual reference frame generation
    frequency_t _virtual_frequency{0.0};
    angle_t _virtual_rotor_angle{0.0};
    torque_pu_t _virtual_torque{0.0};

    // State transition methods
    void transition_to_flux_buildup();
    void transition_to_starting_procedure();
    void transition_to_closed_loop_operation();
    void transition_to_idle();

    // State update methods
    void update_idle();
    void update_flux_buildup(bool is_field_established);
    void update_starting_procedure();
    void update_closed_loop_operation();

    // Virtual reference frame generation
    bool update_virtual_reference_frame();
    bool update_virtual_torque_reference();
    void synchronize_virtual_to_estimated(angle_t estimated_angle);

    // Helper methods
    void update_outputs_for_current_state();
    void reset_timers();
};

}  // namespace yeet::application::sfc
