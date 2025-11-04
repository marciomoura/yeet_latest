/**
 * @file sfc_machine_side_external_ios_datamodel.hpp
 * @brief Machine-side external I/O datamodel
 *
 * Provides parameter-based override interface for all external signals flowing in and out of
 * the machine-side control system:
 *
 * **Inputs:**
 * - Analog measurements (ADC counts from hardware mock)
 * - Fiber-optic signals from grid-side (operating_mode, current_reference, pulse_releases, etc.)
 * - Excitation control commands (start, stop, ready status)
 * - Speed generator profile commands (creep, purge, ignition, assist, hold, decelerate, idle)
 *
 * **Outputs:**
 * - Excitation controller bridge signals
 * - Speed generator profile selection signals
 * - Thyristor firing pulses to external I/O
 *
 * **Override Capability:**
 * Tests can enable override flags and set mock parameter values to inject signals without
 * modifying plant builder or control logic. This provides clean test/simulation interfaces.
 *
 * **Usage Pattern:**
 * 1. Plant builder calls update_analog_inputs() with raw ADC counts at 200µs
 * 2. Plant builder calls update_fiber_optics_inputs() at 1ms with grid-side outputs
 * 3. Plant builder calls update_excitation_inputs() at 10ms with excitation commands
 * 4. Plant builder calls update_speed_generator_inputs() at 10ms with profile commands
 * 5. Plant builder reads from get_thyristor_pulse_outputs() for external I/O
 *
 * @see sfc_grid_side_external_ios_datamodel for parallel grid-side implementation
 */

#pragma once

#include "sfc/sfc_per_unit_system.hpp"
#include "sfc/sfc_state_machine.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/six_pulse_generator.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Machine-side external I/O datamodel
 *
 * Provides parameter-based override interface for all external signals flowing in and out of
 * the machine-side control system. Mirrors the structure and style of sfc_grid_side_external_ios_datamodel
 * to maintain consistency across both sides of the SFC converter.
 *
 * **Signal Categories:**
 * - Analog inputs: ADC counts from hardware mock (voltage measurements)
 * - Fiber-optics inputs: Cross-coupling signals from grid-side (operating_mode, DC link current, etc.)
 * - Excitation outputs: Commands and status for excitation controller
 * - Speed generator outputs: Profile selection signals for turbine starting sequence
 * - Thyristor output: Firing pulses to external I/O
 *
 * **Override Capability:**
 * Tests can enable override flags and set mock parameter values to inject signals without
 * modifying plant builder or control logic. This provides clean test/simulation interfaces.
 *
 * @see sfc_grid_side_external_ios_datamodel for parallel grid-side implementation
 */
class sfc_machine_side_external_ios_datamodel {
public:
    // ==================== PARAMETERS (Override Control) ====================

    /**
     * @brief Parameters for test/simulation signal injection
     *
     * Override flags enable parameter-based mocking:
     * - override_excitation_inputs: Mock excitation control commands
     * - override_speed_generator_profiles: Mock speed generator profile commands
     * - override_fiber_optics_inputs: Mock grid-side cross-coupling signals
     *
     * When override flag is enabled, the corresponding update_*() method will use
     * parameter values instead of the signals provided to the update call.
     */
    struct parameters {
        // Excitation control input overrides
        yeet::parameter<bool> override_excitation_inputs{false};
        yeet::parameter<bool> exciter_ready{false};
        yeet::parameter<bool> excitation_start_command{false};
        yeet::parameter<bool> excitation_stop_command{false};

        // Speed generator profile selection overrides (turbine starting sequence)
        yeet::parameter<bool> override_speed_generator_profiles{false};
        yeet::parameter<bool> run_creep_speed{false};     ///< Profile 1: Creep speed (~5% rated)
        yeet::parameter<bool> run_purge_accel{false};     ///< Profile 2: Purge acceleration (~20% rated)
        yeet::parameter<bool> run_ignition_accel{false};  ///< Profile 3: Ignition acceleration (~55% rated)
        yeet::parameter<bool> run_assist_accel{false};    ///< Profile 4: Assist to self-sustaining (~95% rated)
        yeet::parameter<bool> hold_speed{false};          ///< Hold current speed setpoint
        yeet::parameter<bool> decelerate{false};          ///< Controlled deceleration to zero
        yeet::parameter<bool> idle{false};                ///< Idle mode (zero torque/speed outputs)

        // Fiber-optics input overrides from grid-side
        yeet::parameter<bool> override_fiber_optics_inputs{false};
        yeet::parameter<sfc_operating_mode_t> operating_mode{
            sfc_operating_mode_t::natural_commutation};                    ///< SFC operating mode (pulse or natural)
        yeet::parameter<bool> force_commutation_acknowledged{false};       ///< Forced commutation acknowledgement
        yeet::parameter<bool> is_current_zero{false};                      ///< Machine current below threshold
        yeet::parameter<bool> is_saturated{false};                         ///< Grid current controller saturated
        yeet::parameter<bool> pulse_releases{false};                       ///< Grid pulse releases enable
        yeet::parameter<current_pu_t> dc_link_current{current_pu_t{0.0}};  ///< DC link current from grid-side
        yeet::parameter<bool> freeze_integrator{false};                    ///< Freeze speed controller integrator
        yeet::parameter<bool> is_tripped{false};                           ///< Grid-side trip status
        yeet::parameter<bool> preset_request{false};                       ///< Preset request from grid-side
        yeet::parameter<yeet::application::sfc::sfc_state_id> current_state_id{
            yeet::application::sfc::sfc_state_id::IDLE};  ///< Grid-side state machine state ID
    };

    // ==================== ANALOG INPUTS (ADC Measurements) ====================

    /**
     * @brief Analog input measurements from hardware mock ADC
     *
     * **Machine-Side Architecture:** Measures voltage ONLY (sensorless current)
     * - Voltage: V_AB, V_BC (secondary of step-down transformer)
     * - Current: Not measured (sensorless operation - uses grid-side DC link current)
     *
     * These raw ADC counts flow through measurement runners:
     * - Measurement runner: Converts ADC counts → voltage in per-unit
     * - Observer: Uses voltage measurements for rotor angle/speed estimation
     *
     * Flow: Hardware mock ADC → update_analog_inputs() → measurement_runner → observer
     */
    struct analog_inputs_t {
        signal_output<yeet::control::adc_counts> v_ab_counts{};  ///< Machine voltage AB (ADC counts)
        signal_output<yeet::control::adc_counts> v_bc_counts{};  ///< Machine voltage BC (ADC counts)
    };

    // ==================== FIBER-OPTICS INPUTS FROM GRID-SIDE ====================

    /**
     * @brief Signals FROM grid-side TO machine-side (via fiber-optics)
     *
     * These are **inputs** to the machine-side datamodel perspective.
     * These are **outputs** from the grid-side datamodel perspective.
     *
     * **Signal Sources (Grid-Side Components):**
     * - operating_mode: Grid-side operating_mode_monitor (pulse or natural commutation)
     * - force_commutation_acknowledged: Grid-side state machine handshake ACK
     * - is_current_zero: Grid-side current detection (used for force commutation)
     * - is_saturated: Grid current controller saturation flag
     * - pulse_releases: Grid-side state machine enable (system running)
     * - dc_link_current: Grid-side DC link measurements (machine-side is sensorless)
     * - freeze_integrator: Speed controller integrator freeze for pulse-mode
     * - is_tripped: Grid-side trip status
     * - current_state_id: Grid-side state machine state (for profile awareness)
     *
     * **Timing:** Updated at 1ms (after grid-side update, before machine-side uses them)
     *
     * **Override Capability:** Tests can set override_fiber_optics_inputs flag
     * to inject mock values instead of real grid-side outputs.
     *
     * Flow: Grid-side components → machine-side external_ios → internal components
     */
    struct fiber_optics_inputs_t {
        signal_output<sfc_operating_mode_t> operating_mode{
            sfc_operating_mode_t::natural_commutation};                  ///< SFC operating mode
        signal_output<bool> force_commutation_acknowledged{false};       ///< Forced commutation ACK
        signal_output<bool> is_current_zero{false};                      ///< DC current zero indicator
        signal_output<bool> is_saturated{false};                         ///< Grid controller saturated
        signal_output<bool> pulse_releases{false};                       ///< Grid system running
        signal_output<current_pu_t> dc_link_current{current_pu_t{0.0}};  ///< Grid-measured DC current [pu]
        signal_output<bool> freeze_integrator{false};                    ///< PI integrator freeze
        signal_output<bool> is_tripped{false};                           ///< Grid-side trip status
        signal_output<bool> preset_request{false};                       ///< Preset request from grid-side
        signal_output<yeet::application::sfc::sfc_state_id> current_state_id{
            yeet::application::sfc::sfc_state_id::IDLE};  ///< Grid-side state ID
    };

    // ==================== EXCITATION OUTPUTS (Bridge Signals) ====================

    /**
     * @brief Excitation control output signals (bridge to external excitation controller)
     *
     * These signals drive the external excitation controller through a bridge interface.
     * Allows external systems to monitor exciter status and issue control commands.
     *
     * Flow: External excitation system → machine_side_external_ios → excitation_controller
     */
    struct excitation_outputs_t {
        signal_output<bool> exciter_ready;             ///< Excitation system ready status
        signal_output<bool> excitation_start_command;  ///< Start excitation command
        signal_output<bool> excitation_stop_command;   ///< Stop excitation command
    };

    // ==================== SPEED GENERATOR OUTPUTS (Profile Selection) ====================

    /**
     * @brief Speed generator profile selection output signals (turbine starting sequence)
     *
     * These signals select which turbine starting profile the speed generator uses.
     * Each profile defines a specific acceleration ramp with distinct speed targets.
     *
     * Flow: External turbine controller → machine_side_external_ios → speed_generator
     */
    struct speed_generator_outputs_t {
        signal_output<bool> run_creep_speed;     ///< Profile 1: Creep speed (~5% rated)
        signal_output<bool> run_purge_accel;     ///< Profile 2: Purge acceleration (~20% rated)
        signal_output<bool> run_ignition_accel;  ///< Profile 3: Ignition acceleration (~55% rated)
        signal_output<bool> run_assist_accel;    ///< Profile 4: Assist to self-sustaining (~95% rated)
        signal_output<bool> hold_speed;          ///< Hold current speed setpoint
        signal_output<bool> decelerate;          ///< Controlled deceleration to zero
        signal_output<bool> idle;                ///< Idle mode (zero torque/speed outputs)
    };

    // ==================== THYRISTOR OUTPUT (Pulse Control) ====================

    /**
     * @brief Thyristor firing pulse output to external gate driver
     *
     * Contains the 6-pulse firing signals for the machine-side thyristor converter.
     * These pulses are generated by the machine_side_pulse_generator and output
     * to the gate driver via the external I/O interface.
     *
     * Flow: machine_side_pulse_generator → machine_side_external_ios → gate_driver
     */
    struct thyristor_control_output_t {
        signal_input<yeet::control::six_pulse_generator::pulses> firing_pulses;  ///< 6-pulse firing signals
    };

    sfc_machine_side_external_ios_datamodel();

    // ==================== GETTERS (const and non-const) ====================

    /**
     * @brief Get parameters structure (const)
     * @return Const reference to parameters
     */
    const parameters& get_parameters() const { return _parameters; }

    /**
     * @brief Get parameters structure (mutable)
     * @return Mutable reference to parameters (for tests to set override flags)
     */
    parameters& get_parameters() { return _parameters; }

    /**
     * @brief Get analog inputs structure (const)
     * @return Const reference to analog_inputs
     */
    const analog_inputs_t& get_analog_inputs() const { return _analog_inputs; }

    /**
     * @brief Get analog inputs structure (mutable)
     * @return Mutable reference to analog_inputs
     */
    analog_inputs_t& get_analog_inputs() { return _analog_inputs; }

    /**
     * @brief Get fiber-optics inputs from grid-side (const)
     *
     * Machine-side reads these signals via this getter to know grid-side state
     *
     * @return Const reference to fiber_optics_inputs
     */
    const fiber_optics_inputs_t& get_fiber_optics_inputs() const { return _fiber_optics_inputs; }

    /**
     * @brief Get fiber-optics inputs from grid-side (mutable)
     * @return Mutable reference to fiber_optics_inputs
     */
    fiber_optics_inputs_t& get_fiber_optics_inputs() { return _fiber_optics_inputs; }

    /**
     * @brief Get excitation output signals (const)
     * @return Const reference to excitation outputs
     */
    const excitation_outputs_t& get_excitation_outputs() const { return _excitation_outputs; }

    /**
     * @brief Get excitation output signals (mutable)
     * @return Mutable reference to excitation outputs
     */
    excitation_outputs_t& get_excitation_outputs() { return _excitation_outputs; }

    /**
     * @brief Get speed generator output signals (const)
     * @return Const reference to speed generator outputs
     */
    const speed_generator_outputs_t& get_speed_generator_outputs() const { return _speed_generator_outputs; }

    /**
     * @brief Get speed generator output signals (mutable)
     * @return Mutable reference to speed generator outputs
     */
    speed_generator_outputs_t& get_speed_generator_outputs() { return _speed_generator_outputs; }

    /**
     * @brief Get thyristor output structure (mutable)
     * @return Mutable reference to thyristor_control_output
     */
    thyristor_control_output_t& get_thyristor_control_output() { return _thyristor_output; }

    // ==================== UPDATE METHODS (Called from Plant Builder) ====================

    /**
     * @brief Update analog inputs from hardware mock ADC
     *
     * Called at 200µs (5 kHz) with raw ADC counts from machine-side hardware mock.
     * Stores counts in analog_inputs struct for measurement runner consumption.
     *
     * **Parameters:**
     * @param v_ab_counts Machine voltage AB differential (ADC counts)
     * @param v_bc_counts Machine voltage BC differential (ADC counts)
     *
     * **Timing:** Called at 200µs from plant builder
     *
     * **Override Capability:** Not overrideable (ADC measurements are primary)
     */
    void update_analog_inputs(yeet::control::adc_counts v_ab_counts, yeet::control::adc_counts v_bc_counts);

    /**
     * @brief Update fiber-optics inputs from grid-side
     *
     * Called at 1ms (after grid-side update) to capture grid-side outputs for
     * machine-side use. Respects override_fiber_optics_inputs flag if set for test injection.
     *
     * **Parameters:**
     * @param operating_mode Grid operating mode (pulse or natural commutation)
     * @param current_state_id Grid-side state machine current state ID
     * @param force_commutation_acknowledged Forced commutation handshake acknowledgement
     * @param is_current_zero DC link current below threshold indicator
     * @param is_saturated Grid current controller at output limits
     * @param pulse_releases Pulses enabled / grid system running flag
     * @param dc_link_current Measured DC link current in per-unit
     * @param freeze_integrator Speed controller integrator freeze signal
     * @param is_tripped Grid-side trip status
     *
     * **Timing:** Called at 1ms after grid-side update
     *
     * **Override Logic:** When override_fiber_optics_inputs is true,
     * uses parameter values instead of the values provided to this method.
     *
     * **Plant Builder Usage:**
     * ```cpp
     * machine_side_control.external_ios.update_fiber_optics_inputs(
     *     grid_side_control.get_external_ios().get_fiber_optics_outputs_to_machine().operating_mode.read(),
     *     grid_side_control.get_external_ios().get_fiber_optics_outputs_to_machine().current_state_id.read(),
     *     // ... etc (9 signals total)
     * );
     * ```
     */
    void update_fiber_optics_inputs(sfc_operating_mode_t operating_mode,
        yeet::application::sfc::sfc_state_id current_state_id,
        bool force_commutation_acknowledged,
        bool is_current_zero,
        bool is_saturated,
        bool pulse_releases,
        current_pu_t dc_link_current,
        bool freeze_integrator,
        bool is_tripped,
        bool preset_request);

    /**
     * @brief Update excitation control inputs
     *
     * Called at 10ms with excitation system status and control commands.
     * Respects override_excitation_inputs flag if set for test injection.
     *
     * **Parameters:**
     * @param exciter_ready Excitation system ready status
     * @param excitation_start_command Start excitation command
     * @param excitation_stop_command Stop excitation command
     *
     * **Timing:** Called at 10ms from plant builder
     *
     * **Override Logic:** When override_excitation_inputs is true,
     * uses parameter values instead of the values provided to this method.
     */
    void update_excitation_inputs(bool exciter_ready, bool excitation_start_command, bool excitation_stop_command);

    /**
     * @brief Update speed generator profile selection inputs
     *
     * Called at 10ms with profile selection commands for the turbine starting sequence.
     * Respects override_speed_generator_profiles flag if set for test injection.
     *
     * **Parameters:**
     * @param run_creep_speed Profile 1: Creep speed
     * @param run_purge_accel Profile 2: Purge acceleration
     * @param run_ignition_accel Profile 3: Ignition acceleration
     * @param run_assist_accel Profile 4: Assist to self-sustaining
     * @param hold_speed Hold current speed setpoint
     * @param decelerate Controlled deceleration to zero
     * @param idle Idle mode (zero torque/speed outputs)
     *
     * **Timing:** Called at 10ms from plant builder
     *
     * **Override Logic:** When override_speed_generator_profiles is true,
     * uses parameter values instead of the values provided to this method.
     */
    void update_speed_generator_inputs(bool run_creep_speed,
        bool run_purge_accel,
        bool run_ignition_accel,
        bool run_assist_accel,
        bool hold_speed,
        bool decelerate,
        bool idle);

    /**
     * @brief Get thyristor pulse outputs
     *
     * Returns the current 6-pulse firing signals for external gate driver consumption.
     *
     * @return Current machine-side firing pulses
     */
    yeet::control::six_pulse_generator::pulses get_thyristor_pulse_outputs() const;

    /**
     * @brief Register datamodel with model registry for network/telemetry access
     *
     * Called during application initialization to make datamodel parameters available
     * through the network parameter interface and telemetry system.
     *
     * @param registry Model registry for parameter registration
     */
    void register_datamodel(yeet::model_registry& registry);

    /**
     * @brief Register parameter change callbacks
     *
     * Called during initialization to set up callbacks when override flags are changed.
     * Enables tests to dynamically enable/disable overrides without reinitialization.
     */
    void register_callbacks();

private:
    // ==================== PRIVATE OVERRIDE METHODS ====================

    /**
     * @brief Apply excitation input overrides if enabled
     * @return true if overrides were applied, false if real signals should be used
     */
    bool write_excitation_overrides();

    /**
     * @brief Apply speed generator profile overrides if enabled
     * @return true if overrides were applied, false if real signals should be used
     */
    bool write_speed_generator_overrides();

    /**
     * @brief Apply fiber-optics input overrides if enabled
     * @return true if overrides were applied, false if real signals should be used
     */
    bool write_fiber_optics_input_overrides();

    // ==================== MEMBER STATE ====================

    parameters _parameters;
    analog_inputs_t _analog_inputs;
    fiber_optics_inputs_t _fiber_optics_inputs;
    excitation_outputs_t _excitation_outputs;
    speed_generator_outputs_t _speed_generator_outputs;
    thyristor_control_output_t _thyristor_output;
};

}  // namespace yeet::application::sfc::integration
