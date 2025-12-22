/**
 * @file sfc_machine_side_external_ios_datamodel.hpp
 * @brief Machine-side external I/O datamodel
 *
 * Provides parameter-based override interface for all external signals flowing in and out of
 * the machine-side control system:
 *
 * **Inputs:**
 * - Analog measurements (ADC counts from hardware mock)
 * - Fiber-optic signals from grid-side (operating_mode, current_reference, pulse_releases, state_id, etc.)
 * - Excitation control commands (start, stop, ready status)
 * - Startup sequencer control commands (coordinated with grid-side state machine)
 *
 * **Outputs:**
 * - Excitation controller bridge signals
 * - Startup sequencer control signals
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
 * 4. Plant builder calls update_startup_sequencer_inputs() at 10ms with sequencer commands
 * 5. Plant builder reads from get_thyristor_pulse_outputs() for external I/O
 *
 * @see sfc_grid_side_external_ios_datamodel for parallel grid-side implementation
 */

#pragma once

#include "sfc/sfc_fiber_optic_protocol.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc/sfc_state_machine.hpp"
#include "sfc/sfc_types.hpp"
#include "sfc_integration/sfc_io_types.hpp"
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
     * Override flags enable parameter-based mocking for testing.
     * Each digital input/output has a pair of parameters:
     * - di_<signal>_override / do_<signal>_override: Enable override for this signal
     * - di_<signal>_override_value / do_<signal>_override_value: Value to use when override enabled
     */
    struct parameters {
        // ==================== Digital Input Overrides ====================
        /// Enable override for excitation_status input
        yeet::parameter<bool> di_excitation_status_override{false};
        /// Override value for excitation_status (when override enabled)
        yeet::parameter<bool> di_excitation_status_override_value{false};

        /// Enable override for external_fault input
        yeet::parameter<bool> di_external_fault_override{false};
        /// Override value for external_fault (when override enabled)
        yeet::parameter<bool> di_external_fault_override_value{false};

        /// Enable override for start_command input
        yeet::parameter<bool> di_start_command_override{false};
        /// Override value for start_command (when override enabled)
        yeet::parameter<bool> di_start_command_override_value{false};

        // ==================== Digital Output Overrides ====================
        /// Enable override for excitation_on_command output
        yeet::parameter<bool> do_excitation_on_command_override{false};
        /// Override value for excitation_on_command (when override enabled)
        yeet::parameter<bool> do_excitation_on_command_override_value{false};

        /// Enable override for excitation_off_command output
        yeet::parameter<bool> do_excitation_off_command_override{false};
        /// Override value for excitation_off_command (when override enabled)
        yeet::parameter<bool> do_excitation_off_command_override_value{false};

        // ==================== Analog Input Overrides ====================
        /// Enable override for excitation_current_actual input
        yeet::parameter<bool> ai_excitation_current_actual_override{false};
        /// Override value for excitation_current_actual (when override enabled)
        yeet::parameter<current_pu_t> ai_excitation_current_actual_override_value{current_pu_t{0.0}};

        // ==================== Analog Output Overrides ====================
        /// Enable override for excitation_current_reference output
        yeet::parameter<bool> ao_excitation_current_reference_override{false};
        /// Override value for excitation_current_reference (when override enabled)
        yeet::parameter<current_pu_t> ao_excitation_current_reference_override_value{current_pu_t{0.0}};

        // ==================== Analog Output Scaling Parameters ====================
        /// Gain for excitation_current_reference output (scaling factor)
        yeet::parameter<real_t> ao_excitation_current_reference_gain{real_t{1.0}};
        /// Offset for excitation_current_reference output (added after gain)
        yeet::parameter<real_t> ao_excitation_current_reference_offset{real_t{0.0}};
        /// Minimum value for excitation_current_reference output (after gain and offset)
        yeet::parameter<real_t> ao_excitation_current_reference_minimum{real_t{0.0}};
        /// Maximum value for excitation_current_reference output (after gain and offset)
        yeet::parameter<real_t> ao_excitation_current_reference_maximum{real_t{1.0}};

        // ==================== IO Index Mapping (IO Channel Selection) ====================
        // Naming convention: di_{name}_index, do_{name}_index, ao_{name}_index.
        // N comes from TASK_IO_NUM_DIGITAL_INPUTS / TASK_IO_NUM_DIGITAL_OUTPUTS / TASK_IO_NUM_ANALOG_OUTPUTS.

        // Digital Input Indices
        yeet::parameter<int> di_excitation_status_index{0};
        yeet::parameter<int> di_external_fault_index{1};
        yeet::parameter<int> di_start_command_index{3};

        // Digital Output Indices
        yeet::parameter<int> do_excitation_on_command_index{0};
        yeet::parameter<int> do_excitation_off_command_index{1};

        yeet::parameter<int> ao_excitation_current_reference_index{0};

        // ==================== IO Polarity Configuration ====================
        // Configure signal polarity (active high or active low)

        // Digital Input Polarity
        yeet::parameter<io_polarity_t> di_excitation_status_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_external_fault_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_start_command_polarity{io_polarity_t::ACTIVE_HIGH};

        // Digital Output Polarity
        yeet::parameter<io_polarity_t> do_excitation_on_command_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_excitation_off_command_polarity{io_polarity_t::ACTIVE_HIGH};
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

    // ==================== FIBER-OPTICS OUTPUTS TO GRID-SIDE ====================

    /**
     * @brief Signals FROM machine-side TO grid-side (via fiber-optics)
     *
     * These are **outputs** from the machine-side datamodel perspective.
     * These are **inputs** to the grid-side datamodel perspective (fiber_optics_inputs_from_machine).
     *
     * **Signal Sources (Machine-Side Components):**
     * - measured_speed: From machine_side_observer.estimated_speed
     * - dc_link_current_reference: From torque_controller.current_reference
     * - machine_voltage_magnitude: From machine_side_observer.estimated_voltage_magnitude
     * - force_commutation_request: From machine_side_modulator (when low-speed assist needed)
     * - machine_protection_tripped: From protection system
     * - protection_inhibit: Protection system inhibited (awaiting acknowledgment)
     * - machine_trip_acknowledged: Machine-side trip acknowledgment
     *
     * **Timing:** Updated at 1ms (after machine-side update, before grid-side update)
     *
     * Flow: Machine-side components → fiber_optics_outputs_to_grid → fiber optic protocol → grid-side
     */
    struct fiber_optics_outputs_to_grid_t {
        signal_output<speed_pu_t> measured_speed{speed_pu_t{0.0}};                 ///< Measured rotor speed [pu]
        signal_output<current_pu_t> dc_link_current_reference{current_pu_t{0.0}};  ///< DC link current reference [pu]
        signal_output<voltage_pu_t> machine_voltage_magnitude{voltage_pu_t{0.0}};  ///< Machine voltage magnitude [pu]
        signal_output<bool> force_commutation_request{false};                      ///< Force commutation requested
        signal_output<bool> machine_protection_tripped{false};                     ///< Machine-side protections tripped
        signal_output<bool> protection_inhibit{false};         ///< Protection inhibited (awaiting ack)
        signal_output<bool> machine_trip_acknowledged{false};  ///< Machine-side trip acknowledgment
        signal_output<bool> communication_ok{false};           ///< Machine-side communication status (receiving grid)
    };

    // ==================== FIBER-OPTICS INPUTS FROM GRID-SIDE ====================

    /**
     * @brief Signals FROM grid-side TO machine-side (via fiber-optics)
     *
     * These are **inputs** to the machine-side control system.
     * These are **outputs** from the grid-side control system.
     *
     * **Signal Sources (Grid-Side Components):**
     * - operating_mode: Grid-side state machine (pulse or natural commutation mode)
     * - force_commutation_acknowledged: Grid-side state machine handshake ACK
     * - release_pulses: Grid-side state machine enable (system running)
     * - dc_link_current: Grid-side DC link measurements (machine-side is sensorless)
     * - grid_voltage_magnitude: Grid-side voltage magnitude observer output
     * - current_state_id: Grid-side state machine current state ID
     * - grid_release_protections: Grid-side protection release signal (from state machine)
     * - machine_release_protections: Machine-side protection release signal (via fiber optics from grid state machine)
     * - grid_trip_acknowledged: Grid-side trip acknowledgment (from state machine)
     *
     * **Timing:** Updated at 1ms (after grid-side update, before machine-side uses them)
     *
     * Flow: Grid-side components → fiber optic protocol → machine-side external_ios → internal components
     */
    struct fiber_optics_inputs_from_grid_t {
        external_signal<sfc_operating_mode_t> operating_mode{
            sfc_operating_mode_t::natural_commutation};                    ///< SFC operating mode
        external_signal<bool> force_commutation_acknowledged{false};       ///< Forced commutation ACK
        external_signal<bool> release_pulses{false};                       ///< Grid system running / pulses released
        external_signal<current_pu_t> dc_link_current{current_pu_t{0.0}};  ///< Grid-measured DC current [pu]
        external_signal<voltage_pu_t> grid_voltage_magnitude{voltage_pu_t{0.0}};  ///< Grid voltage magnitude [pu]
        external_signal<yeet::application::sfc::sfc_state_id> current_state_id{
            yeet::application::sfc::sfc_state_id::IDLE};        ///< Grid-side state ID
        external_signal<bool> grid_release_protections{false};  ///< Grid protection release (from grid state machine)
        external_signal<bool> machine_release_protections{false};  ///< Machine protection release (via fiber optics)
        external_signal<bool> communication_ok{false};  ///< Grid-side communication status (receiving machine)
    };

    // ==================== DIGITAL OUTPUTS ====================

    /**
     * @brief Digital output signals from machine-side control
     *
     * These are OUTPUT commands from the machine-side control system.
     * Wired to physical digital output pins via index parameters.
     */
    struct digital_outputs_t {
        signal_input<bool> excitation_on_command;   ///< Excitation controller ON command
        signal_input<bool> excitation_off_command;  ///< Excitation controller OFF command
    };

    // ==================== EXCITATION OUTPUTS (Analog Reference) ====================

    /**
     * @brief Excitation analog output (current reference)
     *
     * Analog output command from the excitation controller to the external excitation system.
     * The excitation_controller_runner generates this reference based on control logic.
     *
     * Flow: excitation_controller → external_ios → embedded/hardware → excitation system
     */
    struct excitation_outputs_t {
        signal_input<current_pu_t> excitation_current_reference;  ///< Excitation current reference (AO)
    };

    // ==================== DIGITAL INPUTS ====================

    /**
     * @brief Digital input signals to machine-side control
     *
     * These are INPUT signals from external systems.
     * Wired from physical digital input pins via index parameters.
     */
    struct digital_inputs_t {
        signal_output<bool> excitation_status;  ///< Excitation controller status feedback
        signal_output<bool> external_fault;     ///< External fault input (OR'd with protections)
        signal_output<bool> start_command;      ///< Start command (used for alarm confirmation)
    };

    // ==================== EXCITATION INPUTS (Analog Feedback) ====================

    /**
     * @brief Excitation analog input (current feedback)
     *
     * Analog input signal from the external excitation system.
     * Optional signal for monitoring and closed-loop control.
     *
     * Flow: excitation system → embedded/hardware → external_ios → excitation_controller
     */
    struct excitation_inputs_t {
        signal_output<current_pu_t> excitation_current_actual;  ///< Actual excitation current (AI - optional)
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
     * @brief Get fiber-optics outputs to grid-side (const)
     *
     * Grid-side reads these signals via this getter to know machine-side state
     *
     * @return Const reference to fiber_optics_outputs_to_grid
     */
    const fiber_optics_outputs_to_grid_t& get_fiber_optics_outputs_to_grid() const
    {
        return _fiber_optics_outputs_to_grid;
    }

    /**
     * @brief Get fiber-optics outputs to grid-side (mutable)
     * @return Mutable reference to fiber_optics_outputs_to_grid
     */
    fiber_optics_outputs_to_grid_t& get_fiber_optics_outputs_to_grid() { return _fiber_optics_outputs_to_grid; }

    /**
     * @brief Get fiber-optics inputs from grid-side (const)
     *
     * Machine-side reads these signals via this getter to know grid-side state
     *
     * @return Const reference to fiber_optics_inputs_from_grid
     */
    const fiber_optics_inputs_from_grid_t& get_fiber_optics_inputs_from_grid() const
    {
        return _fiber_optics_inputs_from_grid;
    }

    /**
     * @brief Get fiber-optics inputs from grid-side (mutable)
     * @return Mutable reference to fiber_optics_inputs_from_grid
     */
    fiber_optics_inputs_from_grid_t& get_fiber_optics_inputs_from_grid() { return _fiber_optics_inputs_from_grid; }

    /**
     * @brief Get digital inputs structure (const)
     * @return Const reference to digital_inputs
     */
    const digital_inputs_t& get_digital_inputs() const { return _digital_inputs; }

    /**
     * @brief Get digital inputs structure (mutable)
     * @return Mutable reference to digital_inputs
     */
    digital_inputs_t& get_digital_inputs() { return _digital_inputs; }

    /**
     * @brief Get digital outputs structure (const)
     * @return Const reference to digital_outputs
     */
    const digital_outputs_t& get_digital_outputs() const { return _digital_outputs; }

    /**
     * @brief Get digital outputs structure (mutable)
     * @return Mutable reference to digital_outputs
     */
    digital_outputs_t& get_digital_outputs() { return _digital_outputs; }

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
     * @brief Get excitation input signals (const)
     * @return Const reference to excitation inputs
     */
    const excitation_inputs_t& get_excitation_inputs() const { return _excitation_inputs; }

    /**
     * @brief Get excitation input signals (mutable)
     * @return Mutable reference to excitation inputs
     */
    excitation_inputs_t& get_excitation_inputs() { return _excitation_inputs; }

    /**
     * @brief Get thyristor output structure (mutable)
     * @return Mutable reference to thyristor_control_output
     */
    thyristor_control_output_t& get_thyristor_control_output() { return _thyristor_output; }

    // ==================== SIMPLIFIED SETTERS (Called from Embedded/Plant) ====================

    /**
     * @brief Set analog inputs from hardware ADC
     *
     * Called from embedded code or plant builder at 200µs with raw ADC measurements.
     * This is the ONLY method the embedded code should call for analog inputs.
     *
     * @param v_ab_counts Machine voltage AB differential (ADC counts)
     * @param v_bc_counts Machine voltage BC differential (ADC counts)
     */
    void set_analog_inputs(yeet::control::adc_counts v_ab_counts, yeet::control::adc_counts v_bc_counts);

    /**
     * @brief Set fiber-optics inputs from grid-side using protocol message
     *
     * Called from embedded code or plant builder after decoding fiber optic RX.
     * Accepts the decoded grid_to_machine_message directly from the fiber optic protocol.
     * This is the ONLY method the embedded code should call for fiber optic inputs.
     *
     * @param msg Decoded message from grid-side (from fiber_optic::decode_grid_to_machine)
     */
    void set_fiber_optics_rx(const fiber_optic::grid_to_machine_message& msg);

    /**
     * @brief Get fiber-optics message to transmit to grid-side
     *
     * Called from embedded code or plant builder to get the encoded message
     * for fiber optic transmission to grid-side.
     * Internal components write to fiber_optics_outputs_to_grid via signals_connector.
     *
     * @return Encoded machine_to_grid_message for fiber optic transmission
     */
    fiber_optic::machine_to_grid_message get_fiber_optics_tx() const;

    /**
     * @brief Set digital inputs from hardware
     *
     * Called from embedded code at 1ms with digital input readings.
     * Applies polarity inversion and override logic.
     *
     * @param excitation_status Excitation controller status feedback
     * @param external_fault External fault input
     * @param start_command Start command (used for alarm confirmation)
     */
    void set_digital_inputs(bool excitation_status, bool external_fault, bool start_command);

    /**
     * @brief Set excitation analog input from external system
     *
     * Called from embedded code or plant builder with feedback from excitation system.
     * Respects override_excitation_inputs flag if set for test injection.
     *
     * @param excitation_current_actual Actual excitation current [pu] (optional)
     */
    void set_excitation_inputs(current_pu_t excitation_current_actual);

    /**
     * @brief Get excitation ON command output
     *
     * Read by embedded code to send ON command to external excitation system.
     * Applies polarity inversion based on configuration.
     *
     * @return true if excitation controller requests ON (field energization)
     */
    bool get_excitation_on_command() const;

    /**
     * @brief Get excitation OFF command output
     *
     * Read by embedded code to send OFF command to external excitation system.
     * Applies polarity inversion based on configuration.
     *
     * @return true if excitation controller requests OFF (field de-energization)
     */
    bool get_excitation_off_command() const;

    /**
     * @brief Get excitation current reference output
     *
     * Read by embedded code to send analog current reference to external excitation system.
     *
     * @return Excitation current reference [pu]
     */
    current_pu_t get_excitation_current_reference() const;

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
    parameters _parameters;
    analog_inputs_t _analog_inputs;
    digital_inputs_t _digital_inputs;
    digital_outputs_t _digital_outputs;
    fiber_optics_outputs_to_grid_t _fiber_optics_outputs_to_grid;
    fiber_optics_inputs_from_grid_t _fiber_optics_inputs_from_grid;
    excitation_outputs_t _excitation_outputs;
    excitation_inputs_t _excitation_inputs;
    thyristor_control_output_t _thyristor_output;
};

}  // namespace yeet::application::sfc::integration
