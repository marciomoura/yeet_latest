#pragma once

#include "sfc/sfc_fiber_optic_protocol.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc/sfc_state_machine.hpp"
#include "sfc/sfc_types.hpp"
#include "sfc_integration/sfc_io_types.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Grid-side external I/O datamodel
 *
 * Provides parameter-based override interface for all external signals flowing in and out of
 * the grid-side control system:
 *
 * **Inputs:**
 * - Supervisory digital commands (auxiliaries_are_on, mv_breaker_is_closed, start/stop)
 * - Analog measurements (ADC counts from hardware mock)
 * - Cross-coupled signals from machine-side (speed, current reference, voltage, force commutation)
 *
 * **Outputs:**
 * - Fiber-optic signals to machine-side (operating_mode, current_controller_saturated, etc.)
 * - Measurement availability for embedded/test consumption
 *
 * **Override Capability:**
 * Tests can enable override flags and set mock parameter values to inject signals without
 * modifying plant builder or control logic. This provides clean test/simulation interfaces.
 *
 * **Usage Pattern:**
 * 1. Plant builder connects supervisory signals to digital_inputs struct via signals_connector
 * 2. Plant builder calls update_analog_inputs() with raw ADC counts at 200µs
 * 3. Plant builder calls update_fiber_optics_outputs_to_machine() at 1ms with grid-side outputs
 * 4. Plant builder calls update_fiber_optics_inputs_from_machine() at 1ms with machine-side outputs
 * 5. Machine-side reads from get_fiber_optics_outputs_to_machine()
 * 6. Grid-side reads from get_fiber_optics_inputs_from_machine()
 *
 * @see sfc_machine_side_external_ios_datamodel for parallel machine-side implementation
 */
class sfc_grid_side_external_ios_datamodel {
public:
    // ==================== PARAMETERS (Override Control) ====================

    /**
     * @brief Parameters for test/simulation signal injection
     *
     * Override flags enable parameter-based mocking for testing.
     * Each digital input has a pair of parameters:
     * - di_<signal>_override: Enable override for this signal
     * - di_<signal>_override_value: Value to use when override enabled
     */
    struct parameters {
        // ==================== Digital Input Overrides ====================
        // When override is true, the override_value is used instead of hardware input

        /// Enable override for auxiliaries_are_on input
        yeet::parameter<bool> di_auxiliaries_are_on_override{false};
        /// Override value for auxiliaries_are_on (when override enabled)
        yeet::parameter<bool> di_auxiliaries_are_on_override_value{false};

        /// Enable override for mv_breaker_is_closed input
        yeet::parameter<bool> di_mv_breaker_is_closed_override{false};
        /// Override value for mv_breaker_is_closed (when override enabled)
        yeet::parameter<bool> di_mv_breaker_is_closed_override_value{false};

        /// Enable override for start_command input
        yeet::parameter<bool> di_start_command_override{false};
        /// Override value for start_command (when override enabled)
        yeet::parameter<bool> di_start_command_override_value{false};

        /// Enable override for stop_command input
        yeet::parameter<bool> di_stop_command_override{false};
        /// Override value for stop_command (when override enabled)
        yeet::parameter<bool> di_stop_command_override_value{false};

        /// Enable override for mv_breaker_is_tripping input
        yeet::parameter<bool> di_mv_breaker_is_tripping_override{false};
        /// Override value for mv_breaker_is_tripping (when override enabled)
        yeet::parameter<bool> di_mv_breaker_is_tripping_override_value{false};

        /// Enable override for external_fault input
        yeet::parameter<bool> di_external_fault_override{false};
        /// Override value for external_fault (when override enabled)
        yeet::parameter<bool> di_external_fault_override_value{false};

        /// Enable override for emergency_off input
        yeet::parameter<bool> di_emergency_off_override{false};
        /// Override value for emergency_off (when override enabled)
        yeet::parameter<bool> di_emergency_off_override_value{false};

        /// Enable override for door_is_open input
        yeet::parameter<bool> di_door_is_open_override{false};
        /// Override value for door_is_open (when override enabled)
        yeet::parameter<bool> di_door_is_open_override_value{false};

        /// Enable override for fans_are_running input
        yeet::parameter<bool> di_fans_are_running_override{false};
        /// Override value for fans_are_running (when override enabled)
        yeet::parameter<bool> di_fans_are_running_override_value{false};

        // ==================== Digital Output Overrides ====================
        // When override is true, the override_value is used instead of control logic

        /// Enable override for mv_breaker_close output
        yeet::parameter<bool> do_mv_breaker_close_override{false};
        /// Override value for mv_breaker_close (when override enabled)
        yeet::parameter<bool> do_mv_breaker_close_override_value{false};

        /// Enable override for mv_breaker_open output
        yeet::parameter<bool> do_mv_breaker_open_override{false};
        /// Override value for mv_breaker_open (when override enabled)
        yeet::parameter<bool> do_mv_breaker_open_override_value{false};

        /// Enable override for auxiliaries_on_command output
        yeet::parameter<bool> do_auxiliaries_on_command_override{false};
        /// Override value for auxiliaries_on_command (when override enabled)
        yeet::parameter<bool> do_auxiliaries_on_command_override_value{false};

        /// Enable override for on_command_acu_fan_1 output
        yeet::parameter<bool> do_on_command_acu_fan_1_override{false};
        /// Override value for on_command_acu_fan_1 (when override enabled)
        yeet::parameter<bool> do_on_command_acu_fan_1_override_value{false};

        /// Enable override for on_command_acu_fan_2 output
        yeet::parameter<bool> do_on_command_acu_fan_2_override{false};
        /// Override value for on_command_acu_fan_2 (when override enabled)
        yeet::parameter<bool> do_on_command_acu_fan_2_override_value{false};

        /// Enable override for on_command_acu_fan_3 output
        yeet::parameter<bool> do_on_command_acu_fan_3_override{false};
        /// Override value for on_command_acu_fan_3 (when override enabled)
        yeet::parameter<bool> do_on_command_acu_fan_3_override_value{false};

        /// Enable override for on_command_acu_fan_4 output
        yeet::parameter<bool> do_on_command_acu_fan_4_override{false};
        /// Override value for on_command_acu_fan_4 (when override enabled)
        yeet::parameter<bool> do_on_command_acu_fan_4_override_value{false};

        /// Enable override for software_trip_1 output
        yeet::parameter<bool> do_software_trip_1_override{false};
        /// Override value for software_trip_1 (when override enabled)
        yeet::parameter<bool> do_software_trip_1_override_value{false};

        /// Enable override for software_trip_2 output
        yeet::parameter<bool> do_software_trip_2_override{false};
        /// Override value for software_trip_2 (when override enabled)
        yeet::parameter<bool> do_software_trip_2_override_value{false};

        /// Enable override for software_trip_3 output
        yeet::parameter<bool> do_software_trip_3_override{false};
        /// Override value for software_trip_3 (when override enabled)
        yeet::parameter<bool> do_software_trip_3_override_value{false};

        /// Enable override for emergency_off_reset output
        yeet::parameter<bool> do_emergency_off_reset_override{false};
        /// Override value for emergency_off_reset (when override enabled)
        yeet::parameter<bool> do_emergency_off_reset_override_value{false};

        // ==================== IO Index Mapping (IO Channel Selection) ====================
        // Select which physical IO channel (0..N-1) maps to each logical signal.
        // Naming convention: di_{name}_index, do_{name}_index, ao_{name}_index.
        // N comes from TASK_IO_NUM_DIGITAL_INPUTS / TASK_IO_NUM_DIGITAL_OUTPUTS / TASK_IO_NUM_ANALOG_OUTPUTS.
        yeet::parameter<int> di_auxiliaries_are_on_index{0};
        yeet::parameter<int> di_mv_breaker_is_closed_index{1};
        yeet::parameter<int> di_start_command_index{2};
        yeet::parameter<int> di_stop_command_index{3};
        yeet::parameter<int> di_mv_breaker_is_tripping_index{6};
        yeet::parameter<int> di_emergency_off_index{7};
        yeet::parameter<int> di_door_is_open_index{8};
        yeet::parameter<int> di_external_fault_index{9};
        yeet::parameter<int> di_fans_are_running_index{10};

        // Digital output indices
        yeet::parameter<int> do_mv_breaker_close_index{0};
        yeet::parameter<int> do_mv_breaker_open_index{1};
        yeet::parameter<int> do_auxiliaries_on_command_index{15};  // Default to some unused index
        yeet::parameter<int> do_on_command_acu_fan_1_index{2};
        yeet::parameter<int> do_on_command_acu_fan_2_index{3};
        yeet::parameter<int> do_on_command_acu_fan_3_index{4};
        yeet::parameter<int> do_on_command_acu_fan_4_index{5};
        yeet::parameter<int> do_software_trip_1_index{6};
        yeet::parameter<int> do_software_trip_2_index{7};
        yeet::parameter<int> do_software_trip_3_index{8};
        yeet::parameter<int> do_emergency_off_reset_index{9};

        // ==================== IO Polarity Configuration ====================
        // Configure signal polarity (active high or active low)
        yeet::parameter<io_polarity_t> di_auxiliaries_are_on_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_mv_breaker_is_closed_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_start_command_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_stop_command_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_mv_breaker_is_tripping_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_emergency_off_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_door_is_open_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_external_fault_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> di_fans_are_running_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_mv_breaker_close_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_mv_breaker_open_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_auxiliaries_on_command_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_on_command_acu_fan_1_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_on_command_acu_fan_2_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_on_command_acu_fan_3_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_on_command_acu_fan_4_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_software_trip_1_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_software_trip_2_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_software_trip_3_polarity{io_polarity_t::ACTIVE_HIGH};
        yeet::parameter<io_polarity_t> do_emergency_off_reset_polarity{io_polarity_t::ACTIVE_HIGH};
    };

    // ==================== DIGITAL INPUTS (Supervisory Commands) ====================

    /**
     * @brief Supervisory digital input signals
     *
     * These are command signals from the external supervisory controller.
     * Typically connected from plant builder's simulation command signals.
     *
     * Flow: External command → signals_connector → digital_inputs → grid-side state machine
     */
    struct digital_inputs_t {
        signal_output<bool> auxiliaries_are_on;      ///< Auxiliary systems ready (cooling, oil pump, etc.)
        signal_output<bool> mv_breaker_is_closed;    ///< Medium voltage breaker closed
        signal_output<bool> start_command;           ///< Start operation command
        signal_output<bool> stop_command;            ///< Stop operation command
        signal_output<bool> mv_breaker_is_tripping;  ///< MV breaker protection relay indicates tripping
        signal_output<bool> emergency_off;           ///< Emergency off button pressed
        signal_output<bool> door_is_open;            ///< Enclosure door is open (prevents breaker close)
        signal_output<bool> external_fault;          ///< External fault signal (ORs to protection trip)
        signal_output<bool> fans_are_running;        ///< ACU fans feedback (auxiliaries ready confirmation)
    };

    // ==================== ANALOG INPUTS (ADC Measurements - Grid Measurements Only) ====================

    /**
     * @brief Analog input measurements from hardware mock ADC
     *
     * **Grid-Side Architecture:** Measures BOTH voltage AND current
     * - Voltage: V_AB, V_BC (secondary of step-down transformer)
     * - Current: I_A, I_C (secondary of current transformer in DC link)
     *
     * These raw ADC counts flow through measurement runners:
     * - Measurement runner: Converts ADC counts → voltage/current in per-unit
     * - Observer: Uses voltage/current measurements for state estimation
     * - Controller: Uses current measurement for control feedback
     *
     * Flow: Hardware mock ADC → update_analog_inputs() → measurement_runner → observer/controller
     */
    struct analog_inputs_t {
        signal_output<yeet::control::adc_counts> v_ab_counts{};  ///< Grid voltage AB (ADC counts)
        signal_output<yeet::control::adc_counts> v_bc_counts{};  ///< Grid voltage BC (ADC counts)
        signal_output<yeet::control::adc_counts> i_a_counts{};   ///< Grid current A (ADC counts)
        signal_output<yeet::control::adc_counts> i_c_counts{};   ///< Grid current C (ADC counts)
    };

    // ==================== FIBER-OPTICS OUTPUTS TO MACHINE-SIDE ====================

    /**
     * @brief Signals FROM grid-side TO machine-side (via fiber-optics)
     *
     * These are **outputs** from the grid-side datamodel perspective.
     * These are **inputs** to the machine-side datamodel perspective (fiber_optics_inputs).
     *
     * **Signal Sources (Grid-Side Components):**
     * - operating_mode: From operation_mode_monitor or state_machine
     * - force_commutation_acknowledged: From state_machine or operation_mode_monitor
     * - is_current_zero: From grid_side_observer or current_controller
     * - is_saturated: From current_controller saturation detection
     * - pulse_releases: From state_machine (pulses enabled / system running)
     * - dc_link_current: From dc_link_measurements (GRID-SIDE measured DC link current)
     *   NOTE: Machine-side has NO current measurement (sensorless), so it needs the DC link
     *   current from grid-side to know the actual link current value for control feedback
     * - freeze_integrator: From sfc_speed_controller_runner (pulse-mode forced commutation signal)
     * - is_requesting_main_pi_reset: From grid_side_modulator (when pulse reset needed)
     * - current_state_id: From state_machine (for machine-side state synchronization)
     *
     * **Timing:** Updated at 1ms (after grid-side update, before machine-side update)
     *
     * Flow: Grid-side components → update_fiber_optics_outputs_to_machine() → machine-side fiber_optics_inputs
     */
    struct fiber_optics_outputs_to_machine_t {
        signal_output<sfc_operating_mode_t> operating_mode{
            sfc_operating_mode_t::natural_commutation};                  ///< SFC operating mode (pulse or natural)
        signal_output<bool> force_commutation_acknowledged{false};       ///< Forced commutation handshake ACK
        signal_output<bool> is_current_zero{false};                      ///< DC link current below threshold
        signal_output<bool> is_saturated{false};                         ///< Current controller at output limits
        signal_output<bool> pulse_releases{false};                       ///< Pulses enabled/system running
        signal_output<current_pu_t> dc_link_current{current_pu_t{0.0}};  ///< DC link current [pu] (grid-measured)
        signal_output<bool> freeze_integrator{false};                    ///< Speed controller integrator freeze
        signal_output<bool> is_requesting_main_pi_reset{false};          ///< Main PI reset request from modulator
        signal_output<bool> communication_ok{false};  ///< Grid-side communication status (receiving machine)
        signal_output<yeet::application::sfc::sfc_state_id> current_state_id{
            yeet::application::sfc::sfc_state_id::IDLE};  ///< Grid-side state machine state ID
    };

    // ==================== FIBER-OPTICS INPUTS FROM MACHINE-SIDE ====================

    /**
     * @brief Signals FROM machine-side TO grid-side (via fiber-optics)
     *
     * These are **inputs** to the grid-side datamodel perspective.
     * These are **outputs** from the machine-side datamodel perspective.
     *
     * **Signal Sources (Machine-Side Components):**
     * - measured_speed: From machine_side_observer.estimated_speed
     * - current_reference: From torque_controller.current_reference
     * - machine_voltage_magnitude: From machine_side_observer.estimated_voltage_magnitude
     * - request_force_commutation: From machine_side_modulator (when low-speed assist needed)
     *
     * **Timing:** Updated at 1ms (after machine-side update, for grid-side next iteration use)
     *
     * Flow: Machine-side components → fiber optic protocol → grid-side external_ios → internal components
     */
    struct fiber_optics_inputs_from_machine_t {
        external_signal<speed_pu_t> measured_speed{speed_pu_t{0.0}};         ///< Machine rotor speed [pu]
        external_signal<current_pu_t> current_reference{current_pu_t{0.0}};  ///< Current ref from torque controller
        external_signal<voltage_pu_t> machine_voltage_magnitude{
            voltage_pu_t{0.0}};                                   ///< Machine back-EMF magnitude [pu]
        external_signal<bool> request_force_commutation{false};   ///< Force commutation request
        external_signal<bool> machine_protection_tripped{false};  ///< Machine-side protections tripped
        external_signal<bool> protection_inhibit{false};          ///< Protection system inhibited (awaiting ack)
        external_signal<bool> communication_ok{false};  ///< Machine-side communication status (receiving grid)
    };

    // ==================== DIGITAL OUTPUTS (Main Circuit Breaker Control) ====================

    /**
     * @brief Digital output commands to main circuit breaker
     *
     * These are OUTPUT commands from the grid-side control system to the main circuit breaker (MCB).
     * The state machine generates these commands based on operational state and safety conditions.
     *
     * Flow: state_machine → external_ios → embedded/hardware → main circuit breaker
     */
    struct digital_outputs_t {
        signal_input<bool> mv_breaker_close;        ///< MV breaker close command (before polarity inversion)
        signal_input<bool> mv_breaker_open;         ///< MV breaker open command (before polarity inversion)
        signal_input<bool> auxiliaries_on_command;  ///< Command to energize auxiliaries (level-high)
        signal_input<bool> on_command_acu_fan_1;    ///< ACU reactor fan ON command (from auxiliaries_on)
        signal_input<bool> on_command_acu_fan_2;    ///< ACU converter 1 fan ON command (from auxiliaries_on)
        signal_input<bool> on_command_acu_fan_3;    ///< ACU converter 2 fan ON command (from auxiliaries_on)
        signal_input<bool> on_command_acu_fan_4;    ///< ACU control fan ON command (from auxiliaries_on)
        signal_input<bool> software_trip_1;         ///< Software trip signal 1 (common trip)
        signal_input<bool> software_trip_2;         ///< Software trip signal 2 (trip/emergency stop)
        signal_input<bool> software_trip_3;         ///< Software trip signal 3 (software trip)
        signal_input<bool> emergency_off_reset;     ///< Pulse to reset external emergency off circuit
    };

    // ==================== PUBLIC INTERFACE ====================

    sfc_grid_side_external_ios_datamodel();

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
     * @brief Get fiber-optics outputs to machine-side (const)
     *
     * Machine-side reads these signals via this getter to know grid-side state
     *
     * @return Const reference to fiber_optics_outputs_to_machine
     */
    const fiber_optics_outputs_to_machine_t& get_fiber_optics_outputs_to_machine() const
    {
        return _fiber_optics_outputs_to_machine;
    }

    /**
     * @brief Get fiber-optics outputs to machine-side (mutable)
     * @return Mutable reference to fiber_optics_outputs_to_machine
     */
    fiber_optics_outputs_to_machine_t& get_fiber_optics_outputs_to_machine()
    {
        return _fiber_optics_outputs_to_machine;
    }

    /**
     * @brief Get fiber-optics inputs from machine-side (const)
     *
     * Grid-side reads these signals via this getter to know machine-side state
     *
     * @return Const reference to fiber_optics_inputs_from_machine
     */
    const fiber_optics_inputs_from_machine_t& get_fiber_optics_inputs_from_machine() const
    {
        return _fiber_optics_inputs_from_machine;
    }

    /**
     * @brief Get fiber-optics inputs from machine-side (mutable)
     * @return Mutable reference to fiber_optics_inputs_from_machine
     */
    fiber_optics_inputs_from_machine_t& get_fiber_optics_inputs_from_machine()
    {
        return _fiber_optics_inputs_from_machine;
    }

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

    // ==================== SIMPLIFIED SETTERS (Called from Embedded/Plant) ====================

    /**
     * @brief Set analog inputs from hardware ADC
     *
     * Called from embedded code or plant builder at 200µs with raw ADC measurements.
     * This is the ONLY method the embedded code should call for analog inputs.
     *
     * @param v_ab_counts Grid voltage AB differential (ADC counts)
     * @param v_bc_counts Grid voltage BC differential (ADC counts)
     * @param i_a_counts Grid current phase A (ADC counts)
     * @param i_c_counts Grid current phase C (ADC counts)
     */
    void set_analog_inputs(yeet::control::adc_counts v_ab_counts,
        yeet::control::adc_counts v_bc_counts,
        yeet::control::adc_counts i_a_counts,
        yeet::control::adc_counts i_c_counts);

    /**
     * @brief Set supervisory digital inputs
     *
     * Called from embedded code or plant builder with external command signals.
     * Respects override_digital_inputs flag if set for test injection.
     * This is the ONLY method the embedded code should call for digital inputs.
     *
     * @param auxiliaries_are_on Auxiliary systems ready
     * @param mv_breaker_is_closed Grid breakers closed
     * @param start_command Start operation
     * @param stop_command Stop operation
     * @param shutdown_request Emergency shutdown request
     * @param mv_breaker_is_tripping Protection relay indicates breaker is tripping
     * @param emergency_off Emergency off button pressed
     * @param door_is_open Enclosure door is open
     * @param external_fault External fault signal
     * @param fans_are_running ACU fans feedback
     */
    void set_digital_inputs(bool auxiliaries_are_on,
        bool mv_breaker_is_closed,
        bool start_command,
        bool stop_command,
        bool mv_breaker_is_tripping,
        bool emergency_off,
        bool door_is_open,
        bool external_fault,
        bool fans_are_running);

    /**
     * @brief Set fiber-optics inputs from machine-side using protocol message
     *
     * Called from embedded code or plant builder after decoding fiber optic RX.
     * Accepts the decoded machine_to_grid_message directly from the fiber optic protocol.
     * This is the ONLY method the embedded code should call for fiber optic inputs.
     *
     * @param msg Decoded message from machine-side (from fiber_optic::decode_machine_to_grid)
     */
    void set_fiber_optics_rx(const fiber_optic::machine_to_grid_message& msg);

    /**
     * @brief Get fiber-optics message to transmit to machine-side
     *
     * Called from embedded code or plant builder to get the encoded message
     * for fiber optic transmission to machine-side.
     * Internal components write to fiber_optics_outputs_to_machine via signals_connector.
     *
     * @return Encoded grid_to_machine_message for fiber optic transmission
     */
    fiber_optic::grid_to_machine_message get_fiber_optics_tx() const;

    /**
     * @brief Get main circuit breaker close signal
     *
     * Read by embedded code to send close signal to main circuit breaker.
     *
     * @return true if state machine requests breaker close (level-high)
     */
    bool get_mv_breaker_close() const;

    /**
     * @brief Get MV breaker open signal
     *
     * Read by embedded code to send open signal to MV breaker.
     * This includes both normal shutdowns and protection trips.
     * Output polarity inversion applied based on do_mv_breaker_open_polarity parameter.
     *
     * @return true if state machine requests breaker open (with polarity applied)
     */
    bool get_mv_breaker_open() const;

    /**
     * @brief Get auxiliaries on command signal
     *
     * Read by embedded code to energize/disable auxiliary systems.
     * Must go low on any trip/shutdown condition.
     *
     * @return true if auxiliaries should be energized (level-high)
     */
    bool get_auxiliaries_on_command() const;

    /**
     * @brief Get ACU fan ON command signals (all driven by auxiliaries_on)
     * These methods apply polarity inversion for each output
     */
    bool get_on_command_acu_fan_1() const;
    bool get_on_command_acu_fan_2() const;
    bool get_on_command_acu_fan_3() const;
    bool get_on_command_acu_fan_4() const;

    /**
     * @brief Get software trip signals (all driven by protection trip/inhibit status)
     * These methods apply polarity inversion for each output
     */
    bool get_software_trip_1() const;
    bool get_software_trip_2() const;
    bool get_software_trip_3() const;

    /**
     * @brief Get emergency off reset pulse signal
     * This method applies polarity inversion based on do_emergency_off_reset_polarity
     *
     * @return true if pulse is active (with polarity applied)
     */
    bool get_emergency_off_reset() const;

    /**
     * @brief Register datamodel with model registry for network/telemetry access
     *
     * Called during application initialization to make datamodel parameters available
     * through the network parameter interface and telemetry system.
     *
     * @param registry Model registry for parameter registration
     */
    void register_datamodel(yeet::telemetry::parameter_handler& handler);

    /**
     * @brief Register parameter change callbacks
     *
     * Called during initialization to set up callbacks when override flags are changed.
     * Enables tests to dynamically enable/disable overrides without reinitialization.
     */
    void register_callbacks();

private:
    /**
     * @brief Apply digital input overrides where enabled
     *
     * For each signal, if its override flag is enabled, use the override value;
     * otherwise use the hardware value passed in.
     */
    void apply_digital_input_overrides(bool auxiliaries_are_on,
        bool mv_breaker_is_closed,
        bool start_command,
        bool stop_command,
        bool mv_breaker_is_tripping,
        bool emergency_off,
        bool door_is_open,
        bool external_fault,
        bool fans_are_running);

    parameters _parameters;
    digital_inputs_t _digital_inputs;
    analog_inputs_t _analog_inputs;
    fiber_optics_outputs_to_machine_t _fiber_optics_outputs_to_machine;
    fiber_optics_inputs_from_machine_t _fiber_optics_inputs_from_machine;
    digital_outputs_t _digital_outputs;
};

}  // namespace yeet::application::sfc::integration
