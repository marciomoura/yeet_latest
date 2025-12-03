#pragma once

#include "sfc/sfc_fiber_optic_protocol.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc/sfc_state_machine.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/model_registry.hpp"
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
 * - Supervisory digital commands (auxiliaries_are_on, breakers_are_on, start/stop)
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
     * When override flag is enabled, parameter values are used instead of
     * values from external inputs.
     */
    struct parameters {
        yeet::parameter<bool> override_digital_inputs{false};
        yeet::parameter<bool> auxiliaries_are_on{false};
        yeet::parameter<bool> breakers_are_on{false};
        yeet::parameter<bool> start_command{false};
        yeet::parameter<bool> stop_command{false};
        yeet::parameter<bool> shutdown_request{false};
        yeet::parameter<bool> breaker_is_closed{false};
        yeet::parameter<bool> breaker_is_tripping{false};
        yeet::parameter<bool> emergency_off{false};
        yeet::parameter<bool> door_is_open{false};
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
        signal_output<bool> auxiliaries_are_on;  ///< Auxiliary systems ready (cooling, oil pump, etc.)
        signal_output<bool> breakers_are_on;     ///< Grid breakers closed
        signal_output<bool> start_command;       ///< Start operation command
        signal_output<bool> stop_command;        ///< Stop operation command
        signal_output<bool> shutdown_request;    ///< Emergency shutdown request
        signal_output<bool> breaker_is_closed;   ///< Main circuit breaker is closed (optional status)
        signal_output<bool> breaker_is_tripping; ///< Protection relay indicates breaker is tripping
        signal_output<bool> emergency_off;       ///< Emergency off button pressed
        signal_output<bool> door_is_open;        ///< Enclosure door is open (prevents breaker close)
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
        external_signal<bool> shutdown_request{false};            ///< Shutdown request from machine side
        external_signal<bool> machine_protection_tripped{false};  ///< Machine-side protections tripped
        external_signal<bool> protection_inhibit{false};          ///< Protection system inhibited (awaiting ack)
        external_signal<bool> machine_trip_acknowledged{false};   ///< Machine-side trip acknowledgment
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
        signal_input<bool> mcb_close_command;  ///< Main circuit breaker close command
        signal_input<bool> mcb_open_command;   ///< Main circuit breaker open command (includes trip)
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
     * @param breakers_are_on Grid breakers closed
     * @param start_command Start operation
     * @param stop_command Stop operation
     * @param shutdown_request Emergency shutdown
     * @param breaker_is_closed Main circuit breaker is closed (optional status)
     * @param breaker_is_tripping Protection relay indicates breaker is tripping
     * @param emergency_off Emergency off button pressed
     * @param door_is_open Enclosure door is open
     */
    void set_digital_inputs(bool auxiliaries_are_on,
        bool breakers_are_on,
        bool start_command,
        bool stop_command,
        bool shutdown_request,
        bool breaker_is_closed,
        bool breaker_is_tripping,
        bool emergency_off,
        bool door_is_open);

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
     * @brief Get main circuit breaker close command
     *
     * Read by embedded code to send close command to main circuit breaker.
     *
     * @return true if state machine requests breaker close
     */
    bool get_mcb_close_command() const;

    /**
     * @brief Get main circuit breaker open command
     *
     * Read by embedded code to send open command to main circuit breaker.
     * This includes both normal shutdowns and protection trips.
     *
     * @return true if state machine requests breaker open
     */
    bool get_mcb_open_command() const;

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
    /**
     * @brief Apply digital input overrides if enabled
     * @return true if overrides were applied, false if real signals should be used
     */
    bool apply_digital_input_overrides();

    parameters _parameters;
    digital_inputs_t _digital_inputs;
    analog_inputs_t _analog_inputs;
    fiber_optics_outputs_to_machine_t _fiber_optics_outputs_to_machine;
    fiber_optics_inputs_from_machine_t _fiber_optics_inputs_from_machine;
    digital_outputs_t _digital_outputs;
};

}  // namespace yeet::application::sfc::integration
