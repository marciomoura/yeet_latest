#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

// Forward declarations
class sfc_grid_side_measurements_runner;
class sfc_dc_link_measurements_runner;
class sfc_state_machine_runner;
class sfc_operation_mode_monitor_runner;
class sfc_current_controller_runner;
class sfc_grid_side_modulator_runner;
class sfc_grid_side_pulse_generator_runner;
class sfc_grid_side_observer_runner;

/**
 * @brief I/O interface for grid-side control system
 *
 * This class provides direct access to internal component signal_inputs and signal_outputs,
 * creating a clean facade for external systems without duplicating signals.
 *
 * Design Pattern: Facade with Direct References
 * - Inputs: Direct references to component signal_input<T>& members
 * - Outputs: Direct references to component signal_output<T>& members
 *
 * Usage:
 * - **Embedded**: Connect hardware ADC/DI/DO to these references
 * - **Simulation**: Connect plant model and machine-side control to these references
 *
 * Benefits:
 * - Zero overhead (no signal duplication)
 * - Works with existing signal_connector framework
 * - Single source of truth for I/O contract
 * - Type-safe signal access
 */
class sfc_grid_side_control_io {
public:
    /**
     * @brief Construct I/O interface with references to internal components
     */
    sfc_grid_side_control_io(sfc_grid_side_measurements_runner& measurements,
        sfc_dc_link_measurements_runner& dc_link_measurements,
        sfc_state_machine_runner& state_machine,
        sfc_operation_mode_monitor_runner& operation_mode_monitor,
        sfc_current_controller_runner& current_controller,
        sfc_grid_side_modulator_runner& modulator,
        sfc_grid_side_pulse_generator_runner& pulse_generator,
        sfc_grid_side_observer_runner& observer);

    // ==================== INPUT SIGNALS (References to component inputs) ====================

    /**
     * @brief Analog inputs from hardware ADC (grid measurements)
     */
    struct analog_inputs_t {
        signal_input<control::adc_counts>& v_ab_counts;  ///< Grid voltage V_AB (ADC counts)
        signal_input<control::adc_counts>& v_bc_counts;  ///< Grid voltage V_BC (ADC counts)
        signal_input<control::adc_counts>& i_a_counts;   ///< Grid current I_A (ADC counts)
        signal_input<control::adc_counts>& i_c_counts;   ///< Grid current I_C (ADC counts)
    } analog_inputs;

    /**
     * @brief Digital inputs from supervisory controller (state machine commands)
     */
    struct digital_inputs_t {
        signal_input<bool>& auxiliaries_are_on;  ///< Auxiliary systems ready (cooling, oil)
        signal_input<bool>& breakers_are_on;     ///< Grid breakers closed
        signal_input<bool>& start_command;       ///< Start operation command
        signal_input<bool>& stop_command;        ///< Stop operation command
        signal_input<bool>& shutdown_request;    ///< Emergency shutdown request
    } digital_inputs;

    /**
     * @brief Cross-coupled inputs from machine-side controller
     */
    struct machine_side_inputs_t {
        signal_input<speed_pu_t>& measured_speed_pu;            ///< Machine rotor speed [pu]
        signal_input<current_pu_t>& dc_link_current_reference;  ///< Current setpoint from torque controller [pu]
        signal_input<voltage_pu_t>& machine_voltage_magnitude;  ///< Machine back-EMF magnitude [pu]
        signal_input<bool>& request_force_commutation;          ///< Force commutation request from machine-side
    } machine_side_inputs;

    // ==================== OUTPUT SIGNALS (References to component outputs) ====================

    /**
     * @brief Thyristor firing pulses (primary control output)
     */
    struct pulse_outputs_t {
        signal_output<control::six_pulse_generator::pulses>& firing_pulses;  ///< 6-pulse firing pattern
    } pulse_outputs;

    /**
     * @brief Digital outputs for status indication
     */
    struct digital_outputs_t {
        signal_output<bool>& pulse_releases;                  ///< System running (pulses enabled)
        signal_output<bool>& is_current_zero;                 ///< DC link current below threshold
        signal_output<bool>& current_controller_saturated;    ///< Controller at output limits
        signal_output<sfc_operating_mode_t>& operating_mode;  ///< Current operating mode
    } digital_outputs;

    /**
     * @brief Cross-coupled outputs to machine-side controller
     */
    struct machine_side_outputs_t {
        signal_output<voltage_pu_t>& grid_voltage_magnitude;    ///< Estimated grid voltage magnitude (from observer)
        signal_output<current_pu_t>& dc_link_current_measured;  ///< Measured DC link current
        signal_output<sfc_operating_mode_t>& operating_mode;    ///< Operating mode for machine-side
        signal_output<bool>& is_current_zero;                   ///< Current zero status
        signal_output<bool>& force_commutation_acknowledged;    ///< Force commutation ACK
        signal_output<bool>& freeze_integrator;                 ///< Freeze integrator command
    } machine_side_outputs;
};

}  // namespace yeet::application::sfc::integration
