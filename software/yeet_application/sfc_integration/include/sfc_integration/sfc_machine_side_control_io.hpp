#pragma once

#include "sfc/sfc_state_machine.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

// Forward declarations for component runners
namespace yeet::application::sfc::integration {
class sfc_excitation_controller_runner;
class sfc_soft_starter_speed_generator_runner;
class sfc_speed_controller_runner;
class sfc_torque_controller_runner;
class sfc_machine_side_controller_runner;
class sfc_machine_side_observer_runner;
class sfc_machine_side_modulator_runner;
class sfc_machine_side_pulse_generator_runner;
class sfc_machine_side_measurements_runner;
}  // namespace yeet::application::sfc::integration

namespace yeet::application::sfc::integration {

/**
 * @brief I/O interface for machine-side control subsystem
 *
 * This class provides a clean facade for all inputs and outputs to the machine-side control.
 * It uses direct references to the actual signal_input/signal_output members of internal components,
 * providing zero-overhead abstraction while maintaining a clear boundary between external I/O
 * and internal implementation.
 *
 * Signal Flow:
 * - Analog Inputs: ADC measurements (v_ab, v_bc) from hardware
 * - Digital Inputs: Supervisory commands (exciter, speed profiles)
 * - Grid-Side Inputs: Cross-coupled signals from grid-side control
 * - Pulse Outputs: Thyristor firing pulses to plant
 * - Grid-Side Outputs: Cross-coupled signals to grid-side control
 */
class sfc_machine_side_control_io {
public:
    // ==================== Analog Inputs (from ADC hardware) ====================

    /**
     * @brief Analog measurement inputs from ADC hardware
     */
    struct analog_inputs_t {
        signal_input<control::adc_counts>& v_ab_counts;  ///< Machine voltage AB differential (ADC counts)
        signal_input<control::adc_counts>& v_bc_counts;  ///< Machine voltage BC differential (ADC counts)
    };

    // ==================== Digital Inputs (from supervisory controller) ====================

    /**
     * @brief Digital command inputs from supervisory controller
     */
    struct digital_inputs_t {
        // Excitation control commands
        signal_input<bool>& exciter_ready;             ///< Exciter system is ready
        signal_input<bool>& excitation_start_command;  ///< Start excitation command
        signal_input<bool>& excitation_stop_command;   ///< Stop excitation command

        // Speed generator profile selection commands (turbine starting sequence)
        signal_input<bool>& run_creep_speed;     ///< Execute Profile 1: Creep speed (~5% rated)
        signal_input<bool>& run_purge_accel;     ///< Execute Profile 2: Purge acceleration (~20% rated)
        signal_input<bool>& run_ignition_accel;  ///< Execute Profile 3: Ignition acceleration (~55% rated)
        signal_input<bool>& run_assist_accel;    ///< Execute Profile 4: Assist to self-sustaining (~95% rated)
        signal_input<bool>& hold_speed;          ///< Hold current speed setpoint
        signal_input<bool>& decelerate;          ///< Controlled deceleration to zero
        signal_input<bool>& idle;                ///< Idle mode (zero torque/speed outputs)
    };

    // ==================== Grid-Side Inputs (cross-coupled from grid-side control) ====================

    /**
     * @brief Inputs from grid-side control subsystem
     */
    struct grid_side_inputs_t {
        signal_input<current_pu_t>& dc_link_current_measured;  ///< DC link current from grid measurements
        signal_input<sfc_operating_mode_t>& operating_mode;    ///< Operating mode from grid mode monitor
        signal_input<bool>& force_commutation_acknowledged;    ///< Force commutation ack from grid modulator
        signal_input<bool>& freeze_integrator;                 ///< Freeze integrator from grid state machine
        signal_input<bool>& current_controller_saturated;      ///< Current controller saturation from grid
        signal_input<bool>& pulse_releases;                    ///< Pulse releases from grid state machine
        signal_input<yeet::application::sfc::sfc_state_id>&
            current_state_id;  ///< Current state ID from grid state machine
    };

    // ==================== Pulse Outputs ====================

    /**
     * @brief Thyristor firing pulse outputs
     */
    struct pulse_outputs_t {
        signal_output<control::six_pulse_generator::pulses>& firing_pulses;  ///< Machine-side firing pulses
    };

    // ==================== Grid-Side Outputs (cross-coupled to grid-side control) ====================

    /**
     * @brief Outputs to grid-side control subsystem
     */
    struct grid_side_outputs_t {
        signal_output<current_pu_t>& current_reference;          ///< Current reference from torque controller
        signal_output<speed_pu_t>& measured_speed_pu;            ///< Estimated speed from observer
        signal_output<voltage_pu_t>& machine_voltage_magnitude;  ///< Estimated voltage magnitude from observer
        signal_output<bool>& request_force_commutation;          ///< Force commutation request from modulator
    };

    // ==================== Constructor ====================

    /**
     * @brief Construct I/O interface with references to component signals
     *
     * This constructor binds all I/O interface references to the actual signal_input/signal_output
     * members of the internal component runners. The binding happens at construction and provides
     * zero-overhead access (direct references, no indirection).
     *
     * @param machine_measurements Machine-side measurements runner
     * @param excitation_controller Excitation controller runner
     * @param speed_generator Speed generator runner
     * @param speed_controller Speed controller runner
     * @param torque_controller Torque controller runner
     * @param machine_side_controller Machine-side controller runner
     * @param machine_side_observer Machine-side observer runner
     * @param machine_side_modulator Machine-side modulator runner
     * @param machine_side_pulse_generator Machine-side pulse generator runner
     */
    sfc_machine_side_control_io(sfc_machine_side_measurements_runner& machine_measurements,
        sfc_excitation_controller_runner& excitation_controller,
        sfc_soft_starter_speed_generator_runner& speed_generator,
        sfc_speed_controller_runner& speed_controller,
        sfc_torque_controller_runner& torque_controller,
        sfc_machine_side_controller_runner& machine_side_controller,
        sfc_machine_side_observer_runner& machine_side_observer,
        sfc_machine_side_modulator_runner& machine_side_modulator,
        sfc_machine_side_pulse_generator_runner& machine_side_pulse_generator);

    // ==================== I/O Interface Access ====================

    analog_inputs_t analog_inputs;          ///< Analog measurement inputs from ADC
    digital_inputs_t digital_inputs;        ///< Digital command inputs from supervisory controller
    grid_side_inputs_t grid_side_inputs;    ///< Cross-coupled inputs from grid-side control
    pulse_outputs_t pulse_outputs;          ///< Thyristor firing pulse outputs
    grid_side_outputs_t grid_side_outputs;  ///< Cross-coupled outputs to grid-side control
};

}  // namespace yeet::application::sfc::integration
