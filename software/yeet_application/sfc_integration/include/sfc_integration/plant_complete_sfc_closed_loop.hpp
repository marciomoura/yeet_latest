#pragma once

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_datamodel.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_datamodel.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_operation_mode_monitor_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "sfc_integration/sfc_state_machine_datamodel.hpp"
#include "sfc_integration/sfc_state_machine_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/parameter_collection.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Sensorless Current Controller Simulation Builder ====================

// Parameter collection with state machine included
using sfc_complete_sfc_closed_loop_parameters = yeet::parameter_collection<sfc_current_controller_datamodel::parameters,
    sfc_grid_side_pulse_generator_datamodel::parameters,
    sfc_machine_side_pulse_generator_datamodel::parameters,
    sfc_machine_side_controller_datamodel::parameters,
    sfc_grid_side_measurements_datamodel::parameters,
    sfc_machine_side_measurements_datamodel::parameters,
    sfc_dc_link_measurements_datamodel::parameters,
    sfc_grid_side_observer_datamodel::parameters,
    sfc_machine_side_observer_datamodel::parameters,
    sfc_operation_mode_monitor_datamodel::parameters,
    sfc_plant_model_datamodel::parameters,
    sfc_speed_controller_datamodel::parameters,
    sfc_state_machine_datamodel::parameters,
    sfc_grid_side_modulator_datamodel::parameters,
    sfc_machine_side_modulator_datamodel::parameters>;

/**
 * @brief Sensorless current controller simulation builder with observers
 *
 * This configuration provides:
 * - Plant model with augmented 6p-6p topology
 * - Grid-side and machine-side observers for sensorless estimation
 * - Grid-side current controller with firing angle control using estimated values
 * - Machine-side lookup table controller using estimated speed
 * - Complete measurement and filtering chain (raw measurements for observers)
 * - Observer-based feedback instead of direct measurements
 * - Per-unit system for proper scaling
 *
 * Signal Flow:
 * Plant → Raw Measurements → Observers → Estimated Values → Current Controller
 */
class sfc_complete_sfc_closed_loop_simulation_builder {
public:
    sfc_complete_sfc_closed_loop_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          plant{_per_unit_system},
          grid_measurements{_per_unit_system},
          machine_measurements(_per_unit_system),
          dc_link_measurements{_per_unit_system},
          current_controller(_per_unit_system)
    {
        // Connect all components with observer-based feedback
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Mock state machine inputs for standalone testing
        // yeet::mock_signal_input::write(
        //     state_machine.get_inputs().auxiliaries_are_on, false);
        // yeet::mock_signal_input::write(state_machine.get_inputs().breakers_are_on,
        // false); yeet::mock_signal_input::write(state_machine.get_inputs().start_command,
        // false); yeet::mock_signal_input::write(state_machine.get_inputs().stop_command,
        // false);
        // yeet::mock_signal_input::write(state_machine.get_inputs().shutdown_request,
        // false); yeet::mock_signal_input::write(
        //     state_machine.get_inputs().is_current_extinguished, false);
    }

    void initialize()
    {
        // Initialize components in dependency order
        state_machine.initialize();           // Initialize state machine first
        operation_mode_monitor.initialize();  // Initialize mode monitor early
        plant.initialize();
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();
        grid_side_observer.initialize();
        machine_side_observer.initialize();
        speed_controller.initialize();
        current_controller.initialize();
        grid_side_modulator.initialize();
        machine_side_modulator.initialize();
        machine_side_controller.initialize();
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();
    }

    void register_applications(yeet::simulator& sim)
    {
        // Register state machine first
        sim.register_lambda_runner(state_machine.get_sampling_time(), [&]() { state_machine.run(); });

        // Register operation mode monitor
        sim.register_lambda_runner(operation_mode_monitor.get_sampling_time(), [&]() { operation_mode_monitor.run(); });

        // Register all runners with their respective sampling times
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });

        sim.register_lambda_runner(grid_measurements.get_sampling_time(), [&]() { grid_measurements.run(); });

        sim.register_lambda_runner(machine_measurements.get_sampling_time(), [&]() { machine_measurements.run(); });

        sim.register_lambda_runner(dc_link_measurements.get_sampling_time(), [&]() { dc_link_measurements.run(); });

        sim.register_lambda_runner(grid_side_observer.get_sampling_time(), [&]() { grid_side_observer.run(); });

        sim.register_lambda_runner(machine_side_observer.get_sampling_time(), [&]() { machine_side_observer.run(); });

        sim.register_lambda_runner(speed_controller.get_sampling_time(), [&]() { speed_controller.run(); });

        sim.register_lambda_runner(current_controller.get_sampling_time(), [&]() { current_controller.run(); });

        sim.register_lambda_runner(grid_side_modulator.get_sampling_time(), [&]() { grid_side_modulator.run(); });

        sim.register_lambda_runner(machine_side_modulator.get_sampling_time(), [&]() { machine_side_modulator.run(); });

        sim.register_lambda_runner(
            machine_side_controller.get_sampling_time(), [&]() { machine_side_controller.run(); });

        sim.register_lambda_runner(
            grid_side_pulse_generator.get_sampling_time(), [&]() { grid_side_pulse_generator.run(); });

        sim.register_lambda_runner(
            machine_side_pulse_generator.get_sampling_time(), [&]() { machine_side_pulse_generator.run(); });
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control components
    sfc_speed_controller_runner speed_controller;
    sfc_current_controller_runner current_controller;
    sfc_machine_side_controller_runner machine_side_controller;
    sfc_grid_side_modulator_runner grid_side_modulator;
    sfc_machine_side_modulator_runner machine_side_modulator;
    sfc_operation_mode_monitor_runner operation_mode_monitor;

    // Observer components (key addition for sensorless operation)
    sfc_grid_side_observer_runner grid_side_observer;
    sfc_machine_side_observer_runner machine_side_observer;

    // Pulse generators
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;

    // Measurements and filtering (raw measurements for observers)
    sfc_grid_side_measurements_runner grid_measurements;
    sfc_machine_side_measurements_runner machine_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;

    // Plant model
    sfc_plant_model_runner plant;

    // State machine
    sfc_state_machine_runner state_machine;

    // External control signals
    signal_output<torque_pu_t> load_torque_pu;
    signal_output<speed_pu_t> speed_reference_pu;

    // External control signals for state machine
    signal_output<bool> auxiliaries_are_on;
    signal_output<bool> breakers_are_on;
    signal_output<bool> start_command;
    signal_output<bool> stop_command;
    signal_output<bool> shutdown_request;
    signal_output<bool> is_current_extinguished;

private:
    static std::shared_ptr<sfc::sfc_per_unit_system> create_per_unit_system()
    {
        // Standard per-unit system for SFC applications
        // Grid side: 2090V RMS, 50Hz, DC link: 1044A, Machine: 1980V RMS, 2950kW
        return std::make_shared<sfc::sfc_per_unit_system>(voltage_t(2090.0),  // Grid line-to-line voltage (V RMS)
            frequency_t(50.0),                                                // Grid frequency (Hz)
            current_t(1044.0),                                                // DC link current base (A)
            power_t(2950e3),                                                  // Rated power (W)
            voltage_t(1980.0),                                                // Machine line-to-line voltage (V RMS)
            frequency_t(50.0),                                                // Machine base frequency (Hz)
            2                                                                 // Number of pole pairs
        );
    }

    void connect_all_components()
    {
        // ==================== State Machine → Pulse Generators & Controllers ====================
        // Connect state machine pulse releases to both pulse generators
        signals_connector::connect_output_to_input(
            state_machine.get_outputs().pulse_releases, grid_side_pulse_generator.get_inputs().release_pulses);

        signals_connector::connect_output_to_input(
            state_machine.get_outputs().pulse_releases, machine_side_pulse_generator.get_inputs().release_pulses);

        // Connect state machine freeze integrator to controllers
        signals_connector::connect_output_to_input(
            state_machine.get_outputs().freeze_integrator, current_controller.get_inputs().freeze_integrator);

        // ==================== Operation Mode Monitor Connections ====================
        // Connect inputs to operation mode monitor
        signals_connector::connect_output_to_input(
            state_machine.get_outputs().pulse_releases, operation_mode_monitor.get_inputs().enable);

        signals_connector::connect_output_to_input(
            machine_side_observer.get_outputs().estimated_speed, operation_mode_monitor.get_inputs().measured_speed_pu);

        signals_connector::connect_output_to_input(
            dc_link_measurements.get_outputs().dc_current_pu, operation_mode_monitor.get_inputs().dc_link_current_pu);

        // Connect outputs from operation mode monitor
        signals_connector::connect_output_to_input(
            operation_mode_monitor.get_outputs().operating_mode, machine_side_modulator.get_inputs().operating_mode);
        signals_connector::connect_output_to_input(
            operation_mode_monitor.get_outputs().operating_mode, grid_side_modulator.get_inputs().operating_mode);

        signals_connector::connect_output_to_input(operation_mode_monitor.get_outputs().is_current_zero,
            grid_side_modulator.get_inputs().dc_link_current_is_zero);

        // ==================== External Control Signals → State Machine ====================
        // Connect external control signals to state machine inputs
        signals_connector::connect_output_to_input(auxiliaries_are_on, state_machine.get_inputs().auxiliaries_are_on);

        signals_connector::connect_output_to_input(breakers_are_on, state_machine.get_inputs().breakers_are_on);

        signals_connector::connect_output_to_input(start_command, state_machine.get_inputs().start_command);

        signals_connector::connect_output_to_input(stop_command, state_machine.get_inputs().stop_command);

        signals_connector::connect_output_to_input(shutdown_request, state_machine.get_inputs().shutdown_request);

        signals_connector::connect_output_to_input(
            is_current_extinguished, state_machine.get_inputs().is_current_extinguished);

        // ==================== Plant Model → Raw Measurements ====================
        // Connect plant outputs to measurement components (as in simplified current control)

        // Grid measurements: Raw grid voltages and currents from plant
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage, grid_measurements.get_inputs().grid_voltage_measurements);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current, grid_measurements.get_inputs().grid_current_measurements);

        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, grid_measurements.get_inputs().grid_pulses);

        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_measurements.get_inputs().grid_side_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_frequency, grid_measurements.get_inputs().grid_side_frequency);

        // Machine measurements: Raw machine voltages and currents from plant
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage, machine_measurements.get_inputs().machine_voltage_measurements);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_current, machine_measurements.get_inputs().machine_current_measurements);

        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, machine_measurements.get_inputs().machine_pulses);

        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage_source_angle, machine_measurements.get_inputs().machine_side_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_frequency, machine_measurements.get_inputs().machine_side_frequency);

        // DC link measurements: Raw DC voltage and current from plant
        signals_connector::connect_output_to_input(
            plant.get_outputs().dc_link_current, dc_link_measurements.get_inputs().dc_current);

        // ==================== Raw Measurements → Observers ====================
        // Connect filtered measurements to observers for estimation

        // Grid side observer inputs: filtered grid voltage and firing pulses
        signals_connector::connect_output_to_input(
            grid_measurements.get_outputs().grid_voltage_abc, grid_side_observer.get_inputs().grid_voltage_pu);

        // Machine side observer inputs: filtered machine voltage and firing pulses
        signals_connector::connect_output_to_input(machine_measurements.get_outputs().machine_voltage_abc,
            machine_side_observer.get_inputs().machine_voltage_pu);

        // ==================== Observers → Current Controller ====================
        // Connect observer-estimated values to current controller (key difference from direct measurement)

        // DC link current feedback (still use direct measurement as it's most reliable)
        signals_connector::connect_output_to_input(dc_link_measurements.get_outputs().dc_current_filtered_pu,
            current_controller.get_inputs().dc_link_current_measured);

        // Feedforward signals from observers instead of direct measurements
        signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_voltage_magnitude,
            current_controller.get_inputs().grid_side_voltage_magnitude);
        signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_voltage_magnitude,
            current_controller.get_inputs().machine_side_voltage_magnitude);

        signals_connector::connect_output_to_input(grid_side_modulator.get_outputs().is_requesting_main_pi_reset,
            current_controller.get_inputs()
                .modulator_freeze_integrator);  // TODO: Cleanup this, or all freeze requests to the modulator.

        // ==================== Observers → Machine Side Controller ====================
        // Connect observer-estimated speed to machine side controller for lookup table

        // DC link current for lookup table input (direct measurement)
        signals_connector::connect_output_to_input(dc_link_measurements.get_outputs().dc_current_filtered_pu,
            machine_side_controller.get_inputs().dc_link_current);

        // Machine speed from observer instead of direct measurement
        signals_connector::connect_output_to_input(
            machine_side_observer.get_outputs().estimated_speed, machine_side_controller.get_inputs().machine_speed);

        // ==================== Current Controller → Grid Side Modulator ====================
        // Connect current controller output to the new grid-side modulator
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().firing_angle, grid_side_modulator.get_inputs().firing_angle);

        // Connect enable signal to grid-side modulator
        signals_connector::connect_output_to_input(
            state_machine.get_outputs().pulse_releases, grid_side_modulator.get_inputs().enable);

        // Connect DC link current zero detection to grid-side modulator (placeholder for now)
        yeet::mock_signal_input::write(grid_side_modulator.get_inputs().dc_link_current_is_zero, false);

        // Connect force commutation request from machine-side (placeholder for future machine-side modulator)
        // yeet::mock_signal_input::write(grid_side_modulator.get_inputs().request_force_commutation, false);

        // ==================== Grid Side Modulator → Grid Pulse Generator ====================
        // Connect modulator output to the grid-side pulse generator
        signals_connector::connect_output_to_input(
            grid_side_modulator.get_outputs().firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);

        // ==================== Machine Side Controller → Machine Side Modulator ====================
        // Connect machine side controller output to the machine-side modulator
        signals_connector::connect_output_to_input(
            machine_side_controller.get_outputs().firing_angle, machine_side_modulator.get_inputs().firing_angle);

        // Connect enable signal to machine-side modulator
        signals_connector::connect_output_to_input(
            state_machine.get_outputs().pulse_releases, machine_side_modulator.get_inputs().enable);

        signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_position,
            machine_side_modulator.get_inputs().machine_side_angle);

        // Connect operating mode to machine-side modulator (mock as pulse_mode for now)
        yeet::mock_signal_input::write(
            machine_side_modulator.get_inputs().operating_mode, sfc_operating_mode_t::load_commutated);

        // Pulse mode request
        signals_connector::connect_output_to_input(machine_side_modulator.get_outputs().request_force_commutation,
            grid_side_modulator.get_inputs().request_force_commutation);

        signals_connector::connect_output_to_input(grid_side_modulator.get_outputs().force_commutation_acknowledged,
            machine_side_modulator.get_inputs().force_commutation_acknowledged);

        // yeet::mock_signal_input::write(machine_side_modulator.get_inputs().force_commutation_acknowledged, true);

        // Connect force commutation acknowledgment from grid-side modulator to machine-side modulator
        // signals_connector::connect_output_to_input(grid_side_modulator.get_outputs().force_commutation_acknowledged,
        // machine_side_modulator.get_inputs().force_commutation_acknowledged);

        // ==================== Machine Side Modulator → Grid Side Modulator (Handshake) ====================
        // Connect force commutation request from machine-side to grid-side modulator
        signals_connector::connect_output_to_input(machine_side_modulator.get_outputs().request_force_commutation,
            grid_side_modulator.get_inputs().request_force_commutation);

        // ==================== Machine Side Modulator → Machine Pulse Generator ====================
        // Connect modulator output to the machine-side pulse generator
        signals_connector::connect_output_to_input(
            machine_side_modulator.get_outputs().firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);

        // Connect pulse release control from machine-side modulator to machine pulse generator
        signals_connector::connect_output_to_input(machine_side_modulator.get_outputs().release_pulses,
            machine_side_pulse_generator.get_inputs().release_pulses);

        // ==================== Pulse Generators → Plant Model ====================
        // Connect firing pulses from both pulse generators to plant model inputs
        signals_connector::connect_output_to_input(load_torque_pu, plant.get_inputs().load_torque_pu);

        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().grid_side_firing_pulses);

        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().machine_side_firing_pulses);

        // ==================== Plant Model → Pulse Generators ====================
        // For sensorless operation, use observer-estimated angles instead of direct plant angles

        // Grid side: Use observer-estimated angle for pulse generator synchronization
        signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_position,
            grid_side_pulse_generator.get_inputs().reference_angle);

        // Machine side: Use observer-estimated position for pulse generator synchronization
        signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_position,
            machine_side_pulse_generator.get_inputs().reference_angle);

        // ==================== External Control Signals → Speed Controller ====================

        yeet::mock_signal_input::write(speed_controller.get_inputs().enable_speed_control, true);

        // Connect external speed reference to speed controller
        signals_connector::connect_output_to_input(
            speed_reference_pu, speed_controller.get_inputs().speed_reference_pu);

        // Connect observed speed to speed controller
        signals_connector::connect_output_to_input(
            machine_side_observer.get_outputs().estimated_speed, speed_controller.get_inputs().measured_speed_pu);

        // Connect state machine freeze integrator to speed controller
        signals_connector::connect_output_to_input(
            state_machine.get_outputs().freeze_integrator, speed_controller.get_inputs().freeze_integrator);

        signals_connector::connect_output_to_input(current_controller.get_outputs().current_controller_is_saturated,
            speed_controller.get_inputs().current_controller_is_saturated);

        // ==================== Speed Controller → Current Controller ====================
        // Connect speed controller output (current reference) to current controller
        signals_connector::connect_output_to_input(speed_controller.get_outputs().current_reference,
            current_controller.get_inputs().dc_link_current_reference);
    }
};

}  // namespace yeet::application::sfc::integration
