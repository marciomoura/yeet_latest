#pragma once

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_datamodel.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_datamodel.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
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
    sfc_plant_model_datamodel::parameters,
    sfc_speed_controller_datamodel::parameters,
    sfc_state_machine_datamodel::parameters>;

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
class sfc_complete_sfc_closed_loop_simulation_builder
    : public yeet::closed_loop_configuration<sfc_complete_sfc_closed_loop_parameters> {
public:
    using parameter_types = sfc_complete_sfc_closed_loop_parameters;

    sfc_complete_sfc_closed_loop_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          plant{_per_unit_system},
          grid_measurements{_per_unit_system},
          machine_measurements(_per_unit_system),
          dc_link_measurements{_per_unit_system}
    {
        // Connect all components with observer-based feedback
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Mock state machine inputs for standalone testing
        // yeet::mock_signal_input::write(
        //     state_machine.get_signal_flow_datamodel().get_inputs().auxiliaries_are_on, false);
        // yeet::mock_signal_input::write(state_machine.get_signal_flow_datamodel().get_inputs().breakers_are_on,
        // false); yeet::mock_signal_input::write(state_machine.get_signal_flow_datamodel().get_inputs().start_command,
        // false); yeet::mock_signal_input::write(state_machine.get_signal_flow_datamodel().get_inputs().stop_command,
        // false);
        // yeet::mock_signal_input::write(state_machine.get_signal_flow_datamodel().get_inputs().shutdown_request,
        // false); yeet::mock_signal_input::write(
        //     state_machine.get_signal_flow_datamodel().get_inputs().is_current_extinguished, false);
    }

    void initialize()
    {
        // Initialize components in dependency order
        state_machine.initialize();  // Initialize state machine first
        plant.initialize();
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();
        grid_side_observer.initialize();
        machine_side_observer.initialize();
        speed_controller.initialize();
        current_controller.initialize();
        machine_side_controller.initialize();
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();
    }

    void register_applications(yeet::simulator& sim) override
    {
        // Register state machine first
        sim.register_lambda_runner(state_machine.get_sampling_time(), [&]() { state_machine.run(); });

        // Register all runners with their respective sampling times
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });

        sim.register_lambda_runner(grid_measurements.get_sampling_time(), [&]() { grid_measurements.run(); });

        sim.register_lambda_runner(machine_measurements.get_sampling_time(), [&]() { machine_measurements.run(); });

        sim.register_lambda_runner(dc_link_measurements.get_sampling_time(), [&]() { dc_link_measurements.run(); });

        sim.register_lambda_runner(grid_side_observer.get_sampling_time(), [&]() { grid_side_observer.run(); });

        sim.register_lambda_runner(machine_side_observer.get_sampling_time(), [&]() { machine_side_observer.run(); });

        sim.register_lambda_runner(speed_controller.get_sampling_time(), [&]() { speed_controller.run(); });

        sim.register_lambda_runner(current_controller.get_sampling_time(), [&]() { current_controller.run(); });

        sim.register_lambda_runner(
            machine_side_controller.get_sampling_time(), [&]() { machine_side_controller.run(); });

        sim.register_lambda_runner(
            grid_side_pulse_generator.get_sampling_time(), [&]() { grid_side_pulse_generator.run(); });

        sim.register_lambda_runner(
            machine_side_pulse_generator.get_sampling_time(), [&]() { machine_side_pulse_generator.run(); });
    }

    void register_datamodel(yeet::model_registry& registry) override
    {
        // Register state machine datamodel
        state_machine.register_datamodel(registry);

        // Register all datamodels
        plant.register_datamodel(registry);
        grid_measurements.register_datamodel(registry);
        machine_measurements.register_datamodel(registry);
        dc_link_measurements.register_datamodel(registry);
        grid_side_observer.register_datamodel(registry);
        machine_side_observer.register_datamodel(registry);
        speed_controller.register_datamodel(registry);
        current_controller.register_datamodel(registry);
        machine_side_controller.register_datamodel(registry);
        grid_side_pulse_generator.register_datamodel(registry);
        machine_side_pulse_generator.register_datamodel(registry);
    }

    parameter_types create_parameter_collection() override
    {
        return parameter_types(current_controller.get_configuration_datamodel().get_parameters(),
            grid_side_pulse_generator.get_configuration_datamodel().get_parameters(),
            machine_side_pulse_generator.get_configuration_datamodel().get_parameters(),
            machine_side_controller.get_configuration_datamodel().get_parameters(),
            grid_measurements.get_configuration_datamodel().get_parameters(),
            machine_measurements.get_configuration_datamodel().get_parameters(),
            dc_link_measurements.get_configuration_datamodel().get_parameters(),
            grid_side_observer.get_configuration_datamodel().get_parameters(),
            machine_side_observer.get_configuration_datamodel().get_parameters(),
            plant.get_configuration_datamodel().get_parameters(),
            speed_controller.get_configuration_datamodel().get_parameters(),
            state_machine.get_configuration_datamodel().get_parameters());
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control components
    sfc_speed_controller_runner speed_controller;
    sfc_current_controller_runner current_controller;
    sfc_machine_side_controller_runner machine_side_controller;

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
        // Get datamodel references
        auto& state_machine_model = state_machine.get_signal_flow_datamodel();
        auto& plant_model = plant.get_signal_flow_datamodel();
        auto& grid_meas_model = grid_measurements.get_signal_flow_datamodel();
        auto& machine_meas_model = machine_measurements.get_signal_flow_datamodel();
        auto& dc_link_model = dc_link_measurements.get_signal_flow_datamodel();
        auto& grid_observer_model = grid_side_observer.get_signal_flow_datamodel();
        auto& machine_observer_model = machine_side_observer.get_signal_flow_datamodel();
        auto& speed_controller_model = speed_controller.get_signal_flow_datamodel();
        auto& current_ctrl_model = current_controller.get_signal_flow_datamodel();
        auto& machine_ctrl_model = machine_side_controller.get_signal_flow_datamodel();
        auto& grid_pulse_gen_model = grid_side_pulse_generator.get_signal_flow_datamodel();
        auto& machine_pulse_gen_model = machine_side_pulse_generator.get_signal_flow_datamodel();

        // ==================== State Machine → Pulse Generators & Controllers ====================
        // Connect state machine pulse releases to both pulse generators
        signals_connector::connect_output_to_input(
            state_machine_model.get_outputs().pulse_releases, grid_pulse_gen_model.get_inputs().release_pulses);

        signals_connector::connect_output_to_input(
            state_machine_model.get_outputs().pulse_releases, machine_pulse_gen_model.get_inputs().release_pulses);

        // Connect state machine freeze integrator to controllers
        signals_connector::connect_output_to_input(
            state_machine_model.get_outputs().freeze_integrator, current_ctrl_model.get_inputs().freeze_integrator);

        // ==================== External Control Signals → State Machine ====================
        // Connect external control signals to state machine inputs
        signals_connector::connect_output_to_input(
            auxiliaries_are_on, state_machine_model.get_inputs().auxiliaries_are_on);

        signals_connector::connect_output_to_input(breakers_are_on, state_machine_model.get_inputs().breakers_are_on);

        signals_connector::connect_output_to_input(start_command, state_machine_model.get_inputs().start_command);

        signals_connector::connect_output_to_input(stop_command, state_machine_model.get_inputs().stop_command);

        signals_connector::connect_output_to_input(shutdown_request, state_machine_model.get_inputs().shutdown_request);

        signals_connector::connect_output_to_input(
            is_current_extinguished, state_machine_model.get_inputs().is_current_extinguished);

        // ==================== Plant Model → Raw Measurements ====================
        // Connect plant outputs to measurement components (as in simplified current control)

        // Grid measurements: Raw grid voltages and currents from plant
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().grid_voltage, grid_meas_model.get_inputs().grid_voltage_measurements);
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().grid_current, grid_meas_model.get_inputs().grid_current_measurements);

        signals_connector::connect_output_to_input(
            grid_pulse_gen_model.get_outputs().firing_pulses, grid_meas_model.get_inputs().grid_pulses);

        signals_connector::connect_output_to_input(
            plant_model.get_outputs().grid_voltage_source_angle, grid_meas_model.get_inputs().grid_side_angle);
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().grid_frequency, grid_meas_model.get_inputs().grid_side_frequency);

        // Machine measurements: Raw machine voltages and currents from plant
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().machine_voltage, machine_meas_model.get_inputs().machine_voltage_measurements);
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().machine_current, machine_meas_model.get_inputs().machine_current_measurements);

        signals_connector::connect_output_to_input(
            machine_pulse_gen_model.get_outputs().firing_pulses, machine_meas_model.get_inputs().machine_pulses);

        signals_connector::connect_output_to_input(
            plant_model.get_outputs().machine_voltage_source_angle, machine_meas_model.get_inputs().machine_side_angle);
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().machine_frequency, machine_meas_model.get_inputs().machine_side_frequency);

        // DC link measurements: Raw DC voltage and current from plant
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().dc_link_current, dc_link_model.get_inputs().dc_current);

        // ==================== Raw Measurements → Observers ====================
        // Connect filtered measurements to observers for estimation

        // Grid side observer inputs: filtered grid voltage and firing pulses
        signals_connector::connect_output_to_input(
            grid_meas_model.get_outputs().grid_voltage_abc, grid_observer_model.get_inputs().grid_voltage_pu);

        // Machine side observer inputs: filtered machine voltage and firing pulses
        signals_connector::connect_output_to_input(machine_meas_model.get_outputs().machine_voltage_abc,
            machine_observer_model.get_inputs().machine_voltage_pu);

        // ==================== Observers → Current Controller ====================
        // Connect observer-estimated values to current controller (key difference from direct measurement)

        // DC link current feedback (still use direct measurement as it's most reliable)
        signals_connector::connect_output_to_input(dc_link_model.get_outputs().dc_current_filtered_pu,
            current_ctrl_model.get_inputs().dc_link_current_measured_pu);

        // Feedforward signals from observers instead of direct measurements
        signals_connector::connect_output_to_input(grid_observer_model.get_outputs().estimated_voltage_magnitude,
            current_ctrl_model.get_inputs().ideal_dc_voltage_pu);
        signals_connector::connect_output_to_input(machine_observer_model.get_outputs().estimated_voltage_magnitude,
            current_ctrl_model.get_inputs().inverter_back_emf_machine_pu);

        // ==================== Observers → Machine Side Controller ====================
        // Connect observer-estimated speed to machine side controller for lookup table

        // DC link current for lookup table input (direct measurement)
        signals_connector::connect_output_to_input(
            dc_link_model.get_outputs().dc_current_filtered_pu, machine_ctrl_model.get_inputs().dc_link_current);

        // Machine speed from observer instead of direct measurement
        signals_connector::connect_output_to_input(
            machine_observer_model.get_outputs().estimated_speed, machine_ctrl_model.get_inputs().machine_speed);

        // ==================== Current Controller → Grid Pulse Generator ====================
        // Connect current controller output to grid-side firing angle control

        // Firing angle command from current controller to grid pulse generator
        signals_connector::connect_output_to_input(
            current_ctrl_model.get_outputs().firing_angle_reference, grid_pulse_gen_model.get_inputs().firing_angle);

        // ==================== Machine Side Controller → Machine Pulse Generator ====================
        // Connect machine side controller output to machine-side firing angle control

        // Firing angle command from machine side controller to machine pulse generator
        signals_connector::connect_output_to_input(machine_ctrl_model.get_outputs().machine_side_firing_angle,
            machine_pulse_gen_model.get_inputs().firing_angle);

        // ==================== Pulse Generators → Plant Model ====================
        // Connect firing pulses from both pulse generators to plant model inputs
        signals_connector::connect_output_to_input(load_torque_pu, plant_model.get_inputs().load_torque_pu);

        signals_connector::connect_output_to_input(
            grid_pulse_gen_model.get_outputs().firing_pulses, plant_model.get_inputs().grid_side_firing_pulses);

        signals_connector::connect_output_to_input(
            machine_pulse_gen_model.get_outputs().firing_pulses, plant_model.get_inputs().machine_side_firing_pulses);

        // ==================== Plant Model → Pulse Generators ====================
        // For sensorless operation, use observer-estimated angles instead of direct plant angles

        // Grid side: Use observer-estimated angle for pulse generator synchronization
        signals_connector::connect_output_to_input(
            grid_observer_model.get_outputs().estimated_angle, grid_pulse_gen_model.get_inputs().reference_angle);

        // Machine side: Use observer-estimated position for pulse generator synchronization
        signals_connector::connect_output_to_input(machine_observer_model.get_outputs().estimated_position,
            machine_pulse_gen_model.get_inputs().reference_angle);

        // ==================== External Control Signals → Speed Controller ====================

        yeet::mock_signal_input::write(speed_controller_model.get_inputs().enable_speed_control, true);

        // Connect external speed reference to speed controller
        signals_connector::connect_output_to_input(
            speed_reference_pu, speed_controller_model.get_inputs().speed_reference_pu);

        // Connect observed speed to speed controller
        signals_connector::connect_output_to_input(machine_observer_model.get_outputs().estimated_speed,
            speed_controller_model.get_inputs().measured_speed_pu);

        // Connect state machine freeze integrator to speed controller
        signals_connector::connect_output_to_input(
            state_machine_model.get_outputs().freeze_integrator, speed_controller_model.get_inputs().freeze_integrator);

        // ==================== Speed Controller → Current Controller ====================
        // Connect speed controller output (current reference) to current controller
        signals_connector::connect_output_to_input(speed_controller_model.get_outputs().current_reference,
            current_ctrl_model.get_inputs().current_reference_pu);
    }
};

}  // namespace yeet::application::sfc::integration
