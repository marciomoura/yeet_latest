
#pragma once

#include "sfc_grid_side_pulse_generator_datamodel.hpp"
#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_machine_side_pulse_generator_datamodel.hpp"
#include "yeet/model_registry.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/parameter_collection.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Current Controller Simulation Builder ====================

// Update the parameter collection to include machine side controller
using sfc_current_controller_simulation_parameters =
    yeet::parameter_collection<sfc_current_controller_datamodel::parameters,
        sfc_grid_side_pulse_generator_datamodel::parameters,
        sfc_machine_side_pulse_generator_datamodel::parameters,
        sfc_machine_side_controller_datamodel::parameters,
        sfc_grid_side_measurements_datamodel::parameters,
        sfc_machine_side_measurements_datamodel::parameters,
        sfc_dc_link_measurements_datamodel::parameters,
        sfc_plant_model_datamodel::parameters>;

/**
 * @brief Current controller simulation builder with plant model and measurements
 *
 * This configuration provides:
 * - Plant model with augmented 6p-6p topology
 * - Grid-side current controller with firing angle control
 * - Machine-side lookup table controller for firing angle calculation
 * - Complete measurement and filtering chain
 * - Per-unit system for proper scaling
 */
class sfc_current_controller_simulation_builder
    : public yeet::closed_loop_configuration<sfc_current_controller_simulation_parameters> {
public:
    using parameter_types = sfc_current_controller_simulation_parameters;

    sfc_current_controller_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          plant{_per_unit_system},
          grid_measurements{_per_unit_system},
          machine_measurements(_per_unit_system),
          dc_link_measurements{_per_unit_system}
    {
        // Connect all components
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        yeet::mock_signal_input::write(
            current_controller.get_signal_flow_datamodel().get_inputs().freeze_integrator, false);
        // Mock pulse release signals for both pulse generators
        yeet::mock_signal_input::write(
            grid_side_pulse_generator.get_signal_flow_datamodel().get_inputs().release_pulses, true);
        yeet::mock_signal_input::write(
            machine_side_pulse_generator.get_signal_flow_datamodel().get_inputs().release_pulses, true);
    }

    void initialize()
    {
        // Initialize components in dependency order
        plant.initialize();
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();
        current_controller.initialize();
        machine_side_controller.initialize();
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();
    }

    void register_applications(yeet::simulator& sim) override
    {
        // Register all runners with their respective sampling times
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
        sim.register_lambda_runner(grid_measurements.get_sampling_time(), [&]() { grid_measurements.run(); });
        sim.register_lambda_runner(machine_measurements.get_sampling_time(), [&]() { machine_measurements.run(); });
        sim.register_lambda_runner(dc_link_measurements.get_sampling_time(), [&]() { dc_link_measurements.run(); });
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
        // Register all datamodels
        plant.register_datamodel(registry);
        grid_measurements.register_datamodel(registry);
        machine_measurements.register_datamodel(registry);
        dc_link_measurements.register_datamodel(registry);
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
            plant.get_configuration_datamodel().get_parameters());
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control components
    sfc_current_controller_runner current_controller;
    sfc_machine_side_controller_runner machine_side_controller;

    // Pulse generators
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;

    // Measurements and filtering
    sfc_grid_side_measurements_runner grid_measurements;
    sfc_machine_side_measurements_runner machine_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;

    // Plant model
    sfc_plant_model_runner plant;

    // External control signals
    signal_output<torque_pu_t> load_torque_pu;
    signal_output<current_pu_t> current_reference_pu;

private:
    static std::shared_ptr<sfc::sfc_per_unit_system> create_per_unit_system()
    {
        return std::make_shared<sfc::sfc_per_unit_system>(voltage_t(2090.0),  // Grid voltage RMS
            frequency_t(50.0),                                                // Grid frequency
            current_t(1044.0),                                                // DC link current base
            power_t(2950e3),                                                  // Machine power
            voltage_t(1980.0),                                                // Machine voltage RMS
            frequency_t(50.0),                                                // Machine frequency
            2                                                                 // Number of poles
        );
    }

    void connect_all_components()
    {
        // Get datamodel references for all components
        auto& current_ctrl_model = current_controller.get_signal_flow_datamodel();
        auto& machine_ctrl_model = machine_side_controller.get_signal_flow_datamodel();
        auto& grid_pulse_gen_model = grid_side_pulse_generator.get_signal_flow_datamodel();
        auto& machine_pulse_gen_model = machine_side_pulse_generator.get_signal_flow_datamodel();
        auto& grid_meas_model = grid_measurements.get_signal_flow_datamodel();
        auto& machine_meas_model = machine_measurements.get_signal_flow_datamodel();
        auto& dc_link_model = dc_link_measurements.get_signal_flow_datamodel();
        auto& plant_model = plant.get_signal_flow_datamodel();

        // ==================== Plant Model → Measurements ====================
        // Connect raw plant outputs to measurement system inputs

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

        // ==================== Measurements → Current Controller ====================
        // Connect filtered measurements to current controller feedback

        // DC link current feedback (primary feedback signal)
        signals_connector::connect_output_to_input(dc_link_model.get_outputs().dc_current_filtered_pu,
            current_ctrl_model.get_inputs().dc_link_current_measured_pu);

        // ==================== DC Link Measurements → Machine Side Controller ====================
        // Connect DC link current and speed to machine side controller for lookup table

        // DC link current for lookup table input
        signals_connector::connect_output_to_input(
            dc_link_model.get_outputs().dc_current_filtered_pu, machine_ctrl_model.get_inputs().dc_link_current);

        // Machine speed for lookup table input
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().machine_speed, machine_ctrl_model.get_inputs().machine_speed);

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
        // Connect reference angles from plant model to pulse generators for synchronization

        signals_connector::connect_output_to_input(
            plant_model.get_outputs().grid_voltage_source_angle, grid_pulse_gen_model.get_inputs().reference_angle);
        signals_connector::connect_output_to_input(plant_model.get_outputs().machine_voltage_source_angle,
            machine_pulse_gen_model.get_inputs().reference_angle);

        // ==================== External Control Signals ====================
        // Connect external control signals to current controller inputs

        // Current reference command (setpoint)
        signals_connector::connect_output_to_input(
            current_reference_pu, current_ctrl_model.get_inputs().current_reference_pu);

        // Feedforward signals for improved control performance
        signals_connector::connect_output_to_input(grid_meas_model.get_outputs().grid_voltage_magnitude_filtered,
            current_ctrl_model.get_inputs().ideal_dc_voltage_pu);
        signals_connector::connect_output_to_input(machine_meas_model.get_outputs().machine_voltage_magnitude_filtered,
            current_ctrl_model.get_inputs().inverter_back_emf_machine_pu);
    }
};

}  // namespace yeet::application::sfc::integration