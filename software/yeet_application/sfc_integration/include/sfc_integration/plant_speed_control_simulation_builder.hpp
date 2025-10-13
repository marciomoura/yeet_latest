#pragma once

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Speed Controller Simulation Builder ====================

/**
 * @brief Speed controller simulation builder with cascaded speed and current control
 *
 * This configuration provides:
 * - Plant model with augmented 6p-6p topology
 * - Cascaded speed and current controller (speed outer loop, current inner loop)
 * - Grid-side current controller with firing angle control
 * - Machine-side lookup table controller for firing angle calculation
 * - Complete measurement and filtering chain
 * - Per-unit system for proper scaling
 * - Simplified configuration without observers, state machine, or modulators
 *
 * Signal Flow:
 * Plant → Raw Measurements → Speed Controller → Current Controller → Pulse Generators → Plant
 */
class plant_speed_control_simulation_builder {
public:
    static constexpr std::string_view name = "PlantSpeedControl";

    plant_speed_control_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          plant{_per_unit_system},
          grid_measurements{_per_unit_system},
          machine_measurements(_per_unit_system),
          dc_link_measurements{_per_unit_system},
          current_controller(_per_unit_system)
    {
        // Connect all components
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Mock control signals for standalone testing
        yeet::mock_signal_input::write(current_controller.get_inputs().freeze_integrator, false);
        yeet::mock_signal_input::write(current_controller.get_inputs().modulator_freeze_integrator, false);
        yeet::mock_signal_input::write(speed_controller.get_inputs().freeze_integrator, false);
        yeet::mock_signal_input::write(speed_controller.get_inputs().modulator_freeze_integrator, false);

        // Mock pulse release signals for both pulse generators
        yeet::mock_signal_input::write(grid_side_pulse_generator.get_inputs().release_pulses, true);
        yeet::mock_signal_input::write(machine_side_pulse_generator.get_inputs().release_pulses, true);
    }

    void initialize()
    {
        // Initialize components in dependency order
        plant.initialize();
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();
        speed_controller.initialize();
        current_controller.initialize();
        machine_side_controller.initialize();
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();
    }

    void register_applications(yeet::simulator& simulator)
    {
        // Register all runners with their respective sampling times
        simulator.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
        simulator.register_lambda_runner(grid_measurements.get_sampling_time(), [&]() { grid_measurements.run(); });
        simulator.register_lambda_runner(
            machine_measurements.get_sampling_time(), [&]() { machine_measurements.run(); });
        simulator.register_lambda_runner(
            dc_link_measurements.get_sampling_time(), [&]() { dc_link_measurements.run(); });
        simulator.register_lambda_runner(speed_controller.get_sampling_time(), [&]() {
            speed_controller.run();
            // Convert torque to current (1:1 assumption for simplified test without torque controller)
            current_reference_from_speed_ctrl.write(
                current_pu_t{speed_controller.get_outputs().torque_reference.read().value()});
        });
        simulator.register_lambda_runner(current_controller.get_sampling_time(), [&]() { current_controller.run(); });
        simulator.register_lambda_runner(
            machine_side_controller.get_sampling_time(), [&]() { machine_side_controller.run(); });
        simulator.register_lambda_runner(
            grid_side_pulse_generator.get_sampling_time(), [&]() { grid_side_pulse_generator.run(); });
        simulator.register_lambda_runner(
            machine_side_pulse_generator.get_sampling_time(), [&]() { machine_side_pulse_generator.run(); });
    }

    void register_datamodel(yeet::model_registry& registry)
    {
        // Register all datamodels
        plant.register_datamodel(registry);
        grid_measurements.register_datamodel(registry);
        machine_measurements.register_datamodel(registry);
        dc_link_measurements.register_datamodel(registry);
        speed_controller.register_datamodel(registry);
        current_controller.register_datamodel(registry);
        machine_side_controller.register_datamodel(registry);
        grid_side_pulse_generator.register_datamodel(registry);
        machine_side_pulse_generator.register_datamodel(registry);
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control components
    sfc_speed_controller_runner speed_controller;
    sfc_current_controller_runner current_controller;
    sfc_machine_side_controller_runner machine_side_controller;

    // Pulse generators
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;

    // Measurements and filtering
    sfc_grid_side_measurements_ideal_runner grid_measurements;
    sfc_machine_side_measurements_ideal_runner machine_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;

    // Plant model
    sfc_plant_model_runner plant;

    // External control signals
    signal_output<torque_pu_t> load_torque_pu;
    signal_output<speed_pu_t> speed_reference_pu;
    signal_output<bool> enable_speed_control;
    signal_output<current_pu_t> external_current_reference_pu;

    // Internal signal for torque-to-current conversion (simplified tests without torque controller)
    signal_output<current_pu_t> current_reference_from_speed_ctrl;

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
        // ==================== Plant Model → Measurements ====================
        // Connect raw plant outputs to measurement system inputs

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
            grid_measurements.get_outputs().grid_current_abc, dc_link_measurements.get_inputs().grid_side_current);

        // ==================== Speed Measurements → Speed Controller ====================
        // Connect speed measurements to speed controller feedback

        // Machine speed feedback (from machine measurements)
        signals_connector::connect_output_to_input(
            machine_measurements.get_outputs().machine_side_speed, speed_controller.get_inputs().measured_speed_pu);

        // ==================== Speed Controller → Current Controller ====================
        // Connect speed controller output to current controller reference (cascade control)

        // Current reference from speed controller to current controller (1:1 torque-to-current assumption)
        // NOTE: In full SFC system, torque controller sits between speed and current controllers
        // Using intermediate signal for type conversion (torque_pu_t → current_pu_t)
        signals_connector::connect_output_to_input(
            current_reference_from_speed_ctrl, current_controller.get_inputs().dc_link_current_reference);

        // ==================== DC Link Measurements → Current Controller ====================
        // Connect DC link current feedback to current controller

        // DC link current feedback (primary feedback signal for current loop)
        signals_connector::connect_output_to_input(
            dc_link_measurements.get_outputs().dc_current_pu, current_controller.get_inputs().dc_link_current_measured);

        // Mock the modulator freeze integrator signal for this simplified configuration
        yeet::mock_signal_input::write(current_controller.get_inputs().modulator_freeze_integrator, false);

        // ==================== Speed Controller → Machine Side Controller ====================
        // Connect speed controller output and speed to machine side controller for lookup table

        // DC link current reference for lookup table input (1:1 torque-to-current assumption)
        signals_connector::connect_output_to_input(
            current_reference_from_speed_ctrl, machine_side_controller.get_inputs().dc_link_current);

        // Machine speed for lookup table input
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_speed, machine_side_controller.get_inputs().machine_speed);

        // ==================== Current Controller → Grid Pulse Generator ====================
        // Connect current controller output to grid-side firing angle control

        // Firing angle command from current controller to grid pulse generator
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);

        // ==================== Machine Side Controller → Machine Pulse Generator ====================
        // Connect machine side controller output to machine-side firing angle control

        // Firing angle command from machine side controller to machine pulse generator
        signals_connector::connect_output_to_input(
            machine_side_controller.get_outputs().firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);

        // ==================== Pulse Generators → Plant Model ====================
        // Connect firing pulses from both pulse generators to plant model inputs

        // Load torque input to plant model
        signals_connector::connect_output_to_input(load_torque_pu, plant.get_inputs().load_torque_pu);

        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().grid_side_firing_pulses);

        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().machine_side_firing_pulses);

        // ==================== Plant Model → Pulse Generators ====================
        // Connect reference angles from plant model to pulse generators for synchronization

        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_side_pulse_generator.get_inputs().reference_angle);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_side_pulse_generator.get_inputs().reference_angle);

        // ==================== External Control Signals ====================
        // Connect external control signals to speed controller inputs

        // Speed reference command (setpoint for outer loop)
        signals_connector::connect_output_to_input(
            speed_reference_pu, speed_controller.get_inputs().speed_reference_pu);

        // Speed control enable/disable
        signals_connector::connect_output_to_input(
            enable_speed_control, speed_controller.get_inputs().enable_speed_control);

        signals_connector::connect_output_to_input(current_controller.get_outputs().current_controller_is_saturated,
            speed_controller.get_inputs().current_controller_is_saturated);

        // Feedforward signals for improved current control performance
        signals_connector::connect_output_to_input(grid_measurements.get_outputs().grid_voltage_magnitude_filtered,
            current_controller.get_inputs().grid_side_voltage_magnitude);
        signals_connector::connect_output_to_input(
            machine_measurements.get_outputs().machine_voltage_magnitude_filtered,
            current_controller.get_inputs().machine_side_voltage_magnitude);
    }
};

}  // namespace yeet::application::sfc::integration