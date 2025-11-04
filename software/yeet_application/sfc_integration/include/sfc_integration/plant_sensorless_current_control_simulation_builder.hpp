#pragma once

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Sensorless Current Controller Simulation Builder ====================

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
 * - Simplified configuration without state machine, modulators, or speed controller
 *
 * Signal Flow:
 * Plant → Raw Measurements → Observers → Estimated Values → Controllers → Pulse Generators → Plant
 */
class plant_sensorless_current_control_simulation_builder {
public:
    static constexpr std::string_view name = "PlantSensorlessCurrentControl";

    plant_sensorless_current_control_simulation_builder()
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
        // Mock control signals for standalone testing
        yeet::mock_signal_input::write(current_controller.get_inputs().freeze_integrator, false);
        yeet::mock_signal_input::write(current_controller.get_inputs().preset_request, false);

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
        grid_side_observer.initialize();
        machine_side_observer.initialize();
        current_controller.initialize();
        machine_side_controller.initialize();
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();
    }

    void register_applications(yeet::simulator& simulator)
    {
        // Register all runners directly in execution order
        simulator.register_runner(plant);
        simulator.register_runner(grid_measurements);
        simulator.register_runner(machine_measurements);
        simulator.register_runner(dc_link_measurements);
        simulator.register_runner(grid_side_observer);
        simulator.register_runner(machine_side_observer);
        simulator.register_runner(current_controller);
        simulator.register_runner(machine_side_controller);
        simulator.register_runner(grid_side_pulse_generator);
        simulator.register_runner(machine_side_pulse_generator);
    }

    void register_datamodel(yeet::model_registry& registry)
    {
        // Register all datamodels
        plant.register_datamodel(registry);
        grid_measurements.register_datamodel(registry);
        machine_measurements.register_datamodel(registry);
        dc_link_measurements.register_datamodel(registry);
        grid_side_observer.register_datamodel(registry);
        machine_side_observer.register_datamodel(registry);
        current_controller.register_datamodel(registry);
        machine_side_controller.register_datamodel(registry);
        grid_side_pulse_generator.register_datamodel(registry);
        machine_side_pulse_generator.register_datamodel(registry);
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control components
    sfc_current_controller_runner current_controller;
    sfc_machine_side_controller_runner machine_side_controller;

    // Observer components (key addition for sensorless operation)
    sfc_grid_side_observer_runner grid_side_observer;
    sfc_machine_side_observer_runner machine_side_observer;

    // Pulse generators
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;

    // Measurements and filtering (raw measurements for observers)
    sfc_grid_side_measurements_ideal_runner grid_measurements;
    sfc_machine_side_measurements_ideal_runner machine_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;

    // Plant model
    sfc_plant_model_runner plant;

    // External control signals
    signal_output<torque_pu_t> load_torque_pu;
    signal_output<current_pu_t> current_reference;

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
        // ==================== Plant Model → Raw Measurements ====================
        // Connect plant outputs to measurement components

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
        signals_connector::connect_output_to_input(
            dc_link_measurements.get_outputs().dc_current_pu, current_controller.get_inputs().current_measured);

        // Feedforward signals from observers instead of direct measurements
        signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_voltage_magnitude,
            current_controller.get_inputs().grid_side_voltage_magnitude);
        signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_voltage_magnitude,
            current_controller.get_inputs().machine_side_voltage_magnitude);

        // Mock the modulator freeze integrator signal for this simplified configuration
        yeet::mock_signal_input::write(current_controller.get_inputs().preset_request, false);

        // ==================== Observers → Machine Side Controller ====================
        // Connect observer-estimated speed to machine side controller for lookup table

        // DC link current for lookup table input (direct measurement)
        signals_connector::connect_output_to_input(dc_link_measurements.get_outputs().dc_current_heavy_filtered_pu,
            machine_side_controller.get_inputs().dc_link_current);

        // Machine speed from observer instead of direct measurement
        signals_connector::connect_output_to_input(
            machine_side_observer.get_outputs().estimated_speed, machine_side_controller.get_inputs().machine_speed);

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

        // ==================== External Control Signals ====================
        // Connect external control signals to current controller inputs

        // Current reference command (setpoint)
        signals_connector::connect_output_to_input(
            current_reference, current_controller.get_inputs().current_reference);
    }
};

}  // namespace yeet::application::sfc::integration
