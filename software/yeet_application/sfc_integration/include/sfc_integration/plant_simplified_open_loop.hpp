#pragma once

#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Open Loop Simulation Builder ====================

/**
 * @brief Open loop simulation builder with plant model and pulse generators
 *
 * This configuration provides:
 * - Plant model with augmented 6p-6p topology
 * - Grid-side and machine-side pulse generators
 * - Direct firing angle control (open loop operation)
 * - Per-unit system for proper scaling
 * - Simplified configuration for open loop testing
 *
 * Signal Flow:
 * External Firing Angles → Pulse Generators → Plant Model
 */
class sfc_open_loop_simulation_builder {
public:
    sfc_open_loop_simulation_builder() : plant{create_per_unit_system()}
    {
        // Connect pulse generators to plant model
        connect_pulse_generators_to_plant();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Mock pulse release signals for both pulse generators
        yeet::mock_signal_input::write(grid_side_pulse_generator.get_inputs().release_pulses, true);
        yeet::mock_signal_input::write(machine_side_pulse_generator.get_inputs().release_pulses, true);
    }

    void initialize()
    {
        // Initialize components in dependency order
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();
        plant.initialize();
    }

    void register_applications(yeet::simulator& sim)
    {
        // Register all runners with their respective sampling times
        sim.register_lambda_runner(
            grid_side_pulse_generator.get_sampling_time(), [&]() { grid_side_pulse_generator.run(); });
        sim.register_lambda_runner(
            machine_side_pulse_generator.get_sampling_time(), [&]() { machine_side_pulse_generator.run(); });
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
    }

    void register_datamodel(yeet::model_registry& registry)
    {
        // Register all datamodels
        grid_side_pulse_generator.register_datamodel(registry);
        machine_side_pulse_generator.register_datamodel(registry);
        plant.register_datamodel(registry);
    }

    // Public members for direct access
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;
    sfc_plant_model_runner plant;

    signal_output<torque_pu_t> load_torque_pu;
    signal_output<angle_wrapped> firing_angle;
    signal_output<angle_wrapped> grid_side_firing_angle;

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

    void connect_pulse_generators_to_plant()
    {
        // ==================== Pulse Generators → Plant Model ====================
        // Connect firing pulses from both pulse generators to plant model inputs
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
        // Connect external firing angle commands to pulse generators

        signals_connector::connect_output_to_input(
            grid_side_firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);
        signals_connector::connect_output_to_input(
            firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);
    }
};

}  // namespace yeet::application::sfc::integration