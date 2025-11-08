#pragma once

#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_plant_model_machine_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Plant Machine Open Loop Simulation Builder ====================

/**
 * @brief Open loop simulation builder with complete machine plant model and pulse generators
 *
 * This configuration provides a simplified testing environment for the complete machine model:
 * - Plant model with 6p-6p topology including complete synchronous machine
 * - Grid-side and machine-side pulse generators
 * - Direct firing angle control (open loop operation)
 * - Fixed field excitation voltage (configurable)
 * - Forced machine speed mode (configurable)
 * - No hardware mocks - direct pulse control
 *
 * Signal Flow:
 * External Firing Angles → Pulse Generators → Plant Model (Machine)
 *
 * Use Cases:
 * - Verification of complete machine model integration
 * - Open-loop converter operation testing
 * - Field excitation and speed control validation
 */
class plant_machine_open_loop_simulation_builder {
public:
    static constexpr std::string_view name = "PlantMachineOpenLoop";

    plant_machine_open_loop_simulation_builder()
    {
        // Initialize firing angles to safe default values
        firing_angle = angle_wrapped::from_degrees(150.0);
        grid_side_firing_angle = angle_wrapped::from_degrees(15.0);

        connect_components();
    }

    void initialize()
    {
        // Initialize pulse generators
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();

        // Initialize plant model
        plant.initialize();
    }

    void register_applications(yeet::simulator& simulator)
    {
        // Register pulse generators at their sampling times
        simulator.register_lambda_runner(
            grid_side_pulse_generator.get_sampling_time(), [&]() { grid_side_pulse_generator.run(); });
        simulator.register_lambda_runner(
            machine_side_pulse_generator.get_sampling_time(), [&]() { machine_side_pulse_generator.run(); });

        // Register plant model
        simulator.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
    }

    void register_datamodel(yeet::model_registry& registry)
    {
        // Register all datamodels
        plant.register_datamodel(registry);
        grid_side_pulse_generator.register_datamodel(registry);
        machine_side_pulse_generator.register_datamodel(registry);
    }

    // Public members for direct access
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;
    sfc_plant_model_machine_runner plant;

    // External control signals
    signal_output<angle_wrapped> firing_angle;
    signal_output<angle_wrapped> grid_side_firing_angle;

private:
    void connect_components()
    {
        // ==================== Pulse Generators → Plant Model ====================
        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().grid_firing_pulses);
        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().machine_firing_pulses);

        // ==================== Plant Model → Pulse Generators ====================
        // Grid side uses grid voltage source angle as reference
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_side_pulse_generator.get_inputs().reference_angle);

        // Machine side uses source voltage phase as reference
        signals_connector::connect_output_to_input(
            plant.get_outputs().source_voltage_phase, machine_side_pulse_generator.get_inputs().reference_angle);

        // ==================== External Control Signals ====================
        signals_connector::connect_output_to_input(
            grid_side_firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);
        signals_connector::connect_output_to_input(
            firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);

        // ==================== Mock pulse release signals ====================
        yeet::mock_signal_input::write(grid_side_pulse_generator.get_inputs().release_pulses, true);
        yeet::mock_signal_input::write(machine_side_pulse_generator.get_inputs().release_pulses, true);
    }
};

}  // namespace yeet::application::sfc::integration
