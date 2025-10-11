#pragma once

#include <string_view>

#include "sfc_integration/sfc_grid_side_control.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/parameter_collection.hpp"
#include "yeet_testing/simulator.hpp"

// Mocks
#include "sfc_grid_side_hardware_mock_runner.hpp"
#include "sfc_integration/plant_hardware_definition.hpp"  // hardware scaling parameter staging
#include "sfc_machine_side_hardware_mock_runner.hpp"

namespace yeet::application::sfc::integration {

// Parameter collection simplified - only plant model parameters
// Control classes manage their own parameters internally
using sfc_complete_sfc_closed_loop_parameters = yeet::parameter_collection<sfc_plant_model_datamodel::parameters>;

/**
 * @brief Complete SFC closed-loop simulation builder using control classes
 *
 * This configuration provides:
 * - Grid-side control: State machine, current controller, observer, modulator, measurements
 * - Machine-side control: Speed controller, machine controller, observer, modulator, measurements
 * - Plant model with augmented 6p-6p topology
 * - Time-based execution: 10ms, 1ms, 200µs sampling rates
 * - Cross-coupling between grid and machine side for operating mode and force commutation
 *
 * Signal Flow:
 * Plant → Control Classes → Observers → Controllers → Modulators → Pulse Generators → Plant
 */
class plant_complete_sfc_closed_loop_simulation_builder {
public:
    static constexpr std::string_view name = "PlantCompleteSFCClosedLoop";

    plant_complete_sfc_closed_loop_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          grid_side_control{_per_unit_system},
          machine_side_control{_per_unit_system},
          plant{_per_unit_system},
          grid_side_hardware_mock{duration_t{10e-6}},
          machine_side_hardware_mock{duration_t{10e-6}}
    {
        // Stage parameters for hardware mocks (ADC + transducers)
        plant_hardware::stage_grid_side_hardware_mock_parameters(grid_side_hardware_mock);
        plant_hardware::stage_machine_side_hardware_mock_parameters(machine_side_hardware_mock);
        // Apply transformer ratio scaling for hardware mocks so ADC counts reflect primary values
        plant_hardware::stage_transformer_ratios_for_grid_side(grid_side_hardware_mock, *_per_unit_system);
        plant_hardware::stage_transformer_ratios_for_machine_side(machine_side_hardware_mock, *_per_unit_system);

        // Stage measurement runner parameters (so runners configure ADC converters during initialize)
        plant_hardware::stage_grid_side_measurement_parameters(grid_side_control.grid_measurements);
        plant_hardware::stage_machine_side_measurement_parameters(machine_side_control.machine_measurements);
        // Apply inverse transformer ratio scaling in measurement reconstruction
        plant_hardware::stage_transformer_ratios_for_grid_side_measurements(
            grid_side_control.grid_measurements, *_per_unit_system);
        plant_hardware::stage_transformer_ratios_for_machine_side_measurements(
            machine_side_control.machine_measurements, *_per_unit_system);

        // Configure plant model to use external excitation voltage control
        configure_plant_for_external_excitation_control();

        // Connect all signal flows between control classes and plant
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Mock external control inputs for standalone testing
        grid_side_control.auxiliaries_are_on.write(false);
        grid_side_control.breakers_are_on.write(false);
        grid_side_control.start_command.write(false);
        grid_side_control.stop_command.write(false);
        grid_side_control.shutdown_request.write(false);
        grid_side_control.is_current_extinguished.write(false);

        // Mock references - REMOVED dc_link_current_reference as it now comes from speed controller
        machine_side_control.speed_reference_pu.write(speed_pu_t{0.0});
        // machine_side_control.enable_speed_control.write(true);

        // Mock excitation control inputs
        machine_side_control.exciter_ready.write(true);  // Exciter is ready (perfect exciter)
        machine_side_control.excitation_start_command.write(true);
        machine_side_control.excitation_stop_command.write(false);

        // Remove these as they're now connected directly:
        // - measured_speed_pu (connected from machine-side observer)
        // - pulse_releases (connected from grid-side state machine)
        // - operating_mode (connected from operation mode monitor)
        // - dc_link_current_pu (connected from dc link measurements)
        // - force_commutation_acknowledged (connected from grid-side modulator)
        // - request_force_commutation (connected from machine-side modulator)
        // - freeze_integrator (connected from state machine)
        // - current_controller_is_saturated (connected from current controller)

        // Only mock the machine voltage magnitude if not available from measurements
        // machine_side_control.machine_side_voltage_magnitude.write(voltage_pu_t{0.0});

        // Mock plant inputs
        load_torque_pu.write(torque_pu_t{0.0});
    }

    void initialize()
    {
        // Initialize in dependency order
        plant.initialize();
        grid_side_hardware_mock.initialize();
        machine_side_hardware_mock.initialize();
        grid_side_control.initialize();
        machine_side_control.initialize();
    }

    void register_applications(yeet::simulator& simulation)
    {
        // Plant model runs at its own sampling time
        simulation.register_lambda_runner(plant.get_sampling_time(), [&]() {
            plant.run();
            grid_side_hardware_mock.run();
            machine_side_hardware_mock.run();
        });

        // 200µs sampling (5 kHz): Measurements, observers, modulators, pulse generators
        simulation.register_lambda_runner(200e-6, [&]() {
            machine_side_control.update_200us();
            grid_side_control.update_200us();
        });

        // 1ms sampling (1 kHz): Current controller, machine controller, operation mode monitor
        simulation.register_lambda_runner(1e-3, [&]() {
            machine_side_control.update_1ms();
            grid_side_control.update_1ms();
        });

        // 10ms sampling (100 Hz): State machine, speed controller
        simulation.register_lambda_runner(10e-3, [&]() {
            machine_side_control.update_10ms();
            grid_side_control.update_10ms();
        });
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control subsystems
    sfc_grid_side_control grid_side_control;
    sfc_machine_side_control machine_side_control;

    // Hardware mocks
    sfc_grid_side_hardware_mock_runner grid_side_hardware_mock;
    sfc_machine_side_hardware_mock_runner machine_side_hardware_mock;

    // Plant model
    sfc_plant_model_runner plant;

    // External control signals
    signal_output<torque_pu_t> load_torque_pu;

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

    void configure_plant_for_external_excitation_control()
    {
        // Configure plant model to use external excitation voltage control mode
        plant.get_parameters().voltage_control_mode =
            sfc_plant_model_datamodel::machine_voltage_control_mode::external_control;
    }

    void connect_all_components()
    {
        // ==================== Plant Inputs to Hardware Mock Measurements ====================
        // Connect plant inputs to hardware mock measurements
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage, grid_side_hardware_mock.get_inputs().grid_voltage);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current, grid_side_hardware_mock.get_inputs().grid_current);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage, machine_side_hardware_mock.get_inputs().machine_voltage);
        // signals_connector::connect_output_to_input(plant.get_outputs().machine_current,
        // machine_side_hardware_mock.get_inputs().machine_current_measurements);

        // ==================== Plant Outputs to Control Class Measurements ====================
        // Connect plant outputs to internal measurement components within control classes

        // Grid side measurements from plant to grid side control internal components
        signals_connector::connect_output_to_input(grid_side_hardware_mock.get_outputs().v_ab_counts,
            grid_side_control.grid_measurements.get_inputs().v_ab_counts);
        signals_connector::connect_output_to_input(grid_side_hardware_mock.get_outputs().v_bc_counts,
            grid_side_control.grid_measurements.get_inputs().v_bc_counts);
        signals_connector::connect_output_to_input(grid_side_hardware_mock.get_outputs().i_a_counts,
            grid_side_control.grid_measurements.get_inputs().i_a_counts);
        signals_connector::connect_output_to_input(grid_side_hardware_mock.get_outputs().i_c_counts,
            grid_side_control.grid_measurements.get_inputs().i_c_counts);

        // Additional grid measurements inputs
        signals_connector::connect_output_to_input(plant.get_outputs().grid_voltage_source_angle,
            grid_side_control.grid_measurements.get_inputs().grid_side_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_frequency, grid_side_control.grid_measurements.get_inputs().grid_side_frequency);
        signals_connector::connect_output_to_input(
            grid_side_control.grid_side_pulse_generator.get_outputs().firing_pulses,
            grid_side_control.grid_measurements.get_inputs().grid_pulses);

        // Machine side measurements from plant to machine side control internal components
        signals_connector::connect_output_to_input(machine_side_hardware_mock.get_outputs().v_ab_counts,
            machine_side_control.machine_measurements.get_inputs().v_ab_counts);
        signals_connector::connect_output_to_input(machine_side_hardware_mock.get_outputs().v_bc_counts,
            machine_side_control.machine_measurements.get_inputs().v_bc_counts);

        // Additional machine measurements inputs
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_side_control.machine_measurements.get_inputs().machine_side_angle);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_frequency,
            machine_side_control.machine_measurements.get_inputs().machine_side_frequency);
        signals_connector::connect_output_to_input(
            machine_side_control.machine_side_pulse_generator.get_outputs().firing_pulses,
            machine_side_control.machine_measurements.get_inputs().machine_pulses);

        // ==================== External Inputs to Control Classes ====================
        // Connect external command signals to appropriate control class components

        // System commands to grid side state machine
        signals_connector::connect_output_to_input(
            auxiliaries_are_on, grid_side_control.state_machine.get_inputs().auxiliaries_are_on);
        signals_connector::connect_output_to_input(
            breakers_are_on, grid_side_control.state_machine.get_inputs().breakers_are_on);
        signals_connector::connect_output_to_input(
            start_command, grid_side_control.state_machine.get_inputs().start_command);
        signals_connector::connect_output_to_input(
            stop_command, grid_side_control.state_machine.get_inputs().stop_command);
        signals_connector::connect_output_to_input(
            shutdown_request, grid_side_control.state_machine.get_inputs().shutdown_request);
        signals_connector::connect_output_to_input(
            is_current_extinguished, grid_side_control.state_machine.get_inputs().is_current_extinguished);

        // Speed command to machine side speed controller
        signals_connector::connect_output_to_input(machine_side_control.speed_reference_pu,
            machine_side_control.speed_controller.get_inputs().speed_reference_pu);

        // Enable speed control
        // machine_side_control.enable_speed_control.write(true);

        // ==================== Cross-Coupling Between Control Classes ====================
        // Connect outputs from one control class to inputs of the other

        // 1. Speed Controller → Current Controller (CRITICAL CASCADE CONNECTION)
        signals_connector::connect_output_to_input(
            machine_side_control.speed_controller.get_outputs().current_reference,
            grid_side_control.current_controller.get_inputs().dc_link_current_reference);

        // 2. Connect pulse releases from grid-side state machine to machine-side modulator enable
        signals_connector::connect_output_to_input(grid_side_control.state_machine.get_outputs().pulse_releases,
            machine_side_control.machine_side_modulator.get_inputs().enable);

        // 3. Connect machine-side estimated speed to grid-side operation mode monitor
        signals_connector::connect_output_to_input(
            machine_side_control.machine_side_observer.get_outputs().estimated_speed,
            grid_side_control.operation_mode_monitor.get_inputs().measured_speed_pu);

        // 4. Connect machine voltage magnitude for current controller feedforward
        // Use machine-side observer estimated voltage magnitude instead of filtered measurement for sensorless parity
        signals_connector::connect_output_to_input(
            machine_side_control.machine_side_observer.get_outputs().estimated_voltage_magnitude,
            grid_side_control.current_controller.get_inputs().machine_side_voltage_magnitude);

        // 5. DC link current to machine-side controller for lookup table
        signals_connector::connect_output_to_input(
            grid_side_control.dc_link_measurements.get_outputs().dc_current_filtered_pu,
            machine_side_control.machine_side_controller.get_inputs().dc_link_current);

        // 6. Operating mode from grid-side to machine-side
        signals_connector::connect_output_to_input(
            grid_side_control.operation_mode_monitor.get_outputs().operating_mode,
            machine_side_control.machine_side_modulator.get_inputs().operating_mode);

        // 7. Force commutation handshake
        signals_connector::connect_output_to_input(
            machine_side_control.machine_side_modulator.get_outputs().request_force_commutation,
            grid_side_control.grid_side_modulator.get_inputs().request_force_commutation);

        signals_connector::connect_output_to_input(
            grid_side_control.grid_side_modulator.get_outputs().force_commutation_acknowledged,
            machine_side_control.machine_side_modulator.get_inputs().force_commutation_acknowledged);

        // 8. Freeze integrator cascading from grid-side to machine-side
        signals_connector::connect_output_to_input(grid_side_control.state_machine.get_outputs().freeze_integrator,
            machine_side_control.speed_controller.get_inputs().freeze_integrator);

        // 9. Current controller saturation to speed controller for anti-windup
        signals_connector::connect_output_to_input(
            grid_side_control.current_controller.get_outputs().current_controller_is_saturated,
            machine_side_control.speed_controller.get_inputs().current_controller_is_saturated);

        signals_connector::connect_output_to_input(grid_side_control.state_machine.get_outputs().pulse_releases,
            machine_side_control.speed_controller.get_inputs().enable_speed_control);

        signals_connector::connect_output_to_input(
            grid_side_control.grid_side_modulator.get_outputs().is_requesting_main_pi_reset,
            machine_side_control.speed_controller.get_inputs().modulator_freeze_integrator);

        // ==================== Control Class Outputs to Plant ====================
        // Connect control class firing pulses directly to plant (bypassing external pulse generators for now)
        signals_connector::connect_output_to_input(
            grid_side_control.grid_side_pulse_generator.get_outputs().firing_pulses,
            plant.get_inputs().grid_side_firing_pulses);

        signals_connector::connect_output_to_input(
            machine_side_control.machine_side_pulse_generator.get_outputs().firing_pulses,
            plant.get_inputs().machine_side_firing_pulses);

        // ==================== Additional Plant Inputs ====================
        // Connect load torque to plant
        signals_connector::connect_output_to_input(load_torque_pu, plant.get_inputs().load_torque_pu);

        // ==================== Excitation Control to Plant ====================
        // Connect excitation voltage setpoint to plant model (external excitation control)
        signals_connector::connect_output_to_input(
            machine_side_control.excitation_controller.get_outputs().excitation_voltage_setpoint_pu,
            plant.get_inputs().excitation_voltage_setpoint_pu);
    }
};

}  // namespace yeet::application::sfc::integration
