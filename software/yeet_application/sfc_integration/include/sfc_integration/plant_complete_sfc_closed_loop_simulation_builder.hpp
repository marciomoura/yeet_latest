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
          grid_side_control{},
          machine_side_control{},
          plant{_per_unit_system},
          grid_side_hardware_mock{duration_t{10e-6}},
          machine_side_hardware_mock{duration_t{10e-6}}
    {
        *grid_side_control.per_unit_system = *_per_unit_system;
        *machine_side_control.per_unit_system = *_per_unit_system;

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
        // Configure machine-side excitation control via datamodel overrides
        machine_side_control.external_ios.get_parameters().override_excitation_inputs.write(true);
        machine_side_control.external_ios.get_parameters().exciter_ready.write(
            true);  // Exciter is ready (perfect exciter)
        machine_side_control.external_ios.get_parameters().excitation_start_command.write(true);
        machine_side_control.external_ios.get_parameters().excitation_stop_command.write(false);

        // Speed generator profile commands via datamodel (all false by default, set manually in tests)
        machine_side_control.external_ios.get_parameters().override_speed_generator_profiles.write(false);
        machine_side_control.external_ios.get_parameters().run_creep_speed.write(false);
        machine_side_control.external_ios.get_parameters().run_purge_accel.write(false);
        machine_side_control.external_ios.get_parameters().run_ignition_accel.write(false);
        machine_side_control.external_ios.get_parameters().run_assist_accel.write(false);
        machine_side_control.external_ios.get_parameters().hold_speed.write(false);
        machine_side_control.external_ios.get_parameters().decelerate.write(false);
        machine_side_control.external_ios.get_parameters().idle.write(false);

        // Grid-side inputs via datamodel
        machine_side_control.external_ios.get_parameters().override_fiber_optics_inputs.write(false);
        machine_side_control.external_ios.get_parameters().pulse_releases.write(true);
        machine_side_control.external_ios.get_parameters().current_controller_is_saturated.write(false);
        machine_side_control.external_ios.get_parameters().is_current_zero.write(false);
        machine_side_control.external_ios.get_parameters().force_commutation_acknowledged.write(false);
        machine_side_control.external_ios.get_parameters().dc_link_current.write(current_pu_t{});

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
            grid_side_control.get_external_ios().update_analog_inputs(
                grid_side_hardware_mock.get_outputs().v_ab_counts.read(),
                grid_side_hardware_mock.get_outputs().v_bc_counts.read(),
                grid_side_hardware_mock.get_outputs().i_a_counts.read(),
                grid_side_hardware_mock.get_outputs().i_c_counts.read());

            // Debug: Check what we're reading from hardware mock
            auto debug_ab = machine_side_hardware_mock.get_outputs().v_ab_counts.read();
            auto debug_bc = machine_side_hardware_mock.get_outputs().v_bc_counts.read();

            machine_side_control.external_ios.update_analog_inputs(debug_ab, debug_bc);

            machine_side_control.update_200us();
            grid_side_control.update_200us();
        });

        // 1ms sampling (1 kHz): Current controller, machine controller, operation mode monitor
        simulation.register_lambda_runner(1e-3, [&]() {
            grid_side_control.update_1ms();

            grid_side_control.get_external_ios().update_fiber_optics_outputs_to_machine(
                grid_side_control.get_operating_mode().read(),
                grid_side_control.get_force_commutation_acknowledged().read(),
                grid_side_control.get_is_current_zero().read(),
                grid_side_control.get_current_controller_is_saturated().read(),
                grid_side_control.get_pulse_releases().read(),
                grid_side_control.get_dc_link_measurements().get_outputs().dc_current_pu.read(),
                grid_side_control.get_freeze_integrator().read(),
                grid_side_control.get_grid_side_modulator().get_outputs().is_requesting_main_pi_reset.read(),
                grid_side_control.get_state_machine().get_outputs().current_state_id.read());

            machine_side_control.external_ios.update_fiber_optics_inputs(
                grid_side_control.get_external_ios().get_fiber_optics_outputs_to_machine().operating_mode.read(),
                grid_side_control.get_external_ios().get_fiber_optics_outputs_to_machine().current_state_id.read(),
                grid_side_control.get_external_ios()
                    .get_fiber_optics_outputs_to_machine()
                    .force_commutation_acknowledged.read(),
                grid_side_control.get_external_ios().get_fiber_optics_outputs_to_machine().is_current_zero.read(),
                grid_side_control.get_external_ios()
                    .get_fiber_optics_outputs_to_machine()
                    .current_controller_is_saturated.read(),
                grid_side_control.get_external_ios().get_fiber_optics_outputs_to_machine().pulse_releases.read(),
                grid_side_control.get_external_ios().get_fiber_optics_outputs_to_machine().dc_link_current.read(),
                grid_side_control.get_external_ios().get_fiber_optics_outputs_to_machine().freeze_integrator.read(),
                grid_side_control.get_external_ios()
                    .get_fiber_optics_outputs_to_machine()
                    .is_requesting_main_pi_reset.read());

            machine_side_control.update_1ms();

            grid_side_control.get_external_ios().update_fiber_optics_inputs_from_machine(
                machine_side_control.machine_side_observer.get_outputs().estimated_speed.read(),
                machine_side_control.torque_controller.get_outputs().current_reference.read(),
                machine_side_control.machine_side_observer.get_outputs().estimated_voltage_magnitude.read(),
                machine_side_control.machine_side_modulator.get_outputs().request_force_commutation.read());
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
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage, grid_side_hardware_mock.get_inputs().grid_voltage);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current, grid_side_hardware_mock.get_inputs().grid_current);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage, machine_side_hardware_mock.get_inputs().machine_voltage);

        signals_connector::connect_output_to_input(
            grid_side_control.get_firing_pulses(), plant.get_inputs().grid_side_firing_pulses);

        signals_connector::connect_output_to_input(
            machine_side_control.machine_side_pulse_generator.get_outputs().firing_pulses,
            plant.get_inputs().machine_side_firing_pulses);

        signals_connector::connect_output_to_input(load_torque_pu, plant.get_inputs().load_torque_pu);

        signals_connector::connect_output_to_input(
            machine_side_control.excitation_controller.get_outputs().excitation_voltage_setpoint_pu,
            plant.get_inputs().excitation_voltage_setpoint_pu);
    }
};

}  // namespace yeet::application::sfc::integration
