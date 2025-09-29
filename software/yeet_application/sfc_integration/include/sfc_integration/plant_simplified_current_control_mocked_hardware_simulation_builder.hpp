#pragma once

#include "sfc_grid_side_hardware_mock_runner.hpp"
#include "sfc_integration/plant_hardware_definition.hpp"
#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_machine_side_hardware_mock_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

class plant_simplified_current_control_mocked_hardware_simulation_builder {
public:
    static constexpr std::string_view name = "PlantSimplifiedCurrentControlMockedHardware";

    plant_simplified_current_control_mocked_hardware_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          plant{_per_unit_system},
          grid_side_hardware_mock{duration_t{10e-6}},
          machine_side_hardware_mock{duration_t{10e-6}},
          grid_measurements{_per_unit_system},
          machine_measurements{_per_unit_system},
          dc_link_measurements{_per_unit_system},
          current_controller(_per_unit_system)
    {
        stage_hardware_and_measurement_parameters();
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        yeet::mock_signal_input::write(current_controller.get_inputs().freeze_integrator, false);
        yeet::mock_signal_input::write(current_controller.get_inputs().modulator_freeze_integrator, false);
        yeet::mock_signal_input::write(grid_side_pulse_generator.get_inputs().release_pulses, true);
        yeet::mock_signal_input::write(machine_side_pulse_generator.get_inputs().release_pulses, true);
    }

    void initialize()
    {
        plant.initialize();
        grid_side_hardware_mock.initialize();
        machine_side_hardware_mock.initialize();
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();
        current_controller.initialize();
        machine_side_controller.initialize();
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();
    }

    void register_applications(yeet::simulator& sim)
    {
        sim.register_runner(plant);
        sim.register_runner(grid_side_hardware_mock);
        sim.register_runner(machine_side_hardware_mock);
        sim.register_runner(grid_measurements);
        sim.register_runner(machine_measurements);
        sim.register_runner(dc_link_measurements);
        sim.register_runner(current_controller);
        sim.register_runner(machine_side_controller);
        sim.register_runner(grid_side_pulse_generator);
        sim.register_runner(machine_side_pulse_generator);
    }

    void register_datamodel(yeet::model_registry& registry)
    {
        plant.register_datamodel(registry);
        grid_side_hardware_mock.register_datamodel(registry);
        machine_side_hardware_mock.register_datamodel(registry);
        grid_measurements.register_datamodel(registry);
        machine_measurements.register_datamodel(registry);
        dc_link_measurements.register_datamodel(registry);
        current_controller.register_datamodel(registry);
        machine_side_controller.register_datamodel(registry);
        grid_side_pulse_generator.register_datamodel(registry);
        machine_side_pulse_generator.register_datamodel(registry);
    }

    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Controllers
    sfc_current_controller_runner current_controller;
    sfc_machine_side_controller_runner machine_side_controller;

    // Pulse generators
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;

    // Hardware mocks
    sfc_grid_side_hardware_mock_runner grid_side_hardware_mock;
    sfc_machine_side_hardware_mock_runner machine_side_hardware_mock;

    // Measurement runners
    sfc_grid_side_measurements_runner grid_measurements;
    sfc_machine_side_measurements_runner machine_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;

    // Plant
    sfc_plant_model_runner plant;

    // External signals
    signal_output<torque_pu_t> load_torque_pu;
    signal_output<current_pu_t> dc_link_current_reference;

private:
    static std::shared_ptr<sfc::sfc_per_unit_system> create_per_unit_system()
    {
        return std::make_shared<sfc::sfc_per_unit_system>(voltage_t(2090.0), frequency_t(50.0), current_t(1044.0),
            power_t(2950e3), voltage_t(1980.0), frequency_t(50.0), 2);
    }

    void stage_hardware_and_measurement_parameters()
    {
        using namespace plant_hardware;
        stage_grid_side_hardware_mock_parameters(grid_side_hardware_mock);
        stage_machine_side_hardware_mock_parameters(machine_side_hardware_mock);
        stage_transformer_ratios_for_grid_side(grid_side_hardware_mock, *_per_unit_system);
        stage_transformer_ratios_for_machine_side(machine_side_hardware_mock, *_per_unit_system);
        stage_grid_side_measurement_parameters(grid_measurements);
        stage_machine_side_measurement_parameters(machine_measurements);
        stage_transformer_ratios_for_grid_side_measurements(grid_measurements, *_per_unit_system);
        stage_transformer_ratios_for_machine_side_measurements(machine_measurements, *_per_unit_system);
    }

    void connect_all_components()
    {
        // Plant → hardware mocks
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage, grid_side_hardware_mock.get_inputs().grid_voltage);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current, grid_side_hardware_mock.get_inputs().grid_current);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage, machine_side_hardware_mock.get_inputs().machine_voltage);

        // Hardware mocks → measurements (counts)
        signals_connector::connect_output_to_input(
            grid_side_hardware_mock.get_outputs().v_ab_counts, grid_measurements.get_inputs().v_ab_counts);
        signals_connector::connect_output_to_input(
            grid_side_hardware_mock.get_outputs().v_bc_counts, grid_measurements.get_inputs().v_bc_counts);
        signals_connector::connect_output_to_input(
            grid_side_hardware_mock.get_outputs().i_a_counts, grid_measurements.get_inputs().i_a_counts);
        signals_connector::connect_output_to_input(
            grid_side_hardware_mock.get_outputs().i_c_counts, grid_measurements.get_inputs().i_c_counts);

        signals_connector::connect_output_to_input(
            machine_side_hardware_mock.get_outputs().v_ab_counts, machine_measurements.get_inputs().v_ab_counts);
        signals_connector::connect_output_to_input(
            machine_side_hardware_mock.get_outputs().v_bc_counts, machine_measurements.get_inputs().v_bc_counts);

        // Pulses / synchronization
        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, grid_measurements.get_inputs().grid_pulses);
        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, machine_measurements.get_inputs().machine_pulses);

        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_measurements.get_inputs().grid_side_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_frequency, grid_measurements.get_inputs().grid_side_frequency);

        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage_source_angle, machine_measurements.get_inputs().machine_side_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_frequency, machine_measurements.get_inputs().machine_side_frequency);

        // Measurements → DC link measurements
        signals_connector::connect_output_to_input(
            grid_measurements.get_outputs().grid_current_abc, dc_link_measurements.get_inputs().grid_side_current);

        // DC link → controllers
        signals_connector::connect_output_to_input(
            dc_link_measurements.get_outputs().dc_current_pu, current_controller.get_inputs().dc_link_current_measured);
        signals_connector::connect_output_to_input(dc_link_measurements.get_outputs().dc_current_heavy_filtered_pu,
            machine_side_controller.get_inputs().dc_link_current);

        // Machine speed
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_speed, machine_side_controller.get_inputs().machine_speed);

        // Feedforward voltages
        signals_connector::connect_output_to_input(grid_measurements.get_outputs().grid_voltage_magnitude_filtered,
            current_controller.get_inputs().grid_side_voltage_magnitude);
        signals_connector::connect_output_to_input(
            machine_measurements.get_outputs().machine_voltage_magnitude_filtered,
            current_controller.get_inputs().machine_side_voltage_magnitude);

        // Controller outputs → pulse generators
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);
        signals_connector::connect_output_to_input(
            machine_side_controller.get_outputs().firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);

        // Pulse generators → plant
        signals_connector::connect_output_to_input(load_torque_pu, plant.get_inputs().load_torque_pu);
        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().grid_side_firing_pulses);
        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().machine_side_firing_pulses);

        // Plant → pulse generators (reference angles)
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_side_pulse_generator.get_inputs().reference_angle);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_side_pulse_generator.get_inputs().reference_angle);

        // External setpoint
        signals_connector::connect_output_to_input(
            dc_link_current_reference, current_controller.get_inputs().dc_link_current_reference);
    }
};

}  // namespace yeet::application::sfc::integration
