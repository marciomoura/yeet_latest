// Auto-generated: Sensorless current controller with mocked hardware and ADC measurement chain
#pragma once

#include <memory>

#include "sfc_integration/plant_hardware_definition.hpp"
#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_hardware_mock_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_hardware_mock_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/signal.hpp"
#include "yeet/signal_wiring.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

class plant_sensorless_current_control_mocked_hardware_simulation_builder {
public:
    static constexpr std::string_view name = "PlantSensorlessCurrentControlMockedHardware";

    plant_sensorless_current_control_mocked_hardware_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          plant{_per_unit_system},
          grid_side_hardware_mock{duration_t{10e-6}},
          machine_side_hardware_mock{duration_t{10e-6}},
          grid_measurements{_per_unit_system},
          machine_measurements{_per_unit_system},
          dc_link_measurements{_per_unit_system},
          current_controller(_per_unit_system),
          machine_side_controller()
    {
        stage_hardware_and_measurement_parameters();
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        yeet::wire(current_controller)
            .mocked(false)
            .to(&sfc_current_controller_datamodel::inputs::freeze_integrator)
            .mocked(false)
            .to(&sfc_current_controller_datamodel::inputs::preset_request);

        // Mock machine-side observer signals (no startup sequencer in this builder)
        yeet::wire(machine_side_observer)
            .mocked(true)
            .to(&sfc_machine_side_observer_datamodel::inputs::release_observer)
            .mocked(false)
            .to(&sfc_machine_side_observer_datamodel::inputs::preset_observer)
            .mocked(angle_wrapped{0.0})
            .to(&sfc_machine_side_observer_datamodel::inputs::preset_angle)
            .mocked(frequency_pu_t{0.0})
            .to(&sfc_machine_side_observer_datamodel::inputs::preset_frequency);

        yeet::wire(grid_side_pulse_generator)
            .mocked(true)
            .to(&sfc_grid_side_pulse_generator_datamodel::inputs::release_pulses);
        yeet::wire(machine_side_pulse_generator)
            .mocked(true)
            .to(&sfc_machine_side_pulse_generator_datamodel::inputs::release_pulses);
    }

    void initialize()
    {
        plant.initialize();
        grid_side_hardware_mock.initialize();
        machine_side_hardware_mock.initialize();
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
        simulator.register_runner(plant);
        simulator.register_runner(grid_side_hardware_mock);
        simulator.register_runner(machine_side_hardware_mock);
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
        plant.register_datamodel(registry);
        grid_side_hardware_mock.register_datamodel(registry);
        machine_side_hardware_mock.register_datamodel(registry);
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

    // Public members
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;
    sfc_current_controller_runner current_controller;
    sfc_machine_side_controller_runner machine_side_controller;
    sfc_grid_side_observer_runner grid_side_observer;
    sfc_machine_side_observer_runner machine_side_observer;
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;
    sfc_grid_side_hardware_mock_runner grid_side_hardware_mock;
    sfc_machine_side_hardware_mock_runner machine_side_hardware_mock;
    sfc_grid_side_measurements_runner grid_measurements;
    sfc_machine_side_measurements_runner machine_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;
    sfc_plant_model_runner plant;
    signal_output<torque_pu_t> load_torque;
    signal_output<current_pu_t> current_reference;

    // Field current reference for variable-speed operation (no excitation controller in this builder)
    signal_output<current_pu_t> field_current_reference_pu{current_pu_t{0.8f}};

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
        // ==================== Plant -> hardware mocks ====================
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_abc, grid_side_hardware_mock.get_inputs().grid_voltage_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current_abc, grid_side_hardware_mock.get_inputs().grid_current_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage_abc, machine_side_hardware_mock.get_inputs().machine_voltage_abc);

        // ==================== Hardware mocks -> measurements (ADC counts) ====================
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

        // ==================== Pulses / sync into measurement runners ====================
        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, grid_measurements.get_inputs().grid_pulses);
        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, machine_measurements.get_inputs().machine_pulses);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_measurements.get_inputs().grid_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_frequency, grid_measurements.get_inputs().grid_frequency);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_measurements.get_inputs().machine_reference_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_frequency, machine_measurements.get_inputs().machine_frequency);

        // ==================== Measurements -> DC link measurements ====================
        signals_connector::connect_output_to_input(
            grid_measurements.get_outputs().grid_current_abc, dc_link_measurements.get_inputs().grid_current_abc);

        // ==================== Measurements -> observers (logging only) ====================
        signals_connector::connect_output_to_input(
            grid_measurements.get_outputs().grid_voltage_abc, grid_side_observer.get_inputs().grid_voltage_abc);
        signals_connector::connect_output_to_input(machine_measurements.get_outputs().machine_voltage_abc,
            machine_side_observer.get_inputs().machine_voltage_pu);

        // ====================
        // Sensorless feed-forward: use observer estimated voltage magnitudes (replaces filtered measurement magnitudes)
        signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_voltage_magnitude,
            current_controller.get_inputs().grid_voltage_magnitude);
        signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_voltage_magnitude,
            current_controller.get_inputs().machine_voltage_magnitude);
        // Use dc link measurement filtered current (already scaled) instead of observer estimate
        signals_connector::connect_output_to_input(
            dc_link_measurements.get_outputs().dc_current_pu, current_controller.get_inputs().current_measured);
        signals_connector::connect_output_to_input(dc_link_measurements.get_outputs().dc_current_heavy_filtered_pu,
            machine_side_controller.get_inputs().dc_link_current);
        // Machine speed from observer (sensorless)
        signals_connector::connect_output_to_input(
            machine_side_observer.get_outputs().estimated_speed, machine_side_controller.get_inputs().machine_speed);

        // ==================== Controller outputs -> pulse generators ====================
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);
        signals_connector::connect_output_to_input(
            machine_side_controller.get_outputs().firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);

        // ==================== Reference angles (observer estimated electrical position) ====================
        signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_reference_angle,
            grid_side_pulse_generator.get_inputs().reference_angle);
        signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_reference_angle,
            machine_side_pulse_generator.get_inputs().reference_angle);

        // ==================== Pulse generators -> plant & load torque ====================

        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().grid_firing_pulses);
        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().machine_firing_pulses);

        // Field current reference for variable-speed voltage control
        signals_connector::connect_output_to_input(
            field_current_reference_pu, plant.get_inputs().field_current_reference);

        // ==================== External setpoint -> current controller ====================
        signals_connector::connect_output_to_input(
            current_reference, current_controller.get_inputs().current_reference);
    }
};

}  // namespace yeet::application::sfc::integration
