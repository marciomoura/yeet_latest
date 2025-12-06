// Auto-generated: Sensorless cascaded speed + current control with mocked hardware and ADC measurement chain
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
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/signal.hpp"
#include "yeet/signal_wiring.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// =====================================================================================
//  Sensorless Speed Controller Mocked-Hardware Simulation Builder
//  Plant → Hardware Mocks (CT/PT + transducers + ADC counts) → Measurement Runners →
//  Observers → Speed Controller (outer) → Current Controller (inner) → Pulse Generators → Plant
// =====================================================================================
class plant_sensorless_speed_control_mocked_hardware_simulation_builder {
public:
    static constexpr std::string_view name = "PlantSensorlessSpeedControlMockedHardware";

    plant_sensorless_speed_control_mocked_hardware_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          plant{_per_unit_system},
          grid_side_hardware_mock{duration_t{10e-6}},
          machine_side_hardware_mock{duration_t{10e-6}},
          grid_measurements{_per_unit_system},
          machine_measurements{_per_unit_system},
          dc_link_measurements{_per_unit_system},
          speed_controller(),
          current_controller(_per_unit_system),
          machine_side_controller()
    {
        stage_hardware_and_measurement_parameters();
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Freeze/integrator management defaults
        yeet::wire(current_controller)
            .mocked(false)
            .to(&sfc_current_controller_datamodel::inputs::freeze_integrator)
            .mocked(false)
            .to(&sfc_current_controller_datamodel::inputs::preset_request);
        yeet::wire(speed_controller)
            .mocked(false)
            .to(&sfc_speed_controller_datamodel::inputs::freeze_integrator)
            .mocked(false)
            .to(&sfc_speed_controller_datamodel::inputs::preset_request)
            // Mock applied torque reference for bumpless transfer (no torque controller in this builder)
            .mocked(torque_pu_t{0.0})
            .to(&sfc_speed_controller_datamodel::inputs::applied_torque_reference);

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

        // Always release pulses for open-loop firing generation
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
        speed_controller.initialize();
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

        // Register speed controller with torque-to-current conversion
        simulator.register_lambda_runner(speed_controller.get_sampling_time(), [&]() {
            speed_controller.run();
            // Convert torque to current (1:1 assumption for simplified test without torque controller)
            current_reference_from_speed_ctrl.write(
                current_pu_t{speed_controller.get_outputs().torque_reference.read().value()});
        });

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
        speed_controller.register_datamodel(registry);
        current_controller.register_datamodel(registry);
        machine_side_controller.register_datamodel(registry);
        grid_side_pulse_generator.register_datamodel(registry);
        machine_side_pulse_generator.register_datamodel(registry);
    }

    // Public accessible members (tests can wire / sample signals)
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;
    sfc_speed_controller_runner speed_controller;                          // Outer loop
    sfc_current_controller_runner current_controller;                      // Inner loop (dc-link current)
    sfc_machine_side_controller_runner machine_side_controller;            // Machine-side lookup (firing angle)
    sfc_grid_side_observer_runner grid_side_observer;                      // Grid-side observer
    sfc_machine_side_observer_runner machine_side_observer;                // Machine-side observer
    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;        // Grid firing pulses
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;  // Machine firing pulses
    sfc_grid_side_hardware_mock_runner grid_side_hardware_mock;            // PT/CT + transducers + ADC (grid)
    sfc_machine_side_hardware_mock_runner machine_side_hardware_mock;      // PT + transducers + ADC (machine)
    sfc_grid_side_measurements_runner grid_measurements;                   // Reconstruction & filtering (grid)
    sfc_machine_side_measurements_runner machine_measurements;             // Reconstruction & filtering (machine)
    sfc_dc_link_measurements_runner dc_link_measurements;                  // DC link current reconstruction
    sfc_plant_model_runner plant;                                          // Core plant model

    // External command / disturbance signals
    signal_output<torque_pu_t> load_torque;     // Mechanical load torque
    signal_output<speed_pu_t> speed_reference;  // Speed setpoint (pu)
    signal_output<bool> enable_speed_control;   // Enable outer loop

    // Internal signal for torque-to-current conversion (simplified tests without torque controller)
    signal_output<current_pu_t> current_reference_from_speed_ctrl;

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
        // Hardware mock scaling (PT/CT + transducers + ADC)
        stage_grid_side_hardware_mock_parameters(grid_side_hardware_mock);
        stage_machine_side_hardware_mock_parameters(machine_side_hardware_mock);
        stage_transformer_ratios_for_grid_side(grid_side_hardware_mock, *_per_unit_system);
        stage_transformer_ratios_for_machine_side(machine_side_hardware_mock, *_per_unit_system);

        // Measurement reconstruction scaling (inverse transducer + transformer ratios)
        stage_grid_side_measurement_parameters(grid_measurements);
        stage_machine_side_measurement_parameters(machine_measurements);
        stage_transformer_ratios_for_grid_side_measurements(grid_measurements, *_per_unit_system);
        stage_transformer_ratios_for_machine_side_measurements(machine_measurements, *_per_unit_system);
    }

    void connect_all_components()
    {
        // ==================== Plant -> Hardware Mocks ====================
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_abc, grid_side_hardware_mock.get_inputs().grid_voltage_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current_abc, grid_side_hardware_mock.get_inputs().grid_current_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage_abc, machine_side_hardware_mock.get_inputs().machine_voltage_abc);

        // ==================== Hardware Mocks -> Measurement Runners (ADC counts) ====================
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

        // ==================== Pulses / Sync into Measurement Runners ====================
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

        // ==================== Measurements -> DC Link ====================
        signals_connector::connect_output_to_input(
            grid_measurements.get_outputs().grid_current_abc, dc_link_measurements.get_inputs().grid_current_abc);

        // ==================== Measurements -> Observers (voltage abc) ====================
        signals_connector::connect_output_to_input(
            grid_measurements.get_outputs().grid_voltage_abc, grid_side_observer.get_inputs().grid_voltage_abc);
        signals_connector::connect_output_to_input(machine_measurements.get_outputs().machine_voltage_abc,
            machine_side_observer.get_inputs().machine_voltage_pu);

        // ==================== Observer Estimates -> Speed Controller (feedback) ====================
        signals_connector::connect_output_to_input(
            machine_side_observer.get_outputs().estimated_speed, speed_controller.get_inputs().measured_speed);

        // ==================== Speed Controller -> Current Controller (cascade) ====================
        // NOTE: In full SFC system, torque controller sits between speed and current controllers
        // Here we use 1:1 torque-to-current assumption for simplified testing (via intermediate signal)
        signals_connector::connect_output_to_input(
            current_reference_from_speed_ctrl, current_controller.get_inputs().current_reference);

        // ==================== DC Link Measurement -> Current Controller (feedback) ====================
        signals_connector::connect_output_to_input(dc_link_measurements.get_outputs().dc_current_filtered_pu,
            current_controller.get_inputs().current_measured);

        // ==================== Observer Voltage Magnitudes -> Current Controller (feedforward) ====================
        signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_voltage_magnitude,
            current_controller.get_inputs().grid_voltage_magnitude);
        signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_voltage_magnitude,
            current_controller.get_inputs().machine_voltage_magnitude);

        // ==================== Cascade Current Ref -> Machine Side Controller (substitute for dc link current)
        // ====================
        // Use torque reference as current reference (1:1 torque-to-current assumption)
        signals_connector::connect_output_to_input(
            current_reference_from_speed_ctrl, machine_side_controller.get_inputs().dc_link_current);
        signals_connector::connect_output_to_input(
            machine_side_observer.get_outputs().estimated_speed, machine_side_controller.get_inputs().machine_speed);

        // ==================== Controller Outputs -> Pulse Generators ====================
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);
        signals_connector::connect_output_to_input(
            machine_side_controller.get_outputs().firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);

        // ==================== Observer Position -> Pulse Generators (sensorless sync) ====================
        signals_connector::connect_output_to_input(grid_side_observer.get_outputs().estimated_reference_angle,
            grid_side_pulse_generator.get_inputs().reference_angle);
        signals_connector::connect_output_to_input(machine_side_observer.get_outputs().estimated_reference_angle,
            machine_side_pulse_generator.get_inputs().reference_angle);

        // ==================== Pulse Generators -> Plant & Load Torque ====================

        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().grid_firing_pulses);
        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().machine_firing_pulses);

        // Field current reference for variable-speed voltage control
        signals_connector::connect_output_to_input(
            field_current_reference_pu, plant.get_inputs().field_current_reference);

        // ==================== External Commands -> Speed Controller ====================
        signals_connector::connect_output_to_input(speed_reference, speed_controller.get_inputs().speed_reference);
        signals_connector::connect_output_to_input(enable_speed_control, speed_controller.get_inputs().release_pulses);

        // ==================== Current Controller Saturation -> Speed Controller (anti-windup) ====================
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().is_saturated, speed_controller.get_inputs().is_saturated);
    }
};

}  // namespace yeet::application::sfc::integration
