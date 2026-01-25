#pragma once

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_excitation_controller_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_observer_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_observer_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "yeet/signal_wiring.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Sensorless Speed Controller Simulation Builder ====================

/**
 * @brief Sensorless speed controller simulation builder with cascaded control and observers
 *
 * This configuration provides:
 * - Plant model with augmented 6p-6p topology
 * - Grid-side and machine-side observers for sensorless estimation
 * - Cascaded speed and current controller (speed outer loop, current inner loop)
 * - Grid-side current controller with firing angle control using estimated values
 * - Machine-side lookup table controller using estimated speed
 * - Complete measurement and filtering chain (raw measurements for observers)
 * - Observer-based feedback instead of direct measurements
 * - Per-unit system for proper scaling
 * - Simplified configuration without state machine, modulators, or operation mode monitor
 *
 * Signal Flow:
 * Plant Ã¢â€ â€™ Raw Measurements Ã¢â€ â€™ Observers Ã¢â€ â€™ Estimated Values Ã¢â€ â€™ Speed Controller Ã¢â€ â€™
 * Current Controller Ã¢â€ â€™ Pulse Generators Ã¢â€ â€™ Plant
 */
class plant_sensorless_speed_control_simulation_builder {
public:
    static constexpr std::string_view name = "PlantSensorlessSpeedControl";

    plant_sensorless_speed_control_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          plant{_per_unit_system},
          grid_measurements{_per_unit_system},
          machine_measurements(_per_unit_system),
          dc_link_measurements{_per_unit_system},
          current_controller(_per_unit_system),
          excitation_controller()
    {
        // Connect all components with observer-based cascaded feedback
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Mock control signals for standalone testing
        yeet::wire(current_controller)
            .mocked(_mock_storage_false)
            .to(&sfc_current_controller_datamodel::inputs::freeze_integrator)
            .mocked(_mock_storage_false)
            .to(&sfc_current_controller_datamodel::inputs::preset_request);
        yeet::wire(speed_controller)
            .mocked(_mock_storage_false)
            .to(&sfc_speed_controller_datamodel::inputs::freeze_integrator)
            .mocked(_mock_storage_false)
            .to(&sfc_speed_controller_datamodel::inputs::preset_request)
            // Mock applied torque reference for bumpless transfer (no torque controller in this builder)
            .mocked(_mock_storage_torque_zero)
            .to(&sfc_speed_controller_datamodel::inputs::applied_torque_reference);

        // Mock machine-side observer signals (no startup sequencer in this builder)
        yeet::wire(machine_side_observer)
            .mocked(_mock_storage_true)
            .to(&sfc_machine_side_observer_datamodel::inputs::release_observer)
            .mocked(_mock_storage_false)
            .to(&sfc_machine_side_observer_datamodel::inputs::preset_observer)
            .mocked(_mock_storage_angle_zero)
            .to(&sfc_machine_side_observer_datamodel::inputs::preset_angle)
            .mocked(_mock_storage_freq_zero)
            .to(&sfc_machine_side_observer_datamodel::inputs::preset_frequency);

        // Mock modulator signals
        yeet::wire(grid_side_modulator)
            .mocked(_mock_storage_true)
            .to(&sfc_grid_side_modulator_datamodel::inputs::release_pulses)
            .mocked(_mock_operating_mode_natural)
            .to(&sfc_grid_side_modulator_datamodel::inputs::operating_mode)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::request_extinguish_dc_current)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::current_is_zero)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::shutdown_request)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::protection_trip)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::undervoltage_request_extinguish_dc_current);

        yeet::wire(machine_side_modulator)
            .mocked(_mock_storage_true)
            .to(&sfc_machine_side_modulator_datamodel::inputs::release_pulses)
            .mocked(_mock_operating_mode_natural)
            .to(&sfc_machine_side_modulator_datamodel::inputs::operating_mode)
            .mocked(_mock_reference_frame_estimated)
            .to(&sfc_machine_side_modulator_datamodel::inputs::reference_frame_source)
            .mocked(_mock_rotor_angle)
            .to(&sfc_machine_side_modulator_datamodel::inputs::virtual_rotor_angle);

        // Excitation command (no startup_sequencer in this builder)
        yeet::wire(excitation_controller)
            .mocked(_mock_storage_true)
            .to(&sfc_excitation_controller_datamodel::inputs::release_excitation);
    }
    void initialize()
    {
        // Configure plant for variable-speed operation and voltage control (excitation controller sets field current)
        plant.get_parameters().operation_mode.write(sfc_plant_model_datamodel::machine_operation_mode::variable_speed);
        plant.get_parameters().voltage_control_mode.write(
            sfc_plant_model_datamodel::machine_voltage_control_mode::variable_speed);

        // Initialize components in dependency order
        plant.initialize();
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();
        excitation_controller.initialize();
        grid_side_observer.initialize();
        machine_side_observer.initialize();
        speed_controller.initialize();
        current_controller.initialize();
        machine_side_controller.initialize();
        grid_side_modulator.initialize();
        machine_side_modulator.initialize();
    }

    void register_applications(yeet::simulator& simulator)
    {
        // Register all runners directly (sampling time encapsulated in each runner)
        simulator.register_runner(plant);
        simulator.register_runner(grid_measurements);
        simulator.register_runner(machine_measurements);
        simulator.register_runner(dc_link_measurements);
        simulator.register_runner(grid_side_observer);
        simulator.register_runner(machine_side_observer);

        // Register speed controller with excitation and torque-to-current conversion
        simulator.register_lambda_runner(speed_controller.get_sampling_time(), [&]() {
            excitation_controller.run();
            speed_controller.run();
            // Convert torque to current (1:1 assumption for simplified test without torque controller)
            current_reference_from_speed_ctrl.write(
                current_pu_t{speed_controller.get_outputs().torque_reference.read().value()});
        });

        simulator.register_runner(current_controller);
        simulator.register_runner(machine_side_controller);
        simulator.register_runner(grid_side_modulator);
        simulator.register_runner(machine_side_modulator);
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control components (cascaded architecture)
    sfc_excitation_controller_runner excitation_controller;
    sfc_speed_controller_runner speed_controller;      // Outer loop
    sfc_current_controller_runner current_controller;  // Inner loop
    sfc_machine_side_controller_runner machine_side_controller;

    // Observer components (key addition for sensorless operation)
    sfc_grid_side_observer_runner grid_side_observer;
    sfc_machine_side_observer_runner machine_side_observer;

    // Modulator components
    sfc_grid_side_modulator_runner grid_side_modulator;
    sfc_machine_side_modulator_runner machine_side_modulator;

    // Measurements and filtering (raw measurements for observers)
    sfc_grid_side_measurements_ideal_runner grid_measurements;
    sfc_machine_side_measurements_ideal_runner machine_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;

    // Plant model
    sfc_plant_model_runner plant;

    // External control signals
    signal_output<torque_pu_t> load_torque;
    signal_output<speed_pu_t> speed_reference;
    signal_output<bool> enable_speed_control;
    signal_output<bool> voltage_dip{false};
    signal_output<current_pu_t> external_current_reference_pu;

    // Internal signal for torque-to-current conversion (simplified tests without torque controller)
    signal_output<current_pu_t> current_reference_from_speed_ctrl;

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
        // Get datamodel references
        auto& plant_model = plant.get_signal_flow_datamodel();
        auto& grid_meas_model = grid_measurements.get_signal_flow_datamodel();
        auto& machine_meas_model = machine_measurements.get_signal_flow_datamodel();
        auto& dc_link_model = dc_link_measurements.get_signal_flow_datamodel();
        auto& grid_observer_model = grid_side_observer.get_signal_flow_datamodel();
        auto& machine_observer_model = machine_side_observer.get_signal_flow_datamodel();
        auto& speed_ctrl_model = speed_controller.get_signal_flow_datamodel();
        auto& current_ctrl_model = current_controller.get_signal_flow_datamodel();
        auto& machine_ctrl_model = machine_side_controller.get_signal_flow_datamodel();
        auto& grid_modulator_model = grid_side_modulator.get_signal_flow_datamodel();
        auto& machine_modulator_model = machine_side_modulator.get_signal_flow_datamodel();

        // ==================== Plant Model Ã¢â€ â€™ Raw Measurements ====================
        // Connect plant outputs to measurement components (as in simplified current control)

        // Grid measurements: Raw grid voltages and currents from plant
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().grid_voltage_abc, grid_meas_model.get_inputs().grid_voltage_abc);
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().grid_current_abc, grid_meas_model.get_inputs().grid_current_abc);

        signals_connector::connect_output_to_input(
            grid_modulator_model.get_outputs().firing_pulses, grid_meas_model.get_inputs().grid_pulses);

        signals_connector::connect_output_to_input(
            plant_model.get_outputs().grid_voltage_source_angle, grid_meas_model.get_inputs().grid_angle);
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().grid_frequency, grid_meas_model.get_inputs().grid_frequency);

        // Machine measurements: Raw machine voltages and currents from plant
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().machine_voltage_abc, machine_meas_model.get_inputs().machine_voltage_abc);
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().machine_current, machine_meas_model.get_inputs().machine_current_abc);

        signals_connector::connect_output_to_input(
            machine_modulator_model.get_outputs().firing_pulses, machine_meas_model.get_inputs().machine_pulses);

        signals_connector::connect_output_to_input(plant_model.get_outputs().machine_voltage_source_angle,
            machine_meas_model.get_inputs().machine_reference_angle);
        signals_connector::connect_output_to_input(
            plant_model.get_outputs().machine_frequency, machine_meas_model.get_inputs().machine_frequency);

        // DC link measurements: Raw DC voltage and current from plant
        signals_connector::connect_output_to_input(
            grid_measurements.get_outputs().grid_current_abc, dc_link_model.get_inputs().grid_current_abc);

        // ==================== Measurements Ã¢â€ â€™ Excitation Controller ====================
        // Provide speed and voltage magnitude feedback for excitation voltage regulation

        signals_connector::connect_output_to_input(machine_observer_model.get_outputs().estimated_speed,
            excitation_controller.get_inputs().machine_rotor_speed_pu);
        signals_connector::connect_output_to_input(machine_meas_model.get_outputs().machine_voltage_magnitude_filtered,
            excitation_controller.get_inputs().machine_voltage_magnitude_pu);

        // ==================== Raw Measurements Ã¢â€ â€™ Observers ====================
        // Connect filtered measurements to observers for estimation

        // Grid side observer inputs: filtered grid voltage and firing pulses
        signals_connector::connect_output_to_input(
            grid_meas_model.get_outputs().grid_voltage_abc, grid_observer_model.get_inputs().grid_voltage_abc);

        // Machine side observer inputs: filtered machine voltage and firing pulses
        signals_connector::connect_output_to_input(machine_meas_model.get_outputs().machine_voltage_abc,
            machine_observer_model.get_inputs().machine_voltage_pu);

        // ==================== Machine Observer Ã¢â€ â€™ Speed Controller ====================
        // Connect machine observer estimated speed to speed controller feedback (key sensorless connection)

        // Speed feedback from machine observer instead of direct measurement
        signals_connector::connect_output_to_input(
            machine_observer_model.get_outputs().estimated_speed, speed_ctrl_model.get_inputs().measured_speed);

        // ==================== Speed Controller Ã¢â€ â€™ Current Controller ====================
        // Connect speed controller output to current controller reference (cascade control)

        // Current reference from speed controller to current controller (1:1 torque-to-current assumption)
        // NOTE: In full SFC system, torque controller sits between speed and current controllers
        // Using intermediate signal for type conversion (torque_pu_t Ã¢â€ â€™ current_pu_t)
        signals_connector::connect_output_to_input(
            current_reference_from_speed_ctrl, current_ctrl_model.get_inputs().current_reference);

        // ==================== DC Link Measurements Ã¢â€ â€™ Current Controller ====================
        // Connect DC link current feedback to current controller

        // DC link current feedback (still use direct measurement as it's most reliable)
        signals_connector::connect_output_to_input(
            dc_link_model.get_outputs().dc_current_filtered_pu, current_ctrl_model.get_inputs().current_measured);

        // ==================== Observers Ã¢â€ â€™ Current Controller (Feedforward) ====================
        // Connect observer-estimated values to current controller feedforward inputs

        // Feedforward signals from observers instead of direct measurements
        signals_connector::connect_output_to_input(grid_observer_model.get_outputs().estimated_voltage_magnitude,
            current_ctrl_model.get_inputs().grid_voltage_magnitude);
        signals_connector::connect_output_to_input(machine_observer_model.get_outputs().estimated_voltage_magnitude,
            current_ctrl_model.get_inputs().machine_voltage_magnitude);

        // ==================== Observers Ã¢â€ â€™ Machine Side Controller ====================
        // Connect observer-estimated values to machine side controller for lookup table

        // DC link current for lookup table input (use torque reference, 1:1 assumption)
        // signals_connector::connect_output_to_input(
        // dc_link_model.get_outputs().dc_current_filtered_pu, machine_ctrl_model.get_inputs().dc_link_current);
        signals_connector::connect_output_to_input(
            current_reference_from_speed_ctrl, machine_ctrl_model.get_inputs().dc_link_current);

        // Machine speed from observer instead of direct measurement
        signals_connector::connect_output_to_input(
            machine_observer_model.get_outputs().estimated_speed, machine_ctrl_model.get_inputs().machine_speed);

        // ==================== Current Controller Ã¢â€ â€™ Grid Modulator ====================
        // Connect current controller output to grid-side modulator

        // Firing angle command from current controller to grid modulator
        signals_connector::connect_output_to_input(
            current_ctrl_model.get_outputs().firing_angle, grid_modulator_model.get_inputs().firing_angle);

        // Grid modulator requesting PI reset/freeze during commutation
        signals_connector::connect_output_to_input(
            grid_modulator_model.get_outputs().freeze_integrator, current_ctrl_model.get_inputs().freeze_integrator);

        // ==================== Machine Side Controller Ã¢â€ â€™ Machine Modulator ====================
        // Connect machine side controller output to machine-side modulator

        // Firing angle command from machine side controller to machine modulator
        signals_connector::connect_output_to_input(
            machine_ctrl_model.get_outputs().firing_angle, machine_modulator_model.get_inputs().firing_angle);

        // ==================== Handshake ====================
        signals_connector::connect_output_to_input(
            voltage_dip, grid_modulator_model.get_inputs().undervoltage_request_extinguish_dc_current);
        signals_connector::connect_output_to_input(machine_modulator_model.get_outputs().request_force_commutation,
            grid_modulator_model.get_inputs().request_force_commutation);
        signals_connector::connect_output_to_input(grid_modulator_model.get_outputs().force_commutation_acknowledged,
            machine_modulator_model.get_inputs().force_commutation_acknowledged);

        // ==================== Modulators Ã¢â€ â€™ Plant Model ====================
        // Connect firing pulses from both modulators to plant model inputs
        // Note: load_torque is now configured via plant_model.get_parameters().constant_torque parameter

        signals_connector::connect_output_to_input(
            grid_modulator_model.get_outputs().firing_pulses, plant_model.get_inputs().grid_firing_pulses);

        signals_connector::connect_output_to_input(
            machine_modulator_model.get_outputs().firing_pulses, plant_model.get_inputs().machine_firing_pulses);

        // Field current reference for variable-speed voltage control from excitation controller
        signals_connector::connect_output_to_input(excitation_controller.get_outputs().field_current_reference,
            plant_model.get_inputs().field_current_reference);

        // ==================== Observer Estimates Ã¢â€ â€™ Modulators ====================
        // For sensorless operation, use observer-estimated angles instead of direct plant angles

        // Grid side: Use observer-estimated angle for modulator synchronization
        signals_connector::connect_output_to_input(grid_observer_model.get_outputs().estimated_reference_angle,
            grid_modulator_model.get_inputs().grid_reference_angle);

        // Machine side: Use observer-estimated position for modulator synchronization
        signals_connector::connect_output_to_input(machine_observer_model.get_outputs().estimated_reference_angle,
            machine_modulator_model.get_inputs().machine_reference_angle);

        // ==================== External Control Signals ====================
        // Connect external control signals to speed controller inputs

        // Speed reference command (setpoint for outer loop)
        signals_connector::connect_output_to_input(speed_reference, speed_ctrl_model.get_inputs().speed_reference);

        // Enable signal for speed control
        signals_connector::connect_output_to_input(enable_speed_control, speed_ctrl_model.get_inputs().release_pulses);

        signals_connector::connect_output_to_input(
            current_ctrl_model.get_outputs().is_saturated, speed_ctrl_model.get_inputs().is_saturated);

        // Optional external current reference override (for manual current control mode)
        // Note: This would typically be switched in via a mode selector in a real application
        // For now, we just provide the signal output for potential use
    }

private:
    bool _mock_storage_true{true};
    bool _mock_storage_false{false};
    angle_wrapped _mock_rotor_angle{0.0f};
    sfc_operating_mode_t _mock_operating_mode_natural{sfc_operating_mode_t::natural_commutation};
    reference_frame_source_t _mock_reference_frame_estimated{reference_frame_source_t::ESTIMATED};
    angle_wrapped _mock_storage_angle_zero{0.0};
    frequency_pu_t _mock_storage_freq_zero{0.0};
    torque_pu_t _mock_storage_torque_zero{0.0};
};

}  // namespace yeet::application::sfc::integration
