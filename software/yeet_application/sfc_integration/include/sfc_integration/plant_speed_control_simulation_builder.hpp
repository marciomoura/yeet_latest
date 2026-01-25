#pragma once

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_excitation_controller_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "sfc_integration/sfc_speed_controller_runner.hpp"
#include "sfc_integration/sfc_torque_controller_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/signal_wiring.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Speed Controller Simulation Builder ====================

/**
 * @brief Speed controller simulation builder with cascaded speed, torque, and current control
 *
 * This configuration provides:
 * - Plant model with augmented 6p-6p topology
 * - Cascaded control: speed controller Ã¢â€ â€™ torque controller Ã¢â€ â€™ current controller
 * - Grid-side current controller with firing angle control
 * - Machine-side lookup table controller for firing angle calculation
 * - Complete measurement and filtering chain
 * - Per-unit system for proper scaling
 * - Simplified configuration without observers, state machine, or modulators
 *
 * Signal Flow:
 * Plant Ã¢â€ â€™ Raw Measurements Ã¢â€ â€™ Speed Controller Ã¢â€ â€™ Torque Controller Ã¢â€ â€™ Current Controller
 * Ã¢â€ â€™ Pulse Generators Ã¢â€ â€™ Plant
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
        yeet::wire(current_controller)
            .mocked(_mock_storage_false)
            .to(&sfc_current_controller_datamodel::inputs::freeze_integrator)
            .mocked(_mock_storage_false)
            .to(&sfc_current_controller_datamodel::inputs::preset_request);
        yeet::wire(speed_controller)
            .mocked(_mock_storage_false)
            .to(&sfc_speed_controller_datamodel::inputs::freeze_integrator)
            .mocked(_mock_storage_false)
            .to(&sfc_speed_controller_datamodel::inputs::preset_request);

        // Torque controller mocks (no speed_generator or startup_sequencer in this builder)
        yeet::wire(torque_controller)
            .mocked(_mock_storage_false)
            .to(&sfc_torque_controller_datamodel::inputs::enable_breakaway_boost)
            .mocked(_mock_storage_control_mode_speed)
            .to(&sfc_torque_controller_datamodel::inputs::control_mode)
            .mocked(_mock_storage_torque_zero)
            .to(&sfc_torque_controller_datamodel::inputs::manual_torque_reference);

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
        speed_controller.initialize();
        torque_controller.initialize();
        current_controller.initialize();
        machine_side_controller.initialize();
        grid_side_modulator.initialize();
        machine_side_modulator.initialize();
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
            excitation_controller.run();
            speed_controller.run();
            torque_controller.run();
        });
        simulator.register_lambda_runner(current_controller.get_sampling_time(), [&]() { current_controller.run(); });
        simulator.register_lambda_runner(
            machine_side_controller.get_sampling_time(), [&]() { machine_side_controller.run(); });
        simulator.register_lambda_runner(grid_side_modulator.get_sampling_time(), [&]() { grid_side_modulator.run(); });
        simulator.register_lambda_runner(
            machine_side_modulator.get_sampling_time(), [&]() { machine_side_modulator.run(); });
    }

    void register_datamodel(yeet::telemetry::parameter_handler& handler)
    {
        // Register all datamodels
        plant.register_datamodel(handler);
        grid_measurements.register_datamodel(handler);
        machine_measurements.register_datamodel(handler);
        dc_link_measurements.register_datamodel(handler);
        excitation_controller.register_datamodel(handler);
        speed_controller.register_datamodel(handler);
        torque_controller.register_datamodel(handler);
        current_controller.register_datamodel(handler);
        machine_side_controller.register_datamodel(handler);
        grid_side_modulator.register_datamodel(handler);
        machine_side_modulator.register_datamodel(handler);
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control components
    sfc_excitation_controller_runner excitation_controller;
    sfc_speed_controller_runner speed_controller;
    sfc_torque_controller_runner torque_controller;
    sfc_current_controller_runner current_controller;
    sfc_machine_side_controller_runner machine_side_controller;

    // Modulator components
    sfc_grid_side_modulator_runner grid_side_modulator;
    sfc_machine_side_modulator_runner machine_side_modulator;

    // Measurements and filtering
    sfc_grid_side_measurements_ideal_runner grid_measurements;
    sfc_machine_side_measurements_ideal_runner machine_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;

    // Plant model
    sfc_plant_model_runner plant;

    // External control signals
    signal_output<speed_pu_t> speed_reference;
    signal_output<bool> enable_speed_control;
    signal_output<bool> voltage_dip{false};
    signal_output<current_pu_t> external_current_reference_pu;

    // State/trip context for torque controller (no full state machine in this builder)
    signal_output<yeet::application::sfc::sfc_state_id> current_state_id{
        yeet::application::sfc::sfc_state_id::OPERATION_ON};
    signal_output<bool> machine_protection_tripped{false};

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
        // ==================== Plant Model Ã¢â€ â€™ Measurements ====================
        // Connect raw plant outputs to measurement system inputs

        // Grid measurements: Raw grid voltages and currents from plant
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_abc, grid_measurements.get_inputs().grid_voltage_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current_abc, grid_measurements.get_inputs().grid_current_abc);
        signals_connector::connect_output_to_input(
            grid_side_modulator.get_outputs().firing_pulses, grid_measurements.get_inputs().grid_pulses);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_measurements.get_inputs().grid_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_frequency, grid_measurements.get_inputs().grid_frequency);

        // Machine measurements: Raw machine voltages and currents from plant
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage_abc, machine_measurements.get_inputs().machine_voltage_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_current, machine_measurements.get_inputs().machine_current_abc);
        signals_connector::connect_output_to_input(
            machine_side_modulator.get_outputs().firing_pulses, machine_measurements.get_inputs().machine_pulses);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_measurements.get_inputs().machine_reference_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_frequency, machine_measurements.get_inputs().machine_frequency);

        // DC link measurements: Raw DC voltage and current from plant
        signals_connector::connect_output_to_input(
            grid_measurements.get_outputs().grid_current_abc, dc_link_measurements.get_inputs().grid_current_abc);

        // ==================== Speed Measurements Ã¢â€ â€™ Speed Controller ====================
        // Connect speed measurements to speed controller feedback

        // Machine speed feedback (from machine measurements)
        signals_connector::connect_output_to_input(
            machine_measurements.get_outputs().machine_speed, speed_controller.get_inputs().measured_speed);

        // ==================== Measurements Ã¢â€ â€™ Excitation Controller ====================
        // Provide speed and voltage magnitude feedback for excitation voltage regulation

        signals_connector::connect_output_to_input(machine_measurements.get_outputs().machine_speed,
            excitation_controller.get_inputs().machine_rotor_speed_pu);
        signals_connector::connect_output_to_input(
            machine_measurements.get_outputs().machine_voltage_magnitude_filtered,
            excitation_controller.get_inputs().machine_voltage_magnitude_pu);

        // ==================== Speed Controller Ã¢â€ â€™ Torque Controller Ã¢â€ â€™ Current Controller
        // ==================== Cascade control: Speed Controller outputs torque reference Ã¢â€ â€™ Torque Controller
        // Ã¢â€ â€™ Current Controller

        // Speed controller torque reference Ã¢â€ â€™ Torque controller input
        signals_connector::connect_output_to_input(
            speed_controller.get_outputs().torque_reference, torque_controller.get_inputs().torque_reference);

        // Machine speed feedback —> Torque controller input (for dynamic limits)
        signals_connector::connect_output_to_input(
            machine_measurements.get_outputs().machine_speed, torque_controller.get_inputs().measured_speed);

        // Provide state/trip context to torque controller
        signals_connector::connect_output_to_input(current_state_id, torque_controller.get_inputs().current_state_id);
        signals_connector::connect_output_to_input(
            machine_protection_tripped, torque_controller.get_inputs().machine_protection_tripped);

        // Torque controller current reference Ã¢â€ â€™ Current controller input
        signals_connector::connect_output_to_input(
            torque_controller.get_outputs().current_reference, current_controller.get_inputs().current_reference);

        // Bumpless transfer: Applied torque (after limiting) fed back to speed controller
        signals_connector::connect_output_to_input(
            torque_controller.get_outputs().torque_reference, speed_controller.get_inputs().applied_torque_reference);

        // Anti-windup saturation cascade: current_controller Ã¢â€ â€™ torque_controller Ã¢â€ â€™ speed_controller
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().is_saturated, torque_controller.get_inputs().is_saturated);
        signals_connector::connect_output_to_input(
            torque_controller.get_outputs().is_saturated, speed_controller.get_inputs().is_saturated);

        // ==================== DC Link Measurements Ã¢â€ â€™ Current Controller ====================
        // Connect DC link current feedback to current controller

        // DC link current feedback (primary feedback signal for current loop)
        signals_connector::connect_output_to_input(
            dc_link_measurements.get_outputs().dc_current_pu, current_controller.get_inputs().current_measured);

        // ==================== Torque Controller Ã¢â€ â€™ Machine Side Controller ====================
        // Connect torque controller current reference to machine side controller for lookup table

        // DC link current reference for lookup table input
        signals_connector::connect_output_to_input(
            torque_controller.get_outputs().current_reference, machine_side_controller.get_inputs().dc_link_current);

        // Machine speed for lookup table input
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_speed, machine_side_controller.get_inputs().machine_speed);

        // ==================== Modulator connections ====================

        // Firing angle command from current controller to grid modulator
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().firing_angle, grid_side_modulator.get_inputs().firing_angle);

        // Grid modulator requesting PI reset/freeze during commutation
        signals_connector::connect_output_to_input(
            grid_side_modulator.get_outputs().freeze_integrator, current_controller.get_inputs().freeze_integrator);

        // Firing angle command from machine side controller to machine modulator
        signals_connector::connect_output_to_input(
            machine_side_controller.get_outputs().firing_angle, machine_side_modulator.get_inputs().firing_angle);

        // Forced commutation handshake
        signals_connector::connect_output_to_input(
            voltage_dip, grid_side_modulator.get_inputs().undervoltage_request_extinguish_dc_current);
        signals_connector::connect_output_to_input(machine_side_modulator.get_outputs().request_force_commutation,
            grid_side_modulator.get_inputs().request_force_commutation);
        signals_connector::connect_output_to_input(grid_side_modulator.get_outputs().force_commutation_acknowledged,
            machine_side_modulator.get_inputs().force_commutation_acknowledged);

        // ==================== Modulators -> Plant Model ====================
        // Connect firing pulses from both modulators to plant model inputs

        signals_connector::connect_output_to_input(
            grid_side_modulator.get_outputs().firing_pulses, plant.get_inputs().grid_firing_pulses);

        signals_connector::connect_output_to_input(
            machine_side_modulator.get_outputs().firing_pulses, plant.get_inputs().machine_firing_pulses);

        // Field current reference for variable-speed voltage control
        signals_connector::connect_output_to_input(
            excitation_controller.get_outputs().field_current_reference, plant.get_inputs().field_current_reference);

        // ==================== Plant Model -> Modulators ====================
        // Connect reference angles from plant model to modulators for synchronization

        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_side_modulator.get_inputs().grid_reference_angle);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_side_modulator.get_inputs().machine_reference_angle);

        // ==================== External Control Signals ====================
        // Connect external control signals to speed controller inputs

        // Speed reference command (setpoint for outer loop)
        signals_connector::connect_output_to_input(speed_reference, speed_controller.get_inputs().speed_reference);

        // Speed control enable/disable
        signals_connector::connect_output_to_input(enable_speed_control, speed_controller.get_inputs().release_pulses);

        // Feedforward signals for improved current control performance
        signals_connector::connect_output_to_input(grid_measurements.get_outputs().grid_voltage_magnitude_filtered,
            current_controller.get_inputs().grid_voltage_magnitude);
        signals_connector::connect_output_to_input(
            machine_measurements.get_outputs().machine_voltage_magnitude_filtered,
            current_controller.get_inputs().machine_voltage_magnitude);
    }

private:
    bool _mock_storage_true{true};
    bool _mock_storage_false{false};
    angle_wrapped _mock_rotor_angle{0.0f};
    sfc_operating_mode_t _mock_operating_mode_natural{sfc_operating_mode_t::natural_commutation};
    reference_frame_source_t _mock_reference_frame_estimated{reference_frame_source_t::ESTIMATED};
    torque_pu_t _mock_storage_torque_zero{0.0};
    sfc::sfc_torque_controller::control_mode _mock_storage_control_mode_speed{
        sfc::sfc_torque_controller::control_mode::SPEED_CONTROL};
};

}  // namespace yeet::application::sfc::integration
