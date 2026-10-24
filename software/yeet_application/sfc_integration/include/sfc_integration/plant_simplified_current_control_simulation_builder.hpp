
#pragma once

#include "sfc/sfc_startup_sequencer.hpp"
#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/signal_wiring.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Current Controller Simulation Builder ====================

/**
 * @brief Current controller simulation builder with plant model and measurements
 *
 * This configuration provides:
 * - Plant model with augmented 6p-6p topology
 * - Grid-side current controller with firing angle control
 * - Machine-side lookup table controller for firing angle calculation
 * - Complete measurement and filtering chain
 * - Per-unit system for proper scaling
 * - Simplified configuration without observers, state machine, or modulators
 *
 * Signal Flow:
 * Plant → Raw Measurements → Controllers → Pulse Generators → Plant
 */
class plant_simplified_current_control_simulation_builder {
public:
    static constexpr std::string_view name = "PlantSimplifiedCurrentControl";

    plant_simplified_current_control_simulation_builder()
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
            .mocked(current_controller_mock.freeze_integrator)
            .to(&sfc_current_controller_datamodel::inputs::freeze_integrator)
            .mocked(current_controller_mock.preset_request)
            .to(&sfc_current_controller_datamodel::inputs::preset_request);

        // Mock pulse release signals for both modulators
        yeet::wire(grid_side_modulator)
            .mocked(modulator_mock.release_pulses)
            .to(&sfc_grid_side_modulator_datamodel::inputs::release_pulses)
            .mocked(modulator_mock.operating_mode)
            .to(&sfc_grid_side_modulator_datamodel::inputs::operating_mode)
            .mocked(modulator_mock.current_is_zero)
            .to(&sfc_grid_side_modulator_datamodel::inputs::current_is_zero)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::request_extinguish_dc_current)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::shutdown_request)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::protection_trip)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::undervoltage_request_extinguish_dc_current);

        yeet::wire(machine_side_modulator)
            .mocked(modulator_mock.release_pulses)
            .to(&sfc_machine_side_modulator_datamodel::inputs::release_pulses)
            .mocked(modulator_mock.operating_mode)
            .to(&sfc_machine_side_modulator_datamodel::inputs::operating_mode)
            .mocked(_mock_reference_frame_estimated)
            .to(&sfc_machine_side_modulator_datamodel::inputs::reference_frame_source)
            .mocked(_mock_rotor_angle)
            .to(&sfc_machine_side_modulator_datamodel::inputs::virtual_rotor_angle);
    }

    void initialize()
    {
        // Initialize components in dependency order
        plant.initialize();
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();
        current_controller.initialize();
        machine_side_controller.initialize();
        grid_side_modulator.initialize();
        machine_side_modulator.initialize();
    }

    void register_applications(yeet::simulator& sim)
    {
        // Register all runners with their respective sampling times
        sim.register_lambda_runner(plant.get_sampling_time(), [&]() { plant.run(); });
        sim.register_lambda_runner(grid_measurements.get_sampling_time(), [&]() { grid_measurements.run(); });
        sim.register_lambda_runner(machine_measurements.get_sampling_time(), [&]() { machine_measurements.run(); });
        sim.register_lambda_runner(dc_link_measurements.get_sampling_time(), [&]() { dc_link_measurements.run(); });
        sim.register_lambda_runner(current_controller.get_sampling_time(), [&]() { current_controller.run(); });
        sim.register_lambda_runner(
            machine_side_controller.get_sampling_time(), [&]() { machine_side_controller.run(); });
        sim.register_lambda_runner(grid_side_modulator.get_sampling_time(), [&]() { grid_side_modulator.run(); });
        sim.register_lambda_runner(machine_side_modulator.get_sampling_time(), [&]() { machine_side_modulator.run(); });
    }

    void register_datamodel(yeet::telemetry::parameter_handler& handler)
    {
        // Register all datamodels
        plant.register_datamodel(handler);
        grid_measurements.register_datamodel(handler);
        machine_measurements.register_datamodel(handler);
        dc_link_measurements.register_datamodel(handler);
        current_controller.register_datamodel(handler);
        machine_side_controller.register_datamodel(handler);
        grid_side_modulator.register_datamodel(handler);
        machine_side_modulator.register_datamodel(handler);
    }

    // Public members for direct access
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Control components
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
    signal_output<torque_pu_t> load_torque;
    signal_output<current_pu_t> current_reference;
    signal_output<bool> voltage_dip{false};

    // Field current reference for variable-speed operation (no excitation controller in this builder)
    signal_output<current_pu_t> field_current_reference_pu{current_pu_t{0.8f}};

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
        // ==================== Plant Model → Measurements ====================
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

        // ==================== Measurements → Current Controller ====================
        // Connect filtered measurements to current controller feedback

        // DC link current feedback (RAW)
        signals_connector::connect_output_to_input(
            dc_link_measurements.get_outputs().dc_current_pu, current_controller.get_inputs().current_measured);

        // ==================== DC Link Measurements → Machine Side Controller ====================
        // Connect DC link current and speed to machine side controller for lookup table

        // DC link current for lookup table input
        signals_connector::connect_output_to_input(dc_link_measurements.get_outputs().dc_current_heavy_filtered_pu,
            machine_side_controller.get_inputs().dc_link_current);

        // Machine speed for lookup table input
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_speed, machine_side_controller.get_inputs().machine_speed);

        // ==================== Controller outputs → Modulators ====================
        // Connect controller outputs to modulator firing angle signals

        // Firing angle command from current controller to grid modulator
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().firing_angle, grid_side_modulator.get_inputs().firing_angle);

        // Firing angle command from machine side controller to machine modulator
        signals_connector::connect_output_to_input(
            machine_side_controller.get_outputs().firing_angle, machine_side_modulator.get_inputs().firing_angle);

        // ==================== Handshake Modulators ====================
        signals_connector::connect_output_to_input(
            voltage_dip, grid_side_modulator.get_inputs().undervoltage_request_extinguish_dc_current);
        signals_connector::connect_output_to_input(machine_side_modulator.get_outputs().request_force_commutation,
            grid_side_modulator.get_inputs().request_force_commutation);

        signals_connector::connect_output_to_input(grid_side_modulator.get_outputs().force_commutation_acknowledged,
            machine_side_modulator.get_inputs().force_commutation_acknowledged);

        // ==================== Modulators → Plant Model ====================
        // Connect firing pulses from both modulators to plant model inputs

        signals_connector::connect_output_to_input(
            grid_side_modulator.get_outputs().firing_pulses, plant.get_inputs().grid_firing_pulses);

        signals_connector::connect_output_to_input(
            machine_side_modulator.get_outputs().firing_pulses, plant.get_inputs().machine_firing_pulses);

        // Field current reference for variable-speed voltage control
        signals_connector::connect_output_to_input(
            field_current_reference_pu, plant.get_inputs().field_current_reference);

        // ==================== Plant Model → Modulators ====================
        // Connect reference angles from plant model to modulators for synchronization

        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_side_modulator.get_inputs().grid_reference_angle);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_side_modulator.get_inputs().machine_reference_angle);

        // ==================== External Control Signals ====================
        // Connect external control signals to current controller inputs

        // Current reference command (setpoint)
        signals_connector::connect_output_to_input(
            current_reference, current_controller.get_inputs().current_reference);

        // Feedforward signals for improved control performance
        signals_connector::connect_output_to_input(grid_measurements.get_outputs().grid_voltage_magnitude_filtered,
            current_controller.get_inputs().grid_voltage_magnitude);
        signals_connector::connect_output_to_input(
            machine_measurements.get_outputs().machine_voltage_magnitude_filtered,
            current_controller.get_inputs().machine_voltage_magnitude);
    }

public:
    struct current_controller_mock_t {
        bool freeze_integrator{true};
        bool preset_request{false};
    };

    struct modulator_mock_t {
        bool release_pulses{false};
        sfc_operating_mode_t operating_mode{sfc_operating_mode_t::natural_commutation};
        bool current_is_zero{false};
    };

    current_controller_mock_t current_controller_mock;
    modulator_mock_t modulator_mock;

private:
    bool _mock_storage_true{true};
    bool _mock_storage_false{false};
    angle_wrapped _mock_rotor_angle{0.0f};
    sfc_operating_mode_t _mock_operating_mode_natural{sfc_operating_mode_t::natural_commutation};
    reference_frame_source_t _mock_reference_frame_estimated{reference_frame_source_t::ESTIMATED};
};

}  // namespace yeet::application::sfc::integration
