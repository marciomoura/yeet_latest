#pragma once

#include "sfc_integration/sfc_current_controller_runner.hpp"
#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_grid_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_machine_side_controller_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_pulse_generator_runner.hpp"
#include "sfc_integration/sfc_operation_mode_monitor_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Current control closed-loop builder including modulators (ideal measurements, no observers/state machine).
 *
 * Composition:
 *  Plant → Ideal Measurements → (DC link aggregation) → Controllers → Modulators → Pulse Generators → Plant
 *
 * Differences vs plant_simplified_current_controller_simulation_builder:
 *  - Inserts grid & machine side modulators between controllers and pulse generators.
 *  - Provides force-commutation handshake path (machine requests → grid acknowledges).
 *  - Integrates operation mode & zero-current monitoring (replaces previous operating mode stubs).
 *  - Still stubs enable / pulse release sequencing that a state machine would normally manage.
 */
class plant_current_control_with_modulators_simulation_builder {
public:
    static constexpr std::string_view name = "PlantCurrentControllerWithModulators";

    plant_current_control_with_modulators_simulation_builder()
        : _per_unit_system{create_per_unit_system()},
          plant{_per_unit_system},
          grid_measurements{_per_unit_system},
          machine_measurements(_per_unit_system),
          dc_link_measurements{_per_unit_system},
          current_controller(_per_unit_system)
    {
        connect_all_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Controller integrator freeze stubs (no higher-level state machine here)
        yeet::mock_signal_input::write(current_controller.get_inputs().freeze_integrator, false);

        // Enable path: set via parameters (not signals) - always true for this simplified builder
        grid_side_modulator.get_parameters().enable = true;
        machine_side_modulator.get_parameters().enable = true;
        yeet::mock_signal_input::write(operation_mode_monitor.get_inputs().enable, true);

        // Grid-side modulator inputs
        yeet::mock_signal_input::write(grid_side_modulator.get_inputs().shutdown_request, false);
        yeet::mock_signal_input::write(grid_side_modulator.get_inputs().release_pulses, true);

        // Machine-side modulator pulse release
        yeet::mock_signal_input::write(machine_side_modulator.get_inputs().release_pulses, true);

        // Pulse generators
        yeet::mock_signal_input::write(grid_side_pulse_generator.get_inputs().release_pulses, true);
        yeet::mock_signal_input::write(machine_side_pulse_generator.get_inputs().release_pulses, true);
    }

    void initialize()
    {
        plant.initialize();
        grid_measurements.initialize();
        machine_measurements.initialize();
        dc_link_measurements.initialize();
        current_controller.initialize();
        machine_side_controller.initialize();
        grid_side_modulator.initialize();
        machine_side_modulator.initialize();
        operation_mode_monitor.initialize();
        grid_side_pulse_generator.initialize();
        machine_side_pulse_generator.initialize();
    }

    void register_applications(yeet::simulator& sim)
    {
        // Order preserved: plant → measurements → dc link → controllers → operation mode monitor → modulators → pulse
        // generators
        sim.register_runner(plant);
        sim.register_runner(grid_measurements);
        sim.register_runner(machine_measurements);
        sim.register_runner(dc_link_measurements);
        sim.register_runner(current_controller);
        sim.register_runner(machine_side_controller);
        sim.register_runner(operation_mode_monitor);
        sim.register_runner(grid_side_modulator);
        sim.register_runner(machine_side_modulator);
        sim.register_runner(grid_side_pulse_generator);
        sim.register_runner(machine_side_pulse_generator);
    }

    void register_datamodel(yeet::model_registry& registry)
    {
        plant.register_datamodel(registry);
        grid_measurements.register_datamodel(registry);
        machine_measurements.register_datamodel(registry);
        dc_link_measurements.register_datamodel(registry);
        current_controller.register_datamodel(registry);
        machine_side_controller.register_datamodel(registry);
        grid_side_modulator.register_datamodel(registry);
        machine_side_modulator.register_datamodel(registry);
        operation_mode_monitor.register_datamodel(registry);
        grid_side_pulse_generator.register_datamodel(registry);
        machine_side_pulse_generator.register_datamodel(registry);
    }

    // Public members
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    sfc_current_controller_runner current_controller;
    sfc_machine_side_controller_runner machine_side_controller;

    sfc_grid_side_modulator_runner grid_side_modulator;
    sfc_machine_side_modulator_runner machine_side_modulator;
    sfc_operation_mode_monitor_runner operation_mode_monitor;

    sfc_grid_side_pulse_generator_runner grid_side_pulse_generator;
    sfc_machine_side_pulse_generator_runner machine_side_pulse_generator;

    sfc_grid_side_measurements_ideal_runner grid_measurements;
    sfc_machine_side_measurements_ideal_runner machine_measurements;
    sfc_dc_link_measurements_runner dc_link_measurements;

    sfc_plant_model_runner plant;

    signal_output<torque_pu_t> load_torque;
    signal_output<current_pu_t> current_reference;

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
        // ==================== Plant → Measurements ====================
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_abc, grid_measurements.get_inputs().grid_voltage_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current_abc, grid_measurements.get_inputs().grid_current_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_measurements.get_inputs().grid_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_frequency, grid_measurements.get_inputs().grid_frequency);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage_abc, machine_measurements.get_inputs().machine_voltage_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_current, machine_measurements.get_inputs().machine_current_abc);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_measurements.get_inputs().machine_reference_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_frequency, machine_measurements.get_inputs().machine_frequency);

        // Pulse feedback to measurements (for masking / commuting detection)
        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, grid_measurements.get_inputs().grid_pulses);
        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, machine_measurements.get_inputs().machine_pulses);

        // DC link current aggregation uses grid phase currents
        signals_connector::connect_output_to_input(
            grid_measurements.get_outputs().grid_current_abc, dc_link_measurements.get_inputs().grid_current_abc);

        // ==================== Measurements → Current Controller ====================
        signals_connector::connect_output_to_input(
            dc_link_measurements.get_outputs().dc_current_pu, current_controller.get_inputs().current_measured);
        signals_connector::connect_output_to_input(grid_measurements.get_outputs().grid_voltage_magnitude_filtered,
            current_controller.get_inputs().grid_voltage_magnitude);
        signals_connector::connect_output_to_input(
            machine_measurements.get_outputs().machine_voltage_magnitude_filtered,
            current_controller.get_inputs().machine_voltage_magnitude);

        // ==================== DC Link Measurements → Machine Side Controller ====================
        signals_connector::connect_output_to_input(dc_link_measurements.get_outputs().dc_current_heavy_filtered_pu,
            machine_side_controller.get_inputs().dc_link_current);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_speed, machine_side_controller.get_inputs().machine_speed);

        // ==================== Controllers → Modulatiors ====================
        signals_connector::connect_output_to_input(
            current_controller.get_outputs().firing_angle, grid_side_modulator.get_inputs().firing_angle);
        signals_connector::connect_output_to_input(
            machine_side_controller.get_outputs().firing_angle, machine_side_modulator.get_inputs().firing_angle);

        // Provide machine electrical angle to machine-side modulator (sector tracking / pulse mode selection)
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_side_modulator.get_inputs().machine_reference_angle);

        // Provide grid voltage source angle to grid-side pulse generator already below; ensure modulator has angle if
        // later needed (Current grid-side modulator implementation does not require angle directly.)

        // ==================== Modulators → Pulse Generators ====================
        signals_connector::connect_output_to_input(
            grid_side_modulator.get_outputs().firing_angle, grid_side_pulse_generator.get_inputs().firing_angle);
        signals_connector::connect_output_to_input(
            machine_side_modulator.get_outputs().firing_angle, machine_side_pulse_generator.get_inputs().firing_angle);

        // Pulse release signals currently fixed via mock_inputs(); if future sequencing added, connect here.

        // ==================== Force Commutation Handshake ====================
        // Machine requests → grid acknowledges
        signals_connector::connect_output_to_input(machine_side_modulator.get_outputs().request_force_commutation,
            grid_side_modulator.get_inputs().request_force_commutation);
        signals_connector::connect_output_to_input(grid_side_modulator.get_outputs().force_commutation_acknowledged,
            machine_side_modulator.get_inputs().force_commutation_acknowledged);

        // ==================== Optional Reset / PI freeze ====================
        signals_connector::connect_output_to_input(
            grid_side_modulator.get_outputs().freeze_integrator, current_controller.get_inputs().preset_request);

        // ==================== DC Link & Speed → Operation Mode Monitor ====================
        signals_connector::connect_output_to_input(
            dc_link_measurements.get_outputs().dc_current_pu, operation_mode_monitor.get_inputs().dc_link_current);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_speed, operation_mode_monitor.get_inputs().measured_speed);

        // ==================== Operation Mode Monitor → Modulatiors ====================
        signals_connector::connect_output_to_input(
            operation_mode_monitor.get_outputs().operating_mode, grid_side_modulator.get_inputs().operating_mode);
        signals_connector::connect_output_to_input(
            operation_mode_monitor.get_outputs().operating_mode, machine_side_modulator.get_inputs().operating_mode);
        signals_connector::connect_output_to_input(
            operation_mode_monitor.get_outputs().current_is_zero, grid_side_modulator.get_inputs().current_is_zero);

        // ==================== DC Current Extinguish Request ====================
        // Note: In this simplified builder without state machine, extinguish is not used.
        // Mock it to false to satisfy the input requirement.
        yeet::mock_signal_input::write(grid_side_modulator.get_inputs().request_extinguish_dc_current, false);

        // ==================== Pulse Generators → Plant ====================
        signals_connector::connect_output_to_input(load_torque, plant.get_inputs().load_torque);
        signals_connector::connect_output_to_input(
            grid_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().grid_firing_pulses);
        signals_connector::connect_output_to_input(
            machine_side_pulse_generator.get_outputs().firing_pulses, plant.get_inputs().machine_firing_pulses);

        // Field current reference for variable-speed voltage control
        signals_connector::connect_output_to_input(
            field_current_reference_pu, plant.get_inputs().field_current_reference);

        // ==================== Plant Angles → Pulse Generators ====================
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_side_pulse_generator.get_inputs().reference_angle);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_side_pulse_generator.get_inputs().reference_angle);

        // ==================== External Setpoints ====================
        signals_connector::connect_output_to_input(
            current_reference, current_controller.get_inputs().current_reference);
    }
};

}  // namespace yeet::application::sfc::integration
