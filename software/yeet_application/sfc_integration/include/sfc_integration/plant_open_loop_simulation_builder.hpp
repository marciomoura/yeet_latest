#pragma once

#include "sfc_integration/sfc_dc_link_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"
#include "sfc_integration/sfc_plant_model_runner.hpp"

// Hardware mocks (ADC front-end simulation)
#include "sfc_grid_side_hardware_mock_runner.hpp"
#include "sfc_integration/plant_hardware_definition.hpp"  // staging helpers for mock parameters
#include "sfc_machine_side_hardware_mock_runner.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/signal_wiring.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc::integration {

// ==================== Open Loop Simulation Builder ====================

/**
 * @brief Open loop simulation builder with plant model and pulse generators
 *
 * This configuration provides:
 * - Plant model with augmented 6p-6p topology
 * - Grid-side and machine-side pulse generators
 * - Grid-side and machine-side hardware mocks (ADC quantization of plant analog signals)
 * - Direct firing angle control (open loop operation)
 * - Per-unit system for proper scaling
 * - Simplified configuration for open loop testing
 *
 * Signal Flow:
 * External Firing Angles → Pulse Generators → Plant Model
 */
class plant_open_loop_simulation_builder {
public:
    static constexpr std::string_view name = "PlantOpenLoop";

    plant_open_loop_simulation_builder()
        : plant{create_per_unit_system()},
          grid_side_hardware_mock{duration_t{10e-6}},
          machine_side_hardware_mock{duration_t{10e-6}},
          grid_side_measurements{plant.get_per_unit_system()},
          machine_side_measurements{plant.get_per_unit_system()},
          dc_link_measurements{plant.get_per_unit_system()}
    {
        // Stage hardware mock parameters (ADC + transducer chain)
        plant_hardware::stage_grid_side_hardware_mock_parameters(grid_side_hardware_mock);
        plant_hardware::stage_machine_side_hardware_mock_parameters(machine_side_hardware_mock);
        // Compute and stage transformer ratios for target spans (1.3 pu voltage, 2.0 pu current)
        plant_hardware::stage_transformer_ratios_for_grid_side(grid_side_hardware_mock, *plant.get_per_unit_system());
        plant_hardware::stage_transformer_ratios_for_machine_side(
            machine_side_hardware_mock, *plant.get_per_unit_system());
        // Mirror ratios to measurement runners so reconstruction uses same scaling
        plant_hardware::stage_transformer_ratios_for_grid_side_measurements(
            grid_side_measurements, *plant.get_per_unit_system());
        plant_hardware::stage_transformer_ratios_for_machine_side_measurements(
            machine_side_measurements, *plant.get_per_unit_system());
        // Stage measurement conversion parameters (ADC + transducer) for runners
        plant_hardware::stage_grid_side_measurement_parameters(grid_side_measurements);
        plant_hardware::stage_machine_side_measurement_parameters(machine_side_measurements);
        // Connect pulse generators and hardware mocks to plant model
        connect_components();
        mock_inputs();
    }

    void mock_inputs()
    {
        // Mock pulse release signals for both modulators
        yeet::wire(grid_side_modulator)
            .mocked(_mock_storage_true)
            .to(&sfc_grid_side_modulator_datamodel::inputs::release_pulses)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::request_extinguish_dc_current)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::shutdown_request)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::protection_trip)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::request_force_commutation)
            .mocked(_mock_storage_false)
            .to(&sfc_grid_side_modulator_datamodel::inputs::current_is_zero)
            .mocked(_mock_storage_mode_natural)
            .to(&sfc_grid_side_modulator_datamodel::inputs::operating_mode);

        yeet::wire(machine_side_modulator)
            .mocked(_mock_storage_true)
            .to(&sfc_machine_side_modulator_datamodel::inputs::release_pulses)
            .mocked(_mock_storage_false)
            .to(&sfc_machine_side_modulator_datamodel::inputs::force_commutation_acknowledged)
            .mocked(_mock_storage_mode_natural)
            .to(&sfc_machine_side_modulator_datamodel::inputs::operating_mode)
            .mocked(_mock_storage_ref_estimated)
            .to(&sfc_machine_side_modulator_datamodel::inputs::reference_frame_source)
            .mocked(_mock_storage_angle_zero)
            .to(&sfc_machine_side_modulator_datamodel::inputs::virtual_rotor_angle);
    }

    void initialize()
    {
        // Initialize components in dependency order (plant first so mocks have valid sources)
        plant.initialize();
        grid_side_hardware_mock.initialize();
        machine_side_hardware_mock.initialize();
        grid_side_measurements.initialize();
        machine_side_measurements.initialize();
        dc_link_measurements.initialize();
        grid_side_modulator.initialize();
        machine_side_modulator.initialize();
    }

    void register_applications(yeet::simulator& simulator)
    {
        // Register modulators at their sampling times
        simulator.register_runner(grid_side_modulator);
        simulator.register_runner(machine_side_modulator);
        // Plant + hardware mocks: run plant then convert analog → ADC counts
        simulator.register_lambda_runner(plant.get_sampling_time(), [&]() {
            plant.run();
            grid_side_hardware_mock.run();
            machine_side_hardware_mock.run();
            grid_side_measurements.run();
            machine_side_measurements.run();
            dc_link_measurements.run();
        });
    }

    void register_datamodel(yeet::telemetry::parameter_handler& handler)
    {
        // Register all datamodels (order: plant, mocks, modulators)
        plant.register_datamodel(handler);
        grid_side_hardware_mock.register_datamodel(handler);
        machine_side_hardware_mock.register_datamodel(handler);
        grid_side_measurements.register_datamodel(handler);
        machine_side_measurements.register_datamodel(handler);
        dc_link_measurements.register_datamodel(handler);
        grid_side_modulator.register_datamodel(handler);
        machine_side_modulator.register_datamodel(handler);
    }

    // Public members for direct access
    sfc_grid_side_modulator_runner grid_side_modulator;
    sfc_machine_side_modulator_runner machine_side_modulator;
    sfc_plant_model_runner plant;
    sfc_grid_side_hardware_mock_runner grid_side_hardware_mock;        // ADC mock for grid-side voltages & currents
    sfc_machine_side_hardware_mock_runner machine_side_hardware_mock;  // ADC mock for machine-side voltages
    sfc_grid_side_measurements_runner grid_side_measurements;          // ADC-based grid-side measurement chain
    sfc_machine_side_measurements_runner machine_side_measurements;    // ADC-based machine-side measurement chain
    sfc_dc_link_measurements_runner dc_link_measurements;              // DC link measurement chain

    signal_output<torque_pu_t> load_torque;
    signal_output<angle_wrapped> firing_angle;
    signal_output<angle_wrapped> grid_side_firing_angle;

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

    void connect_components()
    {
        // ==================== Pulse Generators → Plant Model ====================

        signals_connector::connect_output_to_input(
            grid_side_modulator.get_outputs().firing_pulses, plant.get_inputs().grid_firing_pulses);
        signals_connector::connect_output_to_input(
            machine_side_modulator.get_outputs().firing_pulses, plant.get_inputs().machine_firing_pulses);

        // Field current reference for variable-speed voltage control
        signals_connector::connect_output_to_input(
            field_current_reference_pu, plant.get_inputs().field_current_reference);

        // ==================== Plant Model → Pulse Generators ====================
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_side_modulator.get_inputs().grid_reference_angle);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_side_modulator.get_inputs().machine_reference_angle);

        // ==================== Plant Outputs → Hardware Mocks (analog → ADC counts) ====================
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_abc, grid_side_hardware_mock.get_inputs().grid_voltage_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_current_abc, grid_side_hardware_mock.get_inputs().grid_current_abc);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_voltage_abc, machine_side_hardware_mock.get_inputs().machine_voltage_abc);

        // ==================== Hardware Mocks → Measurement Runners (ADC counts to PU) ====================
        // Grid side: connect ADC counts outputs to measurement runner inputs
        // Voltage line-line counts Vab, Vbc
        signals_connector::connect_output_to_input(
            grid_side_hardware_mock.get_outputs().v_ab_counts, grid_side_measurements.get_inputs().v_ab_counts);
        signals_connector::connect_output_to_input(
            grid_side_hardware_mock.get_outputs().v_bc_counts, grid_side_measurements.get_inputs().v_bc_counts);
        // Currents phase A and C (two-sensor reconstruction)
        signals_connector::connect_output_to_input(
            grid_side_hardware_mock.get_outputs().i_a_counts, grid_side_measurements.get_inputs().i_a_counts);
        signals_connector::connect_output_to_input(
            grid_side_hardware_mock.get_outputs().i_c_counts, grid_side_measurements.get_inputs().i_c_counts);
        signals_connector::connect_output_to_input(
            grid_side_modulator.get_outputs().firing_pulses, grid_side_measurements.get_inputs().grid_pulses);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_voltage_source_angle, grid_side_measurements.get_inputs().grid_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().grid_frequency, grid_side_measurements.get_inputs().grid_frequency);

        // Machine side: connect ADC counts outputs to measurement runner inputs
        signals_connector::connect_output_to_input(
            machine_side_hardware_mock.get_outputs().v_ab_counts, machine_side_measurements.get_inputs().v_ab_counts);
        signals_connector::connect_output_to_input(
            machine_side_hardware_mock.get_outputs().v_bc_counts, machine_side_measurements.get_inputs().v_bc_counts);
        signals_connector::connect_output_to_input(
            machine_side_modulator.get_outputs().firing_pulses, machine_side_measurements.get_inputs().machine_pulses);
        signals_connector::connect_output_to_input(plant.get_outputs().machine_voltage_source_angle,
            machine_side_measurements.get_inputs().machine_reference_angle);
        signals_connector::connect_output_to_input(
            plant.get_outputs().machine_frequency, machine_side_measurements.get_inputs().machine_frequency);

        // ==================== External Control Signals ====================
        signals_connector::connect_output_to_input(
            grid_side_firing_angle, grid_side_modulator.get_inputs().firing_angle);
        signals_connector::connect_output_to_input(firing_angle, machine_side_modulator.get_inputs().firing_angle);

        // ==================== Measurements → DC Link Measurements ====================
        // Use reconstructed grid phase currents (abc) from grid-side measurements as input
        signals_connector::connect_output_to_input(
            grid_side_measurements.get_outputs().grid_current_abc, dc_link_measurements.get_inputs().grid_current_abc);

        // ==================== Handshake ====================
        signals_connector::connect_output_to_input(machine_side_modulator.get_outputs().request_force_commutation,
            grid_side_modulator.get_inputs().request_force_commutation);
        signals_connector::connect_output_to_input(grid_side_modulator.get_outputs().force_commutation_acknowledged,
            machine_side_modulator.get_inputs().force_commutation_acknowledged);
    }

private:
    bool _mock_storage_true{true};
    bool _mock_storage_false{false};
    sfc_operating_mode_t _mock_storage_mode_natural{sfc_operating_mode_t::natural_commutation};
    reference_frame_source_t _mock_storage_ref_estimated{reference_frame_source_t::ESTIMATED};
    angle_wrapped _mock_storage_angle_zero{0.0f};
};

}  // namespace yeet::application::sfc::integration