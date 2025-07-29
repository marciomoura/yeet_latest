/**
 * @file sfc_6p_6p_topology_integration_test.cpp
 * @brief Integration tests for the SFC 6-pulse to 6-pulse topology with pulse generators
 *
 * This test file integrates the auto-generated Simulink SFC model with six-pulse
 * generators for both grid-side and machine-side converters. The test setup includes:
 *
 * - Grid-side converter: Firing angle step from 30° to 40°
 * - Machine-side converter: Fixed firing angle of 160°
 * - Reference angles provided by the plant model's internal voltage sources
 * - Comprehensive logging for visualization and analysis
 *
 * The pulse generators create proper 6-pulse firing patterns based on the
 * reference angles from the plant model, enabling realistic operation of
 * the thyristor-based soft frequency converter.
 */

#include <gtest/gtest.h>

#include "sfc_plant_model_simulator.hpp"
#include "sfc_simulators.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;
using namespace yeet::application::sfc;

class SFCSimulatorMinimalTest : public ::testing::Test {
protected:
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();
    sfc_plant_model_simulator simulator_plant_model;
    sfc_simulator_minimal simulator_sfc{duration_t{10e-6}};

    bool enable_pulse_generators = true;
    angle<zero_to_two_pi> grid_firing_angle_rad = deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg);
    angle<zero_to_two_pi> machine_firing_angle_rad =
        deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg);

    void SetUp() override {
        auto& plant_model = simulator_plant_model.plant();

        // Initialize the model and set parameters
        plant_model.initialize();
        plant_model.set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                        sfc_plant_model_simulator::kNominalGridVoltageRMS);
        plant_model.set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency,
                                           sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        simulator_sfc.configure_simulator(simulator, plant_model, grid_firing_angle_rad, machine_firing_angle_rad,
                                          enable_pulse_generators);

        // Setup simulator to run the open-loop controller and plant model
        simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            // Step the plant model simulation
            plant_model.step();
        });

        // Setup logging for all signals using the simulator's helper
        // simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<voltage_pu_t>>(
        //     "grid_voltage_pu",
        //     [&]() {
        //     return simulator_sfc.get_measurement_and_filtering().get_grid_side_measurements().get_voltage_pu(); }));

        // simulator.register_sampler_entry(
        //     yeet::testing::make_lambda_array_sampler<abc<current_pu_t>>("grid_current_pu", [&]() {
        //         return simulator_sfc.get_measurement_and_filtering().get_grid_side_measurements().get_current_pu();
        //     }));

        // simulator.register_sampler_entry(
        //     yeet::testing::make_lambda_sampler<voltage_pu_t>("grid_voltage_magnitude_pu", [&]() {
        //         return simulator_sfc.get_measurement_and_filtering()
        //             .get_grid_side_measurements()
        //             .get_voltage_magnitude_pu();
        //     }));

        // simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>("dc_link_current_pu", [&]()
        // {
        //     return simulator_sfc.get_measurement_and_filtering().get_dc_link_measurements().get_current_pu();
        // }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<control::six_pulse_generator::pulses>(
            "grid_side_pulses",
            [&]() { return simulator_sfc.get_pulse_generators().grid_side_pulse_generator.get_pulses(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<control::six_pulse_generator::pulses>(
            "machine_side_pulses",
            [&]() { return simulator_sfc.get_pulse_generators().machine_side_pulse_generator.get_pulses(); }));
    }
};

TEST_F(SFCSimulatorMinimalTest, BasicOperationWithFiringAngleStep) { simulator.simulate_for(0.4); }

class SFCSimulatorPerUnitWithObserverTest : public ::testing::Test {
protected:
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();
    sfc_plant_model_simulator simulator_plant_model;
    sfc_simulator_per_unit_with_observer simulator_sfc{duration_t{100e-6}};

    bool enable_pulse_generators = false;
    angle<zero_to_two_pi> grid_firing_angle_rad = deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg);
    angle<zero_to_two_pi> machine_firing_angle_rad =
        deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg);

    void SetUp() override {
        auto& plant_model = simulator_plant_model.plant();

        // Initialize the model and set parameters
        plant_model.initialize();
        plant_model.set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                        sfc_plant_model_simulator::kNominalGridVoltageRMS);
        plant_model.set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency,
                                           sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        simulator_sfc.configure_simulator(simulator, plant_model, grid_firing_angle_rad, machine_firing_angle_rad,
                                          enable_pulse_generators);

        // Setup simulator to run the open-loop controller and plant model
        simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            // Step the plant model simulation
            plant_model.step();
        });

        simulator.register_lambda_runner(100e-6, [&]() {
            // Step the plant model simulation
            // plant_model.step();
            enable_pulse_generators = simulator_sfc.get_observers().grid_side_observer.is_locked();
        });

        // Setup logging for all signals using the simulator's helper
        simulator.register_sampler_entry(
            yeet::testing::make_lambda_array_sampler<abc<voltage_pu_t>>("grid_voltage_pu", [&]() {
                return simulator_sfc.get_measurement_and_filtering().get_grid_side_measurements().get_voltage_pu();
            }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(
            "grid_voltage_observed_pu",
            [&]() { return simulator_sfc.get_observers().grid_side_observer.get_filtered_voltage_alphabeta(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<voltage_pu_t>(
            "grid_voltage_observed_magnitude_pu",
            [&]() { return simulator_sfc.get_observers().grid_side_observer.get_estimated_voltage_magnitude(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<frequency_pu_t>(
            "grid_frequency_observed_pu",
            [&]() { return simulator_sfc.get_observers().grid_side_observer.get_estimated_frequency_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<angle<zero_to_two_pi>>(
            "grid_angle_actual_pu", [&]() { return plant_model.get_grid_voltage_source_angle().get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<angle<zero_to_two_pi>>(
            "machine_angle_actual_pu", [&]() { return plant_model.get_machine_voltage_source_angle().get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<angle<zero_to_two_pi>>(
            "grid_angle_observed_pu",
            [&]() { return simulator_sfc.get_observers().grid_side_observer.get_estimated_angle().get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<angle<zero_to_two_pi>>(
            "machine_angle_observed_pu",
            [&]() { return simulator_sfc.get_observers().machine_side_observer.get_estimated_angle().get_pu(); }));

        simulator.register_sampler_entry(
            yeet::testing::make_lambda_array_sampler<abc<current_pu_t>>("grid_current_pu", [&]() {
                return simulator_sfc.get_measurement_and_filtering().get_grid_side_measurements().get_current_pu();
            }));

        simulator.register_sampler_entry(
            yeet::testing::make_lambda_sampler<voltage_pu_t>("grid_voltage_magnitude_pu", [&]() {
                return simulator_sfc.get_measurement_and_filtering()
                    .get_grid_side_measurements()
                    .get_voltage_magnitude_pu();
            }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>("dc_link_current_pu", [&]() {
            return simulator_sfc.get_measurement_and_filtering().get_dc_link_measurements().get_current_pu();
        }));

        simulator.register_sampler_entry(
            yeet::testing::make_lambda_array_sampler<abc<voltage_pu_t>>("machine_voltage_pu", [&]() {
                return simulator_sfc.get_measurement_and_filtering().get_machine_side_measurements().get_voltage_pu();
            }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(
            "machine_voltage_observed_pu",
            [&]() { return simulator_sfc.get_observers().machine_side_observer.get_filtered_voltage_alphabeta(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<frequency_pu_t>(
            "machine_frequency_observed_pu",
            [&]() { return simulator_sfc.get_observers().machine_side_observer.get_estimated_frequency_pu(); }));
    }
};

TEST_F(SFCSimulatorPerUnitWithObserverTest, BasicOperationWithFiringAngleStep) { simulator.simulate_for(1.0); }

}  // namespace