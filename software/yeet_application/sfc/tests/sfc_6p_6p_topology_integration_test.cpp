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

#include <cmath>
#include <memory>

#include "sfc/sfc_plant_model_simulator.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;
using namespace yeet::application::sfc;

class SFC6p6pTopologyIntegrationTest : public ::testing::Test {
protected:
    static constexpr double kSimulationTime = 0.5;              // 0.5 seconds simulation
    static constexpr double kGridFrequency = 50.0;              // 50 Hz grid frequency
    static constexpr double kMachineFrequency = 50.0;           // 30 Hz machine frequency
    static constexpr double kGridVoltageRMS = 400.0;            // 400V RMS line voltage
    static constexpr double kMachineVoltageRMS = 400.0;         // 400V RMS line voltage
    static constexpr double kInitialGridFiringAngleDeg = 30.0;  // Initial firing angle for grid side
    static constexpr double kMachineFiringAngleDeg = 160.0;     // 160° firing angle for machine side

    sfc_plant_model_simulator _plant_model;
    six_pulse_generator _grid_side_pulse_generator;
    six_pulse_generator _machine_side_pulse_generator;
    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();

    // Configurable firing angle for grid side converter
    double grid_firing_angle_deg_ = kInitialGridFiringAngleDeg;

    void SetUp() override {
        auto& plant_model = _plant_model.plant();

        // Initialize the model and set parameters
        plant_model.initialize();
        plant_model.set_grid_parameters(kGridFrequency, kGridVoltageRMS);
        plant_model.set_machine_parameters(kMachineFrequency, kMachineVoltageRMS);

        // Setup simulator to run the open-loop controller and plant model
        _simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            // In an open-loop test, we manually generate pulses and feed them to the plant
            update_pulse_generators();

            // Step the plant model simulation
            plant_model.step();
        });

        // Setup logging for all signals using the simulator's helper
        _plant_model.setup_all_logging(_simulator);
    }

    void TearDown() override { _plant_model.plant().terminate(); }

    void update_pulse_generators() {
        auto& plant_model = _plant_model.plant();

        // Get reference angles from the plant model
        auto grid_reference_angle = plant_model.get_grid_voltage_source_angle();
        auto machine_reference_angle = plant_model.get_machine_voltage_source_angle();

        // Create firing angle objects
        angle<zero_to_two_pi> grid_firing_angle(deg_to_rad(grid_firing_angle_deg_));
        angle<zero_to_two_pi> machine_firing_angle(deg_to_rad(kMachineFiringAngleDeg));

        // Update pulse generators
        auto grid_pulses = _grid_side_pulse_generator.update(grid_reference_angle, grid_firing_angle, true);
        auto machine_pulses = _machine_side_pulse_generator.update(machine_reference_angle, machine_firing_angle, true);

        // Set the generated pulses as inputs to the plant model
        plant_model.set_grid_side_pulses(grid_pulses.to_array());
        plant_model.set_machine_side_pulses(machine_pulses.to_array());
    }

    void run_simulation(double duration) { _simulator.simulate_for(duration); }

    /**
     * @brief Set the grid firing angle in degrees
     * @param angle_deg Firing angle in degrees
     */
    void set_grid_firing_angle(double angle_deg) { grid_firing_angle_deg_ = angle_deg; }
};

TEST_F(SFC6p6pTopologyIntegrationTest, BasicOperationWithFiringAngleStep) {
    // Test basic operation of the SFC model with a step change in the grid-side firing angle.

    // Phase 1: Run with initial firing angle (30°) for 0.25 seconds
    run_simulation(0.25);

    // Capture initial state for comparison
    // double initial_dc_current = _plant_model.plant().get_dc_link_current();

    // Phase 2: Step change in grid firing angle from 30° to 40°
    set_grid_firing_angle(40.0);

    // Phase 3: Run with new firing angle (40°) for another 0.25 seconds
    run_simulation(0.25);

    // Capture final state
    // double final_dc_current = _plant_model.plant().get_dc_link_current();

    // // Basic sanity checks
    // EXPECT_TRUE(std::isfinite(final_dc_current));
    // EXPECT_TRUE(std::isfinite(_plant_model.plant().get_grid_voltage_source_angle()));
    // EXPECT_TRUE(std::isfinite(_plant_model.plant().get_machine_voltage_source_angle()));

    // // Check that angles are progressing (should increase over time)
    // EXPECT_GT(_plant_model.plant().get_grid_voltage_source_angle(), 0.0);
    // EXPECT_GT(_plant_model.plant().get_machine_voltage_source_angle(), 0.0);

    // Check that three-phase outputs are finite
    // for (int i = 0; i < 3; ++i) {
    //     EXPECT_TRUE(std::isfinite(_plant_model.plant().get_grid_side_converter_voltage_phase()[i]));
    //     EXPECT_TRUE(std::isfinite(_plant_model.plant().get_grid_side_converter_current()[i]));
    //     EXPECT_TRUE(std::isfinite(_plant_model.plant().get_machine_side_converter_voltage_phase()[i]));
    //     EXPECT_TRUE(std::isfinite(_plant_model.plant().get_machine_side_converter_current()[i]));
    // }

    // Verify firing angle step change effect on DC link current
    // Increasing firing angle should affect the converter operation
    // The exact behavior depends on the system dynamics, but we can verify the change occurred
    // EXPECT_NE(initial_dc_current, final_dc_current) << "DC current should change with firing angle step";

    // Simulation should complete without errors
    // EXPECT_EQ(_simulator.get_error_status(), nullptr);
}

}  // namespace