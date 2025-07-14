/**
 * @file sfc_6p_6p_topology_integration_test.cpp
 * @brief Integration tests for the SFC 6-pulse to 6-pulse topology with pulse generators
 *
 * This test file integrates the auto-generated Simulink SFC model with six-pulse
 * generators for both grid-side and machine-side converters. The test setup includes:
 *
 * - Grid-side converter: Fixed firing angle of 15°
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

#include "sfc_6p_6p_topology.h"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;

class SFC6p6pTopologyIntegrationTest : public ::testing::Test {
protected:
    static constexpr double kSamplingTime = 10e-6;            // 100 µs sampling time (10 kHz)
    static constexpr double kSimulationTime = 0.5;            // 0.5 seconds simulation
    static constexpr double kGridFrequency = 50.0;            // 50 Hz grid frequency
    static constexpr double kMachineFrequency = 50.0;         // 50 Hz machine frequency
    static constexpr double kGridVoltageRMS = 400.0;          // 400V RMS line voltage
    static constexpr double kMachineVoltageRMS = 400.0;       // 400V RMS line voltage
    static constexpr double kGridFiringAngleDeg = 30 + 30.0;  // 15° firing angle for grid side
    static constexpr double kMachineFiringAngleDeg = 160.0;   // 160° firing angle for machine side

    std::unique_ptr<generated::sfc::sfc_6p_6p_topology> sfc_model_;
    six_pulse_generator grid_side_pulse_gen_;
    six_pulse_generator machine_side_pulse_gen_;
    yeet::testing::simulator simulator_ = yeet::testing::make_gtest_simulator();
    double current_time_ = 0.0;

    void SetUp() override {
        // Create the SFC model instance
        sfc_model_ = std::make_unique<generated::sfc::sfc_6p_6p_topology>();

        // Initialize the model
        sfc_model_->initialize();

        // Setup default inputs
        setup_default_inputs();

        // Setup simulator
        simulator_.register_lambda_runner(kSamplingTime, [&]() {
            current_time_ += kSamplingTime;

            // Update pulse generators using reference angles from the plant model
            update_pulse_generators();

            // Step the SFC model
            sfc_model_->step();
        });

        // Setup logging
        setup_logging();
    }

    void TearDown() override {
        if (sfc_model_) {
            sfc_model_->terminate();
        }
    }

    void setup_default_inputs() {
        // Get reference to inputs
        auto& inputs = sfc_model_->sfc_6p_6p_topology_U;

        // Set grid side converter pulses (all off initially)
        std::fill(inputs.grid_side_converter_pulses.begin(), inputs.grid_side_converter_pulses.end(), 0.0);

        // Set machine side converter pulses (all off initially)
        std::fill(inputs.machine_side_converter_pulses.begin(), inputs.machine_side_converter_pulses.end(), 0.0);

        // Set voltage source parameters
        inputs.grid_voltage_source_frequency_hz = kGridFrequency;
        inputs.grid_voltage_source_line_voltage_rms = kGridVoltageRMS;
        inputs.machine_voltage_source_frequency_hz = kMachineFrequency;
        inputs.machine_voltage_source_line_voltage_rms = kMachineVoltageRMS;
    }

    void update_pulse_generators() {
        // Get reference to inputs
        auto& inputs = sfc_model_->sfc_6p_6p_topology_U;
        const auto& outputs = sfc_model_->sfc_6p_6p_topology_Y;

        // Create angle objects from the plant model's reference angles
        angle<zero_to_two_pi> grid_reference_angle(outputs.grid_voltage_source_angle_wt);
        angle<zero_to_two_pi> machine_reference_angle(outputs.machine_voltage_source_angle_wt);

        // Create firing angle objects
        angle<zero_to_two_pi> grid_firing_angle(deg_to_rad(kGridFiringAngleDeg));
        angle<zero_to_two_pi> machine_firing_angle(deg_to_rad(kMachineFiringAngleDeg));

        // Update grid side pulse generator
        auto grid_pulses = grid_side_pulse_gen_.update(grid_reference_angle, grid_firing_angle, true);

        // Update machine side pulse generator
        auto machine_pulses = machine_side_pulse_gen_.update(machine_reference_angle, machine_firing_angle, true);

        // Convert pulse generator outputs to plant model inputs
        inputs.grid_side_converter_pulses[0] = grid_pulses.p1 ? 1.0 : 0.0;
        inputs.grid_side_converter_pulses[1] = grid_pulses.p2 ? 1.0 : 0.0;
        inputs.grid_side_converter_pulses[2] = grid_pulses.p3 ? 1.0 : 0.0;
        inputs.grid_side_converter_pulses[3] = grid_pulses.p4 ? 1.0 : 0.0;
        inputs.grid_side_converter_pulses[4] = grid_pulses.p5 ? 1.0 : 0.0;
        inputs.grid_side_converter_pulses[5] = grid_pulses.p6 ? 1.0 : 0.0;

        inputs.machine_side_converter_pulses[0] = machine_pulses.p1 ? 1.0 : 0.0;
        inputs.machine_side_converter_pulses[1] = machine_pulses.p2 ? 1.0 : 0.0;
        inputs.machine_side_converter_pulses[2] = machine_pulses.p3 ? 1.0 : 0.0;
        inputs.machine_side_converter_pulses[3] = machine_pulses.p4 ? 1.0 : 0.0;
        inputs.machine_side_converter_pulses[4] = machine_pulses.p5 ? 1.0 : 0.0;
        inputs.machine_side_converter_pulses[5] = machine_pulses.p6 ? 1.0 : 0.0;
    }

    void update_dynamic_inputs() {
        // Get reference to inputs
        auto& inputs = sfc_model_->sfc_6p_6p_topology_U;

        // Generate simple pulse patterns for testing
        // This creates a basic 6-pulse firing pattern for both converters

        double grid_angle = 2.0 * pi<double> * kGridFrequency * current_time_;
        double machine_angle = 2.0 * pi<double> * kMachineFrequency * current_time_;

        // Generate 6-pulse pattern for grid side converter
        for (int i = 0; i < 6; ++i) {
            double phase_angle = grid_angle + i * pi<double> / 3.0;  // 60° apart
            double normalized_angle = std::fmod(phase_angle, 2.0 * pi<double>);

            // Simple pulse logic: ON for 120° starting at phase angle
            double firing_angle = i * pi<double> / 3.0;   // 0°, 60°, 120°, 180°, 240°, 300°
            double pulse_width = 2.0 * pi<double> / 3.0;  // 120° pulse width

            double relative_angle = std::fmod(normalized_angle - firing_angle + 2.0 * pi<double>, 2.0 * pi<double>);

            inputs.grid_side_converter_pulses[i] = (relative_angle < pulse_width) ? 1.0 : 0.0;
        }

        // Generate 6-pulse pattern for machine side converter with phase shift
        double phase_shift = pi<double> / 6.0;  // 30° phase shift
        for (int i = 0; i < 6; ++i) {
            double phase_angle = machine_angle + phase_shift + i * pi<double> / 3.0;
            double normalized_angle = std::fmod(phase_angle, 2.0 * pi<double>);

            double firing_angle = i * pi<double> / 3.0;
            double pulse_width = 2.0 * pi<double> / 3.0;

            double relative_angle = std::fmod(normalized_angle - firing_angle + 2.0 * pi<double>, 2.0 * pi<double>);

            inputs.machine_side_converter_pulses[i] = (relative_angle < pulse_width) ? 1.0 : 0.0;
        }
    }

    void setup_logging() {
        // Time logging
        simulator_.register_sampler_entry(
            yeet::testing::make_lambda_sampler<double>("time", [&]() -> double { return current_time_; }));

        // Grid side converter inputs
        for (int i = 0; i < 6; ++i) {
            simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
                "grid_pulse_" + std::to_string(i),
                [&, i]() -> double { return sfc_model_->sfc_6p_6p_topology_U.grid_side_converter_pulses[i]; }));
        }

        // Machine side converter inputs
        for (int i = 0; i < 6; ++i) {
            simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
                "machine_pulse_" + std::to_string(i),
                [&, i]() -> double { return sfc_model_->sfc_6p_6p_topology_U.machine_side_converter_pulses[i]; }));
        }

        // Key output signals
        const auto& outputs = sfc_model_->sfc_6p_6p_topology_Y;

        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current", [&]() -> double { return sfc_model_->sfc_6p_6p_topology_Y.dc_link_current_si; }));

        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_angle", [&]() -> double { return sfc_model_->sfc_6p_6p_topology_Y.grid_voltage_source_angle_wt; }));

        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>("machine_angle", [&]() -> double {
            return sfc_model_->sfc_6p_6p_topology_Y.machine_voltage_source_angle_wt;
        }));

        // Grid side three-phase voltages
        for (int i = 0; i < 3; ++i) {
            simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
                "grid_voltage_phase_" + std::string(1, 'a' + i), [&, i]() -> double {
                    return sfc_model_->sfc_6p_6p_topology_Y.grid_side_converter_voltage_phase_si[i];
                }));
        }

        // Grid side three-phase currents
        for (int i = 0; i < 3; ++i) {
            simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
                "grid_current_phase_" + std::string(1, 'a' + i),
                [&, i]() -> double { return sfc_model_->sfc_6p_6p_topology_Y.grid_side_converter_current_si[i]; }));
        }

        // Machine side three-phase voltages
        for (int i = 0; i < 3; ++i) {
            simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
                "machine_voltage_phase_" + std::string(1, 'a' + i), [&, i]() -> double {
                    return sfc_model_->sfc_6p_6p_topology_Y.machine_side_converter_voltage_phase_si[i];
                }));
        }

        // Machine side three-phase currents
        for (int i = 0; i < 3; ++i) {
            simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
                "machine_current_phase_" + std::string(1, 'a' + i),
                [&, i]() -> double { return sfc_model_->sfc_6p_6p_topology_Y.machine_side_converter_current_si[i]; }));
        }

        // Input frequencies and voltages for reference
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_frequency_input",
            [&]() -> double { return sfc_model_->sfc_6p_6p_topology_U.grid_voltage_source_frequency_hz; }));

        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_frequency_input",
            [&]() -> double { return sfc_model_->sfc_6p_6p_topology_U.machine_voltage_source_frequency_hz; }));

        // Firing angles for reference
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_firing_angle_deg", [&]() -> double { return kGridFiringAngleDeg; }));

        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_firing_angle_deg", [&]() -> double { return kMachineFiringAngleDeg; }));
    }

    void run_simulation(double duration) { simulator_.simulate_for(duration); }
};

TEST_F(SFC6p6pTopologyIntegrationTest, BasicOperation) {
    // Test basic operation of the SFC model
    run_simulation(1.0);

    // Basic sanity checks
    const auto& outputs = sfc_model_->sfc_6p_6p_topology_Y;

    // Check that outputs are reasonable (not NaN or infinite)
    EXPECT_TRUE(std::isfinite(outputs.dc_link_current_si));
    EXPECT_TRUE(std::isfinite(outputs.grid_voltage_source_angle_wt));
    EXPECT_TRUE(std::isfinite(outputs.machine_voltage_source_angle_wt));

    // Check that angles are progressing (should increase over time)
    EXPECT_GT(outputs.grid_voltage_source_angle_wt, 0.0);
    EXPECT_GT(outputs.machine_voltage_source_angle_wt, 0.0);

    // Check that three-phase outputs are finite
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(outputs.grid_side_converter_voltage_phase_si[i]));
        EXPECT_TRUE(std::isfinite(outputs.grid_side_converter_current_si[i]));
        EXPECT_TRUE(std::isfinite(outputs.machine_side_converter_voltage_phase_si[i]));
        EXPECT_TRUE(std::isfinite(outputs.machine_side_converter_current_si[i]));
    }

    // Simulation should complete without errors
    EXPECT_EQ(sfc_model_->getRTM()->errorStatus, nullptr);
}

}  // namespace
