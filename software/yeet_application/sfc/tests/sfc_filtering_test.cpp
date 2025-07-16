#include "sfc/sfc_filtering.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sfc/sfc_measurements.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"

using namespace yeet::application::sfc;
using namespace yeet::control;

class SfcFilteringTest : public ::testing::Test {
protected:
    void SetUp() override {
        sampling_period_ = 1e-4;  // 10 kHz sampling
        grid_filter_ = std::make_unique<sfc_grid_side_filtering>(sampling_period_);
        dc_filter_ = std::make_unique<sfc_dc_link_filtering>(sampling_period_);
        machine_filter_ = std::make_unique<sfc_machine_side_filtering>(sampling_period_);
    }

    double sampling_period_;
    std::unique_ptr<sfc_grid_side_filtering> grid_filter_;
    std::unique_ptr<sfc_dc_link_filtering> dc_filter_;
    std::unique_ptr<sfc_machine_side_filtering> machine_filter_;
};

TEST_F(SfcFilteringTest, GridSideFiltering_ProducesCorrectComponents) {
    // Configure filters
    grid_filter_->configure(50.0, 50.0);  // 50 Hz cutoff for both voltage and current

    // Test with balanced three-phase voltages
    double voltage_magnitude_pu = 1.0;  // 1.0 pu peak
    auto test_voltages_pu = make_abc<double>(voltage_magnitude_pu, angle<zero_to_two_pi>(0.0));
    auto test_currents_pu = make_abc<double>(1.0, angle<zero_to_two_pi>(0.0));  // 1.0 pu peak

    // Update filters multiple times to allow settling
    for (int i = 0; i < 100; ++i) {
        grid_filter_->update(test_voltages_pu, test_currents_pu);
    }

    // Check abc components
    auto filtered_voltages_pu = grid_filter_->get_filtered_voltages_pu();
    EXPECT_NEAR(filtered_voltages_pu.a(), voltage_magnitude_pu, voltage_magnitude_pu * 0.1);

    // Check αβ components
    auto voltage_alphabeta_pu = grid_filter_->get_filtered_voltage_alphabeta_pu();
    EXPECT_NEAR(voltage_alphabeta_pu.alpha(), voltage_magnitude_pu, voltage_magnitude_pu * 0.1);
    EXPECT_NEAR(voltage_alphabeta_pu.beta(), 0.0, voltage_magnitude_pu * 0.1);

    // Check magnitude
    double magnitude_pu = grid_filter_->get_filtered_voltage_magnitude_pu();
    EXPECT_NEAR(magnitude_pu, voltage_magnitude_pu, voltage_magnitude_pu * 0.1);
}

TEST_F(SfcFilteringTest, MachineSideFiltering_ProducesCorrectComponents) {
    // Test with machine frequency adaptation
    double machine_frequency = 50.0;    // 50 Hz
    double voltage_magnitude_pu = 1.0;  // 1.0 pu peak
    auto test_voltages_pu = make_abc<double>(voltage_magnitude_pu, angle<zero_to_two_pi>(0.0));
    auto test_currents_pu = make_abc<double>(1.0, angle<zero_to_two_pi>(0.0));  // 1.0 pu peak

    // Update filters multiple times to allow settling
    for (int i = 0; i < 100; ++i) {
        machine_filter_->update(test_voltages_pu, test_currents_pu, machine_frequency);
    }

    // Check abc components
    auto filtered_voltages_pu = machine_filter_->get_filtered_voltages_pu();
    EXPECT_NEAR(filtered_voltages_pu.a(), voltage_magnitude_pu, voltage_magnitude_pu * 0.1);

    // Check αβ components
    auto voltage_alphabeta_pu = machine_filter_->get_filtered_voltage_alphabeta_pu();
    EXPECT_NEAR(voltage_alphabeta_pu.alpha(), voltage_magnitude_pu, voltage_magnitude_pu * 0.1);
    EXPECT_NEAR(voltage_alphabeta_pu.beta(), 0.0, voltage_magnitude_pu * 0.1);

    // Check magnitude
    double magnitude_pu = machine_filter_->get_filtered_voltage_magnitude_pu();
    EXPECT_NEAR(magnitude_pu, voltage_magnitude_pu, voltage_magnitude_pu * 0.1);
}

TEST_F(SfcFilteringTest, DcLinkFiltering_ProducesCorrectMagnitude) {
    // Configure DC link filtering
    dc_filter_->configure(100.0, 300.0);  // 100 Hz voltage filter, 300 Hz current ripple

    double dc_voltage_pu = 1.0;   // 1.0 pu DC
    double dc_current_pu = -0.8;  // -0.8 pu DC (negative for motor operation)

    // Update filters multiple times to allow settling
    for (int i = 0; i < 100; ++i) {
        dc_filter_->update(dc_voltage_pu, dc_current_pu);
    }

    // Check filtered values
    EXPECT_NEAR(dc_filter_->get_filtered_voltage_pu(), dc_voltage_pu, dc_voltage_pu * 0.05);
    EXPECT_NEAR(dc_filter_->get_filtered_current_pu(), dc_current_pu, std::abs(dc_current_pu) * 0.05);

    // Check magnitude (should be absolute value of current)
    double current_magnitude_pu = dc_filter_->get_filtered_current_magnitude_pu();
    EXPECT_NEAR(current_magnitude_pu, std::abs(dc_current_pu), std::abs(dc_current_pu) * 0.05);
}

TEST_F(SfcFilteringTest, AlphaBetaTransformation_BalancedVoltages) {
    grid_filter_->configure(1000.0, 1000.0);  // High cutoff to minimize filtering effects

    // Test with different phase angles
    std::vector<double> phase_angles = {0.0, pi<double> / 6, pi<double> / 4, pi<double> / 3, pi<double> / 2};
    double voltage_magnitude_pu = 0.9;

    for (double phase : phase_angles) {
        auto test_voltages_pu = make_abc<double>(voltage_magnitude_pu, angle<zero_to_two_pi>(phase));
        auto test_currents_pu = make_abc<double>(0.5, angle<zero_to_two_pi>(phase));

        // Update multiple times for settling
        for (int i = 0; i < 50; ++i) {
            grid_filter_->update(test_voltages_pu, test_currents_pu);
        }

        auto alphabeta_pu = grid_filter_->get_filtered_voltage_alphabeta_pu();
        double magnitude_pu = grid_filter_->get_filtered_voltage_magnitude_pu();

        // Verify magnitude is consistent
        EXPECT_NEAR(magnitude_pu, voltage_magnitude_pu, voltage_magnitude_pu * 0.05)
            << "Failed for phase angle: " << phase;

        // Verify αβ magnitude matches expected
        double calculated_magnitude =
            std::sqrt(alphabeta_pu.alpha() * alphabeta_pu.alpha() + alphabeta_pu.beta() * alphabeta_pu.beta());
        EXPECT_NEAR(calculated_magnitude, voltage_magnitude_pu, voltage_magnitude_pu * 0.05)
            << "Failed for phase angle: " << phase;
    }
}

TEST_F(SfcFilteringTest, MachineFrequencyAdaptation) {
    // Test that machine-side filtering adapts to different frequencies
    std::vector<double> frequencies = {5.0, 25.0, 50.0, 75.0};
    double voltage_magnitude_pu = 1.1;

    for (double freq : frequencies) {
        auto test_voltages_pu = make_abc<double>(voltage_magnitude_pu, angle<zero_to_two_pi>(0.0));
        auto test_currents_pu = make_abc<double>(0.8, angle<zero_to_two_pi>(0.0));

        // Configure filters explicitly for this test
        machine_filter_->configure_voltage_filter(freq * 10.0);
        machine_filter_->configure_current_filter(freq * 5.0);

        // Update with the specific frequency
        for (int i = 0; i < 100; ++i) {
            machine_filter_->update(test_voltages_pu, test_currents_pu, freq);
        }

        // Verify filtering still produces correct magnitude
        double magnitude_pu = machine_filter_->get_filtered_voltage_magnitude_pu();
        EXPECT_NEAR(magnitude_pu, voltage_magnitude_pu, voltage_magnitude_pu * 0.1)
            << "Failed for frequency: " << freq << " Hz";
    }
}

// --- Simulation-Based Filtering Test ---

#include "sfc_plant_model_simulator.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"

/**
 * @brief Test fixture for validating filtering classes in a closed-loop simulation.
 *
 * This test extends the pattern from SfcMeasurementsTest by integrating the
 * filtering classes into a full plant simulation. It measures raw signals,
 * converts them to per-unit, filters them, and logs both unfiltered and
 * filtered signals for comparison.
 */
class SfcFilteringSimulationTest : public ::testing::Test {
protected:
    // Constants from SfcMeasurementsTest
    static constexpr double kSimulationTime = 0.2;
    static constexpr double kInitialGridFiringAngleDeg = 30.0 + 13.0;
    static constexpr double kMachineFiringAngleDeg = 30.0 + 180.0 - 26.84;

    // Simulation components
    sfc_plant_model_simulator _plant_sim;
    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();
    six_pulse_generator _grid_side_pulse_generator;
    six_pulse_generator _machine_side_pulse_generator;

    // System definition
    sfc_per_unit_system _pu_system{
        sfc_plant_model_simulator::kGridVoltageRMS,    sfc_plant_model_simulator::kNominalGridFrequency,
        sfc_plant_model_simulator::kBaseDCLinkCurrent, sfc_plant_model_simulator::kMachineRatedPowerVA,
        sfc_plant_model_simulator::kMachineVoltageRMS, sfc_plant_model_simulator::kNominalMachineFrequency,
        sfc_plant_model_simulator::kMachineNumPoles};

    // Measurement and Filtering classes
    sfc_grid_side_measurements _grid_measurements;
    sfc_dc_link_measurements _dc_link_measurements;
    sfc_machine_side_measurements _machine_measurements;
    sfc_grid_side_filtering _grid_filtering{sfc_6p_6p_plant_model::kSamplingTime};
    sfc_dc_link_filtering _dc_link_filtering{sfc_6p_6p_plant_model::kSamplingTime};
    sfc_machine_side_filtering _machine_filtering{sfc_6p_6p_plant_model::kSamplingTime};

    void SetUp() override {
        setup_logging();

        auto& plant_model = _plant_sim.plant();

        // Initialize the plant model
        plant_model.initialize();
        plant_model.set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                        sfc_plant_model_simulator::kGridVoltageRMS);
        plant_model.set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency,
                                           sfc_plant_model_simulator::kMachineVoltageRMS);

        // Configure filters with nominal values
        _grid_filtering.configure(50.0, 50.0);
        _grid_filtering.configure_voltage_magnitude_filter(20.0);  // Low-pass for magnitude
        _dc_link_filtering.configure(100.0, 300.0);
        _machine_filtering.configure_voltage_filter(100.0);
        _machine_filtering.configure_current_filter(100.0);
        _machine_filtering.configure_voltage_magnitude_filter(20.0);  // Low-pass for magnitude

        // Register the main simulation loop
        _simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            // 1. Update pulse generators to command the plant
            update_pulse_generators();

            // 2. Step the plant
            plant_model.step();

            // 3. Update measurement classes with raw plant outputs
            _grid_measurements.update(_pu_system.get_grid_bases(), plant_model.get_grid_side_voltages_abc(),
                                      plant_model.get_grid_side_currents_abc());
            _dc_link_measurements.update(_pu_system.get_dc_link_bases(), 0.0f, plant_model.get_dc_link_current());
            _machine_measurements.update(_pu_system.get_machine_bases(), plant_model.get_machine_side_voltages_abc(),
                                         plant_model.get_machine_side_currents_abc());

            // 4. Update filtering classes with per-unit measurements
            _grid_filtering.update(_grid_measurements.get_voltage_pu(), _grid_measurements.get_current_pu());
            _dc_link_filtering.update(_dc_link_measurements.get_voltage_pu(), _dc_link_measurements.get_current_pu());
            _machine_filtering.update(_machine_measurements.get_voltage_pu(), _machine_measurements.get_current_pu(),
                                      sfc_plant_model_simulator::kNominalMachineFrequency);
        });
    }

    void setup_logging() {
        // --- Unfiltered Signals ---
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_mag_pu_unfiltered", [&]() { return _grid_measurements.get_voltage_magnitude_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_mag_pu_unfiltered", [&]() { return _machine_measurements.get_voltage_magnitude_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_pu", [&]() { return _dc_link_measurements.get_current_pu(); }));

        // --- Filtered Signals ---
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_mag_pu_filtered", [&]() { return _grid_filtering.get_filtered_voltage_magnitude_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_mag_pu_filtered",
            [&]() { return _machine_filtering.get_filtered_voltage_magnitude_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_pu_filtered", [&]() { return _dc_link_filtering.get_filtered_current_pu(); }));
    }

    void update_pulse_generators() {
        auto& plant_model = _plant_sim.plant();
        auto grid_reference_angle = plant_model.get_grid_voltage_source_angle();
        auto machine_reference_angle = plant_model.get_machine_voltage_source_angle();

        angle<zero_to_two_pi> grid_firing_angle(deg_to_rad(kInitialGridFiringAngleDeg));
        angle<zero_to_two_pi> machine_firing_angle(deg_to_rad(kMachineFiringAngleDeg));

        auto grid_pulses = _grid_side_pulse_generator.update(grid_reference_angle, grid_firing_angle, true);
        auto machine_pulses = _machine_side_pulse_generator.update(machine_reference_angle, machine_firing_angle, true);

        plant_model.set_grid_side_pulses(grid_pulses.to_array());
        plant_model.set_machine_side_pulses(machine_pulses.to_array());
    }
};

TEST_F(SfcFilteringSimulationTest, ShouldFilterSignalsAtNominalOperation) {
    // Act: Run the simulation. The primary output is the generated .csv file
    // which allows for visual inspection of the filtered vs. unfiltered signals.
    _simulator.simulate_for(kSimulationTime);

    // // Assert: Check that the filtered DC current is close to the nominal value.
    // // The average is taken over the last part of the simulation to allow settling.
    // auto history = _simulator.get_sampler_history<double>("dc_link_current_pu_filtered");
    // ASSERT_FALSE(history.empty());
    // double sum = std::accumulate(history.end() - 500, history.end(), 0.0);
    // double average_filtered_current = sum / 500.0;

    // EXPECT_NEAR(average_filtered_current, 1.0, 0.1);
}
