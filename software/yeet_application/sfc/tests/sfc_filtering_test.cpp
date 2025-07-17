#include "sfc/sfc_filtering.hpp"

#include <gtest/gtest.h>

#include "sfc/sfc_measurements.hpp"
#include "sfc_plant_model_simulator.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::control;

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

    // auto data = _simulator.get_sampler().get_sampled_data("dc_link_current_pu_filtered");
    // // Assert: Check that the filtered DC current is close to the nominal value.
    // // The average is taken over the last part of the simulation to allow settling.
    // auto history = _simulator.get_sampler_history<double>("dc_link_current_pu_filtered");
    // ASSERT_FALSE(history.empty());
    // double sum = std::accumulate(history.end() - 500, history.end(), 0.0);
    // double average_filtered_current = sum / 500.0;

    // EXPECT_NEAR(average_filtered_current, 1.0, 0.1);
}
