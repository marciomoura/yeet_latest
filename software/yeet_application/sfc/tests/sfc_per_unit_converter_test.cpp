/**
 * @file sfc_per_unit_converter_test.cpp
 * @brief Integration test for the sfc_per_unit_converter class.
 *
 * This test validates the per-unit conversion of raw plant model measurements
 * by running a complete simulation and logging the per-unit outputs.
 */

#include "sfc/sfc_per_unit_converter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "sfc/sfc_per_unit_system.hpp"
#include "sfc/sfc_plant_model_simulator.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace yeet::application::sfc {

using namespace yeet::control;

class SfcPerUnitConverterTest : public ::testing::Test {
protected:
    // Simulation and plant parameters
    static constexpr double kSimulationTime = 0.2;           // 200ms simulation
    static constexpr double kGridFrequency = 50.0;           // 50 Hz grid frequency
    static constexpr double kMachineFrequency = 50.0;        // 30 Hz machine frequency
    static constexpr double kGridVoltageRMS = 2000.0;        // 2000V RMS line voltage
    static constexpr double kMachineVoltageRMS = 2000.0;     // 2000V RMS line voltage
    static constexpr double kGridFiringAngleDeg = 60.0;      // Grid side firing angle
    static constexpr double kMachineFiringAngleDeg = 165.0;  // Machine side firing angle
    static constexpr double kRatedPowerVA = 2e6;             // 2 MVA rated power

    // Test infrastructure
    sfc_plant_model_simulator plant_simulator_;
    six_pulse_generator grid_side_pulse_gen_;
    six_pulse_generator machine_side_pulse_gen_;
    yeet::testing::simulator simulator_ = yeet::testing::make_gtest_simulator();

    // Per-unit system and converter
    std::unique_ptr<sfc_per_unit_system> pu_system_;
    std::unique_ptr<sfc_per_unit_converter> pu_converter_;

    void SetUp() override {
        // 1. Initialize the per-unit system based on plant ratings
        pu_system_ = std::make_unique<sfc_per_unit_system>(kRatedPowerVA, kMachineVoltageRMS, kMachineFrequency);

        // 2. Initialize the per-unit converter with the system definition
        pu_converter_ = std::make_unique<sfc_per_unit_converter>(*pu_system_);

        // 3. Configure the plant model
        auto& plant_model = plant_simulator_.plant();
        plant_model.initialize();
        plant_model.set_grid_parameters(kGridFrequency, kGridVoltageRMS);
        plant_model.set_machine_parameters(kMachineFrequency, kMachineVoltageRMS);

        // 4. Setup the main simulation loop
        simulator_.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            // Update pulse generators based on firing angles
            update_pulse_generators();

            // Step the plant model
            plant_simulator_.plant().step();

            // Get raw measurements from the plant
            auto& plant = plant_simulator_.plant();
            auto grid_voltages_si = plant.get_grid_side_voltages_abc();
            auto grid_currents_si = plant.get_grid_side_currents_abc();
            auto machine_voltages_si = plant.get_machine_side_voltages_abc();
            auto machine_currents_si = plant.get_machine_side_currents_abc();
            auto dc_link_current_si = plant.get_dc_link_current();

            // Convert raw measurements to per-unit
            pu_converter_->update(grid_voltages_si, grid_currents_si, machine_voltages_si, machine_currents_si,
                                  dc_link_current_si);
        });

        // 5. Setup logging for per-unit values
        setup_per_unit_logging();
    }

    void TearDown() override { plant_simulator_.plant().terminate(); }

    void update_pulse_generators() {
        auto& plant_model = plant_simulator_.plant();
        auto grid_reference_angle = plant_model.get_grid_voltage_source_angle();
        auto machine_reference_angle = plant_model.get_machine_voltage_source_angle();

        angle<zero_to_two_pi> grid_firing_angle(deg_to_rad(kGridFiringAngleDeg));
        angle<zero_to_two_pi> machine_firing_angle(deg_to_rad(kMachineFiringAngleDeg));

        auto grid_pulses = grid_side_pulse_gen_.update(grid_reference_angle, grid_firing_angle, true);
        auto machine_pulses = machine_side_pulse_gen_.update(machine_reference_angle, machine_firing_angle, true);

        plant_model.set_grid_side_pulses(grid_pulses.to_array());
        plant_model.set_machine_side_pulses(machine_pulses.to_array());
    }

    void setup_per_unit_logging() {
        // Grid-side per-unit measurements (abc)
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_a_pu", [&]() -> double { return pu_converter_->get_grid_voltages_pu().a(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_b_pu", [&]() -> double { return pu_converter_->get_grid_voltages_pu().b(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_c_pu", [&]() -> double { return pu_converter_->get_grid_voltages_pu().c(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_a_pu", [&]() -> double { return pu_converter_->get_grid_currents_pu().a(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_b_pu", [&]() -> double { return pu_converter_->get_grid_currents_pu().b(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_c_pu", [&]() -> double { return pu_converter_->get_grid_currents_pu().c(); }));

        // Machine-side per-unit measurements (abc)
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_a_pu", [&]() -> double { return pu_converter_->get_machine_voltages_pu().a(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_b_pu", [&]() -> double { return pu_converter_->get_machine_voltages_pu().b(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_c_pu", [&]() -> double { return pu_converter_->get_machine_voltages_pu().c(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_a_pu", [&]() -> double { return pu_converter_->get_machine_currents_pu().a(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_b_pu", [&]() -> double { return pu_converter_->get_machine_currents_pu().b(); }));
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_c_pu", [&]() -> double { return pu_converter_->get_machine_currents_pu().c(); }));

        // DC-link per-unit measurements
        simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_pu", [&]() -> double { return pu_converter_->get_dc_link_current_pu(); }));
    }
};

TEST_F(SfcPerUnitConverterTest, PerUnitConversionAtOperatingPoint) {
    // Run the simulation
    simulator_.simulate_for(kSimulationTime);

    // Get final per-unit values
    auto final_grid_voltage_pu = pu_converter_->get_grid_voltages_pu();
    auto final_machine_current_pu = pu_converter_->get_machine_currents_pu();
    auto final_dc_link_current_pu = pu_converter_->get_dc_link_current_pu();

    // --- Verification ---
    // Check that final per-unit values are finite
    EXPECT_TRUE(std::isfinite(final_grid_voltage_pu.a()));
    EXPECT_TRUE(std::isfinite(final_grid_voltage_pu.b()));
    EXPECT_TRUE(std::isfinite(final_grid_voltage_pu.c()));
    EXPECT_TRUE(std::isfinite(final_machine_current_pu.a()));
    EXPECT_TRUE(std::isfinite(final_machine_current_pu.b()));
    EXPECT_TRUE(std::isfinite(final_machine_current_pu.c()));
    EXPECT_TRUE(std::isfinite(final_dc_link_current_pu));

    // Check that values are non-zero, indicating the system is running
    // A simple RMS-like check on the final snapshot of the three phases
    double grid_voltage_mag_sq = final_grid_voltage_pu.a() * final_grid_voltage_pu.a() +
                                 final_grid_voltage_pu.b() * final_grid_voltage_pu.b() +
                                 final_grid_voltage_pu.c() * final_grid_voltage_pu.c();
    double machine_current_mag_sq = final_machine_current_pu.a() * final_machine_current_pu.a() +
                                    final_machine_current_pu.b() * final_machine_current_pu.b() +
                                    final_machine_current_pu.c() * final_machine_current_pu.c();
    EXPECT_GT(grid_voltage_mag_sq, 0.1);
    EXPECT_GT(machine_current_mag_sq, 0.1);
}

}  // namespace yeet::application::sfc