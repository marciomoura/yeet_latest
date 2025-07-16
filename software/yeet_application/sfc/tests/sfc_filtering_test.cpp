// /**
//  * @file sfc_filtering_test.cpp
//  * @brief Integration test for the SFC filtering components.
//  *
//  * This test validates the filtering of per-unit measurements. It runs a complete
//  * simulation where raw plant model data is first converted to the per-unit system
//  * and then processed by the `sfc_grid_side_filtering`, `sfc_machine_side_filtering`,
//  * and `sfc_dc_link_filtering` classes.
//  *
//  * The test logs both the unfiltered and filtered per-unit values for analysis,
//  * ensuring the filters operate correctly on per-unit signals.
//  */

// #include "sfc/sfc_filtering.hpp"

// #include <gtest/gtest.h>

// #include <cmath>
// #include <memory>

// #include "sfc/sfc_per_unit_converter.hpp"
// #include "sfc/sfc_per_unit_system.hpp"
// #include "sfc/sfc_plant_model_simulator.hpp"
// #include "yeet_control/constants.hpp"
// #include "yeet_control/six_pulse_generator.hpp"
// #include "yeet_testing/simulator_gtest.hpp"

// namespace yeet::application::sfc {

// using namespace yeet::control;

// class SfcFilteringTest : public ::testing::Test {
// protected:
//     // Simulation and plant parameters
//     static constexpr double kSimulationTime = 0.2;
//     static constexpr double kGridFrequency = 50.0;
//     static constexpr double kMachineFrequency = 30.0;
//     static constexpr double kGridVoltageRMS = 400.0;
//     static constexpr double kMachineVoltageRMS = 400.0;
//     static constexpr double kGridFiringAngleDeg = 45.0;
//     static constexpr double kMachineFiringAngleDeg = 150.0;
//     static constexpr double kRatedPowerVA = 2e6;
//     static constexpr double kSamplingTime = sfc_6p_6p_plant_model::kSamplingTime;

//     // Test infrastructure
//     sfc_plant_model_simulator plant_simulator_;
//     six_pulse_generator grid_side_pulse_gen_;
//     six_pulse_generator machine_side_pulse_gen_;
//     yeet::testing::simulator simulator_ = yeet::testing::make_gtest_simulator();

//     // Per-unit system and converter
//     std::unique_ptr<sfc_per_unit_system> pu_system_;
//     std::unique_ptr<sfc_per_unit_converter> pu_converter_;

//     // Filtering components
//     std::unique_ptr<sfc_grid_side_filtering> grid_filter_;
//     std::unique_ptr<sfc_machine_side_filtering> machine_filter_;
//     std::unique_ptr<sfc_dc_link_filtering> dc_link_filter_;

//     void SetUp() override {
//         // 1. Initialize per-unit system and converter
//         pu_system_ = std::make_unique<sfc_per_unit_system>(kRatedPowerVA, kMachineVoltageRMS, kMachineFrequency);
//         pu_converter_ = std::make_unique<sfc_per_unit_converter>(*pu_system_);

//         // 2. Initialize filtering components
//         grid_filter_ = std::make_unique<sfc_grid_side_filtering>(kSamplingTime);
//         machine_filter_ = std::make_unique<sfc_machine_side_filtering>(kSamplingTime);
//         dc_link_filter_ = std::make_unique<sfc_dc_link_filtering>(kSamplingTime);

//         // Configure filters with appropriate cutoff frequencies
//         // Cutoffs are set higher than the fundamental frequencies to preserve the signal
//         // while attenuating high-frequency noise.
//         grid_filter_->configure(kGridFrequency * 10, kGridFrequency * 10);
//         machine_filter_->configure_voltage_filter(kMachineFrequency * 10);
//         machine_filter_->configure_current_filter(kMachineFrequency * 10);
//         // DC link ripple is 6 * fundamental frequency
//         dc_link_filter_->configure(kGridFrequency * 10, kGridFrequency * 6);

//         // 3. Configure the plant model
//         auto& plant_model = plant_simulator_.plant();
//         plant_model.initialize();
//         plant_model.set_grid_parameters(kGridFrequency, kGridVoltageRMS);
//         plant_model.set_machine_parameters(kMachineFrequency, kMachineVoltageRMS);

//         // 4. Setup the main simulation loop
//         simulator_.register_lambda_runner(kSamplingTime, [&]() {
//             // Step 1: Generate pulses and step the plant model
//             update_pulse_generators();
//             plant_simulator_.plant().step();

//             // Step 2: Get raw SI measurements and convert to per-unit
//             update_per_unit_converter();

//             // Step 3: Filter the per-unit measurements
//             update_filters();
//         });

//         // 5. Setup logging for both unfiltered and filtered per-unit values
//         setup_logging();
//     }

//     void update_pulse_generators() {
//         auto& plant_model = plant_simulator_.plant();
//         auto grid_reference_angle = plant_model.get_grid_voltage_source_angle();
//         auto machine_reference_angle = plant_model.get_machine_voltage_source_angle();

//         angle<zero_to_two_pi> grid_firing_angle(deg_to_rad(kGridFiringAngleDeg));
//         angle<zero_to_two_pi> machine_firing_angle(deg_to_rad(kMachineFiringAngleDeg));

//         auto grid_pulses = grid_side_pulse_gen_.update(grid_reference_angle, grid_firing_angle, true);
//         auto machine_pulses = machine_side_pulse_gen_.update(machine_reference_angle, machine_firing_angle, true);

//         plant_model.set_grid_side_pulses(grid_pulses.to_array());
//         plant_model.set_machine_side_pulses(machine_pulses.to_array());
//     }

//     void update_per_unit_converter() {
//         auto& plant = plant_simulator_.plant();
//         pu_converter_->update(plant.get_grid_side_voltages_abc(), plant.get_grid_side_currents_abc(),
//                               plant.get_machine_side_voltages_abc(), plant.get_machine_side_currents_abc(),
//                               plant.get_dc_link_current());
//     }

//     void update_filters() {
//         // Filter grid-side per-unit values
//         grid_filter_->update(pu_converter_->get_grid_voltages_pu(), pu_converter_->get_grid_currents_pu());

//         // Filter machine-side per-unit values
//         machine_filter_->update(pu_converter_->get_machine_voltages_pu(), pu_converter_->get_machine_currents_pu(),
//                                 kMachineFrequency);

//         // Filter DC-link per-unit values (Note: DC link filter expects SI, so we pass raw values)
//         // This is a placeholder as the DC filter is designed for SI. For a pure PU pipeline,
//         // the DC filter would need a PU-based configuration.
//         dc_link_filter_->update(0.0, plant_simulator_.plant().get_dc_link_current());
//     }

//     void setup_logging() {
//         // Log unfiltered and filtered grid-side PU values
//         log_abc_signals(
//             "grid_voltage_pu", [&]() { return pu_converter_->get_grid_voltages_pu(); },
//             [&]() { return grid_filter_->get_filtered_voltages(); });
//         log_abc_signals(
//             "grid_current_pu", [&]() { return pu_converter_->get_grid_currents_pu(); },
//             [&]() { return grid_filter_->get_filtered_currents(); });

//         // Log unfiltered and filtered machine-side PU values
//         log_abc_signals(
//             "machine_voltage_pu", [&]() { return pu_converter_->get_machine_voltages_pu(); },
//             [&]() { return machine_filter_->get_filtered_voltages(); });
//         log_abc_signals(
//             "machine_current_pu", [&]() { return pu_converter_->get_machine_currents_pu(); },
//             [&]() { return machine_filter_->get_filtered_currents(); });

//         // Log unfiltered and filtered DC-link current
//         // simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
//         //     "dc_link_current_pu_unfiltered", [&]() { return pu_converter_->get_dc_link_current_pu(); }));
//         // // Note: DC filter output is SI, so we convert it to PU for logging comparison
//         // simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
//         //     "dc_link_current_pu_filtered",
//         //     [&]() { return dc_link_filter_->get_filtered_current() / pu_system_->get_base_dc_current(); }));
//     }

//     void log_abc_signals(const std::string& base_name, std::function<control::abc<double>()> unfiltered_getter,
//                          std::function<control::abc<double>()> filtered_getter) {
//         // Unfiltered signals
//         simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
//             base_name + "_a_unfiltered", [=]() { return unfiltered_getter().a(); }));
//         simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
//             base_name + "_b_unfiltered", [=]() { return unfiltered_getter().b(); }));
//         simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
//             base_name + "_c_unfiltered", [=]() { return unfiltered_getter().c(); }));

//         // Filtered signals
//         simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
//             base_name + "_a_filtered", [=]() { return filtered_getter().a(); }));
//         simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
//             base_name + "_b_filtered", [=]() { return filtered_getter().b(); }));
//         simulator_.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
//             base_name + "_c_filtered", [=]() { return filtered_getter().c(); }));
//     }
// };

// TEST_F(SfcFilteringTest, FilterPerformanceAtOperatingPoint) {
//     // Run the simulation to generate data
//     simulator_.simulate_for(kSimulationTime);

//     // Get final values for verification
//     auto final_unfiltered_current = pu_converter_->get_grid_currents_pu();
//     auto final_filtered_current = grid_filter_->get_filtered_currents();

//     // --- Verification ---
//     // 1. Check that all final values are finite
//     EXPECT_TRUE(std::isfinite(final_unfiltered_current.a()));
//     EXPECT_TRUE(std::isfinite(final_filtered_current.a()));

//     // 2. Check that the filter has altered the signal.
//     // The instantaneous values might be the same if crossing zero, but the magnitude
//     // of the vector should be different if the filter is active.
//     auto unfiltered_mag_sq = final_unfiltered_current.a() * final_unfiltered_current.a() +
//                              final_unfiltered_current.b() * final_unfiltered_current.b() +
//                              final_unfiltered_current.c() * final_unfiltered_current.c();

//     auto filtered_mag_sq = final_filtered_current.a() * final_filtered_current.a() +
//                            final_filtered_current.b() * final_filtered_current.b() +
//                            final_filtered_current.c() * final_filtered_current.c();

//     // Due to phase shift and magnitude change, the squared magnitudes should differ.
//     // Using a small tolerance to avoid floating point issues.
//     EXPECT_NE(unfiltered_mag_sq, filtered_mag_sq);
// }

// }  // namespace yeet::application::sfc