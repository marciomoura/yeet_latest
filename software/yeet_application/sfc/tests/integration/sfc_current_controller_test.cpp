#include <gtest/gtest.h>

#include "sfc/sfc_simulators.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

/**
 * @brief Test fixture for validating the sfc_current_controller in a closed-loop simulation.
 *
 * This test uses the new simulator builder pattern with observer-based pulse generation
 * to create a realistic closed-loop environment for testing the current controller.
 */
class SfcCurrentControllerSimulationTest : public ::testing::Test {
protected:
    // Simulation signals
    current_pu_t dc_link_current_ref_pu = current_pu_t(0.0);  // Reference current in per-unit

    // Simulation components using the new pattern
    sfc_simulation_components components;
    sfc_pulse_generator_conduit pulse_generator_conduit;
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();
    void SetUp() override {
        // Configure plant model parameters
        components.plant_model_simulator.plant().set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                                                     sfc_plant_model_simulator::kNominalGridVoltageRMS);
        components.plant_model_simulator.plant().set_machine_parameters(
            sfc_plant_model_simulator::kNominalMachineFrequency, sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        // Start with pulse generation disabled to allow system to stabilize
        pulse_generator_conduit.grid_firing_angle_rad =
            deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg);
        pulse_generator_conduit.machine_firing_angle_rad =
            deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg);

        // Use the builder to configure a simulation with current controller
        sfc_simulator_builder(components)
            .with_measurements_and_filtering()
            .with_observers()
            .with_observer_based_pulse_generation(pulse_generator_conduit)
            .with_current_controller(pulse_generator_conduit)
            .configure(simulator);

        // Enable pulse generation once the grid observer has locked
        simulator.register_lambda_runner(components.kControlSamplingTime, [&]() {
            // pulse_generator_conduit.enable = components.observers.grid_side_observer.is_locked();
            pulse_generator_conduit.current_ref_pu = dc_link_current_ref_pu;
            pulse_generator_conduit.measured_current_pu =
                components.measurement_and_filtering.get_dc_link_filtering().get_filtered_current_pu();
            pulse_generator_conduit.enable = components.observers.grid_side_observer.is_locked();
        });

        setup_logging();
    }

    void setup_logging() {
        auto& plant_model = components.plant_model_simulator.plant();
        auto& measurement_and_filtering = components.measurement_and_filtering;
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc_link_current_ref_limited_pu",
            [&]() { return components.current_controller.get_controller().get_current_reference_limited_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc_link_current_ref_pu", [&]() { return dc_link_current_ref_pu; }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc_link_current_measured_pu",
            [&]() { return measurement_and_filtering.get_dc_link_measurements().get_current_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc_link_current_filtered_pu",
            [&]() { return measurement_and_filtering.get_dc_link_filtering().get_filtered_current_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc_link_current_heavy_filtered_pu",
            [&]() { return measurement_and_filtering.get_dc_link_filtering().get_heavy_filtered_current_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc_link_current_error_pu", [&]() { return get_dc_link_current_error_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "grid_firing_angle_pu", [&]() { return pulse_generator_conduit.grid_firing_angle_rad.get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "machine_firing_angle_pu", [&]() { return pulse_generator_conduit.machine_firing_angle_rad.get_pu(); }));
    }

    current_pu_t get_dc_link_current_error_pu() const {
        return components.current_controller.get_controller().get_current_reference_limited_pu() -
               components.measurement_and_filtering.get_dc_link_filtering().get_heavy_filtered_current_pu();
    }
};

// Test regulation of DC-link current to a reference value
TEST_F(SfcCurrentControllerSimulationTest, TestCurrentRegulation) {
    current_pu_t current_error_max_value{0.05f};
    current_pu_t current_error_min_value{-0.05f};

    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::testing::checker_activation_time_range{1.0, 1.5}));

    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "current error after disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::testing::checker_activation_time_range{2.0}));

    // Set high slew-rate to not suffer from any limitation

    // Simulate until observer is locked and system can release pulses.
    dc_link_current_ref_pu = current_pu_t(0.0);
    bool success = simulator.simulate_for(0.5);

    // Set the current reference to 1.0 per-unit, and test for 1.0 second
    dc_link_current_ref_pu = current_pu_t(1.0);
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

TEST_F(SfcCurrentControllerSimulationTest, TestDisturbanceRejection) {
    current_pu_t current_error_max_value{0.05f};
    current_pu_t current_error_min_value{-0.05f};

    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::testing::checker_activation_time_range{1.0, 1.5}));

    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "current error after disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::testing::checker_activation_time_range{2.0}));

    // Set high slew-rate to not suffer from any limitation

    // Simulate until observer is locked and system can release pulses.
    bool success = simulator.simulate_for(0.5);

    // Set the current reference to 1.0 per-unit, and test for 1.0 second
    dc_link_current_ref_pu = current_pu_t(1.0);
    success &= simulator.simulate_for(1.0);

    // Add a little bit of disturbance in the plant model to test the controller's response
    components.plant_model_simulator.plant().set_machine_parameters(
        frequency_t{sfc_plant_model_simulator::kNominalMachineFrequency},
        0.95f * voltage_t{sfc_plant_model_simulator::kNominalMachineVoltageRMS});

    pulse_generator_conduit.machine_firing_angle_rad =
        deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg - 10.0f);

    // Execute the test for another seconds to allow the controller to respond
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

TEST_F(SfcCurrentControllerSimulationTest, TestCurrentLimiters) {
    const current_pu_t current_error_max_value{0.05f};
    const current_pu_t current_error_min_value{-0.05f};

    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::testing::checker_activation_time_range{1.0, 1.5}));

    const current_pu_t current_limit{0.8};
    const current_pu_t current_limit_max_value{current_limit + 0.05f};
    const current_pu_t current_limit_min_value{current_limit - 0.05f};

    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "current limited",
        [&]() -> current_pu_t {
            return components.measurement_and_filtering.get_dc_link_filtering().get_heavy_filtered_current_pu();
        },
        current_limit_min_value, current_limit_max_value, yeet::testing::checker_activation_time_range{1.7}));

    // Set high slew-rate to not suffer from any limitation

    // Simulate until observer is locked and system can release pulses.
    bool success = simulator.simulate_for(0.5);

    // Set the current reference to 1.0 per-unit, and test for 1.0 second
    dc_link_current_ref_pu = current_pu_t(1.0);
    success &= simulator.simulate_for(1.1);

    // Set current limits to test the controller's response
    components.current_controller.configure_current_limits_pu(current_pu_t{0.0}, current_pu_t{0.8});

    // Execute the test for another seconds to allow the controller to respond
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

TEST_F(SfcCurrentControllerSimulationTest, TestCurrentSlewRateLimiters) {
    const real_t slew_rate_pu_per_sec = 1.0;

    components.current_controller.current_controller.enable_current_reference_slew_rate(true);
    components.current_controller.current_controller.configure_current_reference_slew_rate(slew_rate_pu_per_sec);

    // Check that the limited reference is at 0.5 pu halfway through the ramp up
    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "slew_rate_ramp_up_midpoint",
        [&]() { return components.current_controller.get_controller().get_current_reference_limited_pu(); },
        current_pu_t(0.49), current_pu_t(0.51),
        yeet::testing::checker_activation_exact_time{1.0, components.kControlSamplingTime.value()}));

    // Check that the limited reference has reached 1.0 pu at the end of the ramp up
    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "slew_rate_ramp_up_endpoint",
        [&]() { return components.current_controller.get_controller().get_current_reference_limited_pu(); },
        current_pu_t(0.99), current_pu_t(1.01),
        yeet::testing::checker_activation_exact_time{1.5, components.kControlSamplingTime.value()}));

    // Check that the limited reference is at 0.75 pu halfway through the ramp down
    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "slew_rate_ramp_down_midpoint",
        [&]() { return components.current_controller.get_controller().get_current_reference_limited_pu(); },
        current_pu_t(0.74), current_pu_t(0.76),
        yeet::testing::checker_activation_exact_time{2.25, components.kControlSamplingTime.value()}));

    // Check that the limited reference is at 0.5 pu halfway through the ramp down
    simulator.register_checker(yeet::testing::make_range_checker<current_pu_t>(
        "slew_rate_ramp_down_endpoint",
        [&]() { return components.current_controller.get_controller().get_current_reference_limited_pu(); },
        current_pu_t(0.49), current_pu_t(0.51),
        yeet::testing::checker_activation_exact_time{2.5, components.kControlSamplingTime.value()}));

    // Simulate until observer is locked and system can release pulses.
    dc_link_current_ref_pu = current_pu_t(0.0);
    bool success = simulator.simulate_for(0.5);
    ASSERT_TRUE(success) << "Initial stabilization phase failed.";

    // Set the current reference to 1.0 per-unit.
    // With a slew rate of 1.0 pu/s, it should take 1.0s to reach the target.
    dc_link_current_ref_pu = current_pu_t(1.0);
    success &= simulator.simulate_for(1.5);  // Simulate long enough to hit both ramp-up checkers
    ASSERT_TRUE(success) << "Ramp-up phase failed.";

    // Set the current reference back to 0.0 per-unit.
    // It should take 1.0s to ramp down.
    dc_link_current_ref_pu = current_pu_t(0.5);
    success &= simulator.simulate_for(1.5);  // Simulate long enough to hit ramp-down checker

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

TEST_F(SfcCurrentControllerSimulationTest, TestMinimumFiringAngle) {
    const angle_t min_firing_angle{deg_to_rad(10.0f)};

    simulator.register_checker(yeet::testing::make_min_checker<angle_t>(
        "check min angle",
        [&]() -> angle_t { return angle_t{pulse_generator_conduit.grid_firing_angle_rad.get_radians()}; },
        angle_t{min_firing_angle}, yeet::testing::checker_activation_time_range{1.6}));

    // Simulate until observer is locked and system can release pulses.
    bool success = simulator.simulate_for(0.5);

    // Set the current reference to 1.0 per-unit, and test for 1.0 second
    // The slew-rate should bring the current to 1.0 per-unit in 1.0 second
    dc_link_current_ref_pu = current_pu_t(1.0);
    success &= simulator.simulate_for(1.0);

    // At 1.0 per-unit current with the machine-side firing at nominal angle, the grid-side firing angle should be at
    // its minimum Hence, by limiting the firing angle we should see some reaction at the amount of current being
    // controlled.
    components.current_controller.configure_firing_angle_limits_radians(angle_t{min_firing_angle}, angle_t{pi / 2.0});
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

TEST_F(SfcCurrentControllerSimulationTest, TestMaximumFiringAngle) {
    const angle_t max_firing_angle{deg_to_rad(15.0f)};

    simulator.register_checker(yeet::testing::make_max_checker<angle_t>(
        "check min angle",
        [&]() -> angle_t { return angle_t{pulse_generator_conduit.grid_firing_angle_rad.get_radians()}; },
        angle_t{max_firing_angle}, yeet::testing::checker_activation_time_range{1.6}));

    // Simulate until observer is locked and system can release pulses.
    bool success = simulator.simulate_for(0.5);

    // Set the current reference to 1.0 per-unit, and test for 1.0 second
    // The slew-rate should bring the current to 1.0 per-unit in 1.0 second
    dc_link_current_ref_pu = current_pu_t(1.0);
    success &= simulator.simulate_for(1.0);

    // At 1.0 per-unit current with the machine-side firing at nominal angle, the grid-side firing angle should be at
    // its minimum Hence, by limiting the firing angle we should see some reaction at the amount of current being
    // controlled.
    components.current_controller.configure_firing_angle_limits_radians(angle_t{}, angle_t{max_firing_angle});
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}