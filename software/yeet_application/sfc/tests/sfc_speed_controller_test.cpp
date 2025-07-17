#include "sfc/sfc_speed_controller.hpp"

#include <gtest/gtest.h>

#include <numeric>

#include "yeet_control/mechanical_load.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::control;

/**
 * @brief Test fixture for validating the sfc_speed_controller in a closed-loop simulation.
 *
 * This test instantiates the speed controller and a mechanical load model to
 * verify that the controller can regulate the load's speed to a given reference.
 * It simulates an ideal inner torque loop by directly connecting the controller's
 * output to the mechanical load's input.
 */
class SfcSpeedControllerSimulationTest : public ::testing::Test {
protected:
    // Simulation and Control Constants
    static constexpr double kSamplingTime = 0.001;   // 1 ms
    static constexpr double kSimulationTime = 10.0;  // 5 seconds to allow settling
    static constexpr double kLoadTorquePu = 0.0;

    // Components
    sfc_speed_controller _speed_controller{kSamplingTime};
    mechanical_load _mechanical_load{kSamplingTime};
    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();

    //
    double speed_reference_pu = 0.0;

    void SetUp() override {
        // 1. Configure the mechanical load (plant)
        const double inertia_constant_seconds = 2.0;  // Inertia constant in seconds
        const double damping_factor_pu = 0.01;        // Damping factor in per-unit
        _mechanical_load.configure(inertia_constant_seconds, damping_factor_pu);
        _mechanical_load.reset();

        // 2. Configure the speed controller
        _speed_controller.set_pi_gains(10, 0.5);  // Kp, Ti - tuned for this plant
        _speed_controller.set_torque_limits_pu(-10.0, 10.0);
        _speed_controller.configure_speed_reference_filter(0.2);  // 0.2 Hz filter for smooth ramp
        _speed_controller.reset();

        // 3. Setup logging and the main simulation loop
        setup_logging();
        setup_simulation_loop();
    }

    void setup_simulation_loop() {
        _simulator.register_lambda_runner(kSamplingTime, [&]() {
            // Get measurement from the plant
            const double measured_speed_pu = _mechanical_load.get_speed_pu();

            // Update the controller
            const double torque_ref_pu = _speed_controller.update(speed_reference_pu, measured_speed_pu);

            // Apply controller output to the plant
            _mechanical_load.update(torque_ref_pu, kLoadTorquePu);
        });
    }

    void setup_logging() {
        _simulator.register_sampler_entry(
            yeet::testing::make_lambda_sampler<double>("speed_ref_pu", [&]() { return speed_reference_pu; }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "speed_ref_filtered_pu", [&]() { return _speed_controller.get_filtered_speed_ref_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "speed_measured_pu", [&]() { return _mechanical_load.get_speed_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "torque_ref_pu", [&]() { return _speed_controller.get_torque_ref_pu(); }));
    }
};

TEST_F(SfcSpeedControllerSimulationTest, ShouldRegulateSpeedToReference) {
    EXPECT_NEAR(_mechanical_load.get_speed_pu(), speed_reference_pu, 0.01);

    speed_reference_pu = 1.0;
    _simulator.simulate_for(kSimulationTime);
    EXPECT_NEAR(_mechanical_load.get_speed_pu(), speed_reference_pu, 0.01);

    speed_reference_pu = 0.5;
    _simulator.simulate_for(kSimulationTime);
    EXPECT_NEAR(_mechanical_load.get_speed_pu(), speed_reference_pu, 0.01);
}