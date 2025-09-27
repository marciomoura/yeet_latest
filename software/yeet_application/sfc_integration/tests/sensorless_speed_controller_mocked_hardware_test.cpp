// Sensorless speed controller (cascaded) test using mocked hardware (ADC chain)
#include <gtest/gtest.h>

#include "sfc_integration/plant_sensorless_speed_control_closed_loop_mocked_hardware.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcSensorlessSpeedControllerMockedHardwareTest : public ::testing::Test {
public:
    void SetUp() override
    {
        simulator.configure_sampler_sampling_rate(ms(1.0));
        system.register_applications(simulator);
        system.initialize();

        // Register relevant loggers (focus on cascaded loops + observers + dc link)
        register_complete_logger(simulator, system.speed_controller);
        register_complete_logger(simulator, system.current_controller);
        register_complete_logger(simulator, system.dc_link_measurements);
        register_complete_logger(simulator, system.grid_side_observer);
        register_complete_logger(simulator, system.machine_side_observer);
    }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // System under test (mocked hardware builder)
    sfc_sensorless_speed_controller_mocked_hardware_simulation_builder system;
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Basic regulation test: track a speed step and hold within band using sensorless estimates
TEST_F(SfcSensorlessSpeedControllerMockedHardwareTest, TestSpeedRegulation)
{
    // Configure plant for variable speed operation with custom voltage relationship
    yeet::application::sfc::integration::test::variable_speed_config config;
    config.voltage_at_base_speed = 0.9f;  // Custom voltage relationship for this test

    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // Configure speed controller rate limit (not in plant config)
    system.speed_controller.get_parameters().speed_reference_rate_limit.write(2.0);

    const speed_pu_t speed_error_max_value{0.05f};
    const speed_pu_t speed_error_min_value{-0.05f};

    // Check speed error after disturbance
    auto user_activation = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error", [&]() -> speed_pu_t { return system.speed_controller.get_outputs().speed_error.read(); },
        speed_error_min_value, speed_error_max_value, yeet::checker_activation_always{}));

    user_activation->deactivate();  // Deactivate initially to avoid false positives

    // ==================== Test Configuration ====================
    // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
    // The pulse generators are now controlled through signal flow, not parameters
    // grid_side_pulse_generator_parameters()->enable = false;
    // machine_side_pulse_generator_parameters()->enable = false;
    system.enable_speed_control = false;

    simulator.simulate_for(1.0);

    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
    // The pulse generators are now controlled through signal flow, not parameters
    // grid_side_pulse_generator_parameters()->enable = true;
    // machine_side_pulse_generator_parameters()->enable = true;

    // Enable speed control mode
    system.enable_speed_control = true;

    // Configure load torque
    system.load_torque_pu = torque_pu_t(0.3);  // No load initially

    // ==================== Test Execution ====================

    // Simulate until system can stabilize
    system.speed_reference_pu = speed_pu_t(1.0);
    bool success = simulator.simulate_for(5.0);

    user_activation->activate();
    success &= simulator.simulate_for(5.0);
    user_activation->deactivate();

    // // Set the speed reference to 0.5 per-unit, and test for 1.0 second
    system.speed_reference_pu = speed_pu_t(0.95);
    success &= simulator.simulate_for(5.0);

    user_activation->activate();
    success &= simulator.simulate_for(5.0);
    user_activation->deactivate();

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}
