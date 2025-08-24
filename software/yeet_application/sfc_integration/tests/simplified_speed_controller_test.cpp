#include <gtest/gtest.h>

#include "sfc_integration/plant_simplified_speed_control.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcSpeedControllerTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(200e-6);

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Configure plant model
        configure_plant_model();

        // Initialize the system - allocates plant model and all components
        initialize();

        // ==================== Logger Registration ====================
        register_loggers();
    }

    void register_loggers()
    {
        register_complete_logger(simulator, system.speed_controller);
        register_complete_logger(simulator, system.current_controller);
        register_complete_logger(simulator, system.machine_side_controller);
        register_complete_logger(simulator, system.dc_link_measurements);
    }

    void configure_plant_model()
    {
        // Configure plant and speed controller for variable speed operation
        test::variable_speed_config config;
        config.initial_speed = speed_pu_t{0.9f};  // Custom initial speed for this test
        config.voltage_at_base_speed = 0.8f;      // Custom voltage relationship

        test::configure_variable_speed_plant(system.plant, config);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // System under test
    yeet::application::sfc::integration::sfc_speed_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test regulation of machine speed to a reference value
TEST_F(SfcSpeedControllerTest, TestSpeedRegulation)
{
    const speed_pu_t speed_error_max_value{0.05f};
    const speed_pu_t speed_error_min_value{-0.05f};

    // Check speed error during steady state before disturbance
    simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error pre disturbance",
        [&]() -> speed_pu_t { return system.speed_controller.get_outputs().speed_error.read(); }, speed_error_min_value,
        speed_error_max_value, yeet::checker_activation_time_range{4.0, 8.0}));

    // Check speed error after disturbance
    simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error after disturbance",
        [&]() -> speed_pu_t { return system.speed_controller.get_outputs().speed_error.read(); }, speed_error_min_value,
        speed_error_max_value, yeet::checker_activation_time_range{15.0}));

    // // ==================== Test Configuration ====================

    system.grid_side_pulse_generator.get_parameters().enable.write(false);
    system.machine_side_pulse_generator.get_parameters().enable.write(false);
    simulator.simulate_for(1.0);

    // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
    // The pulse generators are now controlled through signal flow, not parameters
    system.grid_side_pulse_generator.get_parameters().enable.write(true);
    system.machine_side_pulse_generator.get_parameters().enable.write(true);

    // Enable speed control mode
    system.enable_speed_control = true;

    // Configure load torque
    system.load_torque_pu = torque_pu_t(0.3);  // No load initially

    // ==================== Test Execution ====================

    // Simulate until system can stabilize
    system.speed_reference_pu = speed_pu_t(0.9);
    bool success = simulator.simulate_for(10.0);

    // Set the speed reference to 0.5 per-unit, and test for 1.0 second
    system.speed_reference_pu = speed_pu_t(1.0);
    success &= simulator.simulate_for(10.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}
