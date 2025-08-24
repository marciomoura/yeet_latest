#include <gtest/gtest.h>

#include "sfc_integration/plant_sensorless_current_control_closed_loop.hpp"
#include "sfc_loggers.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcSensorlessCurrentControllerTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(200e-6);

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Initialize the system - allocates plant model and all components
        initialize();

        system.machine_side_controller.get_parameters().machine_side_commutation_reactance_pu =
            to_pu(system.plant.get_plant_model().get_electrical_parameters().machine_side_inductance,
                system.plant.get_plant_model().get_per_unit_system().get_machine_bases().base_inductance);

        // ==================== Logger Registration ====================
        register_complete_logger(simulator, system.current_controller);
        register_complete_logger(simulator, system.dc_link_measurements);
        register_complete_logger(simulator, system.grid_side_observer);

        // ==================== Test Configuration ====================
        // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
        // The pulse generators are now controlled through signal flow, not parameters
        system.grid_side_pulse_generator.get_parameters().enable.write(false);
        system.machine_side_pulse_generator.get_parameters().enable.write(false);
        simulator.simulate_for(1.0);

        // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
        // The pulse generators are now controlled through signal flow, not parameters
        system.grid_side_pulse_generator.get_parameters().enable.write(true);
        system.machine_side_pulse_generator.get_parameters().enable.write(true);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // System under test
    yeet::application::sfc::integration::sfc_sensorless_current_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test regulation of DC-link current to a reference value
TEST_F(SfcSensorlessCurrentControllerTest, TestCurrentRegulation)
{
    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.16f};
    current_pu_t current_error_min_value{-0.16f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance",
        [&]() -> current_pu_t { return system.current_controller.get_outputs().current_error.read(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{2.0, 2.9}));
    bool success = simulator.simulate_for(2.0);

    system.current_reference_pu = current_pu_t(1.0);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance",
        [&]() -> current_pu_t { return system.current_controller.get_outputs().current_error.read(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{3.5, 4.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

// Test regulation of DC-link current to a reference value
TEST_F(SfcSensorlessCurrentControllerTest, MachineFrequencyStep)
{
    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.15f};
    current_pu_t current_error_min_value{-0.15f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance",
        [&]() -> current_pu_t { return system.current_controller.get_outputs().current_error.read(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{2.0, 2.9}));
    bool success = simulator.simulate_for(2.0);

    system.plant.get_plant_model().set_machine_parameters(frequency_t{40.0}, voltage_t{1980.0});
    // simulator.register_checker(yeet::make_range_checker<current_pu_t>(
    //     "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
    //     current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{3.2, 4.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

// Test regulation of DC-link current to a reference value
TEST_F(SfcSensorlessCurrentControllerTest, MachineFrequencyStepLowFrequency)
{
    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.15f};
    current_pu_t current_error_min_value{-0.15f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance",
        [&]() -> current_pu_t { return system.current_controller.get_outputs().current_error.read(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{2.0, 2.9}));
    bool success = simulator.simulate_for(2.0);

    system.plant.get_plant_model().set_machine_parameters(frequency_t{30.0}, voltage_t{1980.0});
    // simulator.register_checker(yeet::make_range_checker<current_pu_t>(
    //     "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
    //     current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{3.2, 4.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

// Test regulation of DC-link current to a reference value
TEST_F(SfcSensorlessCurrentControllerTest, MachineFrequencyStepHighFrequency)
{
    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.15f};
    current_pu_t current_error_min_value{-0.15f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance",
        [&]() -> current_pu_t { return system.current_controller.get_outputs().current_error.read(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{2.0, 2.9}));
    bool success = simulator.simulate_for(2.0);

    system.plant.get_plant_model().set_machine_parameters(frequency_t{60.0}, voltage_t{1980.0});
    // simulator.register_checker(yeet::make_range_checker<current_pu_t>(
    //     "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
    //     current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{3.2, 4.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

// 1 - Checkers return the instance, so you can just run in between periods of time; user can just deactivate when it
// wants 2 - State-machine setting the booleans 3 -