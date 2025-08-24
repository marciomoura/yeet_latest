#include <gtest/gtest.h>

#include "sfc_integration/plant_sensorless_speed_control_closed_loop.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcSensorlessSpeedControllerTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(ms(0.5));

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Initialize the system - allocates plant model and all components
        initialize();

        system.machine_side_controller.get_parameters().machine_side_commutation_reactance_pu =
            to_pu(system.plant.get_plant_model().get_electrical_parameters().machine_side_inductance,
                system.plant.get_plant_model().get_per_unit_system().get_machine_bases().base_inductance);

        // ==================== Logger Registration ====================
        register_complete_logger(simulator, system.speed_controller);
        register_complete_logger(simulator, system.current_controller);
        register_complete_logger(simulator, system.dc_link_measurements);
        register_complete_logger(simulator, system.grid_side_observer);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // System under test
    yeet::application::sfc::integration::sfc_sensorless_speed_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test regulation of machine speed to a reference value
TEST_F(SfcSensorlessSpeedControllerTest, TestSpeedRegulation)
{
    system.plant.get_parameters().operation_mode.write(
        sfc_plant_model_datamodel::machine_operation_mode::variable_speed);

    system.plant.get_parameters().motor_damping_factor.write(damping_coefficient_pu_t{0.01f});
    system.plant.get_parameters().motor_inertia_constant.write(inertia_constant_t{0.5});
    // system.plant.get_plant_model().configure_mechanical_load(inertia_constant_t{0.5},
    // damping_coefficient_pu_t{0.001f});
    system.plant.get_plant_model().set_operation_mode(plant_model_machine_operation_mode::variable_speed);
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);
    system.plant.get_plant_model().configure_speed_voltage_relationship(0.9, 0.2f, 0.0f);
    system.plant.get_plant_model().set_initial_mechanical_speed_pu(speed_pu_t{1.0f});
    system.plant.get_plant_model().set_load_torque_pu(torque_pu_t{0.3f});
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
