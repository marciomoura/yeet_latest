#include <gtest/gtest.h>

#include "sfc_integration/plant_simplified_speed_control.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
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

        // Initialize the system - allocates plant model and all components
        initialize();

        system.machine_side_controller.get_parameters().machine_side_commutation_reactance_pu =
            to_pu(system.plant.get_plant_model().get_electrical_parameters().machine_side_inductance,
                system.plant.get_plant_model().get_per_unit_system().get_machine_bases().base_inductance);

        // ==================== Speed Control Signals ====================
        register_complete_speed_controller_logger(simulator, system.speed_controller);

        // ==================== Current Control Signals ====================
        register_complete_current_controller_logger(simulator, system.current_controller);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // Helper functions to calculate errors
    current_pu_t get_dc_link_current_error_pu() const
    {
        // Cast away const to access non-const methods - this is safe in test context
        auto& mutable_system = const_cast<sfc_speed_controller_simulation_builder&>(system);
        return mutable_system.speed_controller.get_signal_flow_datamodel().get_outputs().current_reference.read() -
               mutable_system.dc_link_measurements.get_signal_flow_datamodel()
                   .get_outputs()
                   .dc_current_filtered_pu.read();
    }

    speed_pu_t get_speed_error_pu()
    {
        return system.speed_controller.get_inputs().speed_reference_pu.read() -
               system.speed_controller.get_inputs().measured_speed_pu.read();
    }

    // System under test
    yeet::application::sfc::integration::sfc_speed_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test regulation of machine speed to a reference value
TEST_F(SfcSpeedControllerTest, TestSpeedRegulation)
{
    system.plant.get_parameters().operation_mode.write(
        sfc_plant_model_datamodel::machine_operation_mode::variable_speed);

    system.plant.get_parameters().motor_damping_factor.write(damping_coefficient_pu_t{0.01f});
    system.plant.get_parameters().motor_inertia_constant.write(inertia_constant_t{0.5});
    // system.plant.get_plant_model().configure_mechanical_load(inertia_constant_t{0.5},
    // damping_coefficient_pu_t{0.001f});
    system.plant.get_plant_model().set_operation_mode(plant_model_machine_operation_mode::variable_speed);
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);
    system.plant.get_plant_model().configure_speed_voltage_relationship(0.8, 0.2f, 0.0f);
    system.plant.get_plant_model().set_initial_mechanical_speed_pu(speed_pu_t{1.0f});
    system.plant.get_plant_model().set_load_torque_pu(torque_pu_t{0.3f});
    system.plant.get_plant_model().set_initial_mechanical_speed_pu(speed_pu_t{0.9f});
    const speed_pu_t speed_error_max_value{0.05f};
    const speed_pu_t speed_error_min_value{-0.05f};

    // Check speed error during steady state before disturbance
    simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error pre disturbance", [&]() -> speed_pu_t { return get_speed_error_pu(); }, speed_error_min_value,
        speed_error_max_value, yeet::checker_activation_time_range{4.0, 8.0}));

    // Check speed error after disturbance
    simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error after disturbance", [&]() -> speed_pu_t { return get_speed_error_pu(); }, speed_error_min_value,
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
