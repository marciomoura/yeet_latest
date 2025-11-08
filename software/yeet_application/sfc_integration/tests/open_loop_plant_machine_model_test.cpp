#include <gtest/gtest.h>

#include "sfc_integration/plant_machine_open_loop_simulation_builder.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcOpenLoopPlantMachineModel : public ::testing::Test {
public:
    void SetUp() override
    {
        // Register all applications with the simulator
        system.register_applications(simulator);

        // Initialize the system (allocates plant model resources)
        initialize();

        // ==================== Logger Registration ====================
        // register_complete_logger(simulator, system.plant);
        // (simulator, system.plant);
        // register_minimal_logger(simulator, system.grid_side_pulse_generator);
        register_minimal_logger(simulator, system.grid_side_pulse_generator);
        register_complete_plant_model_machine_grid_logger(simulator, system.plant);
        // register_minimal_logger(simulator, system.machine_side_pulse_generator);
        // register_complete_plant_model_machine_side_logger(simulator, system.plant);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    yeet::application::sfc::integration::plant_machine_open_loop_simulation_builder system;

    yeet::simulator simulator = yeet::make_gtest_simulator();
};

TEST_F(SfcOpenLoopPlantMachineModel, InitializationTest)
{
    // Just test that initialization completes without segfault
    SUCCEED();
}

TEST_F(SfcOpenLoopPlantMachineModel, SingleStepTest)
{
    // Set firing angles before simulation
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(15.0);
    system.firing_angle = yeet::angle_wrapped::from_degrees(150.0);

    // Run exactly one timestep (smallest sampling time is 12.5µs)
    simulate_for(13e-6);
}

TEST_F(SfcOpenLoopPlantMachineModel, OpenLoopStepResponseTest)
{
    // Configure plant parameters via datamodel parameters (NOT direct plant model setters)
    system.plant.get_parameters().forced_speed_pu.write(0.0f);
    system.plant.get_parameters().forced_speed_enabled.write(true);
    system.plant.get_parameters().field_voltage.write(voltage_t{0.0f});
    // system.plant.get_parameters().i
    // system.plant.get_parameters().forced_speed_pu.write(0.1f);
    // system.plant.get_parameters().load_coeff_c0.write(0.0f);
    // system.plant.get_parameters().load_coeff_c1.write(0.0f);
    // system.plant.get_parameters().load_coeff_c2.write(0.0f);

    // Set firing angles before simulation
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(0.0);
    system.grid_side_pulse_generator.get_parameters().reference_type.write(
        control::six_pulse_generator::reference_type::line);
    system.plant.get_parameters().dc_isolator_closed.write(true);
    yeet::mock_signal_input::write(system.grid_side_pulse_generator.get_inputs().release_pulses, true);

    // Simulate for another 250ms to observe response
    simulate_for(0.25);

    system.plant.get_parameters().dc_isolator_closed.write(false);
    yeet::mock_signal_input::write(system.grid_side_pulse_generator.get_inputs().release_pulses, true);

    // Simulate for another 250ms to observe response
    simulate_for(0.25);

    // system.plant.get_parameters().field_voltage.write(voltage_t{10.0f});
    // system.plant.get_parameters().dc_isolator_closed.write(false);
    // system.plant.get_parameters().forced_speed_enabled.write(false);
    // system.plant.get_parameters().forced_speed_pu.write(0.1f);
    // simulate_for(0.25);
    // yeet::mock_signal_input::write(system.grid_side_pulse_generator.get_inputs().release_pulses, true);
    // yeet::mock_signal_input::write(system.machine_side_pulse_generator.get_inputs().release_pulses, true);

    // // Configure plant parameters
    // // Field voltage: 100V (default)
    // // Forced speed: 1.0 pu (synchronous speed, default)
    // // Grid voltage: 1707V peak phase (default, from 2090V RMS L-L)

    // // Set initial firing angles BEFORE any simulation
    // // Grid-side: 13 degrees (mild rectification)
    // system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(30.0);

    // // Machine-side: ~153 degrees (180 - 26.84, mild inversion)
    // system.firing_angle = yeet::angle_wrapped::from_degrees(140.0);

    // // Simulate for 250ms to reach steady state
    // // simulate_for(0.25f);  // Apply step change in grid-side firing angle
    // // system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(25.0);

    // // Simulate for another 250ms to observe response
    // simulate_for(0.25);
}

TEST_F(SfcOpenLoopPlantMachineModel, FullRectificationTest)
{
    // Test with near-full rectification on grid side
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(5.0);

    // Machine-side at moderate inversion
    system.firing_angle = yeet::angle_wrapped::from_degrees(150.0);

    // Run simulation
    simulate_for(0.5f);
}

TEST_F(SfcOpenLoopPlantMachineModel, ZeroFiringAngleTest)
{
    // Test with zero firing angles (maximum DC voltage)
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(0.0);
    system.firing_angle = yeet::angle_wrapped::from_degrees(180.0);

    // Run simulation
    simulate_for(0.3f);
}