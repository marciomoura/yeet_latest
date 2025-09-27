#include <gtest/gtest.h>

#include "sfc_integration/plant_simplified_open_loop.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcOpenLoopPlantModel : public ::testing::Test {
public:
    // using parameter_types ::parameter_types;

    void SetUp() override
    {
        // simulator.configure_sampler_sampling_rate(200e-6);
        system.register_applications(simulator);

        // initialize here, it allocates plant model stuff
        initialize();

        // ==================== Logger Registration ====================
        register_minimal_logger(simulator, system.plant);
        register_minimal_logger(simulator, system.grid_side_pulse_generator);
        register_minimal_logger(simulator, system.machine_side_pulse_generator);
        register_complete_logger(simulator, system.grid_side_hardware_mock);
        register_complete_logger(simulator, system.machine_side_hardware_mock);
        register_minimal_logger(simulator, system.grid_side_measurements);
        register_minimal_logger(simulator, system.machine_side_measurements);
        register_complete_logger(simulator, system.dc_link_measurements);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    yeet::application::sfc::integration::sfc_open_loop_simulation_builder system;

    yeet::simulator simulator = yeet::make_gtest_simulator();
};

TEST_F(SfcOpenLoopPlantModel, OpenLoopStepResponseTest)
{
    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(13.0);
    system.firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    simulate_for(0.25f);

    system.grid_side_firing_angle = yeet::angle_wrapped::from_degrees(25.0);

    simulate_for(0.25f);
}
