// Sensorless current controller test using hardware mocks + ADC measurement chain
#include <gtest/gtest.h>

#include "sfc_integration/plant_sensorless_current_control_closed_loop_mocked_hardware.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcSensorlessCurrentControllerMockedHardwareTest : public ::testing::Test {
public:
    void SetUp() override
    {
        simulator.configure_sampler_sampling_rate(200e-6);
        system.register_applications(simulator);
        initialize();

        // Logger registration (core elements)
        register_complete_logger(simulator, system.current_controller);
        register_complete_logger(simulator, system.dc_link_measurements);
        register_complete_logger(simulator, system.grid_side_observer);
        register_complete_logger(simulator, system.machine_side_observer);
        register_complete_logger(simulator, system.grid_measurements);
        register_complete_logger(simulator, system.machine_measurements);

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

    void simulate_for(double t) { simulator.simulate_for(t); }

    // System under test
    sfc_sensorless_current_controller_mocked_hardware_simulation_builder system;
    // Simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// DC-link current regulation with step in reference
TEST_F(SfcSensorlessCurrentControllerMockedHardwareTest, CurrentRegulation)
{
    // Allow larger margin due to added ADC + observer dynamics
    current_pu_t err_max{0.3f};
    current_pu_t err_min{-0.3f};

    system.dc_link_current_reference = current_pu_t(0.5f);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error phase 1",
        [&]() -> current_pu_t { return system.current_controller.get_outputs().current_error.read(); }, err_min,
        err_max, yeet::checker_activation_time_range{1.2, 1.9}));
    bool success = simulator.simulate_for(2.0);

    system.dc_link_current_reference = current_pu_t(1.0f);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error phase 2",
        [&]() -> current_pu_t { return system.current_controller.get_outputs().current_error.read(); }, err_min,
        err_max, yeet::checker_activation_time_range{3.2}));
    success &= simulator.simulate_for(2.0);

    EXPECT_TRUE(success);
}

// Machine frequency decrease scenario
TEST_F(SfcSensorlessCurrentControllerMockedHardwareTest, MachineFrequencyStepDown)
{
    system.dc_link_current_reference = current_pu_t(0.6f);
    simulator.simulate_for(1.5);  // settle
    system.plant.get_plant_model().set_machine_parameters(frequency_t{40.0}, voltage_t{1980.0});
    bool success = simulator.simulate_for(1.0);
    EXPECT_TRUE(success);
}

// Machine frequency low scenario
TEST_F(SfcSensorlessCurrentControllerMockedHardwareTest, MachineFrequencyStepLow)
{
    system.dc_link_current_reference = current_pu_t(0.6f);
    simulator.simulate_for(1.5);
    system.plant.get_plant_model().set_machine_parameters(frequency_t{30.0}, voltage_t{1980.0});
    bool success = simulator.simulate_for(1.0);
    EXPECT_TRUE(success);
}

// Machine frequency high scenario
TEST_F(SfcSensorlessCurrentControllerMockedHardwareTest, MachineFrequencyStepHigh)
{
    system.dc_link_current_reference = current_pu_t(0.6f);
    simulator.simulate_for(1.5);
    system.plant.get_plant_model().set_machine_parameters(frequency_t{60.0}, voltage_t{1980.0});
    bool success = simulator.simulate_for(1.0);
    EXPECT_TRUE(success);
}
