#include <gtest/gtest.h>

#include "sfc_integration/plant_simplified_current_control.hpp"
#include "sfc_loggers.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcCurrentControllerTest : public ::testing::Test {
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

        // ==================== Current Control Signals ====================
        register_complete_current_controller_logger(simulator, system.current_controller);
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // Helper function to calculate current error
    current_pu_t get_dc_link_current_error_pu() const
    {
        // Cast away const to access non-const methods - this is safe in test context
        auto& mutable_system = const_cast<sfc_current_controller_simulation_builder&>(system);
        return mutable_system.current_reference_pu.read() -
               mutable_system.dc_link_measurements.get_signal_flow_datamodel()
                   .get_outputs()
                   .dc_current_filtered_pu.read();
    }

    // System under test
    yeet::application::sfc::integration::sfc_current_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test regulation of DC-link current to a reference value
TEST_F(SfcCurrentControllerTest, TestCurrentRegulation)
{
    // ==================== Test Configuration ====================
    // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
    // The pulse generators are now controlled through signal flow, not parameters
    system.grid_side_pulse_generator.get_parameters().enable.write(true);
    system.machine_side_pulse_generator.get_parameters().enable.write(true);

    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.15f};
    current_pu_t current_error_min_value{-0.15f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{1.0, 1.9}));
    bool success = simulator.simulate_for(2.0);

    system.current_reference_pu = current_pu_t(1.0);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{2.5, 3.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}
