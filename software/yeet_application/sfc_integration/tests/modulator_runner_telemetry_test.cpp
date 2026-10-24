#include <gtest/gtest.h>

#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet::application::sfc;
using angle_wrapped = yeet::angle_wrapped;

TEST(ModulatorRunnerTest, FreezeIntegratorPropagation)
{
    sfc_grid_side_modulator_runner runner;

    // Connect inputs
    bool request_force_commutation = false;
    bool current_is_zero = true;
    angle_wrapped firing_angle = angle_wrapped::from_degrees(90.0f);
    sfc_operating_mode_t operating_mode = sfc_operating_mode_t::forced_commutation;
    bool release_pulses = true;

    angle_wrapped grid_reference_angle = angle_wrapped(0.0f);
    bool shutdown_request = false;
    bool protection_trip = false;
    bool undervoltage_request = false;
    bool extinguish_dc_current = false;

    yeet::utils::mock_signal_input::connect(runner.get_inputs().request_force_commutation, request_force_commutation);
    yeet::utils::mock_signal_input::connect(runner.get_inputs().current_is_zero, current_is_zero);
    yeet::utils::mock_signal_input::connect(runner.get_inputs().firing_angle, firing_angle);
    yeet::utils::mock_signal_input::connect(runner.get_inputs().operating_mode, operating_mode);
    yeet::utils::mock_signal_input::connect(runner.get_inputs().release_pulses, release_pulses);
    yeet::utils::mock_signal_input::connect(runner.get_inputs().grid_reference_angle, grid_reference_angle);
    yeet::utils::mock_signal_input::connect(runner.get_inputs().shutdown_request, shutdown_request);
    yeet::utils::mock_signal_input::connect(runner.get_inputs().protection_trip, protection_trip);
    yeet::utils::mock_signal_input::connect(
        runner.get_inputs().undervoltage_request_extinguish_dc_current, undervoltage_request);
    yeet::utils::mock_signal_input::connect(runner.get_inputs().request_extinguish_dc_current, extinguish_dc_current);

    // Initialize
    runner.initialize();

    // Run
    runner.run();

    // Check output
    EXPECT_TRUE(runner.get_outputs().freeze_integrator.read());
}
