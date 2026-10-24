#include <gtest/gtest.h>

#include "sfc/sfc_grid_side_modulator.hpp"
#include "yeet_control/constants.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

// Test verifying the fix for startup overcurrent due to fiber optic delays
TEST(SfcDelayedStartupTest, ClampsFiringAngleWhenCurrentIsZeroDuringStartup)
{
    constexpr duration_t sampling_time{100e-6};
    sfc_grid_side_modulator modulator(sampling_time);

    // Setup inputs mimicking startup conditions (current is ZERO)
    bool enable = true;
    sfc_operating_mode_t mode = sfc_operating_mode_t::forced_commutation;
    angle_wrapped grid_angle = angle_wrapped::from_radians(angle_t(0.0));
    bool request_force = false;
    bool current_zero = true;
    angle_wrapped pi_angle = angle_wrapped::from_degrees(0.0);  // Saturated PI (Max Voltage)
    bool extinguish = false;

    modulator.update(enable, mode, grid_angle, request_force, current_zero, pi_angle, extinguish);

    angle_t alpha = modulator.get_firing_angle_rad();

    // Check for 60 degree clamp
    EXPECT_GE(alpha.value(), 1.0) << "Firing angle should be clamped to safe value when current is zero";
    EXPECT_NEAR(alpha.value(), yeet::deg_to_rad(60.0).value(), 0.01);

    // Verify reset request is active (to prevent integrator windup)
    EXPECT_TRUE(modulator.get_is_requesting_main_pi_reset())
        << "PI reset should be requested when current is zero at startup";
}

// Ensure normal operation is not affected when current flows
TEST(SfcDelayedStartupTest, NormalOperationPassesThroughPiAngle)
{
    constexpr duration_t sampling_time{100e-6};
    sfc_grid_side_modulator modulator(sampling_time);

    // Normal operation: Current is low but NOT zero
    bool current_zero = false;
    angle_wrapped pi_angle = angle_wrapped::from_degrees(10.0);

    modulator.update(true, sfc_operating_mode_t::forced_commutation, angle_wrapped::from_radians(angle_t(0.0)), false,
        current_zero, pi_angle, false);

    angle_t alpha = modulator.get_firing_angle_rad();
    EXPECT_NEAR(alpha.value(), yeet::deg_to_rad(10.0).value(), 0.001)
        << "Should pass through angle when current is flowing";
    EXPECT_FALSE(modulator.get_is_requesting_main_pi_reset());
}
