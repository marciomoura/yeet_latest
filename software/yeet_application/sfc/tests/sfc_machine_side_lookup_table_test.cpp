#include "sfc/sfc_machine_side_lookup_table.hpp"

#include <gtest/gtest.h>

#include <array>

#include "yeet_control/constants.hpp"

// Define the namespace to match the application structure
namespace yeet::application::sfc {

// Use the class's own constexprs for dimensions to ensure they are in sync
constexpr size_t NUM_POINTS = sfc_machine_side_firing_angle_lookup_table::NUM_POINTS;

// GTest Fixture for the Firing Angle LUT
class SfcMachineSideFiringAngleLUTTest : public ::testing::Test {
protected:
    // This method is called before each test to ensure a known state.
    void SetUp() override
    {
        // Configure the controller with test-specific values.
        // This makes the tests independent of the default values in the class.
        controller.configure_table(test_current_axis, test_angle_axis);
    }

    // --- Test-specific configuration data ---
    const std::array<current_pu_t, NUM_POINTS> test_current_axis = {current_pu_t{0.0}, current_pu_t{0.3},
        current_pu_t{0.5}, current_pu_t{1.0}, current_pu_t{1.88}, current_pu_t{2.0}};
    const std::array<real_t, NUM_POINTS> test_angle_axis = {165.0, 158.0, 152.0, 145.0, 135.0, 130.0};

    // The object under test, constructed before each test run.
    sfc_machine_side_firing_angle_lookup_table controller;
};

// Test 1: Verify that the controller's reset method works as expected.
TEST_F(SfcMachineSideFiringAngleLUTTest, ResetState)
{
    // After setup, the controller is configured with our test data.
    // Update to a non-reset state.
    controller.update(current_pu_t{1.0});
    EXPECT_FLOAT_EQ(controller.get_firing_angle_deg(), test_angle_axis[3]);

    // Now, reset the controller. It should revert to the *default* initial value,
    // not the one from our test configuration.
    controller.reset();
    EXPECT_EQ(controller.get_firing_angle_deg(), sfc_machine_side_firing_angle_lookup_table::kDefaultFiringAngleDeg[0]);
}

// Test 2: Verify the firing angle at exact points defined in the test table.
TEST_F(SfcMachineSideFiringAngleLUTTest, UpdateAtExactTestPoints)
{
    // Test at the first point (zero current)
    controller.update(current_pu_t{test_current_axis[0]});
    EXPECT_FLOAT_EQ(controller.get_firing_angle_deg(), test_angle_axis[0]);

    // Test at a middle point
    controller.update(current_pu_t{test_current_axis[3]});
    EXPECT_FLOAT_EQ(controller.get_firing_angle_deg(), test_angle_axis[3]);

    // Test at the last point
    controller.update(current_pu_t{test_current_axis.back()});
    EXPECT_FLOAT_EQ(controller.get_firing_angle_deg(), test_angle_axis.back());
}

// Test 3: Verify correct linear interpolation between points.
TEST_F(SfcMachineSideFiringAngleLUTTest, InterpolatesBetweenPoints)
{
    // Test halfway between 0.3 (158 deg) and 0.5 (152 deg) -> (158+152)/2 = 155
    controller.update(current_pu_t{0.4});
    EXPECT_FLOAT_EQ(controller.get_firing_angle_deg(), 155.0);

    // Test halfway between 1.0 (145 deg) and 1.88 (135 deg) -> (145+135)/2 = 140
    controller.update(current_pu_t{1.44});
    EXPECT_FLOAT_EQ(controller.get_firing_angle_deg(), 140.0);
}

// Test 4: Verify behavior outside the defined table range.
TEST_F(SfcMachineSideFiringAngleLUTTest, HandlesExtrapolationAndClamping)
{
    // Below the table range, should clamp to the first defined angle (165 deg)
    controller.update(current_pu_t{-0.1});
    EXPECT_EQ(controller.get_firing_angle_deg(), test_angle_axis.front());

    // Above the table range, should clamp to the last defined angle (130 deg)
    controller.update(current_pu_t{2.5});
    EXPECT_EQ(controller.get_firing_angle_deg(), test_angle_axis.back());
}

// Test 5: Verify the new convenience function for radians.
TEST_F(SfcMachineSideFiringAngleLUTTest, GetFiringAngleInRadians)
{
    // Update the controller to a known state
    controller.update(current_pu_t{1.0});

    // Retrieve the angle in both degrees and radians
    real_t angle_deg = controller.get_firing_angle_deg();
    real_t angle_rad = controller.get_firing_angle_rad();

    // Verify the conversion is correct
    EXPECT_NEAR(angle_rad, angle_deg * pi / 180.0, 1e-9);
}

}  // namespace yeet::application::sfc