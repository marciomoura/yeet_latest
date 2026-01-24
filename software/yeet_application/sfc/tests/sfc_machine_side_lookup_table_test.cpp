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
    const std::array<angle_wrapped, NUM_POINTS> test_angle_axis = {angle_wrapped::from_degrees(165.0),
        angle_wrapped::from_degrees(158.0), angle_wrapped::from_degrees(152.0), angle_wrapped::from_degrees(145.0),
        angle_wrapped::from_degrees(135.0), angle_wrapped::from_degrees(130.0)};

    // Helper to get angle in degrees for assertions
    real_t get_deg(angle_wrapped angle) const { return static_cast<real_t>(rad_to_deg(static_cast<angle_t>(angle))); }

    // The object under test, constructed before each test run.
    sfc_machine_side_firing_angle_lookup_table controller;
};

// Test 1: Verify that the controller's reset method works as expected.
TEST_F(SfcMachineSideFiringAngleLUTTest, ResetState)
{
    // After setup, the controller is configured with our test data.
    // Update to a non-reset state.
    controller.update(current_pu_t{1.0}, speed_pu_t{0.0});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), get_deg(test_angle_axis[3]), 1e-3);

    // Now, reset the controller. It should revert to the *default* initial value,
    // not the one from our test configuration.
    controller.reset();
    EXPECT_NEAR(get_deg(controller.get_firing_angle()),
        get_deg(sfc_machine_side_firing_angle_lookup_table::kDefaultFiringAngle[0]), 1e-3);
}

// Test 2: Verify the firing angle at exact points defined in the test table.
TEST_F(SfcMachineSideFiringAngleLUTTest, UpdateAtExactTestPoints)
{
    // Test at the first point (zero current)
    controller.update(current_pu_t{test_current_axis[0]}, speed_pu_t{0.0});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), get_deg(test_angle_axis[0]), 1e-3);

    // Test at a middle point
    controller.update(current_pu_t{test_current_axis[3]}, speed_pu_t{0.0});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), get_deg(test_angle_axis[3]), 1e-3);

    // Test at the last point
    controller.update(current_pu_t{test_current_axis.back()}, speed_pu_t{0.0});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), get_deg(test_angle_axis.back()), 1e-3);
}

// Test 3: Verify correct linear interpolation between points.
TEST_F(SfcMachineSideFiringAngleLUTTest, InterpolatesBetweenPoints)
{
    // Test halfway between 0.3 (158 deg) and 0.5 (152 deg) -> (158+152)/2 = 155
    controller.update(current_pu_t{0.4}, speed_pu_t{0.0});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), 155.0, 1e-3);

    // Test halfway between 1.0 (145 deg) and 1.88 (135 deg) -> (145+135)/2 = 140
    controller.update(current_pu_t{1.44}, speed_pu_t{0.0});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), 140.0, 1e-3);
}

// Test 4: Verify behavior outside the defined table range.
TEST_F(SfcMachineSideFiringAngleLUTTest, HandlesExtrapolationAndClamping)
{
    // Below the table range, should clamp to the first defined angle (165 deg)
    controller.update(current_pu_t{-0.1}, speed_pu_t{0.0});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), get_deg(test_angle_axis.front()), 1e-3);

    // Above the table range, should clamp to the last defined angle (130 deg)
    controller.update(current_pu_t{2.5}, speed_pu_t{0.0});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), get_deg(test_angle_axis.back()), 1e-3);
}

// Test 5: Verify speed-based delta lookup table.
TEST_F(SfcMachineSideFiringAngleLUTTest, UpdateWithSpeedDelta)
{
    // Base: 1.0 pu current -> 145.0 deg
    // Speed points: 0.0, 0.2, 1.05 -> 0.0, 0.0, -7.1

    // Test at 1.0 pu current and 0.2 pu speed (no delta)
    controller.update(current_pu_t{1.0}, speed_pu_t{0.2});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), 145.0, 1e-3);

    // Test at 1.05 pu speed (-7.1 delta) -> 145 - 7.1 = 137.9
    controller.update(current_pu_t{1.0}, speed_pu_t{1.05});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), 137.9, 1e-3);

    // Test halfway between 0.2 (0.0 delta) and 1.05 (-7.1 delta)
    // speed = 0.625 -> delta = -3.55 -> 145 - 3.55 = 141.45
    controller.update(current_pu_t{1.0}, speed_pu_t{0.625});
    EXPECT_NEAR(get_deg(controller.get_firing_angle()), 141.45, 1e-3);
}

}  // namespace yeet::application::sfc