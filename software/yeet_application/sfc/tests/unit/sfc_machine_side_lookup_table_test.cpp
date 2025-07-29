#include "sfc/sfc_machine_side_lookup_table.hpp"

#include <gtest/gtest.h>

#include <array>

#include "yeet_control/constants.hpp"
#include "yeet_control/types.hpp"

// Define the namespace to match the application structure
namespace yeet::application::sfc {

// Use the class's own constexprs for dimensions to ensure they are in sync
constexpr size_t NUM_TEST_CURRENT_POINTS = sfc_machine_side_firing_angle_lookup_table::NUM_CURRENT_POINTS;
constexpr size_t NUM_TEST_SPEED_POINTS = sfc_machine_side_firing_angle_lookup_table::NUM_SPEED_POINTS;

// GTest Fixture for the Firing Angle LUT
class SfcMachineSideFiringAngleLUTTest : public ::testing::Test {
protected:
    // This method is called before each test
    void SetUp() override {
        // --- Test Configuration ---
        // Define the per-unit axes for our test lookup table
        current_axis_pu = {0.0, 0.25, 0.5, 0.75, 1.0, 1.2};  // [p.u.]
        speed_axis_pu = {0.0, 0.2, 0.5, 0.75, 1.0, 1.25};    // [p.u.]

        // Configure the table to put it in a known state
        beta_controller.recalculate_table(initial_gamma, current_axis_pu, speed_axis_pu, machine_reactance_pu);
    }

    // --- Member variables accessible by all tests in this suite ---
    const real_t initial_gamma = 18.0;
    const real_t machine_reactance_pu = 0.0593;  // The base commutation reactance

    std::array<real_t, NUM_TEST_CURRENT_POINTS> current_axis_pu;
    std::array<real_t, NUM_TEST_SPEED_POINTS> speed_axis_pu;

    // Pointer to the object under test
    sfc_machine_side_firing_angle_lookup_table beta_controller{initial_gamma};
};

// Test 1: Verify that the controller is constructed correctly and holds the initial gamma.
TEST_F(SfcMachineSideFiringAngleLUTTest, ConstructionAndInitialState) {
    EXPECT_EQ(beta_controller.get_current_gamma_setpoint(), initial_gamma);
}

// Test 2: Verify the calculated firing angle at the nominal operating point (1.0 p.u. speed, 1.0 p.u. current).
TEST_F(SfcMachineSideFiringAngleLUTTest, GetFiringAngleAtNominalPUPoint) {
    const real_t nominal_speed_pu = 1.0;
    const real_t nominal_current_pu = 1.0;

    // Manually calculate the expected value for verification:
    // d_xM = (3/pi) * 0.0593 * 1.0 * 1.0 = 0.0566
    // cos(beta_adv) = cos(18) - 0.0566 = 0.8945
    // beta_adv = 26.55 deg
    // beta_inv = 180 - 26.55 = 153.45 deg
    const real_t expected_beta = 153.45;

    real_t actual_beta = beta_controller.get_firing_angle_deg(nominal_speed_pu, nominal_current_pu);

    EXPECT_NEAR(actual_beta, expected_beta, 0.05);
}

// Test 3: Verify the firing angles at the boundaries (corners) of the lookup table.
TEST_F(SfcMachineSideFiringAngleLUTTest, GetFiringAngleAtTableCorners) {
    // --- Test Case 1: Minimum speed, minimum current (zero load) ---
    // At this point, d_xM = 0, so beta_adv should equal gamma.
    // beta_inv = 180 - gamma = 180 - 18 = 162.
    real_t beta_at_min = beta_controller.get_firing_angle_deg(speed_axis_pu.front(), current_axis_pu.front());
    EXPECT_NEAR(beta_at_min, 162.0, 1e-9);

    // --- Test Case 2: Maximum speed, maximum current (max defined in table) ---
    const real_t max_speed_pu = speed_axis_pu.back();
    const real_t max_current_pu = current_axis_pu.back();

    // Manually calculate the expected value using PU inputs:
    // d_xM = (3/pi) * 0.0593 * max_speed_pu * max_current_pu
    // d_xM = (3/pi) * 0.0593 * 1.25 * 1.2 = 0.0849
    // cos(beta_adv) = cos(18) - 0.0849 = 0.8662
    // beta_adv = 29.98 deg
    // beta_inv = 180 - 29.98 = 150.02 deg
    const real_t expected_beta_at_max = 150.02;

    real_t beta_at_max = beta_controller.get_firing_angle_deg(max_speed_pu, max_current_pu);
    EXPECT_NEAR(beta_at_max, expected_beta_at_max, 0.01);

    // Verify the value is within the physical inverter range
    EXPECT_GE(beta_at_max, 90.0);
    EXPECT_LE(beta_at_max, 180.0);
}

// Test 4: Verify the online recalculation functionality.
TEST_F(SfcMachineSideFiringAngleLUTTest, RecalculateTableAndVerifyChange) {
    const real_t nominal_speed_pu = 1.0;
    const real_t nominal_current_pu = 1.0;

    // Get the initial firing angle
    real_t initial_beta = beta_controller.get_firing_angle_deg(nominal_speed_pu, nominal_current_pu);
    EXPECT_NEAR(initial_beta, 153.45, 0.05);

    // Now, recommission with a new, more aggressive gamma
    const real_t new_gamma = 15.0;
    beta_controller.recalculate_table(new_gamma, current_axis_pu, speed_axis_pu, machine_reactance_pu);

    // Verify the state has updated
    EXPECT_EQ(beta_controller.get_current_gamma_setpoint(), new_gamma);

    // Get the new firing angle and verify it has changed as expected
    real_t new_beta = beta_controller.get_firing_angle_deg(nominal_speed_pu, nominal_current_pu);

    // Manually calculate the new expected value:
    // cos(beta_adv) = cos(15) - 0.0566 = 0.9093
    // beta_adv = 24.59 deg
    // beta_inv = 180 - 24.59 = 155.41 deg
    const real_t expected_new_beta = 155.41;

    EXPECT_NEAR(new_beta, expected_new_beta, 0.01);
    EXPECT_NE(initial_beta, new_beta);  // Crucially, ensure the value actually changed.
}

// Test 5: Verify the new convenience function for radians.
TEST_F(SfcMachineSideFiringAngleLUTTest, GetFiringAngleInRadians) {
    const real_t nominal_speed_pu = 1.0;
    const real_t nominal_current_pu = 1.0;

    real_t beta_deg = beta_controller.get_firing_angle_deg(nominal_speed_pu, nominal_current_pu);
    real_t beta_rad = beta_controller.get_firing_angle_rad(nominal_speed_pu, nominal_current_pu);

    EXPECT_NEAR(beta_rad, beta_deg * pi / 180.0, 1e-9);
}

}  // namespace yeet::application::sfc