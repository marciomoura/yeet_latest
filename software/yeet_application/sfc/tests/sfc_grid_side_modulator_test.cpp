#include "sfc/sfc_grid_side_modulator.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

namespace {

// Test fixture for grid-side modulator tests
class SfcGridSideModulatorTest : public ::testing::Test {
protected:
    void SetUp() override { modulator = std::make_unique<sfc_grid_side_modulator>(); }

    std::unique_ptr<sfc_grid_side_modulator> modulator;

    // Common test parameters
    static constexpr angle_t test_pi_angle = deg_to_rad(45.0);
    static constexpr angle_t custom_commutation_angle = deg_to_rad(150.0);
};

}  // namespace

// ========================================
// Constructor and Configuration Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, ConstructorInitializesToSafeState)
{
    // After construction, modulator should be in safe default state
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::natural_commutation);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);  // Safe default angle
}

TEST_F(SfcGridSideModulatorTest, ConfigureCommutationAngle)
{
    // Configure custom commutation angle
    modulator->configure_force_commutation_angle_rad(custom_commutation_angle);

    // Trigger force commutation to test the configured angle is used
    modulator->update(true, false, test_pi_angle);  // request_force_commutation = true

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), custom_commutation_angle.value());
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
}

TEST_F(SfcGridSideModulatorTest, ResetFunctionRestoresInitialState)
{
    // Put modulator in force commutation state
    modulator->update(true, false, test_pi_angle);

    // Reset and verify initial state is restored
    modulator->reset();

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::natural_commutation);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);
}

// ========================================
// Normal Operation Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, NormalOperationPassesThroughPiAngle)
{
    // In normal operation, should pass through PI controller firing angle
    modulator->update(false, false, test_pi_angle);  // No force commutation request

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::natural_commutation);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
}

TEST_F(SfcGridSideModulatorTest, NormalOperationIgnoresDcCurrentState)
{
    // In normal operation, DC current state should not affect behavior
    modulator->update(false, true, test_pi_angle);  // dc_link_current_is_zero = true (ignored)

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::natural_commutation);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
}

// ========================================
// Force Commutation Handshake Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, ForceCommutationRequestTriggersInversion)
{
    // Force commutation request should immediately switch to force inversion mode
    modulator->update(true, false, test_pi_angle);  // request_force_commutation = true

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_NE(modulator->get_firing_angle_rad().value(), test_pi_angle.value());  // Should not be PI angle
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());                // Not acknowledged yet
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());                    // PI reset should be requested
}

TEST_F(SfcGridSideModulatorTest, ForceInversionUsesCommutationAngle)
{
    // Test that force inversion mode uses the configured commutation angle
    const angle_t custom_angle = deg_to_rad(160.0);
    modulator->configure_force_commutation_angle_rad(custom_angle);

    modulator->update(true, false, test_pi_angle);  // Trigger force commutation

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), custom_angle.value());
}

TEST_F(SfcGridSideModulatorTest, ZeroCurrentCompletesHandshake)
{
    // Start force commutation
    modulator->update(true, false, test_pi_angle);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Signal zero current - should complete handshake
    modulator->update(true, true, test_pi_angle);  // dc_link_current_is_zero = true

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::natural_commutation);
    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());                 // Should be acknowledged
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());                   // PI reset should be cleared
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());  // Back to PI angle
}

TEST_F(SfcGridSideModulatorTest, AcknowledgmentIsOneCycleOnly)
{
    // Complete force commutation handshake
    modulator->update(true, false, test_pi_angle);  // Start commutation
    modulator->update(true, true, test_pi_angle);   // Complete with zero current

    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());

    // Next cycle should clear the acknowledgment
    modulator->update(false, false, test_pi_angle);  // Normal operation

    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
}

TEST_F(SfcGridSideModulatorTest, ForceInversionPersistsUntilZeroCurrent)
{
    // Start force commutation
    modulator->update(true, false, test_pi_angle);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Continue with force commutation request but no zero current
    for (int i = 0; i < 10; ++i) {
        modulator->update(true, false, test_pi_angle);  // dc_link_current_is_zero = false

        EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
        EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
        EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    }
}

// ========================================
// PI Reset Management Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, PiResetRequestManagedCorrectly)
{
    // Initially, no PI reset should be requested
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());

    // Force commutation should request PI reset
    modulator->update(true, false, test_pi_angle);
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());

    // Completing commutation should clear PI reset request
    modulator->update(true, true, test_pi_angle);
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
}

TEST_F(SfcGridSideModulatorTest, PiResetAcknowledgmentMethod)
{
    // Start force commutation to request PI reset
    modulator->update(true, false, test_pi_angle);
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());

    // Test manual acknowledgment method
    modulator->acknowledge_main_pi_reset();
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
}

// ========================================
// State Transition Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, CompleteHandshakeSequence)
{
    // Test the complete handshake sequence as described in documentation

    // Phase 1: Normal operation
    modulator->update(false, false, test_pi_angle);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::natural_commutation);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());

    // Phase 2: Machine-side requests force commutation
    modulator->update(true, false, test_pi_angle);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());

    // Phase 3: Current reaches zero, grid-side acknowledges
    modulator->update(true, true, test_pi_angle);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::natural_commutation);
    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());

    // Phase 4: Next cycle, acknowledgment is cleared
    modulator->update(false, false, test_pi_angle);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
}

// ========================================
// Edge Case Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, SimultaneousRequestAndZeroCurrent)
{
    // Test edge case where request and zero current arrive simultaneously
    modulator->update(true, true, test_pi_angle);

    // Should transition to force inversion first, then immediately complete
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::natural_commutation);
    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());
}

TEST_F(SfcGridSideModulatorTest, RequestDroppedDuringForceInversion)
{
    // Start force commutation
    modulator->update(true, false, test_pi_angle);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Drop request but no zero current yet - should stay in force inversion
    modulator->update(false, false, test_pi_angle);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Zero current should still complete the handshake
    modulator->update(false, true, test_pi_angle);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::natural_commutation);
    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());
}

TEST_F(SfcGridSideModulatorTest, MultipleConsecutiveRequests)
{
    // Test multiple force commutation requests in sequence

    // First request
    modulator->update(true, false, test_pi_angle);
    modulator->update(true, true, test_pi_angle);  // Complete first
    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());

    // Clear acknowledgment
    modulator->update(false, false, test_pi_angle);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());

    // Second request immediately after
    modulator->update(true, false, test_pi_angle);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    modulator->update(true, true, test_pi_angle);  // Complete second
    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());
}

// ========================================
// Boundary Condition Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, VeryLargeFiringAngles)
{
    // Test with firing angles beyond 2π
    const angle_t large_angle = deg_to_rad(450.0);  // 450° = 90° + 360°

    modulator->update(false, false, large_angle);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), large_angle.value());
}

TEST_F(SfcGridSideModulatorTest, NegativeFiringAngles)
{
    // Test with negative firing angles
    const angle_t negative_angle = deg_to_rad(-30.0);

    modulator->update(false, false, negative_angle);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), negative_angle.value());
}

TEST_F(SfcGridSideModulatorTest, ZeroFiringAngle)
{
    // Test with exactly zero firing angle
    const angle_t zero_angle = deg_to_rad(0.0);

    modulator->update(false, false, zero_angle);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), zero_angle.value());
}

// ========================================
// Default Configuration Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, DefaultCommutationAngleIsApplied)
{
    // Test that default commutation angle (150°) is used when not configured
    modulator->update(true, false, test_pi_angle);

    // Should use default 150° commutation angle
    EXPECT_NEAR(modulator->get_firing_angle_rad().value(), deg_to_rad(150.0).value(), 1e-6);
}

TEST_F(SfcGridSideModulatorTest, ReconfigurationDuringOperation)
{
    // Start with default configuration
    modulator->update(true, false, test_pi_angle);
    EXPECT_NEAR(modulator->get_firing_angle_rad().value(), deg_to_rad(150.0).value(), 1e-6);

    // Reconfigure while in force inversion
    const angle_t new_angle = deg_to_rad(170.0);
    modulator->configure_force_commutation_angle_rad(new_angle);

    // Next update should use new angle
    modulator->update(true, false, test_pi_angle);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), new_angle.value());
}
