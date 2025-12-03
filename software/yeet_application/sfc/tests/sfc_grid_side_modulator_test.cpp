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
    static constexpr angle_t default_commutation_angle = deg_to_rad(150.0);
};

}  // namespace

// ========================================
// Initialization and Reset Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, ConstructorInitializesToSafeState)
{
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);
}

TEST_F(SfcGridSideModulatorTest, ResetFunctionRestoresInitialState)
{
    // Put modulator in force commutation state
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, true, test_pi_angle, false);
    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());

    // Reset and verify initial state is restored
    modulator->reset();

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);
}

TEST_F(SfcGridSideModulatorTest, DisableFunctionRestoresInitialState)
{
    // Put modulator in force commutation state
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, true, test_pi_angle, false);
    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());

    // Disable and verify initial state is restored
    modulator->update(false, sfc_operating_mode_t::forced_commutation, true, true, test_pi_angle, false);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);
}

// ========================================
// Normal Operation Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, NormalOperationPassesThroughPiAngle)
{
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, false);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
}

TEST_F(SfcGridSideModulatorTest, NormalOperationIgnoresDcCurrentState)
{
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, true, test_pi_angle, false);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
}

TEST_F(SfcGridSideModulatorTest, LoadCommutatedModeIgnoresForceCommutationRequests)
{
    // In load commutated mode, force commutation requests should be ignored
    modulator->update(true, sfc_operating_mode_t::natural_commutation, true, false, test_pi_angle, false);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());

    // Even with DC current zero signal, should remain in running state
    modulator->update(true, sfc_operating_mode_t::natural_commutation, true, true, test_pi_angle, false);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
}

// ========================================
// Complete Force Commutation Handshake
// ========================================

TEST_F(SfcGridSideModulatorTest, CompleteHandshakeSequence)
{
    // Phase 1: Normal operation
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());

    // Phase 2: Machine-side requests force commutation
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());

    // Phase 3: DC current reaches zero, but request still active
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, true, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_TRUE(modulator->get_force_commutation_acknowledged());
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());

    // Phase 4: Machine-side clears request (current still zero - freeze remains active)
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, true, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());  // Freeze remains while current is zero
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());

    // Phase 5: Current becomes non-zero - freeze is released
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());  // Freeze released when current flows
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
}

// ========================================
// Edge Cases and Error Conditions
// ========================================

TEST_F(SfcGridSideModulatorTest, ForceInversionPersistsUntilZeroCurrent)
{
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Continue with force commutation request but no zero current
    for (int i = 0; i < 5; ++i) {
        modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
        EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
        EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
        EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    }
}

TEST_F(SfcGridSideModulatorTest, SimultaneousRequestAndZeroCurrent)
{
    // Request and zero current arrive simultaneously
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, true, test_pi_angle, false);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());

    // Next cycle: zero current is evaluated in forced_commutation state
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, true, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
}

TEST_F(SfcGridSideModulatorTest, RequestDroppedDuringForceInversion)
{
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Drop request but no zero current yet
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Zero current should complete the handshake
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, true, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
}

TEST_F(SfcGridSideModulatorTest, ModeChangeInterruptsHandshake)
{
    // Start handshake in pulse mode
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Switch to load commutated mode during handshake
    modulator->update(true, sfc_operating_mode_t::natural_commutation, true, false, test_pi_angle, false);

    // Should clear commutation state and return to running
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
}

TEST_F(SfcGridSideModulatorTest, ModeChangeFromPulseModeToLoadCommutatedDuringForcedCommutation)
{
    // Test specific scenario: modulator is in the middle of pulse mode forced commutation
    // and mode changes to load commutated - should transition to natural commutation

    // Phase 1: Start normal operation in pulse mode
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());

    // Phase 2: Enter forced commutation state due to machine-side request
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());

    // Phase 3: In the middle of forced commutation, DC current still not zero
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());

    // Phase 4: CRITICAL TEST - Mode changes to natural_commutation during active forced commutation
    // The system should immediately transition to natural commutation (running state)
    modulator->update(true, sfc_operating_mode_t::natural_commutation, true, false, test_pi_angle, false);

    // Verify immediate transition to natural commutation behavior
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());  // Should pass through PI angle
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());                // Clear acknowledgment
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());                   // Clear PI reset request

    // Phase 5: Verify that in load commutated mode, force commutation requests are ignored
    modulator->update(true, sfc_operating_mode_t::natural_commutation, true, true, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());

    // Phase 6: Continue in load commutated mode with different PI angles
    const angle_t new_pi_angle = deg_to_rad(60.0);
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, false, new_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), new_pi_angle.value());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
}

// ========================================
// PI Reset Management
// ========================================

TEST_F(SfcGridSideModulatorTest, PiResetAcknowledgmentMethod)
{
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());

    modulator->acknowledge_main_pi_reset();
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
}

// ========================================
// Boundary Value Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, BoundaryFiringAngles)
{
    // Test extreme values
    const angle_t large_angle = deg_to_rad(450.0);
    const angle_t negative_angle = deg_to_rad(-30.0);
    const angle_t zero_angle = deg_to_rad(0.0);

    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, large_angle, false);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), large_angle.value());

    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, negative_angle, false);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), negative_angle.value());

    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, zero_angle, false);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), zero_angle.value());
}

// ========================================
// Extinguish Current Feature Tests
// ========================================

TEST_F(SfcGridSideModulatorTest, ExtinguishCurrentHasHighestPriority)
{
    // Extinguish request should override normal operation
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, true);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::extinguishing_current);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
    EXPECT_TRUE(modulator->get_is_extinguishing_current());
    EXPECT_TRUE(modulator->get_pulse_release());
}

TEST_F(SfcGridSideModulatorTest, ExtinguishCurrentOverridesForceCommutationHandshake)
{
    // Start normal forced commutation handshake
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Extinguish request should immediately override
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, false, test_pi_angle, true);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::extinguishing_current);
    EXPECT_TRUE(modulator->get_is_extinguishing_current());
    EXPECT_FALSE(modulator->get_force_commutation_acknowledged());
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
}

TEST_F(SfcGridSideModulatorTest, ExtinguishCurrentWorksInNaturalCommutationMode)
{
    // Extinguish should work even in natural commutation mode
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, false, test_pi_angle, true);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::extinguishing_current);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());
    EXPECT_TRUE(modulator->get_is_extinguishing_current());
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());
}

TEST_F(SfcGridSideModulatorTest, ExtinguishCurrentPersistsUntilRequestCleared)
{
    // Enter extinguish mode
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, true);
    EXPECT_TRUE(modulator->get_is_extinguishing_current());

    // Continue with extinguish request active
    for (int i = 0; i < 5; ++i) {
        modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, true);
        EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::extinguishing_current);
        EXPECT_TRUE(modulator->get_is_extinguishing_current());
        EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());
    }
}

TEST_F(SfcGridSideModulatorTest, ExtinguishCurrentClearsWhenRequestDropped)
{
    // Enter extinguish mode
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, true);
    EXPECT_TRUE(modulator->get_is_extinguishing_current());

    // Clear extinguish request - should return to normal operation
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, false);

    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(modulator->get_is_extinguishing_current());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
    EXPECT_FALSE(modulator->get_is_requesting_main_pi_reset());
}

TEST_F(SfcGridSideModulatorTest, ExtinguishCurrentWithDcCurrentZero)
{
    // Extinguish with DC current already zero
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, true, test_pi_angle, true);

    // Should still be extinguishing (request is still active)
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::extinguishing_current);
    EXPECT_TRUE(modulator->get_is_extinguishing_current());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());
}

TEST_F(SfcGridSideModulatorTest, ExtinguishCurrentDisableDoesNotOverride)
{
    // Extinguish has priority even over disable
    modulator->update(false, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, true);

    // Extinguish should still be active
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::extinguishing_current);
    EXPECT_TRUE(modulator->get_is_extinguishing_current());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());
}

TEST_F(SfcGridSideModulatorTest, ExtinguishCurrentSequenceWithShutdown)
{
    // Simulate a complete shutdown sequence

    // Phase 1: Normal operation
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);

    // Phase 2: Shutdown requested - enter extinguish mode
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, true);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::extinguishing_current);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());
    EXPECT_TRUE(modulator->get_is_requesting_main_pi_reset());

    // Phase 3: Current reaches zero (but request still active)
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, true, test_pi_angle, true);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::extinguishing_current);
    EXPECT_TRUE(modulator->get_is_extinguishing_current());

    // Phase 4: Request cleared - return to normal
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, true, test_pi_angle, false);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(modulator->get_is_extinguishing_current());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_pi_angle.value());
}

TEST_F(SfcGridSideModulatorTest, ExtinguishCurrentWithModeChanges)
{
    // Enter extinguish in forced commutation mode
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, false, test_pi_angle, true);
    EXPECT_TRUE(modulator->get_is_extinguishing_current());

    // Change to natural commutation mode - extinguish should persist
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, false, test_pi_angle, true);
    EXPECT_EQ(modulator->get_current_state(), grid_side_modulator_state_t::extinguishing_current);
    EXPECT_TRUE(modulator->get_is_extinguishing_current());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), default_commutation_angle.value());
}
