#include "sfc/sfc_machine_side_modulator.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

namespace {

// Test fixture for machine-side modulator tests
class SfcMachineSideModulatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Create modulator with 100µs sampling time
        modulator = std::make_unique<sfc_machine_side_modulator>(duration_t(100e-6));
    }

    std::unique_ptr<sfc_machine_side_modulator> modulator;

    // Common test parameters
    static constexpr duration_t sampling_time{100e-6};  // 100µs
    static constexpr angle_t test_lut_angle = deg_to_rad(45.0);
    static constexpr angle_t forced_commutation_angle =
        deg_to_rad(180.0);  // Fixed pulse mode angle from implementation

    // Helper function to convert degrees to radians for cleaner test code
    constexpr angle_t deg_to_rad_helper(real_t degrees) const { return angle_t(degrees * pi / 180.0); }
};

}  // namespace

// ========================================
// Constructor and Initial State Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, ConstructorInitializesToSafeState)
{
    // After construction, modulator should be in safe default state
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);  // Safe default angle
}

TEST_F(SfcMachineSideModulatorTest, ResetFunctionRestoresInitialState)
{
    // Put modulator in a non-initial state by triggering commutation request
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(30.0), test_lut_angle);
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(70.0), test_lut_angle);

    // Verify we're in waiting state
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_TRUE(modulator->get_request_force_commutation());

    // Reset and verify initial state is restored
    modulator->reset();

    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);
}

// ========================================
// Operating Mode Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, LoadCommutatedModePassesThroughLutAngle)
{
    // Test that in natural_commutation mode, the firing angle from lookup table is passed through
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(45.0), test_lut_angle);

    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_lut_angle.value());
    EXPECT_TRUE(modulator->get_release_pulses());              // Always release pulses in natural commutation
    EXPECT_FALSE(modulator->get_request_force_commutation());  // Never request commutation
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
}

TEST_F(SfcMachineSideModulatorTest, PulseModeUsesFixedFiringAngle)
{
    // Test that in forced_commutation, the firing angle is the fixed 180° value
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(30.0), test_lut_angle);

    EXPECT_EQ(modulator->get_firing_angle_rad().value(), forced_commutation_angle.value());
    EXPECT_TRUE(modulator->get_release_pulses());              // Should start with pulses enabled
    EXPECT_FALSE(modulator->get_request_force_commutation());  // No commutation request initially
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
}

TEST_F(SfcMachineSideModulatorTest, NoCommutationRequestInLoadCommutatedMode)
{
    // Sector changes in natural_commutation mode should not generate commutation requests
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(30.0), test_lut_angle);
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(70.0), test_lut_angle);

    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
}

// ========================================
// System Enable/Disable Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, DisabledSystemOutputsSafeDefaults)
{
    // When system is disabled, should output safe values regardless of inputs
    modulator->update(false, sfc_operating_mode_t::forced_commutation, true, deg_to_rad_helper(100.0), test_lut_angle);

    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);  // Safe default angle
    EXPECT_FALSE(modulator->get_release_pulses());              // Block pulses when disabled
    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
}

TEST_F(SfcMachineSideModulatorTest, MonotonicityRatchetAndReset)
{
    // Ensure we are in a known state
    modulator->reset();

    // 1. Move to 30 degrees (Sector 0)
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(30.0), test_lut_angle);
    EXPECT_EQ(modulator->get_current_rotor_sector(), 0);

    // 2. Observer jitters backward to 25 degrees
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(25.0), test_lut_angle);
    // Should stay at 30 degrees due to ratchet.
    EXPECT_NEAR(modulator->get_monotonic_rotor_position().value(), deg_to_rad_helper(30.0).value(), 1e-4);

    // 3. Observer jitters near a boundary (59 -> 61 -> 59)
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(61.0), test_lut_angle);
    EXPECT_EQ(modulator->get_current_rotor_sector(), 1);  // Jumped to Sector 1

    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(59.0), test_lut_angle);
    EXPECT_EQ(modulator->get_current_rotor_sector(), 1);  // Should stay in Sector 1 (Ratchet!)

    // 4. Large backward jump (default unstick threshold is 60 deg)
    // Jumping from 61 to -10 deg (350.0) -> dist is ~71 deg > 60
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(350.0), test_lut_angle);
    EXPECT_EQ(modulator->get_current_rotor_sector(), 5);  // Should have followed backward jump

    // 5. Reset should clear the ratchet baseline
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(30.0), test_lut_angle);
    EXPECT_EQ(modulator->get_current_rotor_sector(), 0);

    modulator->reset();
    // After reset, the first update establishes a new baseline (smaller than 30)
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(10.0), test_lut_angle);
    EXPECT_NEAR(modulator->get_monotonic_rotor_position().value(), deg_to_rad_helper(10.0).value(), 1e-4);
}

TEST_F(SfcMachineSideModulatorTest, DisableSystemDuringCommutationClears)
{
    // Start commutation sequence
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(30.0), test_lut_angle);
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(70.0), test_lut_angle);

    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_TRUE(modulator->get_request_force_commutation());

    // Disable system - should clear all commutation state
    modulator->update(false, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(70.0), test_lut_angle);

    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_FALSE(modulator->get_release_pulses());  // Pulses blocked when disabled
}

// ========================================
// Rotor Sector Calculation Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, RotorSectorCalculationAndBoundaries)
{
    // Test sector boundaries and wraparound (each sector is 60 degrees)
    const struct {
        real_t rotor_degrees;
        int expected_sector;
    } test_cases[] = {
        {0.0, 0}, {30.0, 0}, {59.999, 0},    // Sector 0: [0°, 60°)
        {60.0, 1}, {90.0, 1}, {119.999, 1},  // Sector 1: [60°, 120°)
        {120.0, 2}, {180.0, 3}, {240.0, 4},  // Sectors 2, 3, 4
        {300.0, 5}, {359.999, 5},            // Sector 5: [300°, 360°)
        {450.0, 1}, {-30.0, 5}               // Wraparound cases
    };

    for (const auto& test_case : test_cases) {
        auto test_modulator = std::make_unique<sfc_machine_side_modulator>(sampling_time);

        test_modulator->update(true, sfc_operating_mode_t::forced_commutation, false,
            deg_to_rad_helper(test_case.rotor_degrees), test_lut_angle);

        // In pulse mode, firing angle should always be 180° regardless of rotor position
        EXPECT_EQ(test_modulator->get_firing_angle_rad().value(), forced_commutation_angle.value())
            << "Failed for rotor angle: " << test_case.rotor_degrees << "°";
    }
}

// ========================================
// Complete Forced Commutation Handshake
// ========================================

TEST_F(SfcMachineSideModulatorTest, CompleteCommutationHandshakeSequence)
{
    // Test the complete handshake sequence including sector changes, acknowledgment, and state transitions

    // Phase 1: Start in sector 0 - running state
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(30.0), test_lut_angle);
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
    EXPECT_FALSE(modulator->get_request_force_commutation());

    // Phase 2: Move to sector 1 - should initiate commutation request
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(70.0), test_lut_angle);
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_FALSE(modulator->get_release_pulses());            // Pulses should be blocked immediately
    EXPECT_TRUE(modulator->get_request_force_commutation());  // Request should be asserted

    // Phase 3: Request persists until acknowledged
    for (int i = 0; i < 3; ++i) {
        modulator->update(
            true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(70.0), test_lut_angle);
        EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
        EXPECT_TRUE(modulator->get_request_force_commutation());
        EXPECT_FALSE(modulator->get_release_pulses());
    }

    // Phase 4: Provide acknowledgment from grid-side
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, deg_to_rad_helper(70.0), test_lut_angle);
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_FALSE(modulator->get_request_force_commutation());  // Request should be cleared
    EXPECT_TRUE(modulator->get_release_pulses());              // Pulses should be released

    // Phase 5: Test multiple consecutive sector changes
    modulator->update(
        true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(130.0), test_lut_angle);  // Sector 1→2
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_TRUE(modulator->get_request_force_commutation());

    modulator->update(
        true, sfc_operating_mode_t::forced_commutation, true, deg_to_rad_helper(130.0), test_lut_angle);  // Acknowledge
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_FALSE(modulator->get_request_force_commutation());
}

// ========================================
// Edge Cases and Error Conditions
// ========================================

TEST_F(SfcMachineSideModulatorTest, AllSectorTransitionsGenerateRequests)
{
    // Test transitions between all adjacent sectors including wraparound
    const std::vector<std::pair<real_t, real_t>> sector_transitions = {
        {30.0, 70.0},    // Sector 0 → 1
        {90.0, 130.0},   // Sector 1 → 2
        {150.0, 190.0},  // Sector 2 → 3
        {210.0, 250.0},  // Sector 3 → 4
        {270.0, 310.0},  // Sector 4 → 5
        {330.0, 10.0}    // Sector 5 → 0 (wraparound)
    };

    for (const auto& transition : sector_transitions) {
        modulator->reset();

        // Start in first sector
        modulator->update(
            true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(transition.first), test_lut_angle);
        EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);

        // Move to second sector
        modulator->update(true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(transition.second),
            test_lut_angle);

        EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack)
            << "Failed for transition " << transition.first << "° → " << transition.second << "°";
        EXPECT_TRUE(modulator->get_request_force_commutation());
        EXPECT_FALSE(modulator->get_release_pulses());
    }
}

TEST_F(SfcMachineSideModulatorTest, RapidSectorChangesAndModeTransitions)
{
    // Test rapid sector changes (faster than typical acknowledgment)
    modulator->update(
        true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(30.0), test_lut_angle);  // Sector 0
    modulator->update(
        true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(70.0), test_lut_angle);  // Sector 1

    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_TRUE(modulator->get_request_force_commutation());

    // Move to another sector before acknowledgment - should remain in waiting state
    modulator->update(
        true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(130.0), test_lut_angle);  // Sector 2
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_TRUE(modulator->get_request_force_commutation());

    // Test mode transition during commutation - should clear commutation state
    modulator->update(true, sfc_operating_mode_t::natural_commutation, false, deg_to_rad_helper(130.0), test_lut_angle);
    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
}

TEST_F(SfcMachineSideModulatorTest, BackwardRotorMovementAndAcknowledgmentEdgeCases)
{
    // Test backward rotor movement
    modulator->update(
        true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(130.0), test_lut_angle);  // Sector 2
    modulator->update(
        true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(70.0), test_lut_angle);  // Sector 1

    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_TRUE(modulator->get_request_force_commutation());

    // Test acknowledgment ignored in running state
    modulator->reset();
    modulator->update(true, sfc_operating_mode_t::forced_commutation, false, deg_to_rad_helper(30.0), test_lut_angle);
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);

    // Provide acknowledgment while in running state (shouldn't affect anything)
    modulator->update(true, sfc_operating_mode_t::forced_commutation, true, deg_to_rad_helper(30.0), test_lut_angle);
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
    EXPECT_FALSE(modulator->get_request_force_commutation());
}