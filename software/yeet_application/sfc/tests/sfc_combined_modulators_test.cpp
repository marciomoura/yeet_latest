#include <gtest/gtest.h>

#include "sfc/sfc_grid_side_modulator.hpp"
#include "sfc/sfc_machine_side_modulator.hpp"
#include "yeet_control/constants.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

namespace {

/**
 * @brief Test fixture for combined grid-side and machine-side modulator tests.
 *
 * This fixture tests the complete handshake protocol between both modulators
 * with direct signal connections (no communication delays).
 */
class SfcCombinedModulatorsTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        grid_modulator = std::make_unique<sfc_grid_side_modulator>();
        machine_modulator = std::make_unique<sfc_machine_side_modulator>(sampling_time);
    }

    std::unique_ptr<sfc_grid_side_modulator> grid_modulator;
    std::unique_ptr<sfc_machine_side_modulator> machine_modulator;

    // Common test parameters
    static constexpr duration_t sampling_time{100e-6};  // 100µs sampling time
    static constexpr angle_t test_pi_angle = deg_to_rad(45.0);
    static constexpr angle_t default_commutation_angle = deg_to_rad(150.0);
    static constexpr angle_t forced_commutation_angle = deg_to_rad(180.0);

    // Helper function for cleaner test code
    constexpr angle_t deg_to_rad_helper(real_t degrees) const { return angle_t(degrees * pi / 180.0); }

    /**
     * @brief Simulates one control cycle with both modulators.
     * @param enable System enable signal
     * @param operating_mode Machine-side operating mode
     * @param rotor_position_rad Current rotor position
     * @param is_current_zero DC link current zero detection
     */
    void simulate_control_cycle(
        bool enable, sfc_operating_mode_t operating_mode, angle_t rotor_position_rad, bool is_current_zero)
    {
        // Direct signal connections - no delay
        bool force_request = machine_modulator->get_request_force_commutation();
        bool force_ack = grid_modulator->get_force_commutation_acknowledged();

        // Update machine-side modulator
        machine_modulator->update(enable, operating_mode, force_ack, rotor_position_rad, test_pi_angle);

        // Update grid-side modulator
        grid_modulator->update(enable, operating_mode, force_request, is_current_zero, test_pi_angle, false);
    }
};

}  // namespace

// ========================================
// Basic Integration Tests
// ========================================

TEST_F(SfcCombinedModulatorsTest, InitialStateCoherence)
{
    // Both modulators should start in safe, coherent states
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);

    EXPECT_FALSE(grid_modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(machine_modulator->get_request_force_commutation());

    EXPECT_TRUE(machine_modulator->get_release_pulses());
    EXPECT_FALSE(grid_modulator->get_is_requesting_main_pi_reset());
}

TEST_F(SfcCombinedModulatorsTest, LoadCommutatedModeNoInteraction)
{
    // In load commutated mode, no handshake should occur even with sector changes
    simulate_control_cycle(true, sfc_operating_mode_t::natural_commutation, deg_to_rad_helper(30.0), false);
    simulate_control_cycle(true, sfc_operating_mode_t::natural_commutation, deg_to_rad_helper(70.0), false);
    simulate_control_cycle(true, sfc_operating_mode_t::natural_commutation, deg_to_rad_helper(130.0), false);

    // Machine-side should never request commutation
    EXPECT_FALSE(machine_modulator->get_request_force_commutation());
    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(machine_modulator->get_release_pulses());

    // Grid-side should remain in natural commutation
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(grid_modulator->get_force_commutation_acknowledged());
    EXPECT_EQ(grid_modulator->get_firing_angle_rad().value(), test_pi_angle.value());
}

// ========================================
// Complete Handshake Protocol Tests
// ========================================

TEST_F(SfcCombinedModulatorsTest, CompleteHandshakeSequence)
{
    // Test complete handshake with direct signal connections

    // Phase 1: Start in sector 0 - both modulators in running state
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(30.0), false);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_TRUE(machine_modulator->get_release_pulses());
    EXPECT_FALSE(machine_modulator->get_request_force_commutation());

    // Phase 2: Move to sector 1 - machine-side initiates commutation request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_FALSE(machine_modulator->get_release_pulses());  // Machine blocks its pulses
    EXPECT_TRUE(machine_modulator->get_request_force_commutation());

    // Extra cycle for grid-side to see the request and respond
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    // Grid-side should enter forced commutation
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_TRUE(grid_modulator->get_is_requesting_main_pi_reset());
    EXPECT_EQ(grid_modulator->get_firing_angle_rad().value(), default_commutation_angle.value());

    // Phase 3: DC current reaches zero - grid-side acknowledges
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);

    EXPECT_TRUE(grid_modulator->get_force_commutation_acknowledged());
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Phase 4: Machine-side receives acknowledgment and clears request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(machine_modulator->get_release_pulses());  // Machine releases pulses
    EXPECT_FALSE(machine_modulator->get_request_force_commutation());

    // Extra cycle for grid-side to see the cleared request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);

    // Grid-side should return to natural commutation
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(grid_modulator->get_force_commutation_acknowledged());
    EXPECT_FALSE(grid_modulator->get_is_requesting_main_pi_reset());
}

TEST_F(SfcCombinedModulatorsTest, IntermediateHandshakeStateWithRequestStillActive)
{
    // Test the intermediate state where DC current is zero but request is still active

    // Start handshake
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(30.0), false);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);

    // Extra cycle for grid-side to see the request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // DC current reaches zero, but machine-side hasn't seen acknowledgment yet
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);

    // This is the intermediate state - grid acknowledges but stays in forced commutation
    EXPECT_TRUE(grid_modulator->get_force_commutation_acknowledged());
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
    EXPECT_TRUE(machine_modulator->get_request_force_commutation());  // Still requesting

    // Machine-side sees acknowledgment and clears request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_FALSE(machine_modulator->get_request_force_commutation());

    // Extra cycle for grid-side to see the cleared request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);

    // Grid-side transitions back to natural commutation
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
    EXPECT_FALSE(grid_modulator->get_force_commutation_acknowledged());
}

// ========================================
// Multiple Sector Changes Tests
// ========================================

TEST_F(SfcCombinedModulatorsTest, ConsecutiveSectorChanges)
{
    // Test multiple consecutive sector changes

    // First commutation: Sector 0 → 1
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(30.0), false);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);

    // Extra cycle for grid-side to see the request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Complete first handshake
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);

    // Second commutation: Sector 1 → 2
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(130.0), false);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_TRUE(machine_modulator->get_request_force_commutation());

    // Extra cycle for grid-side to see the request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(130.0), false);

    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Complete second handshake
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(130.0), true);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(130.0), true);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(130.0), true);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
}

TEST_F(SfcCombinedModulatorsTest, AllSectorTransitions)
{
    // Test all 6 sector transitions including wraparound
    const std::vector<std::pair<real_t, real_t>> sector_transitions = {
        {30.0, 70.0},    // Sector 0 → 1
        {90.0, 130.0},   // Sector 1 → 2
        {150.0, 190.0},  // Sector 2 → 3
        {210.0, 250.0},  // Sector 3 → 4
        {270.0, 310.0},  // Sector 4 → 5
        {330.0, 10.0}    // Sector 5 → 0 (wraparound)
    };

    for (size_t i = 0; i < sector_transitions.size(); ++i) {
        const auto& transition = sector_transitions[i];

        // Reset for clean test
        machine_modulator->reset();
        grid_modulator->reset();

        // Start in first sector
        simulate_control_cycle(
            true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(transition.first), false);

        EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running)
            << "Failed at transition " << i << " initial state";

        // Move to second sector
        simulate_control_cycle(
            true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(transition.second), false);

        EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack)
            << "Failed at transition " << i << " (" << transition.first << "° → " << transition.second << "°)";
        EXPECT_TRUE(machine_modulator->get_request_force_commutation());

        // Extra cycle for grid-side to see the request
        simulate_control_cycle(
            true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(transition.second), false);

        EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

        // Complete handshake
        simulate_control_cycle(
            true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(transition.second), true);
        simulate_control_cycle(
            true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(transition.second), true);
        simulate_control_cycle(
            true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(transition.second), true);

        EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running)
            << "Failed to complete transition " << i;
        EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
    }
}

// ========================================
// Edge Cases and Error Conditions
// ========================================

TEST_F(SfcCombinedModulatorsTest, SystemDisableInterruptsHandshake)
{
    // Start handshake
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(30.0), false);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);

    // Extra cycle for grid-side to see the request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Disable system during handshake
    simulate_control_cycle(false, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    // Machine-side should clear its state immediately
    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_FALSE(machine_modulator->get_request_force_commutation());
    EXPECT_FALSE(machine_modulator->get_release_pulses());  // Blocked when disabled

    // Grid-side should return to natural commutation (request is now false)
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
}

TEST_F(SfcCombinedModulatorsTest, ModeChangeInterruptsHandshake)
{
    // Start handshake in pulse mode
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(30.0), false);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_TRUE(machine_modulator->get_request_force_commutation());

    // Extra cycle for grid-side to see the request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Switch to load commutated mode during handshake
    simulate_control_cycle(true, sfc_operating_mode_t::natural_commutation, deg_to_rad_helper(70.0), false);

    // Machine-side should clear commutation request immediately
    EXPECT_FALSE(machine_modulator->get_request_force_commutation());
    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);

    // Grid-side should return to natural commutation (request is now false)
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
}

TEST_F(SfcCombinedModulatorsTest, DelayedDcCurrentZeroDetection)
{
    // Test scenario where DC current zero detection is delayed

    // Start handshake
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(30.0), false);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    // Extra cycle for grid-side to see the request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // DC current takes several cycles to reach zero (realistic scenario)
    for (int i = 0; i < 5; ++i) {
        simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

        // Grid should stay in forced commutation, no acknowledgment yet
        EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);
        EXPECT_FALSE(grid_modulator->get_force_commutation_acknowledged());
        EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    }

    // Finally, DC current reaches zero
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);

    EXPECT_TRUE(grid_modulator->get_force_commutation_acknowledged());

    // Complete the handshake
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), true);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
}

TEST_F(SfcCombinedModulatorsTest, RapidSectorChangesBeforeAcknowledgment)
{
    // Test rapid sector changes before acknowledgment is received

    // Start first commutation
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(30.0), false);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);

    // Extra cycle for grid-side to see the request
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(70.0), false);

    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Move to another sector before acknowledgment - should remain in waiting state
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(130.0), false);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_TRUE(machine_modulator->get_request_force_commutation());
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::forced_commutation);

    // Complete handshake
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(130.0), true);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(130.0), true);
    simulate_control_cycle(true, sfc_operating_mode_t::forced_commutation, deg_to_rad_helper(130.0), true);

    EXPECT_EQ(machine_modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_EQ(grid_modulator->get_current_state(), grid_side_modulator_state_t::running);
}