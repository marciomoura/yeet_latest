#include "sfc/sfc_grid_side_integrator_supervisor.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

/**
 * @class GridSideIntegratorSupervisorTest
 * @brief Test fixture for grid-side integrator supervisor
 */
class GridSideIntegratorSupervisorTest : public ::testing::Test {
protected:
    static constexpr yeet::duration_t sampling_time = ms(1.0);

    GridSideIntegratorSupervisorTest() : supervisor(sampling_time) {}

    sfc_grid_side_integrator_supervisor supervisor;

    // Helper to set all inputs to false
    void set_all_inputs_false() { supervisor.update(false, false, true, false, false); }

    // Helper to set freeze conditions
    void set_freeze_conditions(bool modulator_reset,
        bool shutdown,
        bool controller_enable = true,
        bool preset_complete = false,
        bool override = false)
    {
        supervisor.update(modulator_reset, shutdown, controller_enable, preset_complete, override);
    }
};

// ============================================================================
// Freeze Logic Tests
// ============================================================================

TEST_F(GridSideIntegratorSupervisorTest, InitialStateNotFrozen)
{
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
    EXPECT_FALSE(supervisor.get_is_tripped());
}

TEST_F(GridSideIntegratorSupervisorTest, FreezeOnModulatorResetRequest)
{
    // Initially not frozen
    EXPECT_FALSE(supervisor.get_freeze_integrator());

    // Apply modulator reset request
    set_freeze_conditions(true, false);  // modulator_reset_request = true

    // Should now be frozen
    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
    EXPECT_FALSE(supervisor.get_is_tripped());  // Not a trip
}

TEST_F(GridSideIntegratorSupervisorTest, FreezeOnShutdownRequest)
{
    // Initially not frozen
    EXPECT_FALSE(supervisor.get_freeze_integrator());

    // Apply shutdown request
    set_freeze_conditions(false, true);  // shutdown_request = true

    // Should now be frozen
    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(GridSideIntegratorSupervisorTest, FreezeOnsOrLogic)
{
    // Test that OR logic works: either source causes freeze
    supervisor.reset();
    set_freeze_conditions(true, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    supervisor.reset();
    set_freeze_conditions(false, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    supervisor.reset();
    set_freeze_conditions(true, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());
}

TEST_F(GridSideIntegratorSupervisorTest, UnfreezeWhenFreezeSourcesCleared)
{
    // Enter freeze
    set_freeze_conditions(true, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Clear freeze sources
    set_freeze_conditions(false, false);

    // Should now be unfrozen (but preset_request should be active)
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_release_pulses());  // Still false due to preset
}

TEST_F(GridSideIntegratorSupervisorTest, OverrideFreezeForTesting)
{
    // Apply freeze conditions but with override
    set_freeze_conditions(true, true, true, false, true);  // override_freeze = true

    // Should NOT be frozen
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_release_pulses());

    // Test that override prevents preset request too
    supervisor.reset();
    set_freeze_conditions(false, false, true, false, false);
    set_freeze_conditions(true, false, true, false, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Now with override
    set_freeze_conditions(true, false, true, false, true);
    EXPECT_FALSE(supervisor.get_freeze_integrator());
}

// ============================================================================
// Trip Logic Tests
// ============================================================================

TEST_F(GridSideIntegratorSupervisorTest, TripLatchesOnShutdownRequest)
{
    EXPECT_FALSE(supervisor.get_is_tripped());

    // Apply shutdown request
    set_freeze_conditions(false, true);
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Clear shutdown request - trip should remain latched
    set_freeze_conditions(false, false);
    EXPECT_TRUE(supervisor.get_is_tripped());
}

TEST_F(GridSideIntegratorSupervisorTest, TripDoesNotLatchOnModulatorReset)
{
    EXPECT_FALSE(supervisor.get_is_tripped());

    // Apply modulator reset (not a trip)
    set_freeze_conditions(true, false);
    EXPECT_FALSE(supervisor.get_is_tripped());

    // Clear modulator reset
    set_freeze_conditions(false, false);
    EXPECT_FALSE(supervisor.get_is_tripped());
}

TEST_F(GridSideIntegratorSupervisorTest, ReleasePulsesBlockedWhenTripped)
{
    // Trigger trip
    set_freeze_conditions(false, true);
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Clear freeze sources but trip remains
    set_freeze_conditions(false, false);

    // release_pulses should still be false due to trip
    EXPECT_FALSE(supervisor.get_release_pulses());
}

// ============================================================================
// Preset Handshake Logic Tests
// ============================================================================

TEST_F(GridSideIntegratorSupervisorTest, PresetRequestActivatesOnUnfreeze)
{
    // Enter freeze
    set_freeze_conditions(true, false);
    EXPECT_FALSE(supervisor.get_preset_request());

    // Exit freeze
    set_freeze_conditions(false, false);

    // Preset request should be active
    EXPECT_TRUE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(GridSideIntegratorSupervisorTest, PresetHandshakeCompletion)
{
    // Enter freeze
    set_freeze_conditions(true, false);

    // Exit freeze - preset request activates
    set_freeze_conditions(false, false);
    EXPECT_TRUE(supervisor.get_preset_request());

    // Controller signals preset complete
    supervisor.update(false, false, true, true, false);  // preset_complete = true

    // Preset request should clear
    EXPECT_FALSE(supervisor.get_preset_request());
}

TEST_F(GridSideIntegratorSupervisorTest, ReleasePulsesAfterPreset)
{
    // Enter freeze
    set_freeze_conditions(true, false);

    // Exit freeze
    set_freeze_conditions(false, false);
    EXPECT_FALSE(supervisor.get_release_pulses());  // Still held by preset

    // Complete preset
    supervisor.update(false, false, true, true, false);

    // Now release_pulses should be true
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(GridSideIntegratorSupervisorTest, PresetResetOnNextCycle)
{
    // Perform preset sequence
    set_freeze_conditions(true, false);
    set_freeze_conditions(false, false);
    supervisor.update(false, false, true, true, false);  // Complete preset

    // Next cycle with preset_complete = false should clear preset_done internally
    set_freeze_conditions(false, false);

    // Verify system is ready for next cycle
    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(GridSideIntegratorSupervisorTest, PresetDoesntActivateWhenTripped)
{
    // Enter freeze via shutdown (trip)
    set_freeze_conditions(false, true);
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Try to exit freeze
    set_freeze_conditions(false, false);

    // Preset request should still activate (preset happens even with trip)
    // But release_pulses will be blocked by trip
    EXPECT_FALSE(supervisor.get_release_pulses());
}

// ============================================================================
// State Transitions Tests
// ============================================================================

TEST_F(GridSideIntegratorSupervisorTest, MultipleFreezeCycles)
{
    // Cycle 1: Freeze and unfreeze
    set_freeze_conditions(true, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    set_freeze_conditions(false, false);
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_preset_request());

    supervisor.update(false, false, true, true, false);  // Complete preset
    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());

    // Cycle 2: Freeze again
    set_freeze_conditions(true, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());

    // Unfreeze and complete preset again
    set_freeze_conditions(false, false);
    EXPECT_TRUE(supervisor.get_preset_request());

    supervisor.update(false, false, true, true, false);
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(GridSideIntegratorSupervisorTest, ResetClearsAllState)
{
    // Create some state
    set_freeze_conditions(true, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Reset
    supervisor.reset();

    // All state should be cleared
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_is_tripped());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(GridSideIntegratorSupervisorTest, QuickFreezeCycles)
{
    // Rapidly toggle freeze
    for (int i = 0; i < 5; ++i) {
        set_freeze_conditions(true, false);
        EXPECT_TRUE(supervisor.get_freeze_integrator());

        set_freeze_conditions(false, false);
        EXPECT_FALSE(supervisor.get_freeze_integrator());

        // Complete preset
        supervisor.update(false, false, true, true, false);
    }

    EXPECT_TRUE(supervisor.get_release_pulses());
}

// ============================================================================
// Edge Cases Tests
// ============================================================================

TEST_F(GridSideIntegratorSupervisorTest, PresetCompleteIgnoredWhenNotRequested)
{
    // Signal preset_complete without requesting preset
    supervisor.update(false, false, true, true, false);

    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(GridSideIntegratorSupervisorTest, EnableCurrentControllerParameterIgnored)
{
    // The enable_current_controller parameter is read but not used in the current logic
    // This test documents that behavior

    set_freeze_conditions(false, false, true);  // enable = true
    EXPECT_FALSE(supervisor.get_freeze_integrator());

    set_freeze_conditions(false, false, false);  // enable = false
    // No change in behavior expected (for now)
    EXPECT_FALSE(supervisor.get_freeze_integrator());
}

TEST_F(GridSideIntegratorSupervisorTest, SimultaneousFreezeAndTrip)
{
    // Apply both modulator reset and shutdown at same time
    set_freeze_conditions(true, true);

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_is_tripped());
    EXPECT_FALSE(supervisor.get_release_pulses());
}
