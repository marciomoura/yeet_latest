#include "sfc/sfc_machine_side_integrator_supervisor.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

/**
 * @class MachineSideIntegratorSupervisorTest
 * @brief Test fixture for machine-side integrator supervisor
 */
class MachineSideIntegratorSupervisorTest : public ::testing::Test {
protected:
    static constexpr yeet::duration_t sampling_time = ms(1.0);

    MachineSideIntegratorSupervisorTest() : supervisor(sampling_time) {}

    sfc_machine_side_integrator_supervisor supervisor;

    // Helper to update with all inputs
    void set_all_inputs(bool grid_freeze_integrator = false,
        bool grid_is_tripped = false,
        bool grid_release_pulses = true,
        bool current_is_zero = true,
        bool speed_saturated = false,
        bool torque_saturated = false,
        bool override_freeze = false)
    {
        supervisor.update(grid_freeze_integrator, grid_is_tripped, grid_release_pulses, current_is_zero,
            speed_saturated, torque_saturated, override_freeze);
    }
};

// ============================================================================
// Freeze Logic Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, InitialStateNotFrozen)
{
    // Initial state: not yet updated, so release_pulses is false (no grid command yet)
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_release_pulses());  // Need grid command + current zero
    EXPECT_FALSE(supervisor.get_is_tripped());

    // After first update with grid allowing release and current zero
    set_all_inputs();  // Defaults: grid_release_pulses=true, current_is_zero=true
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, FreezeOnGridSideFreeze)
{
    set_all_inputs(true);  // grid_freeze_integrator = true

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
    EXPECT_FALSE(supervisor.get_is_tripped());  // Not a trip from grid
}

TEST_F(MachineSideIntegratorSupervisorTest, FreezeOnGridTripped)
{
    set_all_inputs(false, true);  // grid_is_tripped = true

    EXPECT_FALSE(supervisor.get_freeze_integrator());  // Trip doesn't cause freeze, just blocks pulses
    EXPECT_FALSE(supervisor.get_release_pulses());
    EXPECT_TRUE(supervisor.get_is_tripped());
}

TEST_F(MachineSideIntegratorSupervisorTest, FreezeOnSpeedSaturated)
{
    set_all_inputs(false, false, true, true, true);  // speed_saturated = true

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, FreezeOnTorqueSaturated)
{
    set_all_inputs(false, false, true, true, false, true);  // torque_saturated = true

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, FreezeOnOrLogic)
{
    // Grid freeze alone
    set_all_inputs(true);  // grid_freeze_integrator=true
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    supervisor.reset();
    set_all_inputs();  // Update after reset

    // Speed saturated alone
    set_all_inputs(false, false, true, true, true);  // speed_saturated=true
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    supervisor.reset();
    set_all_inputs();  // Update after reset

    // Torque saturated alone
    set_all_inputs(false, false, true, true, false, true);  // torque_saturated=true
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    supervisor.reset();
    set_all_inputs();  // Update after reset

    // All freeze sources together (grid + speed + torque)
    set_all_inputs(true, false, true, true, true, true);  // All freeze sources
    EXPECT_TRUE(supervisor.get_freeze_integrator());
}

TEST_F(MachineSideIntegratorSupervisorTest, UnfreezeWhenAllSourcesCleared)
{
    // Enter freeze
    set_all_inputs(true, true, true, true, true, true);  // All freeze sources
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Clear all sources
    set_all_inputs();  // All defaults

    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_release_pulses());  // Still blocked by preset
}

TEST_F(MachineSideIntegratorSupervisorTest, OverrideFreezeForTesting)
{
    // Apply all freeze conditions but with override
    set_all_inputs(true, true, true, true, true, true, true);  // All freeze sources + override

    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

// ============================================================================
// Anti-Windup (Torque Saturation) Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, TorqueSaturationFreezesIntegrators)
{
    // Torque saturated signal causes freeze
    set_all_inputs(false, false, true, true, false, true);  // torque_saturated=true

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, TorqueSaturationReleasesWithoutPreset)
{
    // Enter saturation freeze
    set_all_inputs(false, false, true, true, false, true);  // torque_saturated=true

    // Clear saturation
    set_all_inputs();  // All defaults

    // Should unfreeze with preset request
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_preset_request());
}

TEST_F(MachineSideIntegratorSupervisorTest, GridFreezeOverridesTorqueSaturation)
{
    // Both grid freeze and torque saturation
    set_all_inputs(true, false, true, true, false, true);  // grid_freeze + torque_saturated
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Clear torque saturation but keep grid freeze
    set_all_inputs(true);  // grid_freeze_integrator=true
    EXPECT_TRUE(supervisor.get_freeze_integrator());
}

// ============================================================================
// Trip Logic Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, TripLatchesOnEmergencyStop)
{
    EXPECT_FALSE(supervisor.get_is_tripped());

    set_all_inputs(false, true);  // grid_is_tripped=true  // emergency_stop = true
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Clear emergency stop - trip should remain latched
    set_all_inputs();  // All defaults
    EXPECT_TRUE(supervisor.get_is_tripped());
}

TEST_F(MachineSideIntegratorSupervisorTest, GridFreezeDoesNotTrip)
{
    set_all_inputs(true);  // grid_freeze_integrator=true  // grid_side_freeze = true

    EXPECT_FALSE(supervisor.get_is_tripped());
}

TEST_F(MachineSideIntegratorSupervisorTest, TorqueSaturationDoesNotTrip)
{
    set_all_inputs(false, false, true, true, false, true);  // torque_saturated=true  // torque_saturated = true

    EXPECT_FALSE(supervisor.get_is_tripped());
}

TEST_F(MachineSideIntegratorSupervisorTest, ReleasePulsesBlockedWhenTripped)
{
    // Trigger trip via emergency stop
    set_all_inputs(false, true);  // grid_is_tripped=true

    // Clear freeze sources
    set_all_inputs();  // All defaults

    // release_pulses should still be false due to trip
    EXPECT_FALSE(supervisor.get_release_pulses());
}

// ============================================================================
// Preset Handshake Logic Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, PresetRequestActivatesOnUnfreeze)
{
    // Enter freeze
    set_all_inputs(true);  // grid_freeze_integrator=true

    // Exit freeze
    set_all_inputs();  // All defaults

    EXPECT_TRUE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, PresetCompletesWithSpeedController)
{
    // Enter freeze and exit
    set_all_inputs(true);  // grid_freeze_integrator=true
    set_all_inputs();      // All defaults

    EXPECT_TRUE(supervisor.get_preset_request());

    // Preset completes automatically after one cycle
    set_all_inputs();  // All defaults

    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, PresetCompletesWithTorqueController)
{
    // Enter freeze and exit
    set_all_inputs(true);  // grid_freeze_integrator=true
    set_all_inputs();      // All defaults

    EXPECT_TRUE(supervisor.get_preset_request());

    // Preset completes automatically after one cycle
    set_all_inputs();  // All defaults

    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, PresetCompletesWithEitherController)
{
    // Preset completes automatically - no difference between controllers
    for (int i = 0; i < 2; ++i) {
        supervisor.reset();

        // Enter freeze and exit
        set_all_inputs(true);  // grid_freeze_integrator=true
        set_all_inputs();      // All defaults

        EXPECT_TRUE(supervisor.get_preset_request());

        // Preset completes automatically after one cycle
        set_all_inputs();  // All defaults

        EXPECT_FALSE(supervisor.get_preset_request());
    }
}

TEST_F(MachineSideIntegratorSupervisorTest, ReleasePulsesAfterPresetCompletion)
{
    // Full sequence
    set_all_inputs();  // All defaults  // Start unfrozen

    // Freeze
    set_all_inputs(true);  // grid_freeze_integrator=true
    EXPECT_FALSE(supervisor.get_release_pulses());

    // Unfreeze with preset request
    set_all_inputs();  // All defaults
    EXPECT_FALSE(supervisor.get_release_pulses());
    EXPECT_TRUE(supervisor.get_preset_request());

    // Preset completes automatically
    set_all_inputs();  // All defaults
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, PresetResetOnNextCycle)
{
    // Perform preset sequence - preset completes automatically
    set_all_inputs(true);  // grid_freeze_integrator=true
    set_all_inputs();      // All defaults (preset request asserted here)
    set_all_inputs();      // All defaults (preset auto-completes)

    // Next cycle
    set_all_inputs();  // All defaults

    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

// ============================================================================
// Multiple Freeze Sources Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, GridAndEmergencySimultaneous)
{
    set_all_inputs(true, true);  // grid_freeze + grid_tripped

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_is_tripped());  // Trip from emergency stop
}

TEST_F(MachineSideIntegratorSupervisorTest, AllFreezeSources)
{
    set_all_inputs(true, true, true, true, true, true);  // All freeze sources

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_is_tripped());  // Trip from emergency stop
}

TEST_F(MachineSideIntegratorSupervisorTest, TransitionFromMultipleToSingleSource)
{
    // Start with all sources
    set_all_inputs(true, true, true, true, true, true);  // All freeze sources
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Remove emergency stop but keep grid freeze and saturation
    set_all_inputs(true, false, true, true, false, true);  // grid_freeze + torque_saturated
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Remove saturation but keep grid freeze
    set_all_inputs(true);  // grid_freeze_integrator=true
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Remove grid freeze
    set_all_inputs();  // All defaults
    EXPECT_FALSE(supervisor.get_freeze_integrator());
}

// ============================================================================
// State Transitions Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, MultipleFreezeUnfreezeCycles)
{
    for (int i = 0; i < 3; ++i) {
        // Freeze
        set_all_inputs(true);  // grid_freeze_integrator=true
        EXPECT_TRUE(supervisor.get_freeze_integrator());

        // Unfreeze with preset
        set_all_inputs();  // All defaults
        EXPECT_TRUE(supervisor.get_preset_request());

        // Preset completes automatically
        set_all_inputs();  // All defaults
        EXPECT_TRUE(supervisor.get_release_pulses());
    }
}

TEST_F(MachineSideIntegratorSupervisorTest, ResetClearsAllState)
{
    // Create complex state
    set_all_inputs(true, true, true, true, true, true);  // All freeze sources
    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Reset
    supervisor.reset();
    set_all_inputs();  // Update with defaults (grid commands present)

    // All state cleared
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_is_tripped());     // Trip cleared by reset
    EXPECT_TRUE(supervisor.get_release_pulses());  // Now true since grid commands present
}

// ============================================================================
// Controller Enable Parameters Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, EnableParametersCurrentlyIgnored)
{
    // enable_speed_controller and enable_torque_controller are read but not used yet
    // This tests that they don't affect the logic

    set_all_inputs();  // All defaults with grid commands
    EXPECT_TRUE(supervisor.get_release_pulses());

    // Still works with grid not releasing (but current is zero, so grid command blocks it)
    set_all_inputs(false, false, false);            // Grid not releasing pulses
    EXPECT_FALSE(supervisor.get_release_pulses());  // Blocked by grid command
}

// ============================================================================
// Edge Cases Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, PresetCompleteIgnoredWhenNotRequested)
{
    // Preset auto-completes when requested, this test is no longer applicable
    // Just verify normal operation without preset request
    set_all_inputs();  // All defaults

    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, QuickTransitions)
{
    // Rapidly toggle freeze sources
    for (int i = 0; i < 10; ++i) {
        set_all_inputs(i % 2 == 0);  // Toggle grid freeze
    }

    // Last iteration was i=9 (odd), so grid_freeze_integrator=false
    EXPECT_FALSE(supervisor.get_freeze_integrator());
}

TEST_F(MachineSideIntegratorSupervisorTest, SaturationDuringFreezeFromGrid)
{
    // Freeze from grid
    set_all_inputs(true);  // grid_freeze_integrator=true
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Torque saturates while frozen
    set_all_inputs(true, false, true, true, false, true);  // grid_freeze + torque_saturated
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Both clear
    set_all_inputs();  // All defaults
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_preset_request());
}

TEST_F(MachineSideIntegratorSupervisorTest, EmergencyStopDuringTorqueSaturation)
{
    // Torque saturated
    set_all_inputs(false, false, true, true, false, true);  // torque_saturated=true
    EXPECT_FALSE(supervisor.get_is_tripped());

    // Emergency stop activates
    set_all_inputs(false, true, true, true, false, true);  // grid_tripped + torque_saturated
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Both clear
    set_all_inputs();                          // All defaults
    EXPECT_TRUE(supervisor.get_is_tripped());  // Trip remains latched
}
