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
    void set_all_inputs(bool grid_freeze = false,
        bool emergency_stop = false,
        bool torque_saturated = false,
        bool speed_enable = true,
        bool speed_preset_complete = false,
        bool torque_enable = true,
        bool torque_preset_complete = false,
        bool override_freeze = false)
    {
        supervisor.update(grid_freeze, emergency_stop, torque_saturated, speed_enable, speed_preset_complete,
            torque_enable, torque_preset_complete, override_freeze);
    }
};

// ============================================================================
// Freeze Logic Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, InitialStateNotFrozen)
{
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
    EXPECT_FALSE(supervisor.get_is_tripped());
}

TEST_F(MachineSideIntegratorSupervisorTest, FreezeOnGridSideFreeze)
{
    set_all_inputs(true, false, false);  // grid_side_freeze = true

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
    EXPECT_FALSE(supervisor.get_is_tripped());  // Not a trip from grid
}

TEST_F(MachineSideIntegratorSupervisorTest, FreezeOnEmergencyStop)
{
    set_all_inputs(false, true, false);  // emergency_stop = true

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, FreezeOnTorqueSaturated)
{
    set_all_inputs(false, false, true);  // torque_saturated = true

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, FreezeOnOrLogic)
{
    // Grid freeze alone
    set_all_inputs(true, false, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    supervisor.reset();

    // Emergency stop alone
    set_all_inputs(false, true, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    supervisor.reset();

    // Torque saturated alone
    set_all_inputs(false, false, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    supervisor.reset();

    // All together
    set_all_inputs(true, true, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());
}

TEST_F(MachineSideIntegratorSupervisorTest, UnfreezeWhenAllSourcesCleared)
{
    // Enter freeze
    set_all_inputs(true, true, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Clear all sources
    set_all_inputs(false, false, false);

    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_release_pulses());  // Still blocked by preset
}

TEST_F(MachineSideIntegratorSupervisorTest, OverrideFreezeForTesting)
{
    // Apply all freeze conditions but with override
    set_all_inputs(true, true, true, true, false, true, false, true);  // override_freeze = true

    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

// ============================================================================
// Anti-Windup (Torque Saturation) Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, TorqueSaturationFreezesIntegrators)
{
    // Torque saturated signal causes freeze
    set_all_inputs(false, false, true);

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, TorqueSaturationReleasesWithoutPreset)
{
    // Enter saturation freeze
    set_all_inputs(false, false, true);

    // Clear saturation
    set_all_inputs(false, false, false);

    // Should unfreeze with preset request
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_preset_request());
}

TEST_F(MachineSideIntegratorSupervisorTest, GridFreezeOverridesTorqueSaturation)
{
    // Both grid freeze and torque saturation
    set_all_inputs(true, false, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Clear torque saturation but keep grid freeze
    set_all_inputs(true, false, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());
}

// ============================================================================
// Trip Logic Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, TripLatchesOnEmergencyStop)
{
    EXPECT_FALSE(supervisor.get_is_tripped());

    set_all_inputs(false, true, false);  // emergency_stop = true
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Clear emergency stop - trip should remain latched
    set_all_inputs(false, false, false);
    EXPECT_TRUE(supervisor.get_is_tripped());
}

TEST_F(MachineSideIntegratorSupervisorTest, GridFreezeDoesNotTrip)
{
    set_all_inputs(true, false, false);  // grid_side_freeze = true

    EXPECT_FALSE(supervisor.get_is_tripped());
}

TEST_F(MachineSideIntegratorSupervisorTest, TorqueSaturationDoesNotTrip)
{
    set_all_inputs(false, false, true);  // torque_saturated = true

    EXPECT_FALSE(supervisor.get_is_tripped());
}

TEST_F(MachineSideIntegratorSupervisorTest, ReleasePulsesBlockedWhenTripped)
{
    // Trigger trip via emergency stop
    set_all_inputs(false, true, false);

    // Clear freeze sources
    set_all_inputs(false, false, false);

    // release_pulses should still be false due to trip
    EXPECT_FALSE(supervisor.get_release_pulses());
}

// ============================================================================
// Preset Handshake Logic Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, PresetRequestActivatesOnUnfreeze)
{
    // Enter freeze
    set_all_inputs(true, false, false);

    // Exit freeze
    set_all_inputs(false, false, false);

    EXPECT_TRUE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, PresetCompletesWithSpeedController)
{
    // Enter freeze and exit
    set_all_inputs(true, false, false);
    set_all_inputs(false, false, false);

    EXPECT_TRUE(supervisor.get_preset_request());

    // Speed controller signals preset complete
    set_all_inputs(false, false, false, true, true, true, false);

    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, PresetCompletesWithTorqueController)
{
    // Enter freeze and exit
    set_all_inputs(true, false, false);
    set_all_inputs(false, false, false);

    EXPECT_TRUE(supervisor.get_preset_request());

    // Torque controller signals preset complete
    set_all_inputs(false, false, false, true, false, true, true);

    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, PresetCompletesWithEitherController)
{
    // Either controller signaling completion should clear preset_request
    for (int i = 0; i < 2; ++i) {
        supervisor.reset();

        // Enter freeze and exit
        set_all_inputs(true, false, false);
        set_all_inputs(false, false, false);

        EXPECT_TRUE(supervisor.get_preset_request());

        // One controller signals preset complete
        if (i == 0) {
            set_all_inputs(false, false, false, true, true, true, false);
        }
        else {
            set_all_inputs(false, false, false, true, false, true, true);
        }

        EXPECT_FALSE(supervisor.get_preset_request());
    }
}

TEST_F(MachineSideIntegratorSupervisorTest, ReleasePulsesAfterPresetCompletion)
{
    // Full sequence
    set_all_inputs(false, false, false);  // Start unfrozen

    // Freeze
    set_all_inputs(true, false, false);
    EXPECT_FALSE(supervisor.get_release_pulses());

    // Unfreeze with preset request
    set_all_inputs(false, false, false);
    EXPECT_FALSE(supervisor.get_release_pulses());
    EXPECT_TRUE(supervisor.get_preset_request());

    // Complete preset
    set_all_inputs(false, false, false, true, true, true, false);
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, PresetResetOnNextCycle)
{
    // Perform preset sequence
    set_all_inputs(true, false, false);
    set_all_inputs(false, false, false);
    set_all_inputs(false, false, false, true, true, true, false);

    // Next cycle
    set_all_inputs(false, false, false);

    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

// ============================================================================
// Multiple Freeze Sources Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, GridAndEmergencySimultaneous)
{
    set_all_inputs(true, true, false);

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_is_tripped());  // Trip from emergency stop
}

TEST_F(MachineSideIntegratorSupervisorTest, AllFreezeSources)
{
    set_all_inputs(true, true, true);

    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_is_tripped());  // Trip from emergency stop
}

TEST_F(MachineSideIntegratorSupervisorTest, TransitionFromMultipleToSingleSource)
{
    // Start with all sources
    set_all_inputs(true, true, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Remove emergency stop but keep grid freeze and saturation
    set_all_inputs(true, false, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Remove saturation but keep grid freeze
    set_all_inputs(true, false, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Remove grid freeze
    set_all_inputs(false, false, false);
    EXPECT_FALSE(supervisor.get_freeze_integrator());
}

// ============================================================================
// State Transitions Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, MultipleFreezeUnfreezeCycles)
{
    for (int i = 0; i < 3; ++i) {
        // Freeze
        set_all_inputs(true, false, false);
        EXPECT_TRUE(supervisor.get_freeze_integrator());

        // Unfreeze with preset
        set_all_inputs(false, false, false);
        EXPECT_TRUE(supervisor.get_preset_request());

        // Complete preset
        set_all_inputs(false, false, false, true, true, true, false);
        EXPECT_TRUE(supervisor.get_release_pulses());
    }
}

TEST_F(MachineSideIntegratorSupervisorTest, ResetClearsAllState)
{
    // Create complex state
    set_all_inputs(true, true, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Reset
    supervisor.reset();

    // All state cleared
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_FALSE(supervisor.get_is_tripped());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

// ============================================================================
// Controller Enable Parameters Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, EnableParametersCurrentlyIgnored)
{
    // enable_speed_controller and enable_torque_controller are read but not used yet
    // This tests that they don't affect the logic

    set_all_inputs(false, false, false, true, false, true, false);  // Both enabled
    EXPECT_TRUE(supervisor.get_release_pulses());

    set_all_inputs(false, false, false, false, false, false, false);  // Both disabled
    // No change expected (for now)
    EXPECT_TRUE(supervisor.get_release_pulses());
}

// ============================================================================
// Edge Cases Tests
// ============================================================================

TEST_F(MachineSideIntegratorSupervisorTest, PresetCompleteIgnoredWhenNotRequested)
{
    // Signal preset complete without requesting preset
    set_all_inputs(false, false, false, true, true, true, true);

    EXPECT_FALSE(supervisor.get_preset_request());
    EXPECT_TRUE(supervisor.get_release_pulses());
}

TEST_F(MachineSideIntegratorSupervisorTest, QuickTransitions)
{
    // Rapidly toggle freeze sources
    for (int i = 0; i < 10; ++i) {
        set_all_inputs(i % 2 == 0, false, false);
    }

    EXPECT_TRUE(supervisor.get_freeze_integrator());
}

TEST_F(MachineSideIntegratorSupervisorTest, SaturationDuringFreezeFromGrid)
{
    // Freeze from grid
    set_all_inputs(true, false, false);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Torque saturates while frozen
    set_all_inputs(true, false, true);
    EXPECT_TRUE(supervisor.get_freeze_integrator());

    // Both clear
    set_all_inputs(false, false, false);
    EXPECT_FALSE(supervisor.get_freeze_integrator());
    EXPECT_TRUE(supervisor.get_preset_request());
}

TEST_F(MachineSideIntegratorSupervisorTest, EmergencyStopDuringTorqueSaturation)
{
    // Torque saturated
    set_all_inputs(false, false, true);
    EXPECT_FALSE(supervisor.get_is_tripped());

    // Emergency stop activates
    set_all_inputs(false, true, true);
    EXPECT_TRUE(supervisor.get_is_tripped());

    // Both clear
    set_all_inputs(false, false, false);
    EXPECT_TRUE(supervisor.get_is_tripped());  // Trip remains latched
}
