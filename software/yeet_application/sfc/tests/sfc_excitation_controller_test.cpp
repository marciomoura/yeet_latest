#include "sfc/sfc_excitation_controller.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
using yeet::current_pu_t;
using yeet::duration_t;
using yeet::real_t;
using yeet::speed_pu_t;
using yeet::voltage_pu_t;

class SfcExcitationControllerTest : public ::testing::Test {
protected:
    static constexpr duration_t kSamplingTime{0.001};  // 1 ms = 1 kHz control loop
    sfc_excitation_controller controller{kSamplingTime};

    void SetUp() override { controller.reset(); }
};

// ============================================================================
// Construction and Initialization Tests
// ============================================================================

TEST_F(SfcExcitationControllerTest, ConstructorInitializesWithDefaultValues)
{
    EXPECT_EQ(controller.get_field_current_reference_pu().value(), 0.0);
    EXPECT_FALSE(controller.get_enable_excitation());
    EXPECT_FALSE(controller.get_is_field_established());
    EXPECT_FALSE(controller.get_is_running());
    EXPECT_EQ(controller.get_flux_buildup_elapsed_time().value(), 0.0);  // Renamed getter
}

TEST_F(SfcExcitationControllerTest, ResetClearsAllStates)
{
    // Start and run for a while
    controller.start_excitation();
    for (int i = 0; i < 10; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
    }

    // Reset and verify all states cleared
    controller.reset();

    EXPECT_EQ(controller.get_field_current_reference_pu().value(), 0.0);
    EXPECT_FALSE(controller.get_enable_excitation());
    EXPECT_FALSE(controller.get_is_field_established());
    EXPECT_FALSE(controller.get_is_running());
    EXPECT_EQ(controller.get_flux_buildup_elapsed_time().value(), 0.0);  // Renamed getter
}

// ============================================================================
// Configuration Tests - Simplified Version
// ============================================================================

TEST_F(SfcExcitationControllerTest, ConfigureFieldCurrentSetpointAcceptsPositiveValues)
{
    EXPECT_NO_FATAL_FAILURE(controller.configure_field_current_setpoint(current_pu_t{0.5}));
    EXPECT_NO_FATAL_FAILURE(controller.configure_field_current_setpoint(current_pu_t{0.8}));
    EXPECT_NO_FATAL_FAILURE(controller.configure_field_current_setpoint(current_pu_t{1.0}));
}

TEST_F(SfcExcitationControllerTest, ConfigureFluxBuildupDurationAcceptsPositiveValues)
{
    EXPECT_NO_FATAL_FAILURE(controller.configure_flux_buildup_duration(duration_t{1.5}));
    EXPECT_NO_FATAL_FAILURE(controller.configure_flux_buildup_duration(duration_t{3.0}));
}

// Commented out - features removed in simplified version
// TEST_F(SfcExcitationControllerTest, ConfigureExcitationVoltageLookupTableAcceptsValidData)
// TEST_F(SfcExcitationControllerTest, ConfigureFieldEstablishedTimeoutAcceptsPositiveValues)
// TEST_F(SfcExcitationControllerTest, ConfigureExcitationRampDownTimeAcceptsPositiveValues)

// ============================================================================
// Start/Stop Command Tests - Simplified Version
// ============================================================================

TEST_F(SfcExcitationControllerTest, StartExcitationEnablesAndSetsConstantCurrent)
{
    controller.configure_field_current_setpoint(current_pu_t{0.75});
    controller.start_excitation();

    EXPECT_TRUE(controller.get_is_running());
    EXPECT_TRUE(controller.get_enable_excitation());

    // Should set field current to configured setpoint immediately
    EXPECT_NEAR(controller.get_field_current_reference_pu().value(), 0.75, 0.01);
}

TEST_F(SfcExcitationControllerTest, UpdateAutoStartsWhenExciterReadyAndNotRunning)
{
    controller.configure_field_current_setpoint(current_pu_t{0.8});

    // Controller not started yet
    ASSERT_FALSE(controller.get_is_running());

    // Calling update(true) when not running automatically starts excitation
    controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);

    EXPECT_TRUE(controller.get_is_running());
    EXPECT_TRUE(controller.get_enable_excitation());
    EXPECT_NEAR(controller.get_field_current_reference_pu().value(), 0.8, 0.01);
}

TEST_F(SfcExcitationControllerTest, ExciterNotReadyStopsExcitation)
{
    // Current implementation: stop_excitation() is not implemented
    // Stopping is achieved by setting exciter_ready = false in update()
    controller.configure_field_current_setpoint(current_pu_t{0.8});
    controller.start_excitation();
    controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);

    ASSERT_TRUE(controller.get_is_running());
    ASSERT_GT(controller.get_field_current_reference_pu().value(), 0.0);

    // Stop by setting exciter_ready = false
    controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, false);

    EXPECT_FALSE(controller.get_is_running());
    EXPECT_FALSE(controller.get_enable_excitation());
    EXPECT_EQ(controller.get_field_current_reference_pu().value(), 0.0);
}

// Commented out - simplified version allows repeated starts
// TEST_F(SfcExcitationControllerTest, StartExcitationReturnsFalseWhenAlreadyRunning)

// ============================================================================
// Flux Build-Up Phase Tests (Simplified - was Pre-Fluxing)
// ============================================================================

TEST_F(SfcExcitationControllerTest, FluxBuildupPhaseMaintainsConstantCurrentForConfiguredDuration)
{
    controller.configure_field_current_setpoint(current_pu_t{0.8});
    controller.configure_flux_buildup_duration(duration_t{2.0});  // 2 seconds
    controller.start_excitation();

    const real_t expected_current = 0.8;

    // Run for less than flux build-up duration
    const int num_cycles = static_cast<int>(1.5 / kSamplingTime.value());  // 1.5 seconds
    for (int i = 0; i < num_cycles; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
    }

    // Field should not be established yet
    EXPECT_FALSE(controller.get_is_field_established());

    // Current should remain at constant setpoint
    EXPECT_NEAR(controller.get_field_current_reference_pu().value(), expected_current, 0.01);
}

TEST_F(SfcExcitationControllerTest, FluxBuildupTimerAdvancesDuringFluxBuildup)
{
    controller.configure_flux_buildup_duration(duration_t{2.0});
    controller.start_excitation();

    // Run for 1 second
    const int num_cycles = static_cast<int>(1.0 / kSamplingTime.value());
    for (int i = 0; i < num_cycles; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
    }

    // Check elapsed time is approximately 1 second
    EXPECT_NEAR(controller.get_flux_buildup_elapsed_time().value(), 1.0, 0.1);
}

TEST_F(SfcExcitationControllerTest, FieldEstablishedAfterFluxBuildupDurationElapses)
{
    controller.configure_flux_buildup_duration(duration_t{1.0});  // 1 second
    controller.start_excitation();

    // Field should not be established initially
    EXPECT_FALSE(controller.get_is_field_established());

    // Run for exactly flux build-up duration + buffer
    const int num_cycles = static_cast<int>(1.2 / kSamplingTime.value());
    for (int i = 0; i < num_cycles; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
    }

    // Field should now be established
    EXPECT_TRUE(controller.get_is_field_established());
}

// ============================================================================
// Normal Operation Tests (Simplified - Constant Flux)
// ============================================================================

TEST_F(SfcExcitationControllerTest, FieldCurrentRemainsConstantAfterFieldEstablished)
{
    controller.configure_field_current_setpoint(current_pu_t{0.9});
    controller.configure_flux_buildup_duration(duration_t{0.1});  // Short for test
    controller.start_excitation();

    // Complete flux build-up
    const int buildup_cycles = static_cast<int>(0.15 / kSamplingTime.value());
    for (int i = 0; i < buildup_cycles; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
    }

    ASSERT_TRUE(controller.get_is_field_established());

    // Run for a while - field current should remain constant (no speed dependency)
    for (int i = 0; i < 1000; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
        EXPECT_NEAR(controller.get_field_current_reference_pu().value(), 0.9, 0.01);
    }
}

// Commented out - removed in simplified version (no speed-dependent control)
// TEST_F(SfcExcitationControllerTest, ExcitationVoltageTracksSpeedAfterFieldEstablished)
// TEST_F(SfcExcitationControllerTest, ExcitationVoltageSaturationAtHighSpeed)

// ============================================================================
// Fault Handling Tests - Simplified Version
// ============================================================================

TEST_F(SfcExcitationControllerTest, ExciterNotReadyDuringOperationTriggersImmediateStop)
{
    controller.configure_flux_buildup_duration(duration_t{0.1});
    controller.start_excitation();

    // Complete flux build-up
    for (int i = 0; i < 150; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
    }

    ASSERT_TRUE(controller.get_is_field_established());
    ASSERT_TRUE(controller.get_is_running());

    // Simulate exciter fault - simplified version stops immediately, no ramp-down
    controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, false);

    // Should stop immediately
    EXPECT_FALSE(controller.get_is_running());
    EXPECT_FALSE(controller.get_enable_excitation());
    EXPECT_EQ(controller.get_field_current_reference_pu().value(), 0.0);
}

TEST_F(SfcExcitationControllerTest, UpdateWhenNotRunningAndExciterNotReadyMaintainsZeroOutput)
{
    // Current implementation: update(true) auto-starts when not running
    // To test zero output, we must use exciter_ready = false
    for (int i = 0; i < 100; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, false);
    }

    EXPECT_EQ(controller.get_field_current_reference_pu().value(), 0.0);
    EXPECT_FALSE(controller.get_enable_excitation());
    EXPECT_FALSE(controller.get_is_running());
}

// ============================================================================
// Complete Operational Sequence Test - Simplified Version
// ============================================================================

TEST_F(SfcExcitationControllerTest, CompleteStartupToShutdownSequence)
{
    controller.configure_field_current_setpoint(current_pu_t{0.85});
    controller.configure_flux_buildup_duration(duration_t{1.0});

    // 1. Initial state
    EXPECT_FALSE(controller.get_is_running());

    // 2. Start excitation - can be done via start_excitation() or update(true) when not running
    controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
    EXPECT_TRUE(controller.get_is_running());
    EXPECT_TRUE(controller.get_enable_excitation());
    EXPECT_NEAR(controller.get_field_current_reference_pu().value(), 0.85, 0.01);

    // 3. Flux build-up phase (1 second)
    for (int i = 0; i < 1200; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
    }
    EXPECT_TRUE(controller.get_is_field_established());

    // 4. Normal operation - current remains constant
    for (int i = 0; i < 1000; ++i) {
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
        EXPECT_NEAR(controller.get_field_current_reference_pu().value(), 0.85, 0.01);
    }

    // 5. Shutdown - set exciter_ready = false
    controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, false);

    // Should be fully stopped immediately
    EXPECT_FALSE(controller.get_is_running());
    EXPECT_FALSE(controller.get_enable_excitation());
    EXPECT_EQ(controller.get_field_current_reference_pu().value(), 0.0);
}

// Commented out - edge cases removed in simplified version (no speed input)
// TEST_F(SfcExcitationControllerTest, NegativeSpeedInputHandledGracefully)
// TEST_F(SfcExcitationControllerTest, OverSpeedInputHandledGracefully)

TEST_F(SfcExcitationControllerTest, RapidStartStopCyclesHandledCorrectly)
{
    for (int cycle = 0; cycle < 5; ++cycle) {
        controller.reset();
        controller.configure_field_current_setpoint(current_pu_t{0.7});

        // Start via update(true) - auto-start when not running
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
        EXPECT_TRUE(controller.get_is_running());

        // Run briefly
        for (int i = 0; i < 50; ++i) {
            controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, true);
        }

        // Stop via exciter_ready = false
        controller.update(speed_pu_t{0.0}, voltage_pu_t{0.0}, false);
        EXPECT_FALSE(controller.get_is_running());
    }
}
