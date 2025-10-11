#include "sfc/sfc_excitation_controller.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
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
    EXPECT_EQ(controller.get_excitation_voltage_setpoint_pu().value(), 0.0);
    EXPECT_FALSE(controller.get_enable_excitation());
    EXPECT_FALSE(controller.get_is_field_established());
    EXPECT_FALSE(controller.get_is_running());
    EXPECT_EQ(controller.get_prefluxing_elapsed_time().value(), 0.0);
}

TEST_F(SfcExcitationControllerTest, ResetClearsAllStates)
{
    // Start and run for a while
    controller.start_excitation();
    for (int i = 0; i < 10; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }

    // Reset and verify all states cleared
    controller.reset();

    EXPECT_EQ(controller.get_excitation_voltage_setpoint_pu().value(), 0.0);
    EXPECT_FALSE(controller.get_enable_excitation());
    EXPECT_FALSE(controller.get_is_field_established());
    EXPECT_FALSE(controller.get_is_running());
    EXPECT_EQ(controller.get_prefluxing_elapsed_time().value(), 0.0);
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(SfcExcitationControllerTest, ConfigureExcitationVoltageLookupTableAcceptsValidData)
{
    std::array<real_t, 6> speed_axis = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    std::array<real_t, 6> voltage_axis = {0.3, 0.5, 0.65, 0.75, 0.85, 0.95};

    // Should not throw or assert
    EXPECT_NO_FATAL_FAILURE(controller.configure_excitation_voltage_lookup_table(speed_axis, voltage_axis));
}

TEST_F(SfcExcitationControllerTest, ConfigurePrefluxingDurationAcceptsPositiveValues)
{
    EXPECT_NO_FATAL_FAILURE(controller.configure_prefluxing_duration(duration_t{1.5}));
    EXPECT_NO_FATAL_FAILURE(controller.configure_prefluxing_duration(duration_t{3.0}));
}

TEST_F(SfcExcitationControllerTest, ConfigureFieldEstablishedTimeoutAcceptsPositiveValues)
{
    EXPECT_NO_FATAL_FAILURE(controller.configure_field_established_timeout(duration_t{5.0}));
    EXPECT_NO_FATAL_FAILURE(controller.configure_field_established_timeout(duration_t{10.0}));
}

TEST_F(SfcExcitationControllerTest, ConfigureExcitationRampDownTimeAcceptsPositiveValues)
{
    EXPECT_NO_FATAL_FAILURE(controller.configure_excitation_ramp_down_time(duration_t{1.0}));
    EXPECT_NO_FATAL_FAILURE(controller.configure_excitation_ramp_down_time(duration_t{2.5}));
}

// ============================================================================
// Start/Stop Command Tests
// ============================================================================

TEST_F(SfcExcitationControllerTest, StartExcitationReturnsTrueWhenNotAlreadyRunning)
{
    EXPECT_TRUE(controller.start_excitation());
    EXPECT_TRUE(controller.get_is_running());
}

TEST_F(SfcExcitationControllerTest, StartExcitationReturnsFalseWhenAlreadyRunning)
{
    controller.start_excitation();
    EXPECT_FALSE(controller.start_excitation());  // Second call should fail
}

TEST_F(SfcExcitationControllerTest, StartExcitationEnablesOutputAndSetsInitialVoltage)
{
    controller.start_excitation();

    // Should enable excitation
    EXPECT_TRUE(controller.get_enable_excitation());

    // Should set excitation voltage to initial value (first point in lookup table)
    EXPECT_GT(controller.get_excitation_voltage_setpoint_pu().value(), 0.0);
}

TEST_F(SfcExcitationControllerTest, StopExcitationInitiatesRampDown)
{
    controller.start_excitation();
    controller.update(speed_pu_t{0.0}, true);

    controller.stop_excitation();

    // After stop command, controller should start ramping down
    // Verify by running several updates and checking voltage decreases
    const real_t initial_voltage = controller.get_excitation_voltage_setpoint_pu().value();

    for (int i = 0; i < 100; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }

    const real_t final_voltage = controller.get_excitation_voltage_setpoint_pu().value();
    EXPECT_LT(final_voltage, initial_voltage);
}

// ============================================================================
// Pre-Fluxing Phase Tests
// ============================================================================

TEST_F(SfcExcitationControllerTest, PrefluxingPhaseMaintainsInitialVoltageForConfiguredDuration)
{
    controller.configure_prefluxing_duration(duration_t{2.0});  // 2 seconds
    controller.start_excitation();

    const real_t initial_voltage = controller.get_excitation_voltage_setpoint_pu().value();

    // Run for less than pre-fluxing duration
    const int num_cycles = static_cast<int>(1.5 / kSamplingTime.value());  // 1.5 seconds
    for (int i = 0; i < num_cycles; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }

    // Field should not be established yet
    EXPECT_FALSE(controller.get_is_field_established());

    // Voltage should remain at initial value
    EXPECT_NEAR(controller.get_excitation_voltage_setpoint_pu().value(), initial_voltage, 0.01);
}

TEST_F(SfcExcitationControllerTest, PrefluxingTimerAdvancesDuringPrefluxing)
{
    controller.configure_prefluxing_duration(duration_t{2.0});
    controller.start_excitation();

    // Run for 1 second
    const int num_cycles = static_cast<int>(1.0 / kSamplingTime.value());
    for (int i = 0; i < num_cycles; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }

    // Check elapsed time is approximately 1 second
    EXPECT_NEAR(controller.get_prefluxing_elapsed_time().value(), 1.0, 0.1);
}

TEST_F(SfcExcitationControllerTest, FieldEstablishedAfterPrefluxingDurationElapses)
{
    controller.configure_prefluxing_duration(duration_t{1.0});  // 1 second
    controller.start_excitation();

    // Field should not be established initially
    EXPECT_FALSE(controller.get_is_field_established());

    // Run for exactly pre-fluxing duration + buffer
    const int num_cycles = static_cast<int>(1.2 / kSamplingTime.value());
    for (int i = 0; i < num_cycles; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }

    // Field should now be established
    EXPECT_TRUE(controller.get_is_field_established());
}

// ============================================================================
// Normal Operation Tests (Field Established)
// ============================================================================

TEST_F(SfcExcitationControllerTest, ExcitationVoltageTracksSpeedAfterFieldEstablished)
{
    controller.configure_prefluxing_duration(duration_t{0.1});  // Short pre-flux for test
    controller.start_excitation();

    // Complete pre-fluxing
    const int preflux_cycles = static_cast<int>(0.15 / kSamplingTime.value());
    for (int i = 0; i < preflux_cycles; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }

    ASSERT_TRUE(controller.get_is_field_established());

    // Now increase speed and verify excitation voltage increases
    const real_t voltage_at_zero_speed = controller.get_excitation_voltage_setpoint_pu().value();

    // Run at 50% speed for a while to allow rate limiter to settle
    for (int i = 0; i < 1000; ++i) {
        controller.update(speed_pu_t{0.5}, true);
    }

    const real_t voltage_at_half_speed = controller.get_excitation_voltage_setpoint_pu().value();

    // Excitation voltage should be higher at 50% speed (in V/Hz region)
    EXPECT_GT(voltage_at_half_speed, voltage_at_zero_speed);
}

TEST_F(SfcExcitationControllerTest, ExcitationVoltageSaturationAtHighSpeed)
{
    controller.configure_prefluxing_duration(duration_t{0.1});
    controller.start_excitation();

    // Complete pre-fluxing
    for (int i = 0; i < 150; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }

    // Run at 80% speed (end of V/Hz region)
    for (int i = 0; i < 2000; ++i) {
        controller.update(speed_pu_t{0.80}, true);
    }
    const real_t voltage_at_80_percent = controller.get_excitation_voltage_setpoint_pu().value();

    // Run at 95% speed (field weakening region)
    for (int i = 0; i < 2000; ++i) {
        controller.update(speed_pu_t{0.95}, true);
    }
    const real_t voltage_at_95_percent = controller.get_excitation_voltage_setpoint_pu().value();

    // In field weakening region, voltage should decrease
    EXPECT_LT(voltage_at_95_percent, voltage_at_80_percent);
}

// ============================================================================
// Fault Handling Tests
// ============================================================================

TEST_F(SfcExcitationControllerTest, ExciterNotReadyDuringOperationTriggersStop)
{
    controller.configure_prefluxing_duration(duration_t{0.1});
    controller.start_excitation();

    // Complete pre-fluxing
    for (int i = 0; i < 150; ++i) {
        controller.update(speed_pu_t{0.5}, true);
    }

    ASSERT_TRUE(controller.get_is_field_established());
    ASSERT_TRUE(controller.get_is_running());

    // Simulate exciter fault
    controller.update(speed_pu_t{0.5}, false);  // exciter_ready = false

    // Controller should initiate shutdown
    // Run a few more cycles to allow ramp-down
    for (int i = 0; i < 2000; ++i) {
        controller.update(speed_pu_t{0.5}, false);
    }

    // Eventually should stop
    EXPECT_FALSE(controller.get_is_running());
    EXPECT_FALSE(controller.get_enable_excitation());
}

TEST_F(SfcExcitationControllerTest, UpdateWhenNotRunningMaintainsZeroVoltage)
{
    // Don't start, just update
    for (int i = 0; i < 100; ++i) {
        controller.update(speed_pu_t{0.5}, true);
    }

    EXPECT_EQ(controller.get_excitation_voltage_setpoint_pu().value(), 0.0);
    EXPECT_FALSE(controller.get_enable_excitation());
}

// ============================================================================
// Complete Operational Sequence Test
// ============================================================================

TEST_F(SfcExcitationControllerTest, CompleteStartupToShutdownSequence)
{
    controller.configure_prefluxing_duration(duration_t{1.0});

    // 1. Initial state
    EXPECT_FALSE(controller.get_is_running());

    // 2. Start excitation
    EXPECT_TRUE(controller.start_excitation());
    EXPECT_TRUE(controller.get_is_running());
    EXPECT_TRUE(controller.get_enable_excitation());

    // 3. Pre-fluxing phase (1 second)
    for (int i = 0; i < 1200; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }
    EXPECT_TRUE(controller.get_is_field_established());

    // 4. Ramp up speed
    for (int i = 0; i < 500; ++i) {
        const real_t speed_ramp = static_cast<real_t>(i) / 500.0;  // 0 to 1.0
        controller.update(speed_pu_t{speed_ramp}, true);
    }

    // Should be at high excitation voltage near rated speed
    EXPECT_GT(controller.get_excitation_voltage_setpoint_pu().value(), 0.5);

    // 5. Shutdown
    controller.stop_excitation();

    // 6. Ramp down (allow sufficient time)
    for (int i = 0; i < 3000; ++i) {
        controller.update(speed_pu_t{1.0}, true);
    }

    // Should be fully stopped
    EXPECT_FALSE(controller.get_is_running());
    EXPECT_FALSE(controller.get_enable_excitation());
    EXPECT_NEAR(controller.get_excitation_voltage_setpoint_pu().value(), 0.0, 0.01);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(SfcExcitationControllerTest, NegativeSpeedInputHandledGracefully)
{
    controller.start_excitation();

    // Run for pre-fluxing
    for (int i = 0; i < 150; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }

    // Pass negative speed (invalid)
    EXPECT_NO_THROW(controller.update(speed_pu_t{-0.1}, true));

    // Controller should handle gracefully (likely clamped to zero in lookup)
    EXPECT_GE(controller.get_excitation_voltage_setpoint_pu().value(), 0.0);
}

TEST_F(SfcExcitationControllerTest, OversSpeedInputHandledGracefully)
{
    controller.start_excitation();

    for (int i = 0; i < 150; ++i) {
        controller.update(speed_pu_t{0.0}, true);
    }

    // Pass overspeed (e.g., 1.2 pu)
    for (int i = 0; i < 1000; ++i) {
        controller.update(speed_pu_t{1.2}, true);
    }

    // Should extrapolate or clamp based on lookup table implementation
    // At minimum, should not crash
    EXPECT_GT(controller.get_excitation_voltage_setpoint_pu().value(), 0.0);
}

TEST_F(SfcExcitationControllerTest, RapidStartStopCyclesHandledCorrectly)
{
    for (int cycle = 0; cycle < 5; ++cycle) {
        controller.reset();
        EXPECT_TRUE(controller.start_excitation());

        // Run briefly
        for (int i = 0; i < 50; ++i) {
            controller.update(speed_pu_t{0.0}, true);
        }

        controller.stop_excitation();

        // Ramp down
        for (int i = 0; i < 2000; ++i) {
            controller.update(speed_pu_t{0.0}, true);
        }

        EXPECT_FALSE(controller.get_is_running());
    }
}
