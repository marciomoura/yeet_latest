#include "sfc/sfc_communication_watchdog.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
using namespace yeet;

class CommunicationWatchdogTest : public ::testing::Test {
protected:
    static constexpr duration_t sampling_period{200e-6f};  // 200µs (5 kHz)

    CommunicationWatchdogTest() : watchdog(sampling_period) {}

    void SetUp() override { watchdog.reset(); }

    // Helper: Run N steps with given data validity
    void run_steps(int steps, bool data_valid)
    {
        for (int i = 0; i < steps; ++i) {
            watchdog.update(data_valid);
        }
    }

    sfc_communication_watchdog watchdog;
};

// ==================== Construction and Initialization Tests ====================

TEST_F(CommunicationWatchdogTest, DefaultConstructorInitializesWithCorrectDefaults)
{
    // Default: 50ms timeout (250 cycles @ 200µs), 5ms reset (25 cycles @ 200µs)
    EXPECT_EQ(watchdog.get_timeout_threshold(), 250);
    EXPECT_EQ(watchdog.get_reset_threshold(), 25);
    EXPECT_FALSE(watchdog.is_tripped());
    EXPECT_FALSE(watchdog.is_warning());
    EXPECT_EQ(watchdog.get_invalid_count(), 0);
}

TEST_F(CommunicationWatchdogTest, ConstructorWithCustomTimeouts)
{
    // 100ms timeout, 10ms reset @ 200µs sampling
    sfc_communication_watchdog custom_watchdog(duration_t{200e-6f}, duration_t{100e-3f}, duration_t{10e-3f});

    EXPECT_EQ(custom_watchdog.get_timeout_threshold(), 500);  // 100ms / 200µs = 500 cycles
    EXPECT_EQ(custom_watchdog.get_reset_threshold(), 50);     // 10ms / 200µs = 50 cycles
    EXPECT_FALSE(custom_watchdog.is_tripped());
}

TEST_F(CommunicationWatchdogTest, ConstructorEnforcesMinimumOneCycle)
{
    // Very short timeout (smaller than sampling period)
    sfc_communication_watchdog min_watchdog(duration_t{200e-6f}, duration_t{50e-6f}, duration_t{10e-6f});

    EXPECT_GE(min_watchdog.get_timeout_threshold(), 1);
    EXPECT_GE(min_watchdog.get_reset_threshold(), 1);
}

// ==================== Timeout Detection Tests ====================

TEST_F(CommunicationWatchdogTest, TripsAfterTimeoutThresholdExceeded)
{
    watchdog.configure_timeout_cycles(10);

    // 9 invalid cycles - should NOT trip
    run_steps(9, false);
    EXPECT_FALSE(watchdog.is_tripped());
    EXPECT_EQ(watchdog.get_invalid_count(), 9);

    // 10th invalid cycle - should trip
    watchdog.update(false);
    EXPECT_TRUE(watchdog.is_tripped());
    EXPECT_EQ(watchdog.get_invalid_count(), 10);
}

TEST_F(CommunicationWatchdogTest, NoTripWithValidCommunication)
{
    watchdog.configure_timeout_cycles(10);

    // 1000 cycles of valid communication - should never trip
    run_steps(1000, true);

    EXPECT_FALSE(watchdog.is_tripped());
    EXPECT_EQ(watchdog.get_invalid_count(), 0);
}

TEST_F(CommunicationWatchdogTest, InvalidCountIncrementsSaturatesAtMax)
{
    watchdog.configure_timeout_cycles(10);

    // Run many invalid cycles (more than timeout threshold)
    run_steps(70000, false);

    EXPECT_TRUE(watchdog.is_tripped());
    EXPECT_EQ(watchdog.get_invalid_count(), 10);  // Saturated at timeout threshold
}

// ==================== Reset and Recovery Tests ====================

TEST_F(CommunicationWatchdogTest, ValidCommunicationDecrementsInvalidCount)
{
    watchdog.configure_timeout_cycles(100);

    // Build up invalid count
    run_steps(50, false);
    EXPECT_EQ(watchdog.get_invalid_count(), 50);

    // Valid communication should decrement
    run_steps(10, true);
    EXPECT_EQ(watchdog.get_invalid_count(), 40);
}

TEST_F(CommunicationWatchdogTest, InvalidCountSaturatesAtZero)
{
    watchdog.configure_timeout_cycles(100);

    // Start with some invalid cycles
    run_steps(10, false);
    EXPECT_EQ(watchdog.get_invalid_count(), 10);

    // More valid cycles than invalid count should saturate at 0
    run_steps(20, true);
    EXPECT_EQ(watchdog.get_invalid_count(), 0);
}

TEST_F(CommunicationWatchdogTest, AutoResetAfterCommunicationRestored)
{
    watchdog.configure_timeout_cycles(100);
    watchdog.configure_reset_cycles(10);

    // Trip the watchdog
    run_steps(100, false);
    EXPECT_TRUE(watchdog.is_tripped());

    // Restore communication - invalid count decrements
    run_steps(89, true);                 // Count down to 11
    EXPECT_TRUE(watchdog.is_tripped());  // Still tripped (count > reset_threshold)

    // One more valid cycle brings count to 10 (at reset threshold)
    watchdog.update(true);
    EXPECT_FALSE(watchdog.is_tripped());  // Auto-reset triggered
    EXPECT_EQ(watchdog.get_invalid_count(), 10);
}

TEST_F(CommunicationWatchdogTest, ManualResetClearsEverything)
{
    watchdog.configure_timeout_cycles(10);

    // Trip the watchdog
    run_steps(20, false);
    EXPECT_TRUE(watchdog.is_tripped());
    EXPECT_EQ(watchdog.get_invalid_count(), 10);  // Counter saturates at timeout threshold

    // Manual reset
    watchdog.reset();
    EXPECT_FALSE(watchdog.is_tripped());
    EXPECT_EQ(watchdog.get_invalid_count(), 0);
}

// ==================== Warning State Tests ====================

TEST_F(CommunicationWatchdogTest, WarningStateActivatesAtHalfThreshold)
{
    watchdog.configure_timeout_cycles(100);

    // 50 invalid cycles - exactly 50% of threshold
    run_steps(50, false);
    EXPECT_FALSE(watchdog.is_warning());  // Warning requires > 50%

    // 51 invalid cycles - over 50%
    watchdog.update(false);
    EXPECT_TRUE(watchdog.is_warning());
    EXPECT_FALSE(watchdog.is_tripped());
}

TEST_F(CommunicationWatchdogTest, WarningClearsWhenCountDropsBelow50Percent)
{
    watchdog.configure_timeout_cycles(100);

    // Trigger warning
    run_steps(60, false);
    EXPECT_TRUE(watchdog.is_warning());

    // Restore communication until below 50%
    run_steps(11, true);  // Count drops to 49
    EXPECT_FALSE(watchdog.is_warning());
}

// ==================== Hysteresis Tests ====================

TEST_F(CommunicationWatchdogTest, HysteresisPreventsChatteringOnTrip)
{
    watchdog.configure_timeout_cycles(100);
    watchdog.configure_reset_cycles(20);

    // Trip the watchdog
    run_steps(100, false);
    EXPECT_TRUE(watchdog.is_tripped());

    // Brief valid communication (less than reset threshold)
    run_steps(10, true);
    EXPECT_TRUE(watchdog.is_tripped());  // Still tripped due to hysteresis

    // Back to invalid - should remain tripped
    run_steps(5, false);
    EXPECT_TRUE(watchdog.is_tripped());
}

TEST_F(CommunicationWatchdogTest, HysteresisAllowsResetAfterSufficientValidCycles)
{
    watchdog.configure_timeout_cycles(100);
    watchdog.configure_reset_cycles(20);

    // Trip the watchdog
    run_steps(100, false);
    EXPECT_TRUE(watchdog.is_tripped());

    // Sufficient valid communication to cross reset threshold
    run_steps(81, true);  // Count drops to 19 (below reset_threshold of 20)
    EXPECT_FALSE(watchdog.is_tripped());
}

// ==================== Configuration Tests (Cycle-based) ====================

TEST_F(CommunicationWatchdogTest, ConfigureTimeoutCyclesUpdatesThreshold)
{
    watchdog.configure_timeout_cycles(500);
    EXPECT_EQ(watchdog.get_timeout_threshold(), 500);

    watchdog.configure_timeout_cycles(100);
    EXPECT_EQ(watchdog.get_timeout_threshold(), 100);
}

TEST_F(CommunicationWatchdogTest, ConfigureTimeoutCyclesEnforcesMinimum)
{
    watchdog.configure_timeout_cycles(0);
    EXPECT_EQ(watchdog.get_timeout_threshold(), 1);
}

TEST_F(CommunicationWatchdogTest, ConfigureResetCyclesUpdatesThreshold)
{
    watchdog.configure_reset_cycles(50);
    EXPECT_EQ(watchdog.get_reset_threshold(), 50);

    watchdog.configure_reset_cycles(10);
    EXPECT_EQ(watchdog.get_reset_threshold(), 10);
}

TEST_F(CommunicationWatchdogTest, ConfigureResetCyclesEnforcesMinimum)
{
    watchdog.configure_reset_cycles(0);
    EXPECT_EQ(watchdog.get_reset_threshold(), 1);
}

// ==================== Configuration Tests (Duration-based) ====================

TEST_F(CommunicationWatchdogTest, ConfigureTimeoutDurationConvertsCorrectly)
{
    // 100ms timeout @ 200µs sampling = 500 cycles
    watchdog.configure_timeout_duration(duration_t{100e-3f});
    EXPECT_EQ(watchdog.get_timeout_threshold(), 500);

    // 50ms timeout @ 200µs sampling = 250 cycles
    watchdog.configure_timeout_duration(duration_t{50e-3f});
    EXPECT_EQ(watchdog.get_timeout_threshold(), 250);

    // 10ms timeout @ 200µs sampling = 50 cycles
    watchdog.configure_timeout_duration(duration_t{10e-3f});
    EXPECT_EQ(watchdog.get_timeout_threshold(), 50);
}

TEST_F(CommunicationWatchdogTest, ConfigureResetDurationConvertsCorrectly)
{
    // 20ms reset @ 200µs sampling = 100 cycles
    watchdog.configure_reset_duration(duration_t{20e-3f});
    EXPECT_EQ(watchdog.get_reset_threshold(), 100);

    // 5ms reset @ 200µs sampling = 25 cycles
    watchdog.configure_reset_duration(duration_t{5e-3f});
    EXPECT_EQ(watchdog.get_reset_threshold(), 25);
}

TEST_F(CommunicationWatchdogTest, ConfigureDurationEnforcesMinimumOneCycle)
{
    // Very short duration (smaller than sampling period)
    watchdog.configure_timeout_duration(duration_t{50e-6f});
    EXPECT_GE(watchdog.get_timeout_threshold(), 1);

    watchdog.configure_reset_duration(duration_t{10e-6f});
    EXPECT_GE(watchdog.get_reset_threshold(), 1);
}

// ==================== Integration Tests ====================

TEST_F(CommunicationWatchdogTest, IntermittentFailuresEventuallyTrip)
{
    watchdog.configure_timeout_cycles(100);

    // Pattern: 2 invalid, 1 valid (net +1 invalid per 3 cycles)
    for (int i = 0; i < 50; ++i) {
        watchdog.update(false);
        watchdog.update(false);
        watchdog.update(true);
    }

    // After 150 cycles with 2:1 invalid:valid ratio, count should be ~50
    EXPECT_FALSE(watchdog.is_tripped());  // Not enough to trip

    // Continue pattern until trip
    for (int i = 0; i < 50; ++i) {
        watchdog.update(false);
        watchdog.update(false);
        watchdog.update(true);
    }

    EXPECT_TRUE(watchdog.is_tripped());
}

TEST_F(CommunicationWatchdogTest, RecoveryFromTripWithIntermittentValid)
{
    watchdog.configure_timeout_cycles(100);
    watchdog.configure_reset_cycles(20);

    // Trip the watchdog
    run_steps(100, false);
    EXPECT_TRUE(watchdog.is_tripped());

    // Gradual recovery with mostly valid communication (3 valid : 1 invalid)
    // Net -2 per iteration, need to go from 100 down to 20 or below = 80 decrease needed = 40 iterations
    for (int i = 0; i < 41; ++i) {
        watchdog.update(true);
        watchdog.update(true);
        watchdog.update(true);
        watchdog.update(false);
    }

    // Net decrease should eventually reset (count should be at or below 20)
    EXPECT_FALSE(watchdog.is_tripped());
}

TEST_F(CommunicationWatchdogTest, DifferentSamplingPeriodScalesCorrectly)
{
    // Create watchdog with 1ms sampling period
    sfc_communication_watchdog slow_watchdog(duration_t{1e-3f});

    // Configure 50ms timeout - should be 50 cycles at 1ms sampling
    slow_watchdog.configure_timeout_duration(duration_t{50e-3f});
    EXPECT_EQ(slow_watchdog.get_timeout_threshold(), 50);

    // Trip after 50 invalid cycles
    for (int i = 0; i < 49; ++i) {
        slow_watchdog.update(false);
    }
    EXPECT_FALSE(slow_watchdog.is_tripped());

    slow_watchdog.update(false);
    EXPECT_TRUE(slow_watchdog.is_tripped());
}

// ==================== Edge Cases ====================

TEST_F(CommunicationWatchdogTest, RepeatedResetDoesNotAffectState)
{
    watchdog.configure_timeout_cycles(10);

    // Build up some invalid count
    run_steps(5, false);

    // Multiple resets
    watchdog.reset();
    watchdog.reset();
    watchdog.reset();

    EXPECT_FALSE(watchdog.is_tripped());
    EXPECT_EQ(watchdog.get_invalid_count(), 0);

    // Watchdog still functions correctly
    run_steps(10, false);
    EXPECT_TRUE(watchdog.is_tripped());
}

TEST_F(CommunicationWatchdogTest, ConfigurationWhileTrippedDoesNotClearTrip)
{
    watchdog.configure_timeout_cycles(10);

    // Trip the watchdog
    run_steps(10, false);
    EXPECT_TRUE(watchdog.is_tripped());

    // Reconfigure thresholds
    watchdog.configure_timeout_cycles(100);
    watchdog.configure_reset_cycles(50);

    // Trip flag should persist (configuration doesn't reset)
    EXPECT_TRUE(watchdog.is_tripped());
}

TEST_F(CommunicationWatchdogTest, MixedCycleAndDurationConfigurationWorks)
{
    // Configure timeout with duration
    watchdog.configure_timeout_duration(duration_t{100e-3f});
    EXPECT_EQ(watchdog.get_timeout_threshold(), 500);

    // Configure reset with cycles
    watchdog.configure_reset_cycles(25);
    EXPECT_EQ(watchdog.get_reset_threshold(), 25);

    // Both configurations should work together
    run_steps(500, false);
    EXPECT_TRUE(watchdog.is_tripped());

    run_steps(476, true);  // Down to 24 (below reset threshold)
    EXPECT_FALSE(watchdog.is_tripped());
}
