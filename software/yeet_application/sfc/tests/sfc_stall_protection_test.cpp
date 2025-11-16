#include "sfc/sfc_stall_protection.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
using namespace yeet;

class StallProtectionTest : public ::testing::Test {
protected:
    static constexpr duration_t sampling_period{0.001};  // 1ms (1 kHz)

    StallProtectionTest() : protection(sampling_period) {}

    void SetUp() override { protection.reset(); }

    // Helper: Run N steps with given current, speed, and startup state
    void run_steps(int steps, current_pu_t current, speed_pu_t speed, bool startup_active)
    {
        for (int i = 0; i < steps; ++i) {
            protection.update(current, speed, startup_active);
        }
    }

    sfc_stall_protection protection;
};

// ==================== Stall Condition Detection Tests ====================

TEST_F(StallProtectionTest, DetectsStallConditionWhenAllThresholdsMet)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});  // 100ms = 100 steps at 1 kHz

    // Stall condition: High current (1.5 pu) + Low speed (0.05 pu) + Startup active
    current_pu_t high_current{1.5};
    speed_pu_t low_speed{0.05};
    bool startup_active = true;

    // Check immediate stall condition (before delay)
    protection.update(high_current, low_speed, startup_active);
    EXPECT_TRUE(protection.is_stall_condition_active());
    EXPECT_FALSE(protection.is_tripped());  // Not tripped yet (delay not elapsed)

    // After delay expires → trip
    run_steps(99, high_current, low_speed, startup_active);
    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(StallProtectionTest, NoStallWhenCurrentBelowThreshold)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});

    // Current below threshold (1.2 pu), speed low, startup active
    run_steps(200, current_pu_t{1.2}, speed_pu_t{0.05}, true);

    EXPECT_FALSE(protection.is_stall_condition_active());
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(StallProtectionTest, NoStallWhenSpeedAboveThreshold)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});

    // Current high, speed above threshold (0.15 pu), startup active
    run_steps(200, current_pu_t{1.5}, speed_pu_t{0.15}, true);

    EXPECT_FALSE(protection.is_stall_condition_active());
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(StallProtectionTest, NoStallWhenStartupNotActive)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});

    // Current high, speed low, but startup NOT active
    run_steps(200, current_pu_t{1.5}, speed_pu_t{0.05}, false);

    EXPECT_FALSE(protection.is_stall_condition_active());
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(StallProtectionTest, AllThreeConditionsRequiredForStall)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});

    // Test 1: High current + low speed, no startup
    run_steps(10, current_pu_t{1.5}, speed_pu_t{0.05}, false);
    EXPECT_FALSE(protection.is_stall_condition_active());

    // Test 2: High current + startup, no low speed
    run_steps(10, current_pu_t{1.5}, speed_pu_t{0.15}, true);
    EXPECT_FALSE(protection.is_stall_condition_active());

    // Test 3: Low speed + startup, no high current
    run_steps(10, current_pu_t{1.0}, speed_pu_t{0.05}, true);
    EXPECT_FALSE(protection.is_stall_condition_active());

    // Test 4: All three conditions met → stall
    protection.update(current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_stall_condition_active());
}

// ==================== State-Dependent Activation Tests ====================

TEST_F(StallProtectionTest, OnlyActiveWhenStartupActive)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});

    // Stall conditions met but startup not active → no trip
    run_steps(150, current_pu_t{1.5}, speed_pu_t{0.05}, false);
    EXPECT_FALSE(protection.is_tripped());

    // Enable startup with same conditions → trip after delay
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(StallProtectionTest, StartupDeactivationClearsStallCondition)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});

    // Stall condition active during startup
    protection.update(current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_stall_condition_active());

    // Deactivate startup (e.g., user abort) → stall condition clears
    protection.update(current_pu_t{1.5}, speed_pu_t{0.05}, false);
    EXPECT_FALSE(protection.is_stall_condition_active());
}

// ==================== Time Delay Tests ====================

TEST_F(StallProtectionTest, BriefStallDoesNotTrip)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});  // 100 steps

    // Brief stall (50 steps) followed by normal acceleration
    run_steps(50, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    run_steps(100, current_pu_t{1.2}, speed_pu_t{0.20}, true);

    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(StallProtectionTest, PersistentStallTripsAfterDelay)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});  // 100 steps

    // Persistent stall for full delay
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);

    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(StallProtectionTest, WarningActivatesBeforeTrip)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_warning_delay(duration_t{0.05});  // 50 steps
    protection.configure_trip_delay(duration_t{0.10});     // 100 steps

    // Apply stall condition for 50 steps
    run_steps(50, current_pu_t{1.5}, speed_pu_t{0.05}, true);

    // Warning should be active, trip should not
    EXPECT_TRUE(protection.is_warning());
    EXPECT_FALSE(protection.is_tripped());

    // Continue for another 50 steps → trip
    run_steps(50, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(StallProtectionTest, WarningClearsImmediately)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_warning_delay(duration_t{0.05});

    // Activate warning
    run_steps(50, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_warning());

    // Clear stall condition → warning clears immediately (not latched)
    protection.update(current_pu_t{1.0}, speed_pu_t{0.20}, true);
    EXPECT_FALSE(protection.is_warning());
}

// ==================== Latch and Reset Tests ====================

TEST_F(StallProtectionTest, TripLatches)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});

    // Trip the protection
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());

    // Return to safe condition → trip remains latched
    run_steps(100, current_pu_t{1.0}, speed_pu_t{0.20}, true);
    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(StallProtectionTest, CanResetWhenCurrentClears)
{
    protection.configure_current_threshold(current_pu_t{1.3}, real_t{0.05});  // 5% hysteresis
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});

    // Trip threshold: 1.3 pu
    // Reset threshold: 1.3 * (1 - 0.05) = 1.235 pu

    // Trip the protection
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());

    // Reduce current to just below trip threshold → still cannot reset
    run_steps(10, current_pu_t{1.25}, speed_pu_t{0.05}, true);
    EXPECT_FALSE(protection.can_reset());

    // Reduce current below reset threshold → can reset
    run_steps(10, current_pu_t{1.20}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.can_reset());

    // Reset clears the trip
    protection.reset();
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(StallProtectionTest, CanResetWhenSpeedIncreases)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10}, real_t{0.02});  // 2% hysteresis
    protection.configure_trip_delay(duration_t{0.1});

    // Trip threshold: 0.10 pu
    // Reset threshold: 0.10 * (1 + 0.02) = 0.102 pu

    // Trip the protection
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());

    // Increase speed to just above trip threshold → still cannot reset
    run_steps(10, current_pu_t{1.5}, speed_pu_t{0.101}, true);
    EXPECT_FALSE(protection.can_reset());

    // Increase speed above reset threshold → can reset
    run_steps(10, current_pu_t{1.5}, speed_pu_t{0.11}, true);
    EXPECT_TRUE(protection.can_reset());

    // Reset clears the trip
    protection.reset();
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(StallProtectionTest, CanResetWithEitherConditionCleared)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});

    // Trip the protection
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());

    // Test 1: Clear via current reduction (speed still low)
    run_steps(10, current_pu_t{1.0}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.can_reset());
    protection.reset();
    EXPECT_FALSE(protection.is_tripped());

    // Trip again
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());

    // Test 2: Clear via speed increase (current still high)
    run_steps(10, current_pu_t{1.5}, speed_pu_t{0.20}, true);
    EXPECT_TRUE(protection.can_reset());
    protection.reset();
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(StallProtectionTest, CannotResetWhileStallConditionActive)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});

    // Trip the protection
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());

    // Stall condition still active → cannot reset
    EXPECT_FALSE(protection.can_reset());
    protection.reset();
    EXPECT_TRUE(protection.is_tripped());  // Still tripped
}

// ==================== Hysteresis Tests ====================

TEST_F(StallProtectionTest, CurrentHysteresisPreventsFalseReset)
{
    protection.configure_current_threshold(current_pu_t{1.3}, real_t{0.05});  // 5% hysteresis
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});

    // Trip the protection
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());

    // Oscillate around trip threshold (1.29 to 1.31 pu)
    for (int i = 0; i < 10; ++i) {
        run_steps(5, current_pu_t{1.29}, speed_pu_t{0.05}, true);
        EXPECT_FALSE(protection.can_reset());  // Still above reset threshold (1.235 pu)
        run_steps(5, current_pu_t{1.31}, speed_pu_t{0.05}, true);
        EXPECT_FALSE(protection.can_reset());
    }

    // Only reset when clearly below reset threshold
    run_steps(10, current_pu_t{1.20}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.can_reset());
}

TEST_F(StallProtectionTest, SpeedHysteresisPreventsFalseReset)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10}, real_t{0.02});  // 2% hysteresis
    protection.configure_trip_delay(duration_t{0.1});

    // Trip the protection
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());

    // Oscillate around trip threshold (0.099 to 0.101 pu)
    for (int i = 0; i < 10; ++i) {
        run_steps(5, current_pu_t{1.5}, speed_pu_t{0.099}, true);
        EXPECT_FALSE(protection.can_reset());  // Still below reset threshold (0.102 pu)
        run_steps(5, current_pu_t{1.5}, speed_pu_t{0.101}, true);
        EXPECT_FALSE(protection.can_reset());
    }

    // Only reset when clearly above reset threshold
    run_steps(10, current_pu_t{1.5}, speed_pu_t{0.11}, true);
    EXPECT_TRUE(protection.can_reset());
}

// ==================== Edge Cases ====================

TEST_F(StallProtectionTest, HandlesZeroCurrentAndSpeed)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});

    // Zero current and speed during startup → no stall (current below threshold)
    run_steps(100, current_pu_t{0.0}, speed_pu_t{0.0}, true);

    EXPECT_FALSE(protection.is_stall_condition_active());
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(StallProtectionTest, HandlesExactThresholdBoundary)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});

    // Exactly at threshold boundaries → no stall (> and < are strict inequalities)
    run_steps(150, current_pu_t{1.3}, speed_pu_t{0.10}, true);

    EXPECT_FALSE(protection.is_stall_condition_active());
    EXPECT_FALSE(protection.is_tripped());

    // Just beyond thresholds → stall
    run_steps(100, current_pu_t{1.30001}, speed_pu_t{0.09999}, true);

    EXPECT_TRUE(protection.is_stall_condition_active());
    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(StallProtectionTest, ResetClearsAllState)
{
    protection.configure_current_threshold(current_pu_t{1.3});
    protection.configure_speed_threshold(speed_pu_t{0.10});
    protection.configure_trip_delay(duration_t{0.1});

    // Trip protection
    run_steps(100, current_pu_t{1.5}, speed_pu_t{0.05}, true);
    EXPECT_TRUE(protection.is_tripped());

    // Clear stall condition and reset
    run_steps(10, current_pu_t{1.0}, speed_pu_t{0.20}, true);
    protection.reset();

    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
    EXPECT_FALSE(protection.is_stall_condition_active());
}

// ==================== Measurement Getter Tests ====================

TEST_F(StallProtectionTest, GettersReturnCurrentMeasurements)
{
    current_pu_t test_current{1.5};
    speed_pu_t test_speed{0.08};

    protection.update(test_current, test_speed, true);

    EXPECT_FLOAT_EQ(protection.get_current_value().value(), test_current.value());
    EXPECT_FLOAT_EQ(protection.get_speed_value().value(), test_speed.value());
}

TEST_F(StallProtectionTest, GettersReturnConfiguredThresholds)
{
    current_pu_t current_threshold{1.4};
    speed_pu_t speed_threshold{0.12};

    protection.configure_current_threshold(current_threshold);
    protection.configure_speed_threshold(speed_threshold);

    EXPECT_FLOAT_EQ(protection.get_current_threshold().value(), current_threshold.value());
    EXPECT_FLOAT_EQ(protection.get_speed_threshold().value(), speed_threshold.value());
}
