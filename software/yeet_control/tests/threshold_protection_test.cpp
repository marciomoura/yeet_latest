#include "yeet_control/threshold_protection.hpp"

#include <gtest/gtest.h>

using namespace yeet::control;
using namespace yeet;

class ThresholdProtectionTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Configure protection with typical overcurrent thresholds
        protection.configure_trip_threshold(current_pu_t{1.5}, real_t{0.05});  // 150% trip, 5% hysteresis
        protection.configure_warning_threshold(current_pu_t{1.2});             // 120% warning
        protection.configure_trip_delay(duration_t{0.05});                     // 50ms trip delay
        protection.configure_warning_delay(duration_t{0.01});                  // 10ms warning delay
    }

    static constexpr double sampling_time = 0.001;  // 1ms sampling
    threshold_protection<current_pu_t> protection{duration_t{sampling_time}};
};

TEST_F(ThresholdProtectionTest, InitialStateNotTripped)
{
    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
    EXPECT_TRUE(protection.can_reset());
}

TEST_F(ThresholdProtectionTest, BelowThresholdsNoActivation)
{
    // Run for 100ms below all thresholds
    for (int i = 0; i < 100; ++i) {
        protection.update(current_pu_t{1.0});  // Nominal current
    }

    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
}

TEST_F(ThresholdProtectionTest, WarningActivatesAfterDelay)
{
    // Apply current above warning threshold (1.2 pu) but below trip (1.5 pu)
    for (int i = 0; i < 5; ++i) {
        protection.update(current_pu_t{1.3});  // 130% current
        EXPECT_FALSE(protection.is_warning()) << "Warning should not activate before 10ms delay";
    }

    // Continue for remaining delay period
    for (int i = 0; i < 10; ++i) {
        protection.update(current_pu_t{1.3});
    }

    EXPECT_TRUE(protection.is_warning()) << "Warning should activate after 10ms delay";
    EXPECT_FALSE(protection.is_tripped()) << "Trip should not activate below trip threshold";
}

TEST_F(ThresholdProtectionTest, WarningClearsWhenBelowThreshold)
{
    // Activate warning
    for (int i = 0; i < 15; ++i) {
        protection.update(current_pu_t{1.3});
    }
    ASSERT_TRUE(protection.is_warning());

    // Drop below warning threshold
    protection.update(current_pu_t{1.0});

    EXPECT_FALSE(protection.is_warning()) << "Warning should clear immediately when below threshold";
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(ThresholdProtectionTest, TripActivatesAfterDelay)
{
    // Apply current above trip threshold (1.5 pu)
    for (int i = 0; i < 25; ++i) {
        protection.update(current_pu_t{1.6});  // 160% current
        EXPECT_FALSE(protection.is_tripped()) << "Trip should not activate before 50ms delay at iteration " << i;
    }

    // Continue for remaining delay period
    for (int i = 0; i < 30; ++i) {
        protection.update(current_pu_t{1.6});
    }

    EXPECT_TRUE(protection.is_tripped()) << "Trip should activate after 50ms delay";
}

TEST_F(ThresholdProtectionTest, TripLatchesAndRemainsActive)
{
    // Trip the protection
    for (int i = 0; i < 60; ++i) {
        protection.update(current_pu_t{1.6});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Reduce current below trip threshold
    for (int i = 0; i < 50; ++i) {
        protection.update(current_pu_t{1.0});
    }

    EXPECT_TRUE(protection.is_tripped()) << "Trip should remain latched even when current returns to normal";
}

TEST_F(ThresholdProtectionTest, CannotResetWhileFaultActive)
{
    // Trip the protection
    for (int i = 0; i < 60; ++i) {
        protection.update(current_pu_t{1.6});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Try to reset while current still high
    protection.update(current_pu_t{1.6});
    EXPECT_FALSE(protection.can_reset()) << "Should not allow reset while fault is active";

    protection.reset();
    EXPECT_TRUE(protection.is_tripped()) << "Reset should not clear trip while fault is active";
}

TEST_F(ThresholdProtectionTest, ResetClearsLatchWhenFaultCleared)
{
    // Trip the protection
    for (int i = 0; i < 60; ++i) {
        protection.update(current_pu_t{1.6});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Clear fault condition (below reset threshold with hysteresis)
    // Reset threshold = 1.5 * (1 - 0.05) = 1.425 pu
    protection.update(current_pu_t{1.0});
    EXPECT_TRUE(protection.can_reset()) << "Should allow reset when current is below reset threshold";

    protection.reset();
    EXPECT_FALSE(protection.is_tripped()) << "Reset should clear trip latch when fault is cleared";
}

TEST_F(ThresholdProtectionTest, HysteresisPreventsChattering)
{
    // Trip threshold = 1.5 pu
    // Reset threshold = 1.5 * 0.95 = 1.425 pu

    // Trip the protection
    for (int i = 0; i < 60; ++i) {
        protection.update(current_pu_t{1.6});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Reduce to just below trip threshold but above reset threshold
    protection.update(current_pu_t{1.45});
    EXPECT_FALSE(protection.can_reset()) << "Should not allow reset in hysteresis band";

    protection.reset();
    EXPECT_TRUE(protection.is_tripped()) << "Should remain tripped in hysteresis band";

    // Reduce below reset threshold
    protection.update(current_pu_t{1.4});
    EXPECT_TRUE(protection.can_reset()) << "Should allow reset below reset threshold";

    protection.reset();
    EXPECT_FALSE(protection.is_tripped()) << "Should clear trip below reset threshold";
}

TEST_F(ThresholdProtectionTest, ImmediateTripWithZeroDelay)
{
    threshold_protection<current_pu_t> fast_protection{duration_t{sampling_time}};
    fast_protection.configure_trip_threshold(current_pu_t{1.5}, real_t{0.05});
    fast_protection.configure_trip_delay(duration_t{0.0});  // No delay

    fast_protection.update(current_pu_t{1.6});

    EXPECT_TRUE(fast_protection.is_tripped()) << "Should trip immediately with zero delay";
}

TEST_F(ThresholdProtectionTest, GettersReturnCorrectValues)
{
    protection.update(current_pu_t{1.3});

    EXPECT_FLOAT_EQ(protection.get_current_value().value(), 1.3f);
    EXPECT_FLOAT_EQ(protection.get_trip_threshold().value(), 1.5f);
    EXPECT_FLOAT_EQ(protection.get_warning_threshold().value(), 1.2f);
}

TEST_F(ThresholdProtectionTest, ResetAlsoClearsTimers)
{
    // Start building up toward trip
    for (int i = 0; i < 30; ++i) {
        protection.update(current_pu_t{1.6});  // Halfway through 50ms delay
    }
    EXPECT_FALSE(protection.is_tripped()) << "Should not have tripped yet";

    // Clear fault and reset
    protection.update(current_pu_t{1.0});
    protection.reset();

    // Apply fault again - should require full delay again
    for (int i = 0; i < 30; ++i) {
        protection.update(current_pu_t{1.6});
    }
    EXPECT_FALSE(protection.is_tripped()) << "Timer should have been reset, requiring full delay again";

    // Complete delay
    for (int i = 0; i < 30; ++i) {
        protection.update(current_pu_t{1.6});
    }
    EXPECT_TRUE(protection.is_tripped()) << "Should trip after full delay from reset";
}

TEST_F(ThresholdProtectionTest, WorksWithVoltageType)
{
    threshold_protection<voltage_pu_t> voltage_protection{duration_t{sampling_time}};
    voltage_protection.configure_trip_threshold(voltage_pu_t{1.25}, real_t{0.05});
    voltage_protection.configure_warning_threshold(voltage_pu_t{1.15});
    voltage_protection.configure_trip_delay(duration_t{0.1});  // 100ms

    // Run below threshold
    for (int i = 0; i < 50; ++i) {
        voltage_protection.update(voltage_pu_t{1.0});
    }
    EXPECT_FALSE(voltage_protection.is_tripped());

    // Apply overvoltage
    for (int i = 0; i < 150; ++i) {
        voltage_protection.update(voltage_pu_t{1.3});
    }
    EXPECT_TRUE(voltage_protection.is_tripped());

    // Clear and reset
    voltage_protection.update(voltage_pu_t{1.0});
    voltage_protection.reset();
    EXPECT_FALSE(voltage_protection.is_tripped());
}

// ==================== Tests for threshold_direction::below (Undervoltage Logic) ====================

class ThresholdProtectionBelowTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Configure protection with undervoltage thresholds
        protection.configure_trip_threshold(voltage_pu_t{0.70}, real_t{0.05});  // Trip at 70%, reset at 73.5%
        protection.configure_warning_threshold(voltage_pu_t{0.85});             // Warn at 85%
        protection.configure_trip_delay(duration_t{0.5});                       // 500ms ride-through
        protection.configure_warning_delay(duration_t{0.1});                    // 100ms warning delay
    }

    static constexpr double sampling_time = 0.001;  // 1ms sampling
    threshold_protection<voltage_pu_t, threshold_direction::below> protection{duration_t{sampling_time}};
};

TEST_F(ThresholdProtectionBelowTest, InitialStateNotTripped)
{
    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
    // Note: can_reset() may be false initially because _current_value is 0 (below reset threshold)
    // This is correct behavior - need to update with valid voltage first
    protection.update(voltage_pu_t{1.0});
    EXPECT_TRUE(protection.can_reset());
}

TEST_F(ThresholdProtectionBelowTest, NominalVoltageNoActivation)
{
    // Run at nominal voltage (1.0 pu) - above all thresholds
    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
}

TEST_F(ThresholdProtectionBelowTest, WarningActivatesWhenBelowThreshold)
{
    // Apply voltage below warning (0.85 pu) but above trip (0.70 pu)
    for (int i = 0; i < 50; ++i) {
        protection.update(voltage_pu_t{0.80});  // 80% voltage
        EXPECT_FALSE(protection.is_warning()) << "Warning should not activate before 100ms delay";
    }

    // Continue for remaining delay period
    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{0.80});
    }

    EXPECT_TRUE(protection.is_warning()) << "Warning should activate after 100ms delay";
    EXPECT_FALSE(protection.is_tripped()) << "Should not trip for voltage sag above trip threshold";
}

TEST_F(ThresholdProtectionBelowTest, TripActivatesWhenBelowThreshold)
{
    // Apply severe undervoltage (below 0.70 pu)
    for (int i = 0; i < 250; ++i) {
        protection.update(voltage_pu_t{0.60});  // 60% voltage
        EXPECT_FALSE(protection.is_tripped()) << "Should not trip before 500ms ride-through";
    }

    // Complete the ride-through delay
    for (int i = 0; i < 300; ++i) {
        protection.update(voltage_pu_t{0.60});
    }

    EXPECT_TRUE(protection.is_tripped()) << "Should trip after 500ms ride-through";
}

TEST_F(ThresholdProtectionBelowTest, TripLatchesUntilReset)
{
    // Trip the protection
    for (int i = 0; i < 600; ++i) {
        protection.update(voltage_pu_t{0.60});
    }
    EXPECT_TRUE(protection.is_tripped());

    // Voltage returns to normal
    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{1.0});
        EXPECT_TRUE(protection.is_tripped()) << "Trip should remain latched even after voltage recovers";
    }
}

TEST_F(ThresholdProtectionBelowTest, HysteresisPreventsChattering)
{
    // Trip at 70%, should reset at 73.5% (70% * 1.05)
    for (int i = 0; i < 600; ++i) {
        protection.update(voltage_pu_t{0.65});
    }
    EXPECT_TRUE(protection.is_tripped());

    // Voltage recovers to just above trip threshold but below reset threshold
    protection.update(voltage_pu_t{0.72});
    EXPECT_FALSE(protection.can_reset()) << "Should not allow reset below hysteresis threshold (73.5%)";

    // Voltage recovers above hysteresis threshold
    protection.update(voltage_pu_t{0.75});
    EXPECT_TRUE(protection.can_reset()) << "Should allow reset above hysteresis threshold";

    protection.reset();
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(ThresholdProtectionBelowTest, ResetClearsLatchWhenAboveThreshold)
{
    // Trip the protection
    for (int i = 0; i < 600; ++i) {
        protection.update(voltage_pu_t{0.60});
    }
    EXPECT_TRUE(protection.is_tripped());

    // Voltage returns above reset threshold
    protection.update(voltage_pu_t{0.80});
    EXPECT_TRUE(protection.can_reset());

    protection.reset();
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(ThresholdProtectionBelowTest, CannotResetWhileFaultActive)
{
    // Trip the protection
    for (int i = 0; i < 600; ++i) {
        protection.update(voltage_pu_t{0.60});
    }
    EXPECT_TRUE(protection.is_tripped());

    // Attempt reset while still low
    protection.update(voltage_pu_t{0.65});
    EXPECT_FALSE(protection.can_reset());

    protection.reset();
    EXPECT_TRUE(protection.is_tripped()) << "Should remain tripped because voltage still below reset threshold";
}

TEST_F(ThresholdProtectionBelowTest, RideThroughBriefVoltageSag)
{
    // Brief voltage sag for 400ms (less than 500ms ride-through)
    for (int i = 0; i < 400; ++i) {
        protection.update(voltage_pu_t{0.60});
    }

    EXPECT_FALSE(protection.is_tripped()) << "Should ride through brief sag";

    // Voltage recovers
    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
}
