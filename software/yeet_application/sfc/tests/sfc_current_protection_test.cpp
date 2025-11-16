#include "sfc/sfc_current_protection.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
using namespace yeet;

class SfcCurrentProtectionTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Protection is constructed with default LCI thresholds
        // Trip: 1.5 pu, Warning: 1.2 pu, Trip delay: 50ms, Warning delay: 10ms
    }

    static constexpr double sampling_time = 0.001;  // 1ms sampling
    sfc_current_protection protection{duration_t{sampling_time}};
};

TEST_F(SfcCurrentProtectionTest, DefaultThresholdsAreConfigured)
{
    EXPECT_FLOAT_EQ(protection.get_trip_threshold().value(), 1.5f);
    EXPECT_FLOAT_EQ(protection.get_warning_threshold().value(), 1.2f);
}

TEST_F(SfcCurrentProtectionTest, InitialStateNotTripped)
{
    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
    EXPECT_TRUE(protection.can_reset());
}

TEST_F(SfcCurrentProtectionTest, NominalCurrentNoActivation)
{
    // Run for 100ms at nominal current
    for (int i = 0; i < 100; ++i) {
        protection.update(current_pu_t{1.0});
    }

    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
    EXPECT_FLOAT_EQ(protection.get_current_value().value(), 1.0f);
}

TEST_F(SfcCurrentProtectionTest, WarningActivatesAt120Percent)
{
    // Apply 130% current (above 120% warning threshold)
    for (int i = 0; i < 5; ++i) {
        protection.update(current_pu_t{1.3});
        EXPECT_FALSE(protection.is_warning()) << "Warning should not activate before 10ms delay";
    }

    // Complete warning delay
    for (int i = 0; i < 10; ++i) {
        protection.update(current_pu_t{1.3});
    }

    EXPECT_TRUE(protection.is_warning()) << "Warning should activate after 10ms at 130% current";
    EXPECT_FALSE(protection.is_tripped()) << "Should not trip below 150% threshold";
}

TEST_F(SfcCurrentProtectionTest, TripActivatesAt150Percent)
{
    // Apply 160% current (above 150% trip threshold)
    for (int i = 0; i < 25; ++i) {
        protection.update(current_pu_t{1.6});
        EXPECT_FALSE(protection.is_tripped()) << "Trip should not activate before 50ms delay at iteration " << i;
    }

    // Complete trip delay (50ms total)
    for (int i = 0; i < 30; ++i) {
        protection.update(current_pu_t{1.6});
    }

    EXPECT_TRUE(protection.is_tripped()) << "Trip should activate after 50ms at 160% current";
}

TEST_F(SfcCurrentProtectionTest, TripLatchesUntilReset)
{
    // Trip the protection
    for (int i = 0; i < 60; ++i) {
        protection.update(current_pu_t{1.6});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Reduce current to nominal
    for (int i = 0; i < 100; ++i) {
        protection.update(current_pu_t{1.0});
    }

    EXPECT_TRUE(protection.is_tripped()) << "Trip should remain latched even after current returns to normal";
}

TEST_F(SfcCurrentProtectionTest, CannotResetWhileFaultActive)
{
    // Trip the protection
    for (int i = 0; i < 60; ++i) {
        protection.update(current_pu_t{1.6});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Current still high
    protection.update(current_pu_t{1.6});
    EXPECT_FALSE(protection.can_reset());

    protection.reset();
    EXPECT_TRUE(protection.is_tripped()) << "Reset should have no effect while fault is active";
}

TEST_F(SfcCurrentProtectionTest, ResetClearsLatchAfterFaultCleared)
{
    // Trip the protection
    for (int i = 0; i < 60; ++i) {
        protection.update(current_pu_t{1.6});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Clear fault (below reset threshold: 1.5 * 0.95 = 1.425 pu)
    protection.update(current_pu_t{1.0});
    EXPECT_TRUE(protection.can_reset());

    protection.reset();
    EXPECT_FALSE(protection.is_tripped()) << "Reset should clear trip after fault is cleared";
}

TEST_F(SfcCurrentProtectionTest, HysteresisPreventsChattering)
{
    // Default hysteresis is 5%, so reset threshold = 1.5 * 0.95 = 1.425 pu

    // Trip the protection
    for (int i = 0; i < 60; ++i) {
        protection.update(current_pu_t{1.6});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Reduce to 1.45 pu (above reset threshold but below trip threshold)
    protection.update(current_pu_t{1.45});
    EXPECT_FALSE(protection.can_reset()) << "Should not allow reset in hysteresis band";

    protection.reset();
    EXPECT_TRUE(protection.is_tripped()) << "Should remain tripped in hysteresis band";

    // Reduce below reset threshold
    protection.update(current_pu_t{1.4});
    EXPECT_TRUE(protection.can_reset());

    protection.reset();
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcCurrentProtectionTest, CustomThresholdsCanBeConfigured)
{
    sfc_current_protection custom_protection{duration_t{sampling_time}};
    custom_protection.configure_trip_threshold(current_pu_t{2.0}, real_t{0.1});  // 200% trip, 10% hysteresis
    custom_protection.configure_warning_threshold(current_pu_t{1.8});            // 180% warning
    custom_protection.configure_trip_delay(duration_t{0.1});                     // 100ms delay
    custom_protection.configure_warning_delay(duration_t{0.02});                 // 20ms delay

    EXPECT_FLOAT_EQ(custom_protection.get_trip_threshold().value(), 2.0f);
    EXPECT_FLOAT_EQ(custom_protection.get_warning_threshold().value(), 1.8f);

    // Should not trip at 150%
    for (int i = 0; i < 150; ++i) {
        custom_protection.update(current_pu_t{1.5});
    }
    EXPECT_FALSE(custom_protection.is_tripped());

    // Should trip at 210% after 100ms
    for (int i = 0; i < 110; ++i) {
        custom_protection.update(current_pu_t{2.1});
    }
    EXPECT_TRUE(custom_protection.is_tripped());
}

TEST_F(SfcCurrentProtectionTest, WarningClearsImmediatelyWhenBelowThreshold)
{
    // Activate warning
    for (int i = 0; i < 15; ++i) {
        protection.update(current_pu_t{1.3});
    }
    ASSERT_TRUE(protection.is_warning());

    // Drop below warning threshold
    protection.update(current_pu_t{1.0});

    EXPECT_FALSE(protection.is_warning()) << "Warning should clear immediately";
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcCurrentProtectionTest, SimulateLCIStartupTransient)
{
    // Simulate a typical LCI startup with brief overcurrent transient

    // Nominal operation
    for (int i = 0; i < 50; ++i) {
        protection.update(current_pu_t{1.0});
    }
    EXPECT_FALSE(protection.is_warning());
    EXPECT_FALSE(protection.is_tripped());

    // Brief transient to 140% (above warning, below trip) lasting 30ms
    for (int i = 0; i < 30; ++i) {
        protection.update(current_pu_t{1.4});
    }
    // Warning should activate after 10ms
    EXPECT_TRUE(protection.is_warning());
    EXPECT_FALSE(protection.is_tripped()) << "Should not trip for brief transient below trip threshold";

    // Return to nominal
    for (int i = 0; i < 50; ++i) {
        protection.update(current_pu_t{1.0});
    }
    EXPECT_FALSE(protection.is_warning());
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcCurrentProtectionTest, SimulateFaultCondition)
{
    // Simulate a sustained overcurrent fault condition

    // Normal operation
    for (int i = 0; i < 100; ++i) {
        protection.update(current_pu_t{1.0});
    }

    // Sudden fault: 180% overcurrent
    for (int i = 0; i < 60; ++i) {
        protection.update(current_pu_t{1.8});
    }

    // Protection should have tripped
    EXPECT_TRUE(protection.is_tripped());
    EXPECT_TRUE(protection.is_warning());

    // System attempts to clear fault (control system reduces current)
    for (int i = 0; i < 100; ++i) {
        protection.update(current_pu_t{0.5});
    }

    // Protection remains latched
    EXPECT_TRUE(protection.is_tripped());

    // Operator resets protection
    EXPECT_TRUE(protection.can_reset());
    protection.reset();

    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
}
