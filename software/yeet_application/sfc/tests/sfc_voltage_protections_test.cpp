#include <gtest/gtest.h>

#include "sfc/sfc_overvoltage_protection.hpp"
#include "sfc/sfc_undervoltage_protection.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

class SfcOvervoltageProtectionTest : public ::testing::Test {
protected:
    static constexpr double sampling_time = 0.001;  // 1ms sampling
    sfc_overvoltage_protection protection{duration_t{sampling_time}};
};

class SfcUndervoltageProtectionTest : public ::testing::Test {
protected:
    static constexpr double sampling_time = 0.001;  // 1ms sampling
    sfc_undervoltage_protection protection{duration_t{sampling_time}};
};

// ==================== Overvoltage Protection Tests ====================

TEST_F(SfcOvervoltageProtectionTest, DefaultThresholdsAreConfigured)
{
    EXPECT_FLOAT_EQ(protection.get_trip_threshold().value(), 1.25f);     // 125%
    EXPECT_FLOAT_EQ(protection.get_warning_threshold().value(), 1.15f);  // 115%
}

TEST_F(SfcOvervoltageProtectionTest, InitialStateNotTripped)
{
    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
    EXPECT_TRUE(protection.can_reset());
}

TEST_F(SfcOvervoltageProtectionTest, NominalVoltageNoActivation)
{
    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
}

TEST_F(SfcOvervoltageProtectionTest, WarningActivatesAt115Percent)
{
    // Apply 120% voltage (above 115% warning, below 125% trip)
    for (int i = 0; i < 10; ++i) {
        protection.update(voltage_pu_t{1.20});
        EXPECT_FALSE(protection.is_warning()) << "Warning should not activate before 20ms delay";
    }

    for (int i = 0; i < 15; ++i) {
        protection.update(voltage_pu_t{1.20});
    }

    EXPECT_TRUE(protection.is_warning());
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcOvervoltageProtectionTest, TripActivatesAt125Percent)
{
    // Apply 130% voltage (above 125% trip)
    for (int i = 0; i < 50; ++i) {
        protection.update(voltage_pu_t{1.30});
        EXPECT_FALSE(protection.is_tripped()) << "Trip should not activate before 100ms delay";
    }

    for (int i = 0; i < 60; ++i) {
        protection.update(voltage_pu_t{1.30});
    }

    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(SfcOvervoltageProtectionTest, TripLatchesUntilReset)
{
    for (int i = 0; i < 120; ++i) {
        protection.update(voltage_pu_t{1.30});
    }
    ASSERT_TRUE(protection.is_tripped());

    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(SfcOvervoltageProtectionTest, ResetClearsLatchAfterFaultCleared)
{
    for (int i = 0; i < 120; ++i) {
        protection.update(voltage_pu_t{1.30});
    }
    ASSERT_TRUE(protection.is_tripped());

    protection.update(voltage_pu_t{1.0});
    EXPECT_TRUE(protection.can_reset());

    protection.reset();
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcOvervoltageProtectionTest, RideThroughBriefVoltageSwellNoTrip)
{
    // Simulate brief voltage swell (70ms) - should not trip with 100ms delay
    for (int i = 0; i < 50; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    for (int i = 0; i < 70; ++i) {
        protection.update(voltage_pu_t{1.30});
    }

    EXPECT_FALSE(protection.is_tripped()) << "Should ride through brief 70ms swell";

    for (int i = 0; i < 50; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    EXPECT_FALSE(protection.is_tripped());
}

// ==================== Undervoltage Protection Tests ====================

TEST_F(SfcUndervoltageProtectionTest, DefaultThresholdsAreConfigured)
{
    EXPECT_FLOAT_EQ(protection.get_detection_threshold().value(), 0.85f);  // 85%
    EXPECT_FLOAT_EQ(protection.get_recovery_threshold().value(), 0.90f);   // 90%
    EXPECT_FLOAT_EQ(protection.get_warning_threshold().value(), 0.92f);    // 92%
}

TEST_F(SfcUndervoltageProtectionTest, InitialStateNotTripped)
{
    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
    // Note: can_reset() will be false initially because voltage is 0
}

TEST_F(SfcUndervoltageProtectionTest, NominalVoltageNoActivation)
{
    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    EXPECT_FALSE(protection.is_tripped());
    EXPECT_FALSE(protection.is_warning());
    EXPECT_TRUE(protection.can_reset());
}

TEST_F(SfcUndervoltageProtectionTest, WarningActivatesBelow92Percent)
{
    // Apply 90% voltage (below 92% warning, above 85% detection)
    for (int i = 0; i < 50; ++i) {
        protection.update(voltage_pu_t{0.90});
        EXPECT_FALSE(protection.is_warning()) << "Warning should not activate before 100ms delay";
    }

    for (int i = 0; i < 60; ++i) {
        protection.update(voltage_pu_t{0.90});
    }

    EXPECT_TRUE(protection.is_warning());
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcUndervoltageProtectionTest, TripActivatesBelow85Percent)
{
    // Apply 80% voltage (below 85% detection/trip)
    for (int i = 0; i < 250; ++i) {
        protection.update(voltage_pu_t{0.80});
        EXPECT_FALSE(protection.is_tripped()) << "Trip should not activate before 500ms delay";
    }

    for (int i = 0; i < 300; ++i) {
        protection.update(voltage_pu_t{0.80});
    }

    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(SfcUndervoltageProtectionTest, TripLatchesUntilReset)
{
    for (int i = 0; i < 550; ++i) {
        protection.update(voltage_pu_t{0.65});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Voltage returns to normal
    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(SfcUndervoltageProtectionTest, CannotResetWhileFaultActive)
{
    for (int i = 0; i < 550; ++i) {
        protection.update(voltage_pu_t{0.65});
    }
    ASSERT_TRUE(protection.is_tripped());

    protection.update(voltage_pu_t{0.65});
    EXPECT_FALSE(protection.can_reset()) << "Should not allow reset while voltage still low";

    protection.reset();
    EXPECT_TRUE(protection.is_tripped());
}

TEST_F(SfcUndervoltageProtectionTest, ResetClearsLatchAfterFaultCleared)
{
    for (int i = 0; i < 550; ++i) {
        protection.update(voltage_pu_t{0.80});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Voltage recovers above recovery threshold (0.90 pu)
    protection.update(voltage_pu_t{0.95});
    EXPECT_TRUE(protection.can_reset());

    protection.reset();
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcUndervoltageProtectionTest, HysteresisPreventsChattering)
{
    // Detection threshold = 0.85 pu
    // Recovery threshold = 0.90 pu

    for (int i = 0; i < 550; ++i) {
        protection.update(voltage_pu_t{0.80});
    }
    ASSERT_TRUE(protection.is_tripped());

    // Increase to 0.87 (above detection but below recovery)
    protection.update(voltage_pu_t{0.87});
    EXPECT_FALSE(protection.can_reset()) << "Should not allow reset below recovery threshold";

    protection.reset();
    EXPECT_TRUE(protection.is_tripped());

    // Increase above recovery threshold
    protection.update(voltage_pu_t{0.92});
    EXPECT_TRUE(protection.can_reset());

    protection.reset();
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcUndervoltageProtectionTest, RideThroughBriefVoltageSagNoTrip)
{
    // Simulate brief voltage sag (400ms) - should not trip with 500ms delay
    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    for (int i = 0; i < 400; ++i) {
        protection.update(voltage_pu_t{0.80});
    }

    EXPECT_FALSE(protection.is_tripped()) << "Should ride through brief 400ms sag";

    for (int i = 0; i < 100; ++i) {
        protection.update(voltage_pu_t{1.0});
    }

    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcUndervoltageProtectionTest, CustomThresholdsCanBeConfigured)
{
    sfc_undervoltage_protection custom_protection{duration_t{sampling_time}};
    custom_protection.configure_trip_threshold(voltage_pu_t{0.5}, real_t{0.1});  // 50% trip, 10% hysteresis
    custom_protection.configure_warning_threshold(voltage_pu_t{0.75});           // 75% warning
    custom_protection.configure_trip_delay(duration_t{0.2});                     // 200ms delay

    EXPECT_FLOAT_EQ(custom_protection.get_trip_threshold().value(), 0.5f);
    EXPECT_FLOAT_EQ(custom_protection.get_warning_threshold().value(), 0.75f);

    // Should not trip at 0.60
    for (int i = 0; i < 300; ++i) {
        custom_protection.update(voltage_pu_t{0.6});
    }
    EXPECT_FALSE(custom_protection.is_tripped());

    // Should trip at 0.40 after 200ms
    for (int i = 0; i < 210; ++i) {
        custom_protection.update(voltage_pu_t{0.4});
    }
    EXPECT_TRUE(custom_protection.is_tripped());
}

TEST_F(SfcUndervoltageProtectionTest, WarningClearsImmediatelyWhenAboveThreshold)
{
    // Activate warning (below 0.92)
    for (int i = 0; i < 120; ++i) {
        protection.update(voltage_pu_t{0.90});
    }
    ASSERT_TRUE(protection.is_warning());

    // Rise above warning threshold
    protection.update(voltage_pu_t{1.0});

    EXPECT_FALSE(protection.is_warning());
    EXPECT_FALSE(protection.is_tripped());
}

TEST_F(SfcUndervoltageProtectionTest, DipDetectionIncludesRecoveryHysteresis)
{
    // Start healthy
    protection.update(voltage_pu_t{1.0});
    ASSERT_FALSE(protection.is_detected());

    // Drop below detection threshold (0.85 default in hpp, but constructor used 0.70 via legacy)
    // Actually, I changed the defaults in hpp to 0.85/0.90
    protection.update(voltage_pu_t{0.80});
    EXPECT_TRUE(protection.is_detected()) << "Should detect dip immediately";

    // Rise above detection but below recovery (0.90 default)
    protection.update(voltage_pu_t{0.87});
    EXPECT_TRUE(protection.is_detected()) << "Should stay detected in hysteresis band";

    // Rise above recovery threshold
    protection.update(voltage_pu_t{0.95});
    EXPECT_TRUE(protection.is_detected()) << "Should stay detected during recovery delay";

    // Wait for recovery delay (100ms default)
    for (int i = 0; i < 110; ++i) {
        protection.update(voltage_pu_t{0.95});
    }
    EXPECT_FALSE(protection.is_detected()) << "Should clear after recovery delay";
}
