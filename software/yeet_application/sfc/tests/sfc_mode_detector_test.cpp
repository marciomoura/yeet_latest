#include "sfc/sfc_mode_detector.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
using namespace yeet;

class SfcModeDetectorTest : public ::testing::Test {
protected:
    const speed_pu_t handover_speed{0.1F};   // 10% speed
    const real_t hysteresis_factor{0.1F};    // 10% hysteresis -> [0.09, 0.11] band
    const real_t switch_down_speed = 0.09F;  // 0.1 * (1 - 0.1)
    const real_t switch_up_speed = 0.11F;    // 0.1 * (1 + 0.1)
    const bool enable = true;

    void SetUp() override
    {
        detector.configure_mode_handover_speed_pu(handover_speed);
        detector.configure_hysteresis_factor(hysteresis_factor);
    }

    sfc_mode_detector detector;
};

TEST_F(SfcModeDetectorTest, InitialStateIsPulseMode)
{
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
    EXPECT_FALSE(detector.has_mode_changed());
}

TEST_F(SfcModeDetectorTest, ResetReturnsToPulseMode)
{
    // Go to load commutated mode first
    detector.update(enable, speed_pu_t{switch_up_speed + 0.01F});
    ASSERT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);

    detector.reset();
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
}

TEST_F(SfcModeDetectorTest, DisabledStateForcesPulseMode)
{
    // Go to load commutated mode first
    detector.update(enable, speed_pu_t{switch_up_speed + 0.01F});
    ASSERT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);

    // Now disable the system, it should immediately revert to pulse mode
    detector.update(!enable, speed_pu_t{switch_up_speed + 0.01F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
    EXPECT_TRUE(detector.has_mode_changed());
}

TEST_F(SfcModeDetectorTest, TransitionToLoadCommutatedMode)
{
    // Speed is below the high threshold, should not switch
    detector.update(enable, speed_pu_t{switch_up_speed - 0.001F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
    EXPECT_FALSE(detector.has_mode_changed());

    // Speed crosses the high threshold, should switch
    detector.update(enable, speed_pu_t{switch_up_speed + 0.001F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
    EXPECT_TRUE(detector.has_mode_changed());

    // Call update again with the same speed, mode should not change again
    detector.update(enable, speed_pu_t{switch_up_speed + 0.001F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
    EXPECT_FALSE(detector.has_mode_changed());
}

TEST_F(SfcModeDetectorTest, TransitionToPulseMode)
{
    // First, go to load commutated mode
    detector.update(enable, speed_pu_t{switch_up_speed + 0.01F});
    detector.update(enable, speed_pu_t{switch_up_speed + 0.01F});  // second step to clear has_mode_changed
    ASSERT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
    ASSERT_FALSE(detector.has_mode_changed());

    // Speed is above the low threshold, should not switch back
    detector.update(enable, speed_pu_t{switch_down_speed + 0.001F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
    EXPECT_FALSE(detector.has_mode_changed());

    // Speed crosses the low threshold, should switch back to pulse mode
    detector.update(enable, speed_pu_t{switch_down_speed - 0.001F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
    EXPECT_TRUE(detector.has_mode_changed());
}

TEST_F(SfcModeDetectorTest, HysteresisPreventsChattering)
{
    // 1. Go to load commutated mode
    detector.update(enable, speed_pu_t{switch_up_speed + 0.01F});
    ASSERT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);

    // 2. Jitter inside the hysteresis band, should stay in load commutated mode
    detector.update(enable, speed_pu_t{switch_up_speed - 0.005F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
    detector.update(enable, speed_pu_t{switch_down_speed + 0.005F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);

    // 3. Go back to pulse mode
    detector.update(enable, speed_pu_t{switch_down_speed - 0.01F});
    ASSERT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);

    // 4. Jitter inside the hysteresis band again, should stay in pulse mode
    detector.update(enable, speed_pu_t{switch_down_speed + 0.005F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
    detector.update(enable, speed_pu_t{switch_up_speed - 0.005F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
}

TEST_F(SfcModeDetectorTest, ConfigurationChangesThresholds)
{
    // Test with different handover speed
    detector.configure_mode_handover_speed_pu(speed_pu_t{0.2F});
    detector.configure_hysteresis_factor(0.1F);

    // New thresholds should be 0.18 and 0.22
    detector.update(enable, speed_pu_t{0.21F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);

    detector.update(enable, speed_pu_t{0.22F + 0.001F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
    EXPECT_TRUE(detector.has_mode_changed());
}

TEST_F(SfcModeDetectorTest, ZeroHysteresisCreatesSharpTransition)
{
    // Configure with zero hysteresis
    detector.configure_hysteresis_factor(0.0F);

    // Both thresholds should be at 0.1
    detector.update(enable, speed_pu_t{0.099F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);

    detector.update(enable, speed_pu_t{0.101F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
    EXPECT_TRUE(detector.has_mode_changed());

    detector.update(enable, speed_pu_t{0.099F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
    EXPECT_TRUE(detector.has_mode_changed());
}

TEST_F(SfcModeDetectorTest, LargeHysteresisCreatesWideDeadBand)
{
    // Configure with large hysteresis (50%)
    detector.configure_hysteresis_factor(0.5F);

    // Thresholds should be 0.05 and 0.15
    detector.update(enable, speed_pu_t{0.14F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);

    detector.update(enable, speed_pu_t{0.16F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
    EXPECT_TRUE(detector.has_mode_changed());

    // Large dead band - should stay in load commutated mode
    detector.update(enable, speed_pu_t{0.06F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
    EXPECT_FALSE(detector.has_mode_changed());

    detector.update(enable, speed_pu_t{0.04F});
    EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
    EXPECT_TRUE(detector.has_mode_changed());
}

TEST_F(SfcModeDetectorTest, RepeatedEnableDisableCycles)
{
    // Go to load commutated mode
    detector.update(enable, speed_pu_t{switch_up_speed + 0.01F});
    ASSERT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);

    // Disable and re-enable multiple times
    for (int i = 0; i < 3; ++i) {
        detector.update(!enable, speed_pu_t{switch_up_speed + 0.01F});
        EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::pulse_mode);
        EXPECT_TRUE(detector.has_mode_changed());

        detector.update(enable, speed_pu_t{switch_up_speed + 0.01F});
        EXPECT_EQ(detector.get_operating_mode(), sfc_operating_mode_t::load_commutated);
        EXPECT_TRUE(detector.has_mode_changed());
    }
}