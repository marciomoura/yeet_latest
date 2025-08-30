#include "yeet_control/hysteresis_limiter.hpp"

#include <gtest/gtest.h>

using namespace yeet::control;

class HysteresisLimiterTest : public ::testing::Test {
protected:
    void SetUp() override { limiter.configure_thresholds(10.0F, 20.0F); }

    hysteresis_limiter limiter;
};

TEST_F(HysteresisLimiterTest, InitialStateIsFalse) { EXPECT_FALSE(limiter.get_output()); }

TEST_F(HysteresisLimiterTest, ResetReturnsToFalse)
{
    limiter.update(25.0F);  // Go high
    ASSERT_TRUE(limiter.get_output());

    limiter.reset();
    EXPECT_FALSE(limiter.get_output());
}

TEST_F(HysteresisLimiterTest, DoesNotTriggerBelowHighThreshold)
{
    limiter.update(19.9F);
    EXPECT_FALSE(limiter.get_output());
}

TEST_F(HysteresisLimiterTest, TriggersWhenCrossingHighThreshold)
{
    limiter.update(20.1F);
    EXPECT_TRUE(limiter.get_output());
}

TEST_F(HysteresisLimiterTest, StaysHighWhenAboveLowThreshold)
{
    limiter.update(25.0F);  // Go high
    ASSERT_TRUE(limiter.get_output());

    limiter.update(10.1F);  // Drop into hysteresis band
    EXPECT_TRUE(limiter.get_output());
}

TEST_F(HysteresisLimiterTest, GoesLowWhenCrossingLowThreshold)
{
    limiter.update(25.0F);  // Go high
    ASSERT_TRUE(limiter.get_output());

    limiter.update(9.9F);  // Drop below low threshold
    EXPECT_FALSE(limiter.get_output());
}

TEST_F(HysteresisLimiterTest, ChatteringInHysteresisBand)
{
    limiter.update(25.0F);  // Go high
    ASSERT_TRUE(limiter.get_output());

    // Jitter above low threshold
    limiter.update(11.0F);
    limiter.update(15.0F);
    limiter.update(12.0F);
    EXPECT_TRUE(limiter.get_output()) << "Output should not chatter while in the hysteresis band.";

    limiter.update(9.0F);  // Go low
    ASSERT_FALSE(limiter.get_output());

    // Jitter below high threshold
    limiter.update(18.0F);
    limiter.update(15.0F);
    limiter.update(19.0F);
    EXPECT_FALSE(limiter.get_output()) << "Output should not chatter while in the hysteresis band.";
}