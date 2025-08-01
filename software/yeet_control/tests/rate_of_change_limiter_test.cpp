#include "yeet_control/rate_of_change_limiter.hpp"

#include <gtest/gtest.h>

namespace {

using namespace yeet;
using namespace yeet::control;

// Test fixture for common setup
class RateOfChangeLimiterTest : public ::testing::Test {
protected:
    const real_t sampling_time = 0.1f;  // 100 ms sampling time for easy calculations
};

TEST_F(RateOfChangeLimiterTest, InitialStateIsCorrect) {
    const real_t initial_value = 5.0f;
    rate_of_change_limiter limiter(10.0f, sampling_time, initial_value);
    EXPECT_FLOAT_EQ(limiter.get_output(), initial_value);
    EXPECT_TRUE(limiter.is_enabled());  // Should be enabled by default

    rate_of_change_limiter limiter_default_init(10.0f, sampling_time);
    EXPECT_FLOAT_EQ(limiter_default_init.get_output(), 0.0f);
    EXPECT_TRUE(limiter_default_init.is_enabled());
}

TEST_F(RateOfChangeLimiterTest, RampsUpAtCorrectRate) {
    const real_t rate_limit = 10.0f;  // 10 units / sec
    // With 0.1s sampling, max change per sample is 1.0 unit
    rate_of_change_limiter limiter(rate_limit, sampling_time);

    // Target a large positive value
    limiter.update(100.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 1.0f);

    limiter.update(100.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 2.0f);

    limiter.update(100.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 3.0f);
}

TEST_F(RateOfChangeLimiterTest, RampsDownAtCorrectRate) {
    const real_t rate_limit = 10.0f;  // 10 units / sec
    // With 0.1s sampling, max change per sample is 1.0 unit
    rate_of_change_limiter limiter(rate_limit, sampling_time, 50.0f);

    // Target a large negative value
    limiter.update(0.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 49.0f);

    limiter.update(0.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 48.0f);

    limiter.update(0.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 47.0f);
}

TEST_F(RateOfChangeLimiterTest, ReachesTargetWithoutOvershoot) {
    const real_t rate_limit = 10.0f;  // max change per sample = 1.0
    rate_of_change_limiter limiter(rate_limit, sampling_time);

    limiter.update(0.5f);
    // The change is less than the limit, so it should go directly to the target
    EXPECT_FLOAT_EQ(limiter.get_output(), 0.5f);

    limiter.update(0.5f);
    // Should stay at the target
    EXPECT_FLOAT_EQ(limiter.get_output(), 0.5f);
}

TEST_F(RateOfChangeLimiterTest, ReachesFinalValueExactly) {
    const real_t rate_limit = 10.0f;  // max change per sample = 1.0
    rate_of_change_limiter limiter(rate_limit, sampling_time);
    const real_t target = 3.5f;

    limiter.update(target);  // output: 1.0
    limiter.update(target);  // output: 2.0
    limiter.update(target);  // output: 3.0

    // The remaining error is 0.5, which is less than the max change of 1.0
    limiter.update(target);
    EXPECT_FLOAT_EQ(limiter.get_output(), 3.5f);

    // Should stay at the target
    limiter.update(target);
    EXPECT_FLOAT_EQ(limiter.get_output(), 3.5f);
}

TEST_F(RateOfChangeLimiterTest, ResetMethodWorks) {
    rate_of_change_limiter limiter(10.0f, sampling_time, 50.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 50.0f);

    limiter.reset(123.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 123.0f);

    // Test default reset
    limiter.reset();
    EXPECT_FLOAT_EQ(limiter.get_output(), 0.0f);
}

TEST_F(RateOfChangeLimiterTest, ConfigureChangesRate) {
    rate_of_change_limiter limiter(10.0f, sampling_time);  // max change = 1.0

    limiter.update(100.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 1.0f);

    // Increase the rate limit
    limiter.configure(50.0f);  // max change = 5.0

    limiter.update(100.0f);
    // New output should be old output + new max change = 1.0 + 5.0
    EXPECT_FLOAT_EQ(limiter.get_output(), 6.0f);
}

TEST_F(RateOfChangeLimiterTest, DisableEnablesPassThroughMode) {
    const real_t rate_limit = 10.0f;  // max change per sample = 1.0
    rate_of_change_limiter limiter(rate_limit, sampling_time);

    // First, demonstrate normal rate limiting behavior
    limiter.update(100.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 1.0f);

    // Now disable the rate limiter
    limiter.configure_enable(false);
    EXPECT_FALSE(limiter.is_enabled());

    // Input should pass through directly
    limiter.update(50.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 50.0f);

    limiter.update(25.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 25.0f);

    limiter.update(100.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 100.0f);
}

TEST_F(RateOfChangeLimiterTest, ReEnableRestoresRateLimiting) {
    const real_t rate_limit = 10.0f;  // max change per sample = 1.0
    rate_of_change_limiter limiter(rate_limit, sampling_time);

    // Disable and set to a specific value
    limiter.configure_enable(false);
    limiter.update(50.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 50.0f);

    // Re-enable rate limiting
    limiter.configure_enable(true);
    EXPECT_TRUE(limiter.is_enabled());

    // Rate limiting should now be active again
    limiter.update(100.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 51.0f);  // 50.0 + 1.0 (max change)

    limiter.update(100.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 52.0f);  // 51.0 + 1.0 (max change)
}

TEST_F(RateOfChangeLimiterTest, DisabledModeIgnoresRateLimit) {
    const real_t rate_limit = 1.0f;  // Very slow rate limit
    rate_of_change_limiter limiter(rate_limit, sampling_time);

    // Disable the limiter
    limiter.configure_enable(false);

    // Large step changes should pass through immediately
    limiter.update(1000.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 1000.0f);

    limiter.update(-500.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), -500.0f);

    limiter.update(0.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 0.0f);
}

TEST_F(RateOfChangeLimiterTest, EnableStatePreservedAfterConfigure) {
    rate_of_change_limiter limiter(10.0f, sampling_time);

    // Disable the limiter
    limiter.configure_enable(false);
    EXPECT_FALSE(limiter.is_enabled());

    // Reconfigure the rate limit
    limiter.configure(20.0f);

    // Enable state should be preserved
    EXPECT_FALSE(limiter.is_enabled());

    // Behavior should still be pass-through
    limiter.update(100.0f);
    EXPECT_FLOAT_EQ(limiter.get_output(), 100.0f);
}

}  // namespace