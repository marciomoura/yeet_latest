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

    rate_of_change_limiter limiter_default_init(10.0f, sampling_time);
    EXPECT_FLOAT_EQ(limiter_default_init.get_output(), 0.0f);
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

}  // namespace