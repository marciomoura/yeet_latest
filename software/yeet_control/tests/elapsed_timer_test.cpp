#include "yeet_control/elapsed_timer.hpp"

#include <gtest/gtest.h>

using namespace yeet;
using namespace yeet::control;

class ElapsedTimerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Standard 1ms sampling time for most tests
        sampling_time = duration_t(0.001);
        timer = std::make_unique<elapsed_timer>(sampling_time);
    }

    duration_t sampling_time;
    std::unique_ptr<elapsed_timer> timer;
};

// Test basic construction and initial state
TEST_F(ElapsedTimerTest, InitialState)
{
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.0, 1e-5);
    EXPECT_FALSE(timer->is_enabled());
    EXPECT_FALSE(timer->has_elapsed(duration_t(0.001)));
    EXPECT_NEAR(timer->get_remaining_time(duration_t(0.005)).value(), 0.005, 1e-5);
}

// Test basic timing functionality
TEST_F(ElapsedTimerTest, BasicTiming)
{
    // Timer should not increment when disabled
    timer->update(false);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.0, 1e-5);
    EXPECT_FALSE(timer->is_enabled());

    // Timer should increment when enabled
    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.001, 1e-5);
    EXPECT_TRUE(timer->is_enabled());

    // Multiple updates should accumulate time
    timer->update(true);
    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.003, 1e-5);
}

// Test pause and resume functionality
TEST_F(ElapsedTimerTest, PauseAndResume)
{
    // Run for 3 cycles
    timer->update(true);
    timer->update(true);
    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.003, 1e-5);

    // Pause for 2 cycles
    timer->update(false);
    timer->update(false);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.003, 1e-5);  // Should not change
    EXPECT_FALSE(timer->is_enabled());

    // Resume for 1 cycle
    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.004, 1e-5);
    EXPECT_TRUE(timer->is_enabled());
}

// Test reset functionality
TEST_F(ElapsedTimerTest, Reset)
{
    // Accumulate some time
    timer->update(true);
    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.002, 1e-5);

    // Reset should clear elapsed time but preserve enable state
    timer->reset();
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.0, 1e-5);
    EXPECT_TRUE(timer->is_enabled());  // Should preserve enabled state

    // Timer should continue working after reset
    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.001, 1e-5);
}

// Test reset with initial value
TEST_F(ElapsedTimerTest, ResetWithInitialValue)
{
    const duration_t initial_time(0.005);
    timer->reset(initial_time);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.005, 1e-5);

    // Timer should continue from initial value
    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.006, 1e-5);
}

// Test threshold detection
TEST_F(ElapsedTimerTest, ThresholdDetection)
{
    const duration_t threshold(0.0025);  // 2.5ms threshold

    // Before threshold
    timer->update(true);
    timer->update(true);  // 2ms elapsed
    EXPECT_FALSE(timer->has_elapsed(threshold));

    // At threshold
    timer->update(true);  // 3ms elapsed (>= 2.5ms)
    EXPECT_TRUE(timer->has_elapsed(threshold));

    // After threshold
    timer->update(true);  // 4ms elapsed
    EXPECT_TRUE(timer->has_elapsed(threshold));
}

// Test remaining time calculation
TEST_F(ElapsedTimerTest, RemainingTime)
{
    const duration_t target_time(0.005);  // 5ms target

    // Initially, full time remaining
    EXPECT_NEAR(timer->get_remaining_time(target_time).value(), 0.005, 1e-5);

    // After 2ms elapsed
    timer->update(true);
    timer->update(true);
    EXPECT_NEAR(timer->get_remaining_time(target_time).value(), 0.003, 1e-5);

    // After 5ms elapsed (at target)
    timer->update(true);
    timer->update(true);
    timer->update(true);
    EXPECT_NEAR(timer->get_remaining_time(target_time).value(), 0.0, 1e-5);

    // After 6ms elapsed (past target)
    timer->update(true);
    EXPECT_NEAR(timer->get_remaining_time(target_time).value(), 0.0, 1e-5);
}

// Test sampling time configuration
TEST_F(ElapsedTimerTest, SamplingTimeConfiguration)
{
    // Change sampling time to 2ms
    const duration_t new_sampling_time(0.002);
    timer->configure_sampling_time(new_sampling_time);

    // Timer should now increment by 2ms per update
    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.002, 1e-5);

    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.004, 1e-5);
}

// Test with different sampling times
TEST_F(ElapsedTimerTest, DifferentSamplingTimes)
{
    // Test with 100µs sampling time
    elapsed_timer fast_timer(duration_t(0.0001));

    fast_timer.update(true);
    fast_timer.update(true);
    EXPECT_NEAR(fast_timer.get_elapsed_time().value(), 0.0002, 1e-5);

    // Test with 10ms sampling time
    elapsed_timer slow_timer(duration_t(0.01));

    slow_timer.update(true);
    slow_timer.update(true);
    EXPECT_NEAR(slow_timer.get_elapsed_time().value(), 0.02, 1e-5);
}

// Test edge cases
TEST_F(ElapsedTimerTest, EdgeCases)
{
    // Test with zero threshold
    EXPECT_TRUE(timer->has_elapsed(duration_t(0.0)));
    EXPECT_NEAR(timer->get_remaining_time(duration_t(0.0)).value(), 0.0, 1e-5);

    // Test negative initial time (should work)
    timer->reset(duration_t(-0.001));
    EXPECT_NEAR(timer->get_elapsed_time().value(), -0.001, 1e-5);

    timer->update(true);
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.0, 1e-5);
}

// Test rapid enable/disable cycling
TEST_F(ElapsedTimerTest, RapidEnableDisable)
{
    for (int i = 0; i < 10; ++i) {
        timer->update(i % 2 == 0);  // Alternate enable/disable
    }

    // Only 5 enabled cycles should have passed
    EXPECT_NEAR(timer->get_elapsed_time().value(), 0.005, 1e-5);
}

// Test precision with many small increments
TEST_F(ElapsedTimerTest, PrecisionTest)
{
    // Use very small sampling time
    elapsed_timer precise_timer(duration_t(1e-6));  // 1µs

    // Run for 1000 cycles (1ms total)
    for (int i = 0; i < 1000; ++i) {
        precise_timer.update(true);
    }

    EXPECT_NEAR(precise_timer.get_elapsed_time().value(), 0.001, 1e-5);
    EXPECT_TRUE(precise_timer.has_elapsed(duration_t(0.0005)));
    EXPECT_FALSE(precise_timer.has_elapsed(duration_t(0.002)));
}

// Test construction with different sampling times
TEST(ElapsedTimerConstructionTest, DifferentSamplingTimes)
{
    elapsed_timer timer1(duration_t(0.001));
    elapsed_timer timer2(duration_t(0.0001));
    elapsed_timer timer3(duration_t(0.01));

    // All should start at zero
    EXPECT_NEAR(timer1.get_elapsed_time().value(), 0.0, 1e-5);
    EXPECT_NEAR(timer2.get_elapsed_time().value(), 0.0, 1e-5);
    EXPECT_NEAR(timer3.get_elapsed_time().value(), 0.0, 1e-5);

    // All should be initially disabled
    EXPECT_FALSE(timer1.is_enabled());
    EXPECT_FALSE(timer2.is_enabled());
    EXPECT_FALSE(timer3.is_enabled());
}