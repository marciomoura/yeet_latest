#include "yeet_control/interval_timer.hpp"

#include <gtest/gtest.h>

namespace {

using namespace yeet;
using namespace yeet::control;

// Test fixture for common setup
class IntervalTimerTest : public ::testing::Test {
protected:
    static constexpr real_t sampling_time = 0.001;      // 1ms sampling time in seconds
    static constexpr real_t sampling_time_us = 1000.0;  // 1000µs equivalent
    static constexpr real_t tolerance = 1e-6;           // Relaxed tolerance for floating point conversion
};

TEST_F(IntervalTimerTest, InitialStateIsNotRunning) {
    interval_timer timer(sampling_time);

    EXPECT_FALSE(timer.is_running());
    EXPECT_NEAR(timer.get_elapsed_time(), 0.0, tolerance);
}

TEST_F(IntervalTimerTest, StartsAndAccumulatesTime) {
    interval_timer timer(sampling_time);

    timer.start();
    EXPECT_TRUE(timer.is_running());
    EXPECT_NEAR(timer.get_elapsed_time(), 0.0, tolerance);

    // Simulate 5 control cycles
    for (int i = 0; i < 5; ++i) {
        timer.update();
    }

    EXPECT_TRUE(timer.is_running());
    EXPECT_NEAR(timer.get_elapsed_time(), 5.0 * sampling_time_us, tolerance);
}

TEST_F(IntervalTimerTest, StopReturnsElapsedTime) {
    interval_timer timer(sampling_time);

    timer.start();

    // Simulate 3 control cycles
    for (int i = 0; i < 3; ++i) {
        timer.update();
    }

    const real_t elapsed = timer.stop();
    const real_t expected_time = 3.0 * sampling_time_us;

    EXPECT_NEAR(elapsed, expected_time, tolerance);
    EXPECT_FALSE(timer.is_running());
    EXPECT_NEAR(timer.get_elapsed_time(), expected_time, tolerance);
}

TEST_F(IntervalTimerTest, UpdateDoesNothingWhenNotRunning) {
    interval_timer timer(sampling_time);

    // Timer is not started, updates should have no effect
    for (int i = 0; i < 5; ++i) {
        timer.update();
    }

    EXPECT_FALSE(timer.is_running());
    EXPECT_NEAR(timer.get_elapsed_time(), 0.0, tolerance);
}

TEST_F(IntervalTimerTest, UpdateDoesNothingAfterStop) {
    interval_timer timer(sampling_time);

    timer.start();
    timer.update();
    timer.update();

    const real_t elapsed_before_stop = timer.get_elapsed_time();
    timer.stop();

    // Updates after stop should have no effect
    timer.update();
    timer.update();

    EXPECT_FALSE(timer.is_running());
    EXPECT_NEAR(timer.get_elapsed_time(), elapsed_before_stop, tolerance);
}

TEST_F(IntervalTimerTest, ResetClearsStateAndStopsTimer) {
    interval_timer timer(sampling_time);

    timer.start();
    timer.update();
    timer.update();
    timer.update();

    EXPECT_TRUE(timer.is_running());
    EXPECT_GT(timer.get_elapsed_time(), 0.0);

    timer.reset();

    EXPECT_FALSE(timer.is_running());
    EXPECT_NEAR(timer.get_elapsed_time(), 0.0, tolerance);
}

TEST_F(IntervalTimerTest, StartRestartsClearingPreviousTime) {
    interval_timer timer(sampling_time);

    timer.start();
    timer.update();
    timer.update();

    EXPECT_GT(timer.get_elapsed_time(), 0.0);

    // Restart the timer
    timer.start();

    EXPECT_TRUE(timer.is_running());
    EXPECT_NEAR(timer.get_elapsed_time(), 0.0, tolerance);
}

TEST_F(IntervalTimerTest, ConfigureChangesSamplingTimeAndResets) {
    interval_timer timer(sampling_time);

    timer.start();
    timer.update();
    timer.update();

    EXPECT_TRUE(timer.is_running());
    EXPECT_GT(timer.get_elapsed_time(), 0.0);

    const real_t new_sampling_time = 0.0005;  // 500µs in seconds
    timer.configure(new_sampling_time);

    // Timer should be reset after configure
    EXPECT_FALSE(timer.is_running());
    EXPECT_NEAR(timer.get_elapsed_time(), 0.0, tolerance);

    // Verify new sampling time is used
    timer.start();
    timer.update();

    EXPECT_NEAR(timer.get_elapsed_time(), 500.0, tolerance);  // 500µs
}

TEST_F(IntervalTimerTest, PreciseTimingMeasurement) {
    const real_t precise_sampling_time = 50e-6;  // 50µs in seconds
    interval_timer timer(precise_sampling_time);

    timer.start();

    // Simulate 20 precise updates
    for (int i = 0; i < 20; ++i) {
        timer.update();
    }

    const real_t expected_time = 20.0 * 50.0;  // 1000µs
    EXPECT_NEAR(timer.get_elapsed_time(), expected_time, tolerance);

    const real_t stopped_time = timer.stop();
    EXPECT_NEAR(stopped_time, expected_time, tolerance);
}

TEST_F(IntervalTimerTest, MultipleStartStopCycles) {
    interval_timer timer(sampling_time);

    // First cycle
    timer.start();
    timer.update();
    timer.update();
    real_t first_elapsed = timer.stop();
    EXPECT_NEAR(first_elapsed, 2.0 * sampling_time_us, tolerance);

    // Second cycle
    timer.start();
    timer.update();
    timer.update();
    timer.update();
    real_t second_elapsed = timer.stop();
    EXPECT_NEAR(second_elapsed, 3.0 * sampling_time_us, tolerance);

    // Verify independence
    EXPECT_NE(first_elapsed, second_elapsed);
}

TEST_F(IntervalTimerTest, GetElapsedTimeDoesNotAffectRunningState) {
    interval_timer timer(sampling_time);

    timer.start();
    timer.update();
    timer.update();

    // Multiple calls to get_elapsed_time should not affect state
    real_t time1 = timer.get_elapsed_time();
    real_t time2 = timer.get_elapsed_time();
    real_t time3 = timer.get_elapsed_time();

    EXPECT_NEAR(time1, time2, tolerance);
    EXPECT_NEAR(time2, time3, tolerance);
    EXPECT_TRUE(timer.is_running());

    timer.update();
    real_t time4 = timer.get_elapsed_time();
    EXPECT_GT(time4, time1);
}

TEST_F(IntervalTimerTest, SecondsToMicrosecondsConversion) {
    const real_t sampling_time_10ms = 0.01;  // 10ms in seconds
    interval_timer timer(sampling_time_10ms);

    timer.start();
    timer.update();

    // Should return 10,000 microseconds
    EXPECT_NEAR(timer.get_elapsed_time(), 10000.0, tolerance);
}

// Death tests for assertion validation
#ifdef NDEBUG
// Skip death tests in release builds where assertions are disabled
#else
TEST_F(IntervalTimerTest, ConstructorAssertionOnNegativeSamplingTime) {
    EXPECT_DEATH(interval_timer timer(-0.001), "Sampling time must be positive");
}

TEST_F(IntervalTimerTest, ConstructorAssertionOnZeroSamplingTime) {
    EXPECT_DEATH(interval_timer timer(0.0), "Sampling time must be positive");
}

TEST_F(IntervalTimerTest, ConfigureAssertionOnNegativeSamplingTime) {
    interval_timer timer(sampling_time);
    EXPECT_DEATH(timer.configure(-0.001), "Sampling time must be positive");
}

TEST_F(IntervalTimerTest, ConfigureAssertionOnZeroSamplingTime) {
    interval_timer timer(sampling_time);
    EXPECT_DEATH(timer.configure(0.0), "Sampling time must be positive");
}
#endif

}  // namespace