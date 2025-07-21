#include "yeet_control/on_delay.hpp"

#include <gtest/gtest.h>

namespace {

using namespace yeet;
using namespace yeet::control;

// Test fixture for common setup
class OnDelayTest : public ::testing::Test {
protected:
    const real_t sampling_time = 0.01f;  // 10 ms sampling time
};

TEST_F(OnDelayTest, InitialStateIsFalse) {
    const real_t delay = 0.1f;  // 100 ms delay
    on_delay timer(delay, sampling_time);

    // The output should be false immediately after construction.
    EXPECT_FALSE(timer.get_output());
}

TEST_F(OnDelayTest, TurnsOnExactlyWhenDelayIsMet) {
    const real_t delay = 0.1f;  // 100 ms delay
    on_delay timer(delay, sampling_time);
    const int required_samples = static_cast<int>(delay / sampling_time);  // 10 samples

    // Update n-1 times
    for (int i = 0; i < required_samples - 1; ++i) {
        timer.update(true);
        EXPECT_FALSE(timer.get_output()) << "Timer turned on prematurely at sample " << i + 1;
    }

    // The final update that meets the delay requirement
    timer.update(true);
    EXPECT_TRUE(timer.get_output()) << "Timer did not turn on at the correct sample";

    // It should stay on after the delay is met
    timer.update(true);
    EXPECT_TRUE(timer.get_output()) << "Timer turned off after being active";
}

TEST_F(OnDelayTest, TimerResetsWhenInputGoesFalse) {
    const real_t delay = 0.1f;  // 100 ms delay
    on_delay timer(delay, sampling_time);
    const int required_samples = static_cast<int>(delay / sampling_time);  // 10 samples

    // Update for some time, but not enough to turn on
    for (int i = 0; i < required_samples / 2; ++i) {
        timer.update(true);
    }
    EXPECT_FALSE(timer.get_output());

    // Send a 'false' input, which should reset the internal counter
    timer.update(false);
    EXPECT_FALSE(timer.get_output());

    // Now, it should require the full delay time again
    for (int i = 0; i < required_samples - 1; ++i) {
        timer.update(true);
        EXPECT_FALSE(timer.get_output()) << "Timer did not reset properly";
    }

    timer.update(true);
    EXPECT_TRUE(timer.get_output()) << "Timer did not turn on after reset and full delay";
}

TEST_F(OnDelayTest, OutputGoesFalseImmediately) {
    const real_t delay = 0.1f;  // 100 ms delay
    on_delay timer(delay, sampling_time);
    const int required_samples = static_cast<int>(delay / sampling_time);  // 10 samples

    // Turn the timer on
    for (int i = 0; i < required_samples; ++i) {
        timer.update(true);
    }
    EXPECT_TRUE(timer.get_output());

    // Update with a false signal
    timer.update(false);

    // Output should immediately go to false
    EXPECT_FALSE(timer.get_output());
}

TEST_F(OnDelayTest, ResetMethodWorksWhenActive) {
    const real_t delay = 0.1f;  // 100 ms delay
    on_delay timer(delay, sampling_time);
    const int required_samples = static_cast<int>(delay / sampling_time);  // 10 samples

    // Turn the timer on
    for (int i = 0; i < required_samples; ++i) {
        timer.update(true);
    }
    EXPECT_TRUE(timer.get_output());

    // Now, reset it
    timer.reset();
    EXPECT_FALSE(timer.get_output());
}

TEST_F(OnDelayTest, ConfigureResetsTheTimer) {
    const real_t delay = 0.1f;  // 100 ms delay
    on_delay timer(delay, sampling_time);
    const int required_samples = static_cast<int>(delay / sampling_time);  // 10 samples

    // Turn the timer on
    for (int i = 0; i < required_samples; ++i) {
        timer.update(true);
    }
    EXPECT_TRUE(timer.get_output());

    // Re-configure with a new delay
    timer.configure(0.2f);

    // The timer should have been reset by configure()
    EXPECT_FALSE(timer.get_output());
}

TEST_F(OnDelayTest, ZeroDelayTurnsOnImmediately) {
    const real_t delay = 0.0f;
    on_delay timer(delay, sampling_time);

    // Output should still be false initially
    EXPECT_FALSE(timer.get_output());

    // A single true input should make the output true
    timer.update(true);
    EXPECT_TRUE(timer.get_output());

    // A single false input should make the output false
    timer.update(false);
    EXPECT_FALSE(timer.get_output());
}

}  // namespace