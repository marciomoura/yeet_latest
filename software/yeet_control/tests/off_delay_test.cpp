#include "yeet_control/off_delay.hpp"

#include <gtest/gtest.h>

namespace {

using namespace yeet;
using namespace yeet::control;

// Test fixture for common setup
class OffDelayTest : public ::testing::Test {
protected:
    const real_t sampling_time = 0.01f;  // 10 ms sampling time
};

TEST_F(OffDelayTest, InitialStateIsFalse) {
    const real_t delay = 0.1f;  // 100 ms delay
    off_delay timer(delay, sampling_time);

    // The output should be false immediately after construction.
    EXPECT_FALSE(timer.get_output());
}

TEST_F(OffDelayTest, TurnsOnImmediately) {
    const real_t delay = 0.1f;
    off_delay timer(delay, sampling_time);

    timer.update(true);
    EXPECT_TRUE(timer.get_output());
}

TEST_F(OffDelayTest, TurnsOffExactlyWhenDelayIsMet) {
    const real_t delay = 0.1f;  // 100 ms delay
    off_delay timer(delay, sampling_time);
    const int required_samples = static_cast<int>(delay / sampling_time);  // 10 samples

    // Turn it on
    timer.update(true);
    EXPECT_TRUE(timer.get_output());

    // Update n-1 times with false input
    for (int i = 0; i < required_samples - 1; ++i) {
        timer.update(false);
        EXPECT_TRUE(timer.get_output()) << "Timer turned off prematurely at sample " << i + 1;
    }

    // The final update that meets the delay requirement should turn it off
    timer.update(false);
    EXPECT_FALSE(timer.get_output()) << "Timer did not turn off at the correct sample";

    // It should stay off
    timer.update(false);
    EXPECT_FALSE(timer.get_output()) << "Timer turned back on after being off";
}

TEST_F(OffDelayTest, TimerResetsWhenInputGoesTrue) {
    const real_t delay = 0.1f;  // 100 ms delay
    off_delay timer(delay, sampling_time);
    const int required_samples = static_cast<int>(delay / sampling_time);  // 10 samples

    // Turn on, then start the off-delay timer
    timer.update(true);
    for (int i = 0; i < required_samples / 2; ++i) {
        timer.update(false);
    }
    EXPECT_TRUE(timer.get_output());

    // Send a 'true' input, which should reset the off-delay counter
    timer.update(true);
    EXPECT_TRUE(timer.get_output());

    // Now, it should require the full delay time again to turn off
    for (int i = 0; i < required_samples - 1; ++i) {
        timer.update(false);
        EXPECT_TRUE(timer.get_output()) << "Timer did not reset properly";
    }

    timer.update(false);
    EXPECT_FALSE(timer.get_output()) << "Timer did not turn off after reset and full delay";
}

TEST_F(OffDelayTest, ResetMethodWorksWhenActive) {
    const real_t delay = 0.1f;  // 100 ms delay
    off_delay timer(delay, sampling_time);

    // Turn the timer on
    timer.update(true);
    EXPECT_TRUE(timer.get_output());

    // Now, reset it
    timer.reset();
    EXPECT_FALSE(timer.get_output());
}

TEST_F(OffDelayTest, ZeroDelayTurnsOffImmediately) {
    const real_t delay = 0.0f;
    off_delay timer(delay, sampling_time);

    // Turn it on
    timer.update(true);
    EXPECT_TRUE(timer.get_output());

    // A single false input should make the output false immediately
    timer.update(false);
    EXPECT_FALSE(timer.get_output());
}

}  // namespace