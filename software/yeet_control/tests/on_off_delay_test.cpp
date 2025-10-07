#include "yeet_control/on_off_delay.hpp"

#include <gtest/gtest.h>

namespace {

using namespace yeet;
using namespace yeet::control;

// Test fixture for common setup
class OnOffDelayTest : public ::testing::Test {
protected:
    const double sampling_time = 0.01f;  // 10 ms sampling time

    void simulate_steps(on_off_delay& timer, int steps, bool input)
    {
        for (int i = 0; i < steps; ++i) {
            timer.update(input);
        }
    }
};

TEST_F(OnOffDelayTest, InitialStateIsFalse)
{
    on_off_delay timer(sampling_time);

    // The output should be false immediately after configuration.
    EXPECT_FALSE(timer.get_output());
}

TEST_F(OnOffDelayTest, OnDelayTurnsOnCorrectly)
{
    const real_t on_delay = 0.1f;                                       // 100 ms
    const real_t off_delay = 0.05f;                                     // 50 ms
    const int on_samples = static_cast<int>(on_delay / sampling_time);  // 10 samples
    on_off_delay timer(sampling_time);

    timer.configure(on_delay, off_delay);

    // Update n-1 times, output should remain false (input hasn't been true long enough)
    simulate_steps(timer, on_samples - 1, true);
    EXPECT_FALSE(timer.get_output()) << "Timer turned on prematurely.";

    // The final update that meets the on-delay requirement
    timer.update(true);
    EXPECT_TRUE(timer.get_output()) << "Timer did not turn on after input was true for on-delay duration.";

    // It should stay on while input remains true
    timer.update(true);
    EXPECT_TRUE(timer.get_output()) << "Timer turned off after being active.";
}

TEST_F(OnOffDelayTest, OffDelayTurnsOffCorrectly)
{
    const real_t on_delay = 0.02f;                                        // 20 ms
    const real_t off_delay = 0.1f;                                        // 100 ms
    const int on_samples = static_cast<int>(on_delay / sampling_time);    // 2 samples
    const int off_samples = static_cast<int>(off_delay / sampling_time);  // 10 samples
    on_off_delay timer(sampling_time);

    timer.configure(on_delay, off_delay);

    // First, turn the timer on (input true for longer than on-delay)
    simulate_steps(timer, on_samples + 1, true);  // 3 samples > 2 samples needed for on_delay
    EXPECT_TRUE(timer.get_output());

    // Update n-1 times with false input, output should remain true (input hasn't been false long enough)
    simulate_steps(timer, off_samples - 1, false);
    EXPECT_TRUE(timer.get_output()) << "Timer turned off prematurely.";

    // The final update that meets the off-delay requirement
    timer.update(false);
    EXPECT_FALSE(timer.get_output()) << "Timer did not turn off after input was false for off-delay duration.";
}

TEST_F(OnOffDelayTest, OnDelayCounterResetsIfInputGoesFalse)
{
    const real_t on_delay = 0.1f;                                       // 100 ms
    const real_t off_delay = 0.1f;                                      // 100 ms
    const int on_samples = static_cast<int>(on_delay / sampling_time);  // 10 samples
    on_off_delay timer(sampling_time);

    timer.configure(on_delay, off_delay);

    // Update for some time, but not enough to trigger on-delay
    simulate_steps(timer, on_samples / 2, true);
    EXPECT_FALSE(timer.get_output());

    // A single 'false' input should reset the on-delay counter
    timer.update(false);
    EXPECT_FALSE(timer.get_output());

    // Now, it should require the full on-delay time again
    simulate_steps(timer, on_samples - 1, true);
    EXPECT_FALSE(timer.get_output()) << "Timer did not reset its on-delay counter properly.";

    timer.update(true);
    EXPECT_TRUE(timer.get_output()) << "Timer did not turn on after reset and full delay.";
}

TEST_F(OnOffDelayTest, OffDelayCounterResetsIfInputGoesTrue)
{
    const real_t off_delay = 0.1f;                                        // 100 ms
    const int off_samples = static_cast<int>(off_delay / sampling_time);  // 10 samples
    on_off_delay timer(sampling_time);

    timer.configure(0.0f, off_delay);

    // Turn the timer on (input true for longer than on-delay)
    simulate_steps(timer, 3, true);  // 3 samples > 2 samples needed for on_delay
    EXPECT_TRUE(timer.get_output());

    // Start the off-delay sequence, but interrupt it (input false but not long enough)
    simulate_steps(timer, off_samples / 2, false);
    EXPECT_TRUE(timer.get_output());

    // A single 'true' input should reset the off-delay counter
    timer.update(true);
    EXPECT_TRUE(timer.get_output());

    // Now, it should require the full off-delay time again
    simulate_steps(timer, off_samples - 1, false);
    EXPECT_TRUE(timer.get_output()) << "Timer did not reset its off-delay counter properly.";

    timer.update(false);
    EXPECT_FALSE(timer.get_output()) << "Timer did not turn off after reset and full delay.";
}

TEST_F(OnOffDelayTest, ResetMethodWorks)
{
    on_off_delay timer(sampling_time);

    timer.configure(0.02f, 0.1f);

    // Turn the timer on (input true for longer than on-delay)
    simulate_steps(timer, 3, true);  // 3 samples > 2 samples needed for on_delay
    EXPECT_TRUE(timer.get_output());

    // Reset it
    timer.reset();
    EXPECT_FALSE(timer.get_output());
}

TEST_F(OnOffDelayTest, ZeroDelaysFollowsInput)
{
    on_off_delay timer(sampling_time);

    timer.configure(0.0f, 0.0f);

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