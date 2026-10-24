#include "yeet_control/boolean_debouncer.hpp"

#include <gtest/gtest.h>

namespace {

using namespace yeet;
using namespace yeet::control;

class BooleanDebouncerTest : public ::testing::Test {
protected:
    const duration_t sampling_time{0.001f};  // 1ms sampling time
    const duration_t on_delay{0.010f};       // 10ms on delay
    const duration_t off_delay{0.010f};      // 10ms off delay

    void simulate_steps(boolean_debouncer& debouncer, int steps, bool input)
    {
        for (int i = 0; i < steps; ++i) {
            debouncer.update(input);
        }
    }
};

TEST_F(BooleanDebouncerTest, InitialStateIsFalse)
{
    boolean_debouncer debouncer;
    debouncer.configure_sampling_time(sampling_time);
    EXPECT_FALSE(debouncer.get_output());
}

TEST_F(BooleanDebouncerTest, OnDelayWorksCorrectly)
{
    boolean_debouncer debouncer;
    debouncer.configure_sampling_time(sampling_time);
    debouncer.configure_delay(on_delay, off_delay);

    // 9ms of true input - should stay false
    simulate_steps(debouncer, 9, true);
    EXPECT_FALSE(debouncer.get_output());

    // 10th ms of true input - should turn true
    debouncer.update(true);
    EXPECT_TRUE(debouncer.get_output());
}

TEST_F(BooleanDebouncerTest, OffDelayWorksCorrectly)
{
    boolean_debouncer debouncer;
    debouncer.configure_sampling_time(sampling_time);
    const duration_t on_delay{0.010f};
    const duration_t off_delay{0.005f};
    debouncer.configure_delay(on_delay, off_delay);

    // Stabilize at TRUE
    simulate_steps(debouncer, 10, true);
    ASSERT_TRUE(debouncer.get_output());

    // 4ms of false input - should stay true
    simulate_steps(debouncer, 4, false);
    EXPECT_TRUE(debouncer.get_output());

    // 5th ms of false input - should turn false
    debouncer.update(false);
    EXPECT_FALSE(debouncer.get_output());
}

TEST_F(BooleanDebouncerTest, NoiseImmunityTest)
{
    // Test that short spikes don't trigger the output
    boolean_debouncer debouncer;
    debouncer.configure_sampling_time(sampling_time);
    debouncer.configure_delay(duration_t{0.010f}, duration_t{0.010f});

    // 5ms TRUE (spike), then back to FALSE
    simulate_steps(debouncer, 5, true);
    EXPECT_FALSE(debouncer.get_output());
    simulate_steps(debouncer, 1, false);
    EXPECT_FALSE(debouncer.get_output());

    // 5ms FALSE (notch in a TRUE signal)
    simulate_steps(debouncer, 10, true);  // Get it to TRUE first
    ASSERT_TRUE(debouncer.get_output());

    simulate_steps(debouncer, 5, false);  // Notch
    EXPECT_TRUE(debouncer.get_output());
    debouncer.update(true);
    EXPECT_TRUE(debouncer.get_output());
}

TEST_F(BooleanDebouncerTest, CumulativeNoiseImmunity)
{
    // Test that the integrator handles chatty signals better than a reset-based timer
    boolean_debouncer debouncer;
    debouncer.configure_sampling_time(sampling_time);
    debouncer.configure_delay(duration_t{0.010f}, duration_t{0.010f});

    // Pattern of 2 steps TRUE, 1 step FALSE (predominantly TRUE)
    // 2/3 * speed should eventually reach 10ms target
    // In a reset-based debouncer, this would NEVER turn ON.
    for (int i = 0; i < 10; ++i) {
        debouncer.update(true);
        debouncer.update(true);
        debouncer.update(false);
    }

    // After 30 steps (20 true, 10 false), it should eventually hit the limit
    // because net gain is positive.
    // Increment per step = 1ms / 10ms = 0.1
    // Net gain per cycle = 0.1 + 0.1 - 0.1 = 0.1
    // 10 cycles * 0.1 = 1.0 (Saturation reached)
    EXPECT_TRUE(debouncer.get_output());
}

TEST_F(BooleanDebouncerTest, ResetClearsState)
{
    boolean_debouncer debouncer;
    debouncer.configure_sampling_time(sampling_time);
    debouncer.configure_delay(duration_t{0.010f}, duration_t{0.010f});

    simulate_steps(debouncer, 10, true);
    ASSERT_TRUE(debouncer.get_output());

    debouncer.reset();
    EXPECT_FALSE(debouncer.get_output());

    // Should require another 10ms to turn on again
    simulate_steps(debouncer, 9, true);
    EXPECT_FALSE(debouncer.get_output());
}

}  // namespace
