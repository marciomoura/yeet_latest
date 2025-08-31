#include "sfc/sfc_discontinuous_conduction_detector.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
using namespace yeet;

class DiscontinuousConductionDetectorTest : public ::testing::Test {
protected:
    const current_pu_t zero_current_threshold{0.01f};
    const duration_t is_zero_on_delay{0.01f};  // 10ms on-delay
    const duration_t discontinuous_on_delay{0.01f};
    const duration_t discontinuous_off_delay{0.5f};

    void SetUp() override
    {
        // Configure with typical values
        // is_zero: 10ms on-delay (output true after input true for 10ms)
        // discontinuous_mode: 500ms on-delay to confirm exit, 10ms off-delay to detect entry
        detector.configure(zero_current_threshold, is_zero_on_delay, discontinuous_on_delay, discontinuous_off_delay);
    }

    void simulate_steps(int steps, current_pu_t current)
    {
        for (int i = 0; i < steps; ++i) {
            detector.update(current);
        }
    }

    sfc_discontinuous_conduction_detector detector{duration_t{0.001f}};
};

TEST_F(DiscontinuousConductionDetectorTest, InitialStateIsFalse)
{
    EXPECT_FALSE(detector.get_is_current_zero());
    EXPECT_FALSE(detector.get_is_discontinuous_mode());
}

TEST_F(DiscontinuousConductionDetectorTest, ResetReturnsToInitialState)
{
    simulate_steps(20, current_pu_t{0.0f});  // Current goes to zero, timers start
    EXPECT_TRUE(detector.get_is_current_zero());
    EXPECT_TRUE(detector.get_is_discontinuous_mode());

    detector.reset();
    EXPECT_FALSE(detector.get_is_current_zero());
    EXPECT_FALSE(detector.get_is_discontinuous_mode());
}

TEST_F(DiscontinuousConductionDetectorTest, IsCurrentZeroOnDelayBehavior)
{
    // Current drops below threshold - with on_delay, output stays false until delay is met
    simulate_steps(1, current_pu_t{0.0f});
    EXPECT_FALSE(detector.get_is_current_zero()) << "Should remain false until on-delay is met";

    // Continue with current below threshold for 9 more steps (total 10ms)
    simulate_steps(7, current_pu_t{0.0f});
    EXPECT_FALSE(detector.get_is_current_zero()) << "Should still be false - on-delay not yet met";

    // One more step completes the 10ms on-delay
    simulate_steps(1, current_pu_t{0.0f});
    EXPECT_TRUE(detector.get_is_current_zero()) << "Should be true after on-delay is met";

    // Current rises above threshold - with on_delay, output immediately goes false
    simulate_steps(1, current_pu_t{0.1f});
    EXPECT_FALSE(detector.get_is_current_zero()) << "Should immediately go false when current rises";
}

TEST_F(DiscontinuousConductionDetectorTest, IsDiscontinuousModeLongOnDelay)
{
    // Enter discontinuous mode (fast off-delay)
    simulate_steps(10, current_pu_t{0.0f});
    EXPECT_TRUE(detector.get_is_discontinuous_mode());

    // Attempt to exit discontinuous mode
    // Simulate for 499ms with current above threshold
    simulate_steps(498, current_pu_t{0.1f});
    // The long on-delay (500ms) has not been met yet
    EXPECT_TRUE(detector.get_is_discontinuous_mode()) << "Should not exit discontinuous mode before on-delay is met";

    // Simulate one more step to cross the 500ms threshold
    simulate_steps(2, current_pu_t{0.1f});
    EXPECT_FALSE(detector.get_is_discontinuous_mode()) << "Should exit discontinuous mode after on-delay is met";
}

TEST_F(DiscontinuousConductionDetectorTest, JitterAroundThresholdWithOnDelay)
{
    // Start with current below threshold but don't meet the on-delay yet
    simulate_steps(5, current_pu_t{0.0f});
    EXPECT_FALSE(detector.get_is_current_zero()) << "Should be false - on-delay not met";

    // Jitter: current goes above threshold, resets the on-delay timer
    simulate_steps(1, current_pu_t{0.1f});
    EXPECT_FALSE(detector.get_is_current_zero()) << "Should remain false";

    // Current goes back below threshold, timer restarts
    simulate_steps(5, current_pu_t{0.0f});
    EXPECT_FALSE(detector.get_is_current_zero()) << "Should still be false - timer was reset by jitter";

    // More jitter - current above threshold again
    simulate_steps(1, current_pu_t{0.1f});
    EXPECT_FALSE(detector.get_is_current_zero()) << "Should remain false";

    // Finally, current stays below threshold long enough to meet on-delay
    simulate_steps(10, current_pu_t{0.0f});
    EXPECT_TRUE(detector.get_is_current_zero()) << "Should be true after uninterrupted on-delay period";
}

TEST_F(DiscontinuousConductionDetectorTest, OnDelayDebouncesProperly)
{
    // Test that the on-delay properly debounces the input

    // Current below threshold for exactly the on_delay period
    simulate_steps(10, current_pu_t{0.0f});
    EXPECT_TRUE(detector.get_is_current_zero()) << "Should be true after exactly on_delay period";

    // Current above threshold - should immediately go false
    simulate_steps(1, current_pu_t{0.1f});
    EXPECT_FALSE(detector.get_is_current_zero()) << "Should immediately go false when current rises";

    // Current below threshold again - debounce should reset
    simulate_steps(5, current_pu_t{0.0f});
    EXPECT_FALSE(detector.get_is_current_zero()) << "Should be false - on-delay timer reset by current rise";

    // Current stays below threshold for 15ms (1.5 times the on_delay)
    simulate_steps(15, current_pu_t{0.0f});
    EXPECT_TRUE(detector.get_is_current_zero()) << "Should be true after sufficient time below threshold";
}