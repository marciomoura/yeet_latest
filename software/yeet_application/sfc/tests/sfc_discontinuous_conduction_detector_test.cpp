#include "sfc/sfc_discontinuous_conduction_detector.hpp"

#include <gtest/gtest.h>

using namespace yeet::application::sfc;
using namespace yeet;

class DiscontinuousConductionDetectorTest : public ::testing::Test {
protected:
    const current_pu_t zero_current_threshold{0.01f};
    const duration_t is_zero_on_delay{0.01f};
    const duration_t discontinuous_on_delay{0.01f};
    const duration_t discontinuous_off_delay{0.5f};

    void SetUp() override
    {
        // Configure with typical values
        // is_zero: 10ms on/off delay for debouncing
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

TEST_F(DiscontinuousConductionDetectorTest, IsCurrentZeroDebouncing)
{
    // Current drops below threshold
    simulate_steps(1, current_pu_t{0.0f});
    EXPECT_TRUE(detector.get_is_current_zero());

    // Current rises above threshold
    simulate_steps(8, current_pu_t{0.1f});
    EXPECT_TRUE(detector.get_is_current_zero());  // Not yet met on-delay
    simulate_steps(2, current_pu_t{0.1f});
    EXPECT_FALSE(detector.get_is_current_zero());  // Met on-delay (10ms)
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

TEST_F(DiscontinuousConductionDetectorTest, JitterAroundThreshold)
{
    // Enter zero current state
    simulate_steps(10, current_pu_t{0.0f});
    EXPECT_TRUE(detector.get_is_current_zero());

    // Jitter: one cycle above threshold
    simulate_steps(1, current_pu_t{0.1f});
    EXPECT_TRUE(detector.get_is_current_zero());  // Output should not change

    // Jitter: a few more cycles
    simulate_steps(5, current_pu_t{0.0f});
    EXPECT_TRUE(detector.get_is_current_zero());  // Output should remain stable
    simulate_steps(2, current_pu_t{0.1f});        // This will immediately reset the off-delay
    simulate_steps(2, current_pu_t{0.0f});
}