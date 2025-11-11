#include "yeet_control/linear_ramp.hpp"

#include <gtest/gtest.h>

#include "yeet_control/units.hpp"

using namespace yeet;

// Helper function to extract value from both quantity types and raw floats
template <typename T>
inline auto get_value(const T& val) -> decltype(val.value())
{
    return val.value();
}

inline real_t get_value(real_t val) { return val; }

class LinearRampTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        sampling_time = duration_t{0.001};  // 1ms sampling
        ramp = std::make_unique<linear_ramp<real_t>>(sampling_time);
    }

    duration_t sampling_time;
    std::unique_ptr<linear_ramp<real_t>> ramp;
};

// ========== Construction and Configuration Tests ==========

TEST_F(LinearRampTest, ConstructorCreatesValidObject) { EXPECT_NO_THROW(linear_ramp<real_t> r(duration_t{0.001})); }

TEST_F(LinearRampTest, ConfigurationIsApplied)
{
    linear_ramp<real_t>::config cfg{.initial = 0.0f, .final = 10.0f, .duration = duration_t{1.0}};

    ramp->configure(cfg);

    auto retrieved_cfg = ramp->get_config();
    EXPECT_FLOAT_EQ(retrieved_cfg.initial, 0.0f);
    EXPECT_FLOAT_EQ(retrieved_cfg.final, 10.0f);
    EXPECT_FLOAT_EQ(retrieved_cfg.duration.value(), 1.0f);
}

// ========== Basic Ramp Functionality Tests ==========

TEST_F(LinearRampTest, RampStartsAtInitialValue)
{
    linear_ramp<real_t>::config cfg{.initial = 5.0f, .final = 15.0f, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    real_t output = ramp->update(true);
    EXPECT_NEAR(output, 5.0f, 0.01f);
}

TEST_F(LinearRampTest, RampReachesFinalValueAfterDuration)
{
    linear_ramp<real_t>::config cfg{.initial = 0.0f, .final = 10.0f, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    // Run for 1000 cycles (1 second at 1ms sampling)
    for (int i = 0; i < 1000; ++i) {
        ramp->update(true);
    }

    // Should still be ramping (need > 1000 cycles due to timer comparison)
    EXPECT_FALSE(ramp->is_finished());

    // One more cycle should finish it
    real_t output = ramp->update(true);
    EXPECT_NEAR(output, 10.0f, 0.1f);
    EXPECT_TRUE(ramp->is_finished());
}

TEST_F(LinearRampTest, RampInterpolatesLinearly)
{
    linear_ramp<real_t>::config cfg{.initial = 0.0f, .final = 100.0f, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    // At 25% (250ms)
    for (int i = 0; i < 250; ++i) {
        ramp->update(true);
    }
    real_t output_25 = ramp->update(true);
    EXPECT_NEAR(output_25, 25.0f, 1.0f);

    // At 50% (250ms more = 500ms total)
    for (int i = 0; i < 249; ++i) {
        ramp->update(true);
    }
    real_t output_50 = ramp->update(true);
    EXPECT_NEAR(output_50, 50.0f, 1.0f);

    // At 75% (250ms more = 750ms total)
    for (int i = 0; i < 249; ++i) {
        ramp->update(true);
    }
    real_t output_75 = ramp->update(true);
    EXPECT_NEAR(output_75, 75.0f, 1.0f);
}

// ========== Zero Duration Tests ==========

TEST_F(LinearRampTest, ZeroDurationImmediatelyReturnsFinalValue)
{
    linear_ramp<real_t>::config cfg{.initial = 0.0f, .final = 10.0f, .duration = duration_t{0.0}};
    ramp->configure(cfg);

    real_t output = ramp->update(true);
    EXPECT_FLOAT_EQ(output, 10.0f);
    EXPECT_TRUE(ramp->is_finished());
}

TEST_F(LinearRampTest, NegativeDurationImmediatelyReturnsFinalValue)
{
    linear_ramp<real_t>::config cfg{.initial = 5.0f, .final = 20.0f, .duration = duration_t{-0.5}};
    ramp->configure(cfg);

    real_t output = ramp->update(true);
    EXPECT_FLOAT_EQ(output, 20.0f);
    EXPECT_TRUE(ramp->is_finished());
}

// ========== Constant Value Tests (Initial == Final) ==========

TEST_F(LinearRampTest, ConstantValueWaitsForDurationToComplete)
{
    linear_ramp<real_t>::config cfg{.initial = 7.5f, .final = 7.5f, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    // At start (elapsed = 0)
    real_t output = ramp->update(true);
    EXPECT_FLOAT_EQ(output, 7.5f);
    EXPECT_FALSE(ramp->is_finished());  // elapsed = 0.001s < 1.0s

    // At 50% through duration (elapsed = 0.5s)
    for (int i = 0; i < 499; ++i) {
        ramp->update(true);
    }
    output = ramp->update(true);
    EXPECT_FLOAT_EQ(output, 7.5f);
    EXPECT_FALSE(ramp->is_finished());  // elapsed = 0.5s < 1.0s

    // Approach the duration (elapsed = 0.999s)
    for (int i = 0; i < 498; ++i) {
        ramp->update(true);
    }
    output = ramp->update(true);
    EXPECT_FLOAT_EQ(output, 7.5f);
    EXPECT_FALSE(ramp->is_finished());  // elapsed = 0.999s < 1.0s

    // Complete the duration (elapsed = 1.0s)
    output = ramp->update(true);
    EXPECT_FLOAT_EQ(output, 7.5f);
    EXPECT_TRUE(ramp->is_finished());  // elapsed = 1.0s >= 1.0s
}

// ========== Negative Slope Tests ==========

TEST_F(LinearRampTest, NegativeSlopeRampWorks)
{
    linear_ramp<real_t>::config cfg{.initial = 100.0f, .final = 0.0f, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    // At start
    real_t output = ramp->update(true);
    EXPECT_NEAR(output, 100.0f, 0.1f);

    // At 50%
    for (int i = 0; i < 499; ++i) {
        ramp->update(true);
    }
    output = ramp->update(true);
    EXPECT_NEAR(output, 50.0f, 1.0f);

    // At end
    for (int i = 0; i < 500; ++i) {
        ramp->update(true);
    }
    output = ramp->update(true);
    EXPECT_NEAR(output, 0.0f, 0.1f);
    EXPECT_TRUE(ramp->is_finished());
}

// ========== Reset Tests ==========

TEST_F(LinearRampTest, ResetRestartsRamp)
{
    linear_ramp<real_t>::config cfg{.initial = 0.0f, .final = 10.0f, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    // Run halfway
    for (int i = 0; i < 500; ++i) {
        ramp->update(true);
    }
    real_t output_mid = ramp->update(true);
    EXPECT_NEAR(output_mid, 5.0f, 0.5f);

    // Reset
    ramp->reset();

    // Should be back at initial
    real_t output_after_reset = ramp->update(true);
    EXPECT_NEAR(output_after_reset, 0.0f, 0.01f);
    EXPECT_FALSE(ramp->is_finished());
}

// ========== Enable/Disable Tests ==========

TEST_F(LinearRampTest, DisabledRampDoesNotProgress)
{
    linear_ramp<real_t>::config cfg{.initial = 0.0f, .final = 10.0f, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    // Run with enable=false
    for (int i = 0; i < 500; ++i) {
        ramp->update(false);
    }

    real_t output = ramp->update(false);
    EXPECT_NEAR(output, 0.0f, 0.01f);  // Should still be at initial
    EXPECT_FALSE(ramp->is_finished());
}

TEST_F(LinearRampTest, EnabledThenDisabledHoldsValue)
{
    linear_ramp<real_t>::config cfg{.initial = 0.0f, .final = 10.0f, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    // Run halfway with enable=true
    for (int i = 0; i < 500; ++i) {
        ramp->update(true);
    }
    real_t output_mid = ramp->update(true);
    EXPECT_NEAR(output_mid, 5.0f, 0.5f);

    // Continue with enable=false (should not progress)
    for (int i = 0; i < 500; ++i) {
        ramp->update(false);
    }
    real_t output_held = ramp->update(false);
    EXPECT_NEAR(output_held, 5.0f, 0.5f);  // Should hold at ~5.0
}

// ========== Typed Value Tests (frequency_t, torque_pu_t) ==========

TEST_F(LinearRampTest, WorksWithFrequencyType)
{
    linear_ramp<frequency_t> freq_ramp(duration_t{0.001});
    linear_ramp<frequency_t>::config cfg{
        .initial = frequency_t{1.0}, .final = frequency_t{10.0}, .duration = duration_t{1.0}};
    freq_ramp.configure(cfg);

    frequency_t output = freq_ramp.update(true);
    EXPECT_NEAR(get_value(output), 1.0f, 0.01f);

    // Run to completion
    for (int i = 0; i < 1000; ++i) {
        freq_ramp.update(true);
    }
    output = freq_ramp.update(true);
    EXPECT_NEAR(get_value(output), 10.0f, 0.1f);
    EXPECT_TRUE(freq_ramp.is_finished());
}

TEST_F(LinearRampTest, WorksWithTorqueType)
{
    linear_ramp<torque_pu_t> torque_ramp(duration_t{0.001});
    linear_ramp<torque_pu_t>::config cfg{
        .initial = torque_pu_t{0.2}, .final = torque_pu_t{0.8}, .duration = duration_t{0.5}};
    torque_ramp.configure(cfg);

    torque_pu_t output = torque_ramp.update(true);
    EXPECT_NEAR(get_value(output), 0.2f, 0.01f);

    // Run halfway (250ms)
    for (int i = 0; i < 250; ++i) {
        torque_ramp.update(true);
    }
    output = torque_ramp.update(true);
    EXPECT_NEAR(get_value(output), 0.5f, 0.05f);
}

// ========== Elapsed Time Tests ==========

TEST_F(LinearRampTest, GetElapsedTimeReturnsCorrectValue)
{
    linear_ramp<real_t>::config cfg{.initial = real_t{0.0}, .final = real_t{10.0}, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    // At start
    EXPECT_FLOAT_EQ(ramp->get_elapsed_time().value(), 0.0f);

    // After 100 cycles (100ms)
    for (int i = 0; i < 100; ++i) {
        ramp->update(true);
    }
    EXPECT_NEAR(ramp->get_elapsed_time().value(), 0.1f, 0.001f);

    // After 500 cycles (500ms)
    for (int i = 0; i < 400; ++i) {
        ramp->update(true);
    }
    EXPECT_NEAR(ramp->get_elapsed_time().value(), 0.5f, 0.001f);
}

// ========== Edge Case Tests ==========

TEST_F(LinearRampTest, VeryShortDurationWorks)
{
    linear_ramp<real_t>::config cfg{.initial = 0.0f, .final = 10.0f, .duration = duration_t{0.001}};
    ramp->configure(cfg);

    // First cycle: elapsed = 0.001s which equals duration
    ramp->update(true);
    EXPECT_TRUE(ramp->is_finished());  // elapsed = 0.001s >= 0.001s
}

TEST_F(LinearRampTest, OutputDoesNotExceedFinalValue)
{
    linear_ramp<real_t>::config cfg{.initial = 0.0f, .final = 10.0f, .duration = duration_t{1.0}};
    ramp->configure(cfg);

    // Run way past the duration
    for (int i = 0; i < 5000; ++i) {
        real_t output = ramp->update(true);
        EXPECT_LE(output, 10.0f);  // Should never exceed final
    }
}

TEST_F(LinearRampTest, ReconfigurationResetsInternalState)
{
    // First configuration
    linear_ramp<real_t>::config cfg1{.initial = 0.0f, .final = 10.0f, .duration = duration_t{1.0}};
    ramp->configure(cfg1);

    // Run halfway
    for (int i = 0; i < 500; ++i) {
        ramp->update(true);
    }

    // Reconfigure with different parameters
    linear_ramp<real_t>::config cfg2{.initial = 20.0f, .final = 30.0f, .duration = duration_t{0.5}};
    ramp->configure(cfg2);

    // Reset needed after reconfiguration to restart from beginning
    ramp->reset();

    // Should start from new initial value
    real_t output = ramp->update(true);
    EXPECT_NEAR(output, 20.0f, 0.01f);
}
