#include "yeet_control/hardware_filter_compensator.hpp"

#include <gtest/gtest.h>

using namespace yeet;
using namespace yeet::control;

TEST(HardwareFilterCompensatorTest, IdealFirstOrderLPF)
{
    hardware_filter_compensator compensator;
    const frequency_t fc{50.0f};
    const frequency_t f_max{100.0f};

    compensator.configure_as_first_order_lpf(fc, f_max);

    // At DC (0 Hz), gain should be 1.0 and phase 0.0
    abc<real_t> input{1.0f, -0.5f, -0.5f};
    abc<real_t> output = compensator.update(input, frequency_t{0.0f});

    EXPECT_NEAR(output.a(), input.a(), 1e-6);
    EXPECT_NEAR(output.b(), input.b(), 1e-6);
    EXPECT_NEAR(output.c(), input.c(), 1e-6);

    // At cutoff frequency (50 Hz), gain should be sqrt(2) and phase 45 degrees (pi/4)
    // Inverse LPF is (1 + j f/fc). At f=fc, it's 1 + j1.
    // Magnitude: sqrt(1^2 + 1^2) = sqrt(2)
    // Phase: atan(1/1) = 45 degrees
    output = compensator.update(input, fc);

    const real_t expected_gain = std::sqrt(2.0f);
    const real_t expected_phase = std::atan(1.0f);

    alphabeta<real_t> ab_input = input.to_alphabeta();
    alphabeta<real_t> ab_expected{
        expected_gain * (ab_input.alpha() * std::cos(expected_phase) - ab_input.beta() * std::sin(expected_phase)),
        expected_gain * (ab_input.alpha() * std::sin(expected_phase) + ab_input.beta() * std::cos(expected_phase))};

    abc<real_t> expected_abc = ab_expected.to_abc();

    EXPECT_NEAR(output.a(), expected_abc.a(), 1e-6);
    EXPECT_NEAR(output.b(), expected_abc.b(), 1e-6);
    EXPECT_NEAR(output.c(), expected_abc.c(), 1e-6);
}

TEST(HardwareFilterCompensatorTest, ResetBypasses)
{
    hardware_filter_compensator compensator;
    compensator.configure_as_first_order_lpf(frequency_t{50.0f}, frequency_t{100.0f});
    compensator.reset();

    abc<real_t> input{1.0f, 0.0f, -1.0f};
    abc<real_t> output = compensator.update(input, frequency_t{50.0f});

    EXPECT_NEAR(output.a(), input.a(), 1e-6);
    EXPECT_NEAR(output.b(), input.b(), 1e-6);
    EXPECT_NEAR(output.c(), input.c(), 1e-6);
}
