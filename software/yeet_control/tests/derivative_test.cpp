#include "yeet_control/derivative.hpp"

#include <gtest/gtest.h>

#include "gtest/gtest.h"

using namespace yeet::control;

TEST(DerivativeTest, InitialStateIsZero) {
    derivative deriv(0.01F);
    EXPECT_FLOAT_EQ(deriv.get_output(), 0.0F);
}

TEST(DerivativeTest, CalculatesCorrectDerivative) {
    constexpr yeet::real_t sampling_time = 0.1F;
    derivative deriv(sampling_time);

    // First update, previous value is 0
    deriv.update(1.0F);
    EXPECT_FLOAT_EQ(deriv.get_output(), (1.0F - 0.0F) / sampling_time);  // 10.0

    // Second update
    deriv.update(1.5F);
    EXPECT_FLOAT_EQ(deriv.get_output(), (1.5F - 1.0F) / sampling_time);  // 5.0

    // Third update with negative change
    deriv.update(1.2F);
    EXPECT_NEAR(deriv.get_output(), (1.2F - 1.5F) / sampling_time, 1e-6);  // -3.0
}

TEST(DerivativeTest, ResetWorksCorrectly) {
    derivative deriv(0.1F);
    deriv.update(5.0F);
    ASSERT_NE(deriv.get_output(), 0.0F);

    deriv.reset(10.0F);
    EXPECT_FLOAT_EQ(deriv.get_output(), 0.0F);

    // Next update uses the reset value as previous input
    deriv.update(12.0F);
    EXPECT_FLOAT_EQ(deriv.get_output(), (12.0F - 10.0F) / 0.1F);  // 20.0
}

TEST(DerivativeTest, ConfigureSamplingTime) {
    derivative deriv(0.1F);
    deriv.update(1.0F);
    EXPECT_FLOAT_EQ(deriv.get_output(), 10.0F);

    deriv.configure_sampling_time(0.05F);
    deriv.update(1.5F);                                          // Previous value was 1.0
    EXPECT_FLOAT_EQ(deriv.get_output(), (1.5F - 1.0F) / 0.05F);  // 10.0
}

TEST(DerivativeTest, InitialValueWorksCorrectly) {
    derivative deriv(0.1F, 5.0F);  // Initial value of 5.0
    deriv.update(6.0F);
    EXPECT_FLOAT_EQ(deriv.get_output(), (6.0F - 5.0F) / 0.1F);  // 10.0
}
