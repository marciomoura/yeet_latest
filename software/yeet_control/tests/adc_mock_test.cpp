#include "yeet_control/adc_mock.hpp"

#include <gtest/gtest.h>

using namespace yeet;
using namespace yeet::control;
using namespace yeet::control::mock;

class AdcMockTest : public ::testing::Test {
protected:
    static constexpr real_t sampling_period = 0.001;  // 1 kHz
    adc_mock adc{sampling_period};
};

TEST_F(AdcMockTest, InitialStateIsZero) { EXPECT_EQ(adc.get_output(), 0); }

TEST_F(AdcMockTest, ResetReturnsToZero)
{
    adc.configure_gain(2.0);
    adc.configure_filter(100.0);
    adc.configure_quantization(12, 5.0);
    adc.update(1.0);
    ASSERT_NE(adc.get_output(), 0);
    adc.reset();
    EXPECT_EQ(adc.get_output(), 0);
}

TEST_F(AdcMockTest, QuantizationNoGainOrFilter)
{
    const int bits = 12;
    const real_t full_scale_voltage = 5.0;
    adc.configure_gain(1.0);
    adc.configure_filter(1.0 / (2.0 * sampling_period) - 1);  // Set filter near Nyquist to minimize effect
    adc.configure_quantization(bits, full_scale_voltage);

    // Wait for filter to settle
    for (int i = 0; i < 100; ++i) {
        adc.update(2.5);
    }

    const std::int32_t max_count = (1 << (bits - 1)) - 1;
    const std::int32_t expected_output = static_cast<std::int32_t>(std::round((2.5 / full_scale_voltage) * max_count));

    EXPECT_NEAR(adc.get_output(), expected_output, 1);
}

TEST_F(AdcMockTest, SaturationPositive)
{
    const int bits = 16;
    const real_t full_scale_voltage = 3.3;
    adc.configure_gain(1.0);
    adc.configure_filter(499.0);
    adc.configure_quantization(bits, full_scale_voltage);

    // Wait for filter to settle with an input voltage greater than full scale
    for (int i = 0; i < 100; ++i) {
        adc.update(4.0);
    }

    const std::int32_t max_count = (1 << (bits - 1)) - 1;
    EXPECT_EQ(adc.get_output(), max_count);
}

TEST_F(AdcMockTest, SaturationNegative)
{
    const int bits = 16;
    const real_t full_scale_voltage = 3.3;
    adc.configure_gain(1.0);
    adc.configure_filter(499.0);
    adc.configure_quantization(bits, full_scale_voltage);

    // Wait for filter to settle with an input voltage less than negative full scale
    for (int i = 0; i < 100; ++i) {
        adc.update(-4.0);
    }

    const std::int32_t min_count = -(1 << (bits - 1));
    // Note: The minimum value for two's complement is -2^(n-1), not -2^(n-1)+1
    // However, our scaling is symmetric around zero, so the output will be clamped to -max_count.
    const std::int32_t expected_min_count = -((1 << (bits - 1)) - 1);
    EXPECT_EQ(adc.get_output(), expected_min_count);
}

TEST_F(AdcMockTest, GainAndFilterIntegration)
{
    adc.configure_gain(2.0);
    adc.configure_filter(10.0);  // 10 Hz cutoff
    adc.configure_quantization(12, 10.0);

    // Apply a DC input of 3.0V. With gain of 2, it becomes 6.0V.
    // The filter will pass this DC value.
    for (int i = 0; i < 200; ++i) {
        adc.update(3.0);
    }

    const std::int32_t max_count = (1 << 11) - 1;
    const std::int32_t expected_output = static_cast<std::int32_t>(std::round((6.0 / 10.0) * max_count));

    EXPECT_NEAR(adc.get_output(), expected_output, 1);
}