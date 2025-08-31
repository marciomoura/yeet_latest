#include "yeet_control/adc_value_t.hpp"

#include <gtest/gtest.h>


using namespace yeet::control;
using yeet::real_t;

class AdcValueTest : public ::testing::Test {
protected:
    adc_value_t adc_value;
};

TEST_F(AdcValueTest, DefaultConstructorCreatesZeroValue)
{
    EXPECT_EQ(adc_value.get_counts(), 0);
    EXPECT_EQ(adc_value.to_real(), 0.0);
}

TEST_F(AdcValueTest, ConstructorWithParameters)
{
    const int bits = 12;
    const real_t full_scale_value = 5.0;
    const std::int32_t counts = 1000;

    adc_value_t value(bits, full_scale_value, counts);

    EXPECT_EQ(value.get_counts(), counts);

    // Calculate expected real value
    const std::int32_t max_counts = (1 << (bits - 1)) - 1;  // 2047
    const real_t expected_real = (static_cast<real_t>(counts) / max_counts) * full_scale_value;
    EXPECT_NEAR(value.to_real(), expected_real, 1e-9);
}

TEST_F(AdcValueTest, FromRealStaticConstructor)
{
    const int bits = 12;
    const real_t full_scale_value = 5.0;
    const real_t real_value = 2.5;

    auto value = adc_value_t::from_real(bits, full_scale_value, real_value);

    // Should be close to the original real value when converted back
    EXPECT_NEAR(value.to_real(), real_value, 0.01);  // Allow for quantization error
}

TEST_F(AdcValueTest, SetAndGetCounts)
{
    const int bits = 16;
    const real_t full_scale_value = 10.0;
    adc_value_t value(bits, full_scale_value);

    const std::int32_t test_counts = 12345;
    value.set_counts(test_counts);

    EXPECT_EQ(value.get_counts(), test_counts);
}

TEST_F(AdcValueTest, SetRealValue)
{
    const int bits = 12;
    const real_t full_scale_value = 5.0;
    adc_value_t value(bits, full_scale_value);

    const real_t test_real = 2.5;
    value.set_real(test_real);

    // Should be close to the original value when converted back
    EXPECT_NEAR(value.to_real(), test_real, 0.01);
}

TEST_F(AdcValueTest, ImplicitConversionToReal)
{
    const int bits = 12;
    const real_t full_scale_value = 5.0;
    const std::int32_t counts = 1000;

    adc_value_t value(bits, full_scale_value, counts);

    // Test implicit conversion
    real_t converted_value = value;
    EXPECT_EQ(converted_value, value.to_real());
}

TEST_F(AdcValueTest, AssignmentFromCounts)
{
    const int bits = 12;
    const real_t full_scale_value = 5.0;
    adc_value_t value(bits, full_scale_value);

    const std::int32_t test_counts = 500;
    value = test_counts;

    EXPECT_EQ(value.get_counts(), test_counts);
}

TEST_F(AdcValueTest, AssignmentFromReal)
{
    const int bits = 12;
    const real_t full_scale_value = 5.0;
    adc_value_t value(bits, full_scale_value);

    const real_t test_real = 1.25;
    value = test_real;

    EXPECT_NEAR(value.to_real(), test_real, 0.01);
}

TEST_F(AdcValueTest, ConversionFor12Bit)
{
    const int bits = 12;
    const real_t full_scale_value = 5.0;
    const std::int32_t max_counts = (1 << (bits - 1)) - 1;  // 2047

    adc_value_t value(bits, full_scale_value);

    // Test zero
    value.set_counts(0);
    EXPECT_NEAR(value.to_real(), 0.0, 1e-9);

    // Test positive full scale
    value.set_counts(max_counts);
    EXPECT_NEAR(value.to_real(), full_scale_value, 1e-9);

    // Test negative full scale
    value.set_counts(-max_counts);
    EXPECT_NEAR(value.to_real(), -full_scale_value, 1e-9);

    // Test half scale
    value.set_counts(max_counts / 2);
    EXPECT_NEAR(value.to_real(), full_scale_value / 2.0, 0.002);
}

TEST_F(AdcValueTest, ConversionFor16Bit)
{
    const int bits = 16;
    const real_t full_scale_value = 10.0;
    const std::int32_t max_counts = (1 << (bits - 1)) - 1;  // 32767

    adc_value_t value(bits, full_scale_value);

    // Test positive full scale
    value.set_counts(max_counts);
    EXPECT_NEAR(value.to_real(), full_scale_value, 1e-9);

    // Test some intermediate value
    value.set_counts(16384);
    const real_t expected_value = (16384.0 / max_counts) * full_scale_value;
    EXPECT_NEAR(value.to_real(), expected_value, 1e-9);
}

TEST_F(AdcValueTest, ClampingBehavior)
{
    const int bits = 12;
    const real_t full_scale_value = 5.0;
    const std::int32_t max_counts = (1 << (bits - 1)) - 1;  // 2047

    adc_value_t value(bits, full_scale_value);

    // Test that values beyond full scale are clamped
    value.set_real(full_scale_value * 2.0);     // Try to set beyond full scale
    EXPECT_LE(value.get_counts(), max_counts);  // Should be clamped to max

    value.set_real(-full_scale_value * 2.0);         // Try to set beyond negative full scale
    EXPECT_GE(value.get_counts(), -max_counts - 1);  // Should be clamped to min
}