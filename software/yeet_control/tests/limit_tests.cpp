#include <gtest/gtest.h>

#include "yeet_control/limit.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control::test {

// Test fixture for templated tests
template <typename T>
class LimitTest : public ::testing::Test {
protected:
    limit<T> limiter;
};

using TestedTypes = ::testing::Types<int, float, double>;
TYPED_TEST_SUITE(LimitTest, TestedTypes);

// Test upper limit
TYPED_TEST(LimitTest, UpperLimit)
{
    // Test value below limit
    EXPECT_EQ(limit<TypeParam>::upper(TypeParam(5), TypeParam(10)), TypeParam(5));

    // Test value at limit
    EXPECT_EQ(limit<TypeParam>::upper(TypeParam(10), TypeParam(10)), TypeParam(10));

    // Test value above limit
    EXPECT_EQ(limit<TypeParam>::upper(TypeParam(15), TypeParam(10)), TypeParam(10));
}

// Test lower limit
TYPED_TEST(LimitTest, LowerLimit)
{
    // Test value above limit
    EXPECT_EQ(limit<TypeParam>::lower(TypeParam(5), TypeParam(0)), TypeParam(5));

    // Test value at limit
    EXPECT_EQ(limit<TypeParam>::lower(TypeParam(0), TypeParam(0)), TypeParam(0));

    // Test value below limit
    EXPECT_EQ(limit<TypeParam>::lower(TypeParam(-5), TypeParam(0)), TypeParam(0));
}

// Test range limit
TYPED_TEST(LimitTest, RangeLimit)
{
    // Test value within range
    EXPECT_EQ(limit<TypeParam>::range(TypeParam(5), TypeParam(0), TypeParam(10)), TypeParam(5));

    // Test value at lower bound
    EXPECT_EQ(limit<TypeParam>::range(TypeParam(0), TypeParam(0), TypeParam(10)), TypeParam(0));

    // Test value at upper bound
    EXPECT_EQ(limit<TypeParam>::range(TypeParam(10), TypeParam(0), TypeParam(10)), TypeParam(10));

    // Test value below range
    EXPECT_EQ(limit<TypeParam>::range(TypeParam(-5), TypeParam(0), TypeParam(10)), TypeParam(0));

    // Test value above range
    EXPECT_EQ(limit<TypeParam>::range(TypeParam(15), TypeParam(0), TypeParam(10)), TypeParam(10));
}

// Test upper limit violation detection for integral types
TEST(LimitTest, IsAboveUpperLimitIntegral)
{
    // Test value below limit
    EXPECT_FALSE(limit<int>::is_above_upper_limit(5, 10));

    // Test value at limit
    EXPECT_FALSE(limit<int>::is_above_upper_limit(10, 10));

    // Test value above limit
    EXPECT_TRUE(limit<int>::is_above_upper_limit(15, 10));

    // Test value just above limit
    EXPECT_TRUE(limit<int>::is_above_upper_limit(11, 10));

    // Test negative values
    EXPECT_FALSE(limit<int>::is_above_upper_limit(-5, 0));
    EXPECT_FALSE(limit<int>::is_above_upper_limit(0, 0));
    EXPECT_TRUE(limit<int>::is_above_upper_limit(1, 0));

    // Test negative limit
    EXPECT_FALSE(limit<int>::is_above_upper_limit(-10, -5));
    EXPECT_FALSE(limit<int>::is_above_upper_limit(-5, -5));
    EXPECT_TRUE(limit<int>::is_above_upper_limit(-4, -5));
}

// Test upper limit violation detection for floating point types
TEST(LimitTest, IsAboveUpperLimitFloat)
{
    // Test value below limit
    EXPECT_FALSE(limit<double>::is_above_upper_limit(5.0, 10.0));

    // Test value at limit
    EXPECT_FALSE(limit<double>::is_above_upper_limit(10.0, 10.0));

    // Test value above limit
    EXPECT_TRUE(limit<double>::is_above_upper_limit(15.0, 10.0));

    // Test epsilon comparison - just above epsilon threshold
    EXPECT_FALSE(limit<double>::is_above_upper_limit(10.0 + 1e-11, 10.0));

    // Test epsilon comparison - beyond epsilon threshold
    EXPECT_TRUE(limit<double>::is_above_upper_limit(10.0 + 1e-9, 10.0));

    // Test very small difference (should not trigger due to epsilon)
    EXPECT_FALSE(limit<double>::is_above_upper_limit(10.0 + 1e-12, 10.0));

    // Test negative values
    EXPECT_FALSE(limit<double>::is_above_upper_limit(-5.0, 0.0));
    EXPECT_FALSE(limit<double>::is_above_upper_limit(0.0, 0.0));
    EXPECT_TRUE(limit<double>::is_above_upper_limit(0.5, 0.0));

    // Test negative limit
    EXPECT_FALSE(limit<double>::is_above_upper_limit(-10.0, -5.0));
    EXPECT_FALSE(limit<double>::is_above_upper_limit(-5.0, -5.0));
    EXPECT_TRUE(limit<double>::is_above_upper_limit(-4.0, -5.0));

    // Test with float type
    EXPECT_FALSE(limit<float>::is_above_upper_limit(5.0f, 10.0f));
    EXPECT_TRUE(limit<float>::is_above_upper_limit(15.0f, 10.0f));
    EXPECT_FALSE(limit<float>::is_above_upper_limit(10.0f + 1e-11f, 10.0f));
}

// Test range limit with status for integral types
TEST(LimitTest, RangeWithStatusIntegral)
{
    // Test value within range
    auto result1 = limit<int>::range_with_status(5, 0, 10);
    EXPECT_EQ(result1.value, 5);
    EXPECT_FALSE(result1.was_limited);

    // Test value below range
    auto result2 = limit<int>::range_with_status(-5, 0, 10);
    EXPECT_EQ(result2.value, 0);
    EXPECT_TRUE(result2.was_limited);

    // Test value above range
    auto result3 = limit<int>::range_with_status(15, 0, 10);
    EXPECT_EQ(result3.value, 10);
    EXPECT_TRUE(result3.was_limited);
}

// Test range limit with status for floating point types
TEST(LimitTest, RangeWithStatusFloat)
{
    // Test value within range
    auto result1 = limit<double>::range_with_status(5.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result1.value, 5.0);
    EXPECT_FALSE(result1.was_limited);

    // Test value below range
    auto result2 = limit<double>::range_with_status(-5.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result2.value, 0.0);
    EXPECT_TRUE(result2.was_limited);

    // Test value above range
    auto result3 = limit<double>::range_with_status(15.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result3.value, 10.0);
    EXPECT_TRUE(result3.was_limited);

    // Test epsilon comparison
    auto result4 = limit<double>::range_with_status(10.0 + 1e-11, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result4.value, 10.0);
    EXPECT_FALSE(result4.was_limited);  // Should not be considered limited due to epsilon

    auto result5 = limit<double>::range_with_status(10.0 + 1e-9, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result5.value, 10.0);
    EXPECT_TRUE(result5.was_limited);  // Should be considered limited (difference > epsilon)
}

// Test range limit with upper limit status for integral types
TEST(LimitTest, RangeWithUpperLimitStatusIntegral)
{
    // Test value within range - should not report as limited
    auto result1 = limit<int>::range_with_upper_limit_status(5, 0, 10);
    EXPECT_EQ(result1.value, 5);
    EXPECT_FALSE(result1.was_limited);

    // Test value below range - should NOT report as limited (lower limit hit)
    auto result2 = limit<int>::range_with_upper_limit_status(-5, 0, 10);
    EXPECT_EQ(result2.value, 0);
    EXPECT_FALSE(result2.was_limited);

    // Test value above range - SHOULD report as limited (upper limit hit)
    auto result3 = limit<int>::range_with_upper_limit_status(15, 0, 10);
    EXPECT_EQ(result3.value, 10);
    EXPECT_TRUE(result3.was_limited);

    // Test value at upper bound - should not report as limited
    auto result4 = limit<int>::range_with_upper_limit_status(10, 0, 10);
    EXPECT_EQ(result4.value, 10);
    EXPECT_FALSE(result4.was_limited);

    // Test value at lower bound - should not report as limited
    auto result5 = limit<int>::range_with_upper_limit_status(0, 0, 10);
    EXPECT_EQ(result5.value, 0);
    EXPECT_FALSE(result5.was_limited);
}

// Test range limit with upper limit status for floating point types
TEST(LimitTest, RangeWithUpperLimitStatusFloat)
{
    // Test value within range - should not report as limited
    auto result1 = limit<double>::range_with_upper_limit_status(5.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result1.value, 5.0);
    EXPECT_FALSE(result1.was_limited);

    // Test value below range - should NOT report as limited (lower limit hit)
    auto result2 = limit<double>::range_with_upper_limit_status(-5.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result2.value, 0.0);
    EXPECT_FALSE(result2.was_limited);

    // Test value above range - SHOULD report as limited (upper limit hit)
    auto result3 = limit<double>::range_with_upper_limit_status(15.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result3.value, 10.0);
    EXPECT_TRUE(result3.was_limited);

    // Test epsilon comparison - just above epsilon threshold
    auto result4 = limit<double>::range_with_upper_limit_status(10.0 + 1e-11, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result4.value, 10.0);
    EXPECT_FALSE(result4.was_limited);  // Should not be considered limited due to epsilon

    // Test epsilon comparison - beyond epsilon threshold
    auto result5 = limit<double>::range_with_upper_limit_status(10.0 + 1e-9, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result5.value, 10.0);
    EXPECT_TRUE(result5.was_limited);  // Should be considered limited (difference > epsilon)

    // Test value at upper bound - should not report as limited
    auto result6 = limit<double>::range_with_upper_limit_status(10.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result6.value, 10.0);
    EXPECT_FALSE(result6.was_limited);

    // Test value at lower bound - should not report as limited
    auto result7 = limit<double>::range_with_upper_limit_status(0.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result7.value, 0.0);
    EXPECT_FALSE(result7.was_limited);

    // Test negative range values
    auto result8 = limit<double>::range_with_upper_limit_status(-2.5, -10.0, -1.0);
    EXPECT_DOUBLE_EQ(result8.value, -2.5);
    EXPECT_FALSE(result8.was_limited);

    // Test negative range - exceeding upper limit
    auto result9 = limit<double>::range_with_upper_limit_status(0.5, -10.0, -1.0);
    EXPECT_DOUBLE_EQ(result9.value, -1.0);
    EXPECT_TRUE(result9.was_limited);

    // Test negative range - exceeding lower limit (should NOT report as limited)
    auto result10 = limit<double>::range_with_upper_limit_status(-15.0, -10.0, -1.0);
    EXPECT_DOUBLE_EQ(result10.value, -10.0);
    EXPECT_FALSE(result10.was_limited);
}

// Test invalid range assertion
TEST(LimitDeathTest, InvalidRange)
{
    // Test death cases - these should trigger assertions in debug builds
#ifndef NDEBUG
    EXPECT_DEATH(limit<int>::range(5, 10, 0), "Wrong range limits");
    EXPECT_DEATH(limit<double>::range(5.0, 10.0, 0.0), "Wrong range limits");
#else
    // In release builds, just verify the function doesn't crash
    // These should not crash in release mode (assertions disabled)
    // but the behavior is undefined - just test that calls complete
    EXPECT_NO_THROW(limit<int>::range(5, 10, 0));
    EXPECT_NO_THROW(limit<double>::range(5.0, 10.0, 0.0));
#endif
}

// Test upper limit with quantity types
TEST(LimitTest, UpperLimitQuantity)
{
    using yeet::current_pu_t;

    // Test value below limit
    auto result1 = limit<current_pu_t>::upper(current_pu_t(0.5), current_pu_t(1.0));
    EXPECT_DOUBLE_EQ(result1.value(), 0.5);

    // Test value at limit
    auto result2 = limit<current_pu_t>::upper(current_pu_t(1.0), current_pu_t(1.0));
    EXPECT_DOUBLE_EQ(result2.value(), 1.0);

    // Test value above limit
    auto result3 = limit<current_pu_t>::upper(current_pu_t(1.5), current_pu_t(1.0));
    EXPECT_DOUBLE_EQ(result3.value(), 1.0);
}

// Test lower limit with quantity types
TEST(LimitTest, LowerLimitQuantity)
{
    using yeet::voltage_pu_t;

    // Test value above limit
    auto result1 = limit<voltage_pu_t>::lower(voltage_pu_t(0.5), voltage_pu_t(0.0));
    EXPECT_DOUBLE_EQ(result1.value(), 0.5);

    // Test value at limit
    auto result2 = limit<voltage_pu_t>::lower(voltage_pu_t(0.0), voltage_pu_t(0.0));
    EXPECT_DOUBLE_EQ(result2.value(), 0.0);

    // Test value below limit
    auto result3 = limit<voltage_pu_t>::lower(voltage_pu_t(-0.5), voltage_pu_t(0.0));
    EXPECT_DOUBLE_EQ(result3.value(), 0.0);
}

// Test range limit with quantity types
TEST(LimitTest, RangeLimitQuantity)
{
    using yeet::current_pu_t;

    // Test value within range
    auto result1 = limit<current_pu_t>::range(current_pu_t(0.5), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result1.value(), 0.5);

    // Test value at lower bound
    auto result2 = limit<current_pu_t>::range(current_pu_t(-1.5), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result2.value(), -1.5);

    // Test value at upper bound
    auto result3 = limit<current_pu_t>::range(current_pu_t(1.5), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result3.value(), 1.5);

    // Test value below range
    auto result4 = limit<current_pu_t>::range(current_pu_t(-2.0), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result4.value(), -1.5);

    // Test value above range
    auto result5 = limit<current_pu_t>::range(current_pu_t(2.0), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result5.value(), 1.5);
}

// Test is_above_upper_limit with quantity types
TEST(LimitTest, IsAboveUpperLimitQuantity)
{
    using yeet::current_pu_t;

    // Test value below limit
    EXPECT_FALSE(limit<current_pu_t>::is_above_upper_limit(current_pu_t(0.5), current_pu_t(1.0)));

    // Test value at limit
    EXPECT_FALSE(limit<current_pu_t>::is_above_upper_limit(current_pu_t(1.0), current_pu_t(1.0)));

    // Test value above limit
    EXPECT_TRUE(limit<current_pu_t>::is_above_upper_limit(current_pu_t(1.5), current_pu_t(1.0)));

    // Test epsilon comparison - just above epsilon threshold
    EXPECT_FALSE(limit<current_pu_t>::is_above_upper_limit(current_pu_t(1.0 + 1e-11), current_pu_t(1.0)));

    // Test epsilon comparison - beyond epsilon threshold
    EXPECT_TRUE(limit<current_pu_t>::is_above_upper_limit(current_pu_t(1.0 + 1e-6), current_pu_t(1.0)));

    // Test negative values
    EXPECT_FALSE(limit<current_pu_t>::is_above_upper_limit(current_pu_t(-1.5), current_pu_t(-1.0)));
    EXPECT_FALSE(limit<current_pu_t>::is_above_upper_limit(current_pu_t(-1.0), current_pu_t(-1.0)));
    EXPECT_TRUE(limit<current_pu_t>::is_above_upper_limit(current_pu_t(-0.5), current_pu_t(-1.0)));
}

// Test range_with_status with quantity types
TEST(LimitTest, RangeWithStatusQuantity)
{
    using yeet::voltage_pu_t;

    // Test value within range
    auto result1 = limit<voltage_pu_t>::range_with_status(voltage_pu_t(0.5), voltage_pu_t(0.0), voltage_pu_t(1.0));
    EXPECT_DOUBLE_EQ(result1.value.value(), 0.5);
    EXPECT_FALSE(result1.was_limited);

    // Test value below range
    auto result2 = limit<voltage_pu_t>::range_with_status(voltage_pu_t(-0.5), voltage_pu_t(0.0), voltage_pu_t(1.0));
    EXPECT_DOUBLE_EQ(result2.value.value(), 0.0);
    EXPECT_TRUE(result2.was_limited);

    // Test value above range
    auto result3 = limit<voltage_pu_t>::range_with_status(voltage_pu_t(1.5), voltage_pu_t(0.0), voltage_pu_t(1.0));
    EXPECT_DOUBLE_EQ(result3.value.value(), 1.0);
    EXPECT_TRUE(result3.was_limited);

    // Test epsilon comparison - just above epsilon threshold
    auto result4 =
        limit<voltage_pu_t>::range_with_status(voltage_pu_t(1.0 + 1e-11), voltage_pu_t(0.0), voltage_pu_t(1.0));
    EXPECT_DOUBLE_EQ(result4.value.value(), 1.0);
    EXPECT_FALSE(result4.was_limited);  // Should not be considered limited due to epsilon

    // Test epsilon comparison - beyond epsilon threshold
    auto result5 =
        limit<voltage_pu_t>::range_with_status(voltage_pu_t(1.0 + 1e-6), voltage_pu_t(0.0), voltage_pu_t(1.0));
    EXPECT_DOUBLE_EQ(result5.value.value(), 1.0);
    EXPECT_TRUE(result5.was_limited);  // Should be considered limited (difference > epsilon)
}

// Test range_with_upper_limit_status with quantity types
TEST(LimitTest, RangeWithUpperLimitStatusQuantity)
{
    using yeet::current_pu_t;

    // Test value within range - should not report as limited
    auto result1 =
        limit<current_pu_t>::range_with_upper_limit_status(current_pu_t(0.5), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result1.value.value(), 0.5);
    EXPECT_FALSE(result1.was_limited);

    // Test value below range - should NOT report as limited (lower limit hit)
    auto result2 =
        limit<current_pu_t>::range_with_upper_limit_status(current_pu_t(-2.0), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result2.value.value(), -1.5);
    EXPECT_FALSE(result2.was_limited);

    // Test value above range - SHOULD report as limited (upper limit hit)
    auto result3 =
        limit<current_pu_t>::range_with_upper_limit_status(current_pu_t(2.0), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result3.value.value(), 1.5);
    EXPECT_TRUE(result3.was_limited);

    // Test epsilon comparison - just above epsilon threshold
    auto result4 = limit<current_pu_t>::range_with_upper_limit_status(
        current_pu_t(1.5 + 1e-11), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result4.value.value(), 1.5);
    EXPECT_FALSE(result4.was_limited);  // Should not be considered limited due to epsilon

    // Test epsilon comparison - beyond epsilon threshold
    auto result5 = limit<current_pu_t>::range_with_upper_limit_status(
        current_pu_t(1.5 + 1e-6), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result5.value.value(), 1.5);
    EXPECT_TRUE(result5.was_limited);  // Should be considered limited (difference > epsilon)

    // Test value at upper bound - should not report as limited
    auto result6 =
        limit<current_pu_t>::range_with_upper_limit_status(current_pu_t(1.5), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result6.value.value(), 1.5);
    EXPECT_FALSE(result6.was_limited);

    // Test value at lower bound - should not report as limited
    auto result7 =
        limit<current_pu_t>::range_with_upper_limit_status(current_pu_t(-1.5), current_pu_t(-1.5), current_pu_t(1.5));
    EXPECT_DOUBLE_EQ(result7.value.value(), -1.5);
    EXPECT_FALSE(result7.was_limited);
}

// Test with different quantity types to ensure template works universally
TEST(LimitTest, MultipleQuantityTypes)
{
    using yeet::duration_t;
    using yeet::frequency_t;
    using yeet::torque_pu_t;

    // Test with frequency_t
    auto freq_result = limit<frequency_t>::is_above_upper_limit(frequency_t(60.0), frequency_t(50.0));
    EXPECT_TRUE(freq_result);

    // Test with duration_t
    auto dur_result = limit<duration_t>::range(duration_t(0.5), duration_t(0.0), duration_t(1.0));
    EXPECT_DOUBLE_EQ(dur_result.value(), 0.5);

    // Test with torque_pu_t
    auto torque_result =
        limit<torque_pu_t>::range_with_upper_limit_status(torque_pu_t(1.2), torque_pu_t(0.0), torque_pu_t(1.0));
    EXPECT_DOUBLE_EQ(torque_result.value.value(), 1.0);
    EXPECT_TRUE(torque_result.was_limited);
}

}  // namespace yeet::control::test