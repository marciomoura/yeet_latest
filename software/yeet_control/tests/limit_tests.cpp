#include <gtest/gtest.h>

#include "yeet_control/limit.hpp"

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
TYPED_TEST(LimitTest, UpperLimit) {
    // Test value below limit
    EXPECT_EQ(limit<TypeParam>::upper(TypeParam(5), TypeParam(10)), TypeParam(5));

    // Test value at limit
    EXPECT_EQ(limit<TypeParam>::upper(TypeParam(10), TypeParam(10)), TypeParam(10));

    // Test value above limit
    EXPECT_EQ(limit<TypeParam>::upper(TypeParam(15), TypeParam(10)), TypeParam(10));
}

// Test lower limit
TYPED_TEST(LimitTest, LowerLimit) {
    // Test value above limit
    EXPECT_EQ(limit<TypeParam>::lower(TypeParam(5), TypeParam(0)), TypeParam(5));

    // Test value at limit
    EXPECT_EQ(limit<TypeParam>::lower(TypeParam(0), TypeParam(0)), TypeParam(0));

    // Test value below limit
    EXPECT_EQ(limit<TypeParam>::lower(TypeParam(-5), TypeParam(0)), TypeParam(0));
}

// Test range limit
TYPED_TEST(LimitTest, RangeLimit) {
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

// Test range limit with status for integral types
TEST(LimitTest, RangeWithStatusIntegral) {
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
TEST(LimitTest, RangeWithStatusFloat) {
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

// Test invalid range assertion
TEST(LimitDeathTest, InvalidRange) {
    // This test should be run in debug mode to check for assertion failures
    if constexpr (!__OPTIMIZE__) {
        EXPECT_DEATH(limit<int>::range(5, 10, 0), "Wrong range limits");
        EXPECT_DEATH(limit<double>::range(5.0, 10.0, 0.0), "Wrong range limits");
    }
}

}  // namespace yeet::control::test