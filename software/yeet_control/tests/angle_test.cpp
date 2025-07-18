#include "yeet_control/angle.hpp"

#include <gtest/gtest.h>

namespace {

using namespace yeet;

class AngleMinusPiToPiTest : public ::testing::Test {
protected:
    static constexpr double kEpsilon = 1e-10;
};

class AngleZeroToTwoPiTest : public ::testing::Test {
protected:
    static constexpr double kEpsilon = 1e-10;
};

TEST_F(AngleMinusPiToPiTest, DefaultConstructor) {
    angle<minus_pi_to_pi> a;
    EXPECT_NEAR(a.get_radians(), 0.0, kEpsilon);
    EXPECT_NEAR(a.get_degrees(), 0.0, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, Constructor) {
    angle<minus_pi_to_pi> a(1.0);
    EXPECT_NEAR(a.get_radians(), 1.0, kEpsilon);
    EXPECT_NEAR(a.get_degrees(), 180.0 / pi, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, NegationOperator) {
    angle<minus_pi_to_pi> a;
    a.set_value(1.0);
    angle<minus_pi_to_pi> a_negated = -a;
    EXPECT_NEAR(a, 1.0, kEpsilon);
    EXPECT_NEAR(a_negated, -1.0, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, ConversionalOperator) {
    angle<minus_pi_to_pi> a;
    a.set_value(1.0);
    EXPECT_NEAR(a, 1.0, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, SetValueWrapPositive) {
    angle<minus_pi_to_pi> a;
    a.set_value(4.0);  // > pi
    EXPECT_NEAR(a.get_radians(), 4.0 - 2 * pi, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, SetValueWrapNegative) {
    angle<minus_pi_to_pi> a;
    a.set_value(-4.0);  // < -pi
    EXPECT_NEAR(a.get_radians(), -4.0 + 2 * pi, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, Addition) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(2.0);
    auto c = a + b;
    EXPECT_NEAR(c.get_radians(), 3.0, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, AdditionWithWrapping) {
    angle<minus_pi_to_pi> a(2.0);
    angle<minus_pi_to_pi> b(2.0);
    auto c = a + b;  // Should wrap from 4.0 to 4.0 - 2pi
    EXPECT_NEAR(c.get_radians(), 4.0 - 2 * pi, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, Subtraction) {
    angle<minus_pi_to_pi> a(2.0);
    angle<minus_pi_to_pi> b(1.0);
    auto c = a - b;
    EXPECT_NEAR(c.get_radians(), 1.0, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, SubtractionWithWrapping) {
    angle<minus_pi_to_pi> a(-2.0);
    angle<minus_pi_to_pi> b(2.0);
    auto c = a - b;  // Should wrap from -4.0 to -4.0 + 2pi
    EXPECT_NEAR(c.get_radians(), -4.0 + 2 * pi, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, AdditionAssignment) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(2.0);
    a += b;
    EXPECT_NEAR(a.get_radians(), 3.0, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, SubtractionAssignment) {
    angle<minus_pi_to_pi> a(2.0);
    angle<minus_pi_to_pi> b(1.0);
    a -= b;
    EXPECT_NEAR(a.get_radians(), 1.0, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, ScalarMultiplication) {
    angle<minus_pi_to_pi> a(1.0);
    auto b = a * 2.0;
    EXPECT_NEAR(b.get_radians(), 2.0, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, Equality) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(1.0);
    EXPECT_EQ(a, b);
}

TEST_F(AngleMinusPiToPiTest, EqualityWithWrapping) {
    angle<minus_pi_to_pi> a(pi);
    angle<minus_pi_to_pi> b(-pi);
    EXPECT_EQ(a, b);
}

TEST_F(AngleMinusPiToPiTest, Inequality) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(2.0);
    EXPECT_NE(a, b);
}

// Comparison operator tests for minus_pi_to_pi

TEST_F(AngleMinusPiToPiTest, LessThanBasic) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(2.0);
    EXPECT_TRUE(a < b);
    EXPECT_FALSE(b < a);
}

TEST_F(AngleMinusPiToPiTest, LessThanEqual) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(1.0);
    EXPECT_FALSE(a < b);
    EXPECT_FALSE(b < a);
}

TEST_F(AngleMinusPiToPiTest, LessThanWithWrapping) {
    angle<minus_pi_to_pi> a(pi - 0.1);   // Near positive boundary
    angle<minus_pi_to_pi> b(-pi + 0.1);  // Near negative boundary (wraps to same as pi - 0.1)
    // After wrapping both to [0, 2π]: a ≈ π-0.1+π ≈ 2π-0.1, b ≈ π+0.1
    EXPECT_TRUE(b < a);
}

TEST_F(AngleMinusPiToPiTest, LessThanOrEqualBasic) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(2.0);
    EXPECT_TRUE(a <= b);
    EXPECT_FALSE(b <= a);
}

TEST_F(AngleMinusPiToPiTest, LessThanOrEqualEqual) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(1.0);
    EXPECT_TRUE(a <= b);
    EXPECT_TRUE(b <= a);
}

TEST_F(AngleMinusPiToPiTest, LessThanOrEqualWithWrapping) {
    angle<minus_pi_to_pi> a(pi);   // Should wrap to -π (equivalent)
    angle<minus_pi_to_pi> b(-pi);  // At negative boundary
    EXPECT_TRUE(a <= b);
    EXPECT_TRUE(b <= a);
}

TEST_F(AngleMinusPiToPiTest, GreaterThanBasic) {
    angle<minus_pi_to_pi> a(2.0);
    angle<minus_pi_to_pi> b(1.0);
    EXPECT_TRUE(a > b);
    EXPECT_FALSE(b > a);
}

TEST_F(AngleMinusPiToPiTest, GreaterThanEqual) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(1.0);
    EXPECT_FALSE(a > b);
    EXPECT_FALSE(b > a);
}

TEST_F(AngleMinusPiToPiTest, GreaterThanWithWrapping) {
    angle<minus_pi_to_pi> a(-pi + 0.1);  // Near negative boundary
    angle<minus_pi_to_pi> b(pi - 0.1);   // Near positive boundary
    // After wrapping both to [0, 2π]: a ≈ π+0.1, b ≈ 2π-0.1
    EXPECT_TRUE(b > a);
}

TEST_F(AngleMinusPiToPiTest, GreaterThanOrEqualBasic) {
    angle<minus_pi_to_pi> a(2.0);
    angle<minus_pi_to_pi> b(1.0);
    EXPECT_TRUE(a >= b);
    EXPECT_FALSE(b >= a);
}

TEST_F(AngleMinusPiToPiTest, GreaterThanOrEqualEqual) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(1.0);
    EXPECT_TRUE(a >= b);
    EXPECT_TRUE(b >= a);
}

TEST_F(AngleMinusPiToPiTest, GreaterThanOrEqualWithWrapping) {
    angle<minus_pi_to_pi> a(pi);   // Should wrap to -π (equivalent)
    angle<minus_pi_to_pi> b(-pi);  // At negative boundary
    EXPECT_TRUE(a >= b);
    EXPECT_TRUE(b >= a);
}

TEST_F(AngleMinusPiToPiTest, ComparisonTransitivity) {
    angle<minus_pi_to_pi> a(-1.0);
    angle<minus_pi_to_pi> b(0.0);
    angle<minus_pi_to_pi> c(1.0);

    EXPECT_TRUE(a < b);
    EXPECT_TRUE(b < c);
    EXPECT_TRUE(a < c);  // Transitivity
}

// Zero to 2Pi tests

TEST_F(AngleZeroToTwoPiTest, DefaultConstructor) {
    angle<zero_to_two_pi> a;
    EXPECT_NEAR(a.get_radians(), 0.0, kEpsilon);
    EXPECT_NEAR(a.get_degrees(), 0.0, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, Constructor) {
    angle<zero_to_two_pi> a(1.0);
    EXPECT_NEAR(a.get_radians(), 1.0, kEpsilon);
    EXPECT_NEAR(a.get_degrees(), 180.0 / pi, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SetValueWrapNegative) {
    angle<zero_to_two_pi> a;
    a.set_value(-1.0);
    EXPECT_NEAR(a.get_radians(), -1.0 + 2 * pi, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SetValueWrapPositive) {
    angle<zero_to_two_pi> a;
    a.set_value(2 * pi + 0.5);
    EXPECT_NEAR(a.get_radians(), 0.5, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SetValueExactlyTwoPi) {
    angle<zero_to_two_pi> a;
    a.set_value(2 * pi);
    EXPECT_NEAR(a.get_radians(), 2 * pi, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SetValueExactlyZero) {
    angle<zero_to_two_pi> a;
    a.set_value(0.0);
    EXPECT_NEAR(a.get_radians(), 0.0, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, Addition) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(2.0);
    auto c = a + b;
    EXPECT_NEAR(c.get_radians(), 3.0, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, NegationOperator) {
    angle<zero_to_two_pi> a;
    a.set_value(1.0);
    angle<zero_to_two_pi> a_negated = -a;
    EXPECT_NEAR(a, 1.0, kEpsilon);
    EXPECT_NEAR(a_negated, 2.0 * pi - 1.0, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, AdditionWithWrapping) {
    angle<zero_to_two_pi> a(5.0);
    angle<zero_to_two_pi> b(2.0);
    auto c = a + b;
    EXPECT_NEAR(c.get_radians(), 7.0 - 2 * pi, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SubtractionWithWrapping) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(2.0);
    auto c = a - b;  // Should wrap from -1.0 to -1.0 + 2pi
    EXPECT_NEAR(c.get_radians(), -1.0 + 2 * pi, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, AdditionAssignment) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(2.0);
    a += b;
    EXPECT_NEAR(a.get_radians(), 3.0, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SubtractionAssignment) {
    angle<zero_to_two_pi> a(4.0);
    angle<zero_to_two_pi> b(1.0);
    a -= b;
    EXPECT_NEAR(a.get_radians(), 3.0, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, ScalarMultiplication) {
    angle<zero_to_two_pi> a(1.0);
    auto b = a * 2.0;
    EXPECT_NEAR(b.get_radians(), 2.0, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, Equality) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(1.0);
    EXPECT_EQ(a, b);
}

TEST_F(AngleZeroToTwoPiTest, EqualityWithWrapping) {
    angle<zero_to_two_pi> a(0.0);
    angle<zero_to_two_pi> b(2 * pi);
    EXPECT_EQ(a, b);
}

TEST_F(AngleZeroToTwoPiTest, Inequality) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(2.0);
    EXPECT_NE(a, b);
}

// Comparison operator tests for zero_to_two_pi

TEST_F(AngleZeroToTwoPiTest, LessThanBasic) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(2.0);
    EXPECT_TRUE(a < b);
    EXPECT_FALSE(b < a);
}

TEST_F(AngleZeroToTwoPiTest, LessThanEqual) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(1.0);
    EXPECT_FALSE(a < b);
    EXPECT_FALSE(b < a);
}

TEST_F(AngleZeroToTwoPiTest, LessThanWithWrapping) {
    angle<zero_to_two_pi> a(-0.1);  // Wraps to 2π - 0.1
    angle<zero_to_two_pi> b(0.1);   // Near zero
    // After wrapping both to [0, 2π]: a ≈ 2π-0.1, b ≈ 0.1
    EXPECT_TRUE(b < a);
}

TEST_F(AngleZeroToTwoPiTest, LessThanOrEqualBasic) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(2.0);
    EXPECT_TRUE(a <= b);
    EXPECT_FALSE(b <= a);
}

TEST_F(AngleZeroToTwoPiTest, LessThanOrEqualEqual) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(1.0);
    EXPECT_TRUE(a <= b);
    EXPECT_TRUE(b <= a);
}

TEST_F(AngleZeroToTwoPiTest, LessThanOrEqualWithWrapping) {
    angle<zero_to_two_pi> a(0.0);     // At zero boundary
    angle<zero_to_two_pi> b(2 * pi);  // Should wrap to 0.0 (equivalent)
    EXPECT_TRUE(a <= b);
    EXPECT_TRUE(b <= a);
}

TEST_F(AngleZeroToTwoPiTest, GreaterThanBasic) {
    angle<zero_to_two_pi> a(2.0);
    angle<zero_to_two_pi> b(1.0);
    EXPECT_TRUE(a > b);
    EXPECT_FALSE(b > a);
}

TEST_F(AngleZeroToTwoPiTest, GreaterThanEqual) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(1.0);
    EXPECT_FALSE(a > b);
    EXPECT_FALSE(b > a);
}

TEST_F(AngleZeroToTwoPiTest, GreaterThanWithWrapping) {
    angle<zero_to_two_pi> a(0.1);   // Near zero
    angle<zero_to_two_pi> b(-0.1);  // Wraps to 2π - 0.1
    // After wrapping both to [0, 2π]: a ≈ 0.1, b ≈ 2π-0.1
    EXPECT_TRUE(b > a);
}

TEST_F(AngleZeroToTwoPiTest, GreaterThanOrEqualBasic) {
    angle<zero_to_two_pi> a(2.0);
    angle<zero_to_two_pi> b(1.0);
    EXPECT_TRUE(a >= b);
    EXPECT_FALSE(b >= a);
}

TEST_F(AngleZeroToTwoPiTest, GreaterThanOrEqualEqual) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(1.0);
    EXPECT_TRUE(a >= b);
    EXPECT_TRUE(b >= a);
}

TEST_F(AngleZeroToTwoPiTest, GreaterThanOrEqualWithWrapping) {
    angle<zero_to_two_pi> a(0.0);     // At zero boundary
    angle<zero_to_two_pi> b(2 * pi);  // Should wrap to 0.0 (equivalent)
    EXPECT_TRUE(a >= b);
    EXPECT_TRUE(b >= a);
}

TEST_F(AngleZeroToTwoPiTest, ComparisonTransitivity) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(2.0);
    angle<zero_to_two_pi> c(3.0);

    EXPECT_TRUE(a < b);
    EXPECT_TRUE(b < c);
    EXPECT_TRUE(a < c);  // Transitivity
}

TEST_F(AngleZeroToTwoPiTest, ComparisonBoundaryValues) {
    angle<zero_to_two_pi> a(0.0);           // At lower boundary
    angle<zero_to_two_pi> b(pi);            // At middle
    angle<zero_to_two_pi> c(2 * pi - 0.1);  // Near upper boundary

    EXPECT_TRUE(a < b);
    EXPECT_TRUE(b < c);
    EXPECT_TRUE(a < c);
}

}  // namespace