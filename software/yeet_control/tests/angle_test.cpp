#include "yeet_control/angle.hpp"

#include <gtest/gtest.h>

namespace {

using namespace yeet::control;

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
    EXPECT_NEAR(a.get_degrees(), 180.0 / pi<double>, kEpsilon);
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
    EXPECT_NEAR(a.get_radians(), 4.0 - 2 * pi<double>, kEpsilon);
}

TEST_F(AngleMinusPiToPiTest, SetValueWrapNegative) {
    angle<minus_pi_to_pi> a;
    a.set_value(-4.0);  // < -pi
    EXPECT_NEAR(a.get_radians(), -4.0 + 2 * pi<double>, kEpsilon);
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
    EXPECT_NEAR(c.get_radians(), 4.0 - 2 * pi<double>, kEpsilon);
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
    EXPECT_NEAR(c.get_radians(), -4.0 + 2 * pi<double>, kEpsilon);
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
    angle<minus_pi_to_pi> a(pi<double>);
    angle<minus_pi_to_pi> b(-pi<double>);
    EXPECT_EQ(a, b);
}

TEST_F(AngleMinusPiToPiTest, Inequality) {
    angle<minus_pi_to_pi> a(1.0);
    angle<minus_pi_to_pi> b(2.0);
    EXPECT_NE(a, b);
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
    EXPECT_NEAR(a.get_degrees(), 180.0 / pi<double>, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SetValueWrapNegative) {
    angle<zero_to_two_pi> a;
    a.set_value(-1.0);
    EXPECT_NEAR(a.get_radians(), -1.0 + 2 * pi<double>, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SetValueWrapPositive) {
    angle<zero_to_two_pi> a;
    a.set_value(2 * pi<double> + 0.5);
    EXPECT_NEAR(a.get_radians(), 0.5, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SetValueExactlyTwoPi) {
    angle<zero_to_two_pi> a;
    a.set_value(2 * pi<double>);
    EXPECT_NEAR(a.get_radians(), 0.0, kEpsilon);
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
    EXPECT_NEAR(a_negated, 2.0 * pi<double> - 1.0, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, AdditionWithWrapping) {
    angle<zero_to_two_pi> a(5.0);
    angle<zero_to_two_pi> b(2.0);
    auto c = a + b;
    EXPECT_NEAR(c.get_radians(), 7.0 - 2 * pi<double>, kEpsilon);
}

TEST_F(AngleZeroToTwoPiTest, SubtractionWithWrapping) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(2.0);
    auto c = a - b;  // Should wrap from -1.0 to -1.0 + 2pi
    EXPECT_NEAR(c.get_radians(), -1.0 + 2 * pi<double>, kEpsilon);
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
    angle<zero_to_two_pi> b(2 * pi<double>);
    EXPECT_EQ(a, b);
}

TEST_F(AngleZeroToTwoPiTest, Inequality) {
    angle<zero_to_two_pi> a(1.0);
    angle<zero_to_two_pi> b(2.0);
    EXPECT_NE(a, b);
}

}  // namespace