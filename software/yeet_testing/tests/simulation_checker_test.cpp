#include <gtest/gtest.h>

#include "yeet_testing/simulator_checker.hpp"

namespace {

using namespace yeet;

// Test fixture for simulator checker tests
class SimulatorCheckerTest : public ::testing::Test {
protected:
    static constexpr double sampling_time = 0.001;  // 1ms sampling time
    static constexpr double default_tolerance = 1e-6;
};

// Lambda checker tests
TEST_F(SimulatorCheckerTest, LambdaCheckerBasicOperation)
{
    bool check_called = false;
    auto checker = lambda_checker("test_checker", [&check_called]() {
        check_called = true;
        return true;
    });

    EXPECT_TRUE(checker.check());
    EXPECT_TRUE(check_called);
    EXPECT_EQ(checker.get_name(), "test_checker");
}

TEST_F(SimulatorCheckerTest, LambdaCheckerCustomFailureMessage)
{
    auto checker = lambda_checker("test_checker", []() { return false; }, []() { return "Custom failure message"; });

    EXPECT_FALSE(checker.check());
    EXPECT_EQ(checker.get_failure_message(), "Custom failure message");
}

// Range checker tests
TEST_F(SimulatorCheckerTest, RangeCheckerWithinBounds)
{
    auto value = 5.0;
    auto checker = make_range_checker<double>(
        "range_test", [&value]() { return value; }, 0.0, 10.0, checker_activation_time_range{0.0, 1.0});

    EXPECT_TRUE(checker.check());
    EXPECT_TRUE(checker.is_active(0.5));
}

TEST_F(SimulatorCheckerTest, RangeCheckerOutOfBounds)
{
    auto value = 15.0;
    auto checker = make_range_checker<double>(
        "range_test", [&value]() { return value; }, 0.0, 10.0, checker_activation_time_range{0.0, 1.0});

    EXPECT_FALSE(checker.check());
    EXPECT_FALSE(checker.is_active(2.0));
}

// Equal checker tests
TEST_F(SimulatorCheckerTest, EqualCheckerExactMatch)
{
    auto value = 5.0;
    auto checker = make_equal_checker<double>(
        "equal_test", [&value]() { return value; }, 5.0, checker_activation_exact_time{0.5, sampling_time});

    EXPECT_TRUE(checker.check());
    EXPECT_TRUE(checker.is_active(0.5));
}

TEST_F(SimulatorCheckerTest, EqualCheckerMismatch)
{
    auto value = 5.1;
    auto checker = make_equal_checker<double>(
        "equal_test", [&value]() { return value; }, 5.0, checker_activation_exact_time{0.5, sampling_time});

    EXPECT_FALSE(checker.check());
}

// Near checker tests
TEST_F(SimulatorCheckerTest, NearCheckerWithinTolerance)
{
    auto value = 5.0001;
    auto checker = checker_type_near<double>(5.0, 0.001);

    EXPECT_TRUE(checker.check(value));
}

TEST_F(SimulatorCheckerTest, NearCheckerOutsideTolerance)
{
    auto value = 5.1;
    auto checker = checker_type_near<double>(5.0, 0.001);

    EXPECT_FALSE(checker.check(value));
}

// Min checker tests
TEST_F(SimulatorCheckerTest, MinCheckerAboveMinimum)
{
    auto value = 5.0;
    auto checker = make_min_checker<double>(
        "min_test", [&value]() { return value; }, 0.0, checker_activation_time_range{0.0, 1.0});

    EXPECT_TRUE(checker.check());
}

TEST_F(SimulatorCheckerTest, MinCheckerBelowMinimum)
{
    auto value = -1.0;
    auto checker = make_min_checker<double>(
        "min_test", [&value]() { return value; }, 0.0, checker_activation_time_range{0.0, 1.0});

    EXPECT_FALSE(checker.check());
}

// Max checker tests
TEST_F(SimulatorCheckerTest, MaxCheckerBelowMaximum)
{
    auto value = 5.0;
    auto checker = make_max_checker<double>(
        "max_test", [&value]() { return value; }, 10.0, checker_activation_time_range{0.0, 1.0});

    EXPECT_TRUE(checker.check());
}

TEST_F(SimulatorCheckerTest, MaxCheckerAboveMaximum)
{
    auto value = 15.0;
    auto checker = make_max_checker<double>(
        "max_test", [&value]() { return value; }, 10.0, checker_activation_time_range{0.0, 1.0});

    EXPECT_FALSE(checker.check());
}

// Activation time tests
TEST_F(SimulatorCheckerTest, CheckerActivationTimeRange)
{
    auto checker = make_range_checker<double>(
        "activation_test", []() { return 5.0; }, 0.0, 10.0, checker_activation_time_range{1.0, 2.0});

    EXPECT_FALSE(checker.is_active(0.5));  // Before range
    EXPECT_TRUE(checker.is_active(1.5));   // Within range
    EXPECT_FALSE(checker.is_active(2.5));  // After range
}

TEST_F(SimulatorCheckerTest, CheckerActivationExactTime)
{
    auto checker = make_range_checker<double>(
        "activation_test", []() { return 5.0; }, 0.0, 10.0, checker_activation_exact_time{1.0, sampling_time});

    EXPECT_FALSE(checker.is_active(0.999));  // Just before
    EXPECT_TRUE(checker.is_active(1.0));     // Exact time
    EXPECT_FALSE(checker.is_active(1.002));  // Just after
}

}  // namespace