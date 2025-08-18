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

TEST_F(SimulatorCheckerTest, LambdaCheckerDefaultFailureMessage)
{
    auto checker = lambda_checker("test_checker", []() { return false; });

    EXPECT_FALSE(checker.check());
    EXPECT_EQ(checker.get_failure_message(), "Check 'test_checker' failed");
}

TEST_F(SimulatorCheckerTest, LambdaCheckerWithTimeRange)
{
    auto checker = lambda_checker("timed_lambda", []() { return true; }, nullptr, 1.0, 2.0);

    EXPECT_FALSE(checker.is_active(0.5));
    EXPECT_TRUE(checker.is_active(1.5));
    EXPECT_FALSE(checker.is_active(2.5));
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

TEST_F(SimulatorCheckerTest, RangeCheckerBoundaryValues)
{
    auto value = 0.0;
    auto checker = make_range_checker<double>(
        "boundary_test", [&value]() { return value; }, 0.0, 10.0, checker_activation_time_range{0.0, 1.0});

    // Test lower boundary
    EXPECT_TRUE(checker.check());

    // Test upper boundary
    value = 10.0;
    EXPECT_TRUE(checker.check());

    // Just outside lower boundary
    value = -0.001;
    EXPECT_FALSE(checker.check());

    // Just outside upper boundary
    value = 10.001;
    EXPECT_FALSE(checker.check());
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

TEST_F(SimulatorCheckerTest, EqualCheckerWithIntegers)
{
    auto value = 42;
    auto checker =
        make_equal_checker<int>("int_equal", [&value]() { return value; }, 42, checker_activation_time_range{0.0, 1.0});

    EXPECT_TRUE(checker.check());

    value = 41;
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

TEST_F(SimulatorCheckerTest, NearCheckerExactValue)
{
    auto value = 5.0;
    auto checker = checker_type_near<double>(5.0, 0.001);

    EXPECT_TRUE(checker.check(value));
}

TEST_F(SimulatorCheckerTest, NearCheckerBoundaryTolerance)
{
    auto checker = checker_type_near<double>(5.0, 0.1);

    // Exactly at tolerance boundary
    EXPECT_TRUE(checker.check(5.1));
    EXPECT_TRUE(checker.check(4.9));

    // Just outside tolerance
    EXPECT_FALSE(checker.check(5.101));
    EXPECT_FALSE(checker.check(4.899));
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

TEST_F(SimulatorCheckerTest, MinCheckerAtMinimum)
{
    auto value = 0.0;
    auto checker = make_min_checker<double>(
        "min_boundary", [&value]() { return value; }, 0.0, checker_activation_time_range{0.0, 1.0});

    EXPECT_TRUE(checker.check());
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

TEST_F(SimulatorCheckerTest, MaxCheckerAtMaximum)
{
    auto value = 10.0;
    auto checker = make_max_checker<double>(
        "max_boundary", [&value]() { return value; }, 10.0, checker_activation_time_range{0.0, 1.0});

    EXPECT_TRUE(checker.check());
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

TEST_F(SimulatorCheckerTest, CheckerActivationTimeRangeBoundaries)
{
    auto checker = make_range_checker<double>(
        "boundary_test", []() { return 5.0; }, 0.0, 10.0, checker_activation_time_range{1.0, 2.0});

    // Test exact start time
    EXPECT_TRUE(checker.is_active(1.0));

    // Test just before start
    EXPECT_FALSE(checker.is_active(0.9999));

    // Test exact end time (should be exclusive)
    EXPECT_FALSE(checker.is_active(2.0));

    // Test just before end
    EXPECT_TRUE(checker.is_active(1.9999));
}

TEST_F(SimulatorCheckerTest, CheckerActivationOpenEndedTimeRange)
{
    // Constructor with single parameter (open-ended)
    auto checker =
        make_range_checker<double>("open_ended", []() { return 5.0; }, 0.0, 10.0, checker_activation_time_range{1.0});

    EXPECT_FALSE(checker.is_active(0.5));   // Before start
    EXPECT_TRUE(checker.is_active(1.5));    // After start
    EXPECT_TRUE(checker.is_active(100.0));  // Far in the future
    EXPECT_TRUE(checker.is_active(1e6));    // Very far in the future
}

TEST_F(SimulatorCheckerTest, CheckerActivationExactTime)
{
    auto checker = make_range_checker<double>(
        "activation_test", []() { return 5.0; }, 0.0, 10.0, checker_activation_exact_time{1.0, sampling_time});

    EXPECT_FALSE(checker.is_active(0.999));  // Just before
    EXPECT_TRUE(checker.is_active(1.0));     // Exact time
    EXPECT_FALSE(checker.is_active(1.002));  // Just after
}

TEST_F(SimulatorCheckerTest, CheckerActivationExactTimeWindow)
{
    auto checker = make_range_checker<double>(
        "window_test", []() { return 5.0; }, 0.0, 10.0, checker_activation_exact_time{1.0, 0.01});

    // Test window boundaries (1.0 ± 0.005)
    EXPECT_FALSE(checker.is_active(0.994));
    EXPECT_TRUE(checker.is_active(0.995));
    EXPECT_TRUE(checker.is_active(1.0));
    EXPECT_TRUE(checker.is_active(1.004));
    EXPECT_FALSE(checker.is_active(1.006));
}

// Always active checker tests
TEST_F(SimulatorCheckerTest, CheckerActivationAlwaysActive)
{
    auto value = 5.0;
    auto checker = make_range_checker<double>(
        "always_active_test", [&value]() { return value; }, 0.0, 10.0, checker_activation_always{});

    // Should be active at any time
    EXPECT_TRUE(checker.is_active(-100.0));
    EXPECT_TRUE(checker.is_active(0.0));
    EXPECT_TRUE(checker.is_active(1.0));
    EXPECT_TRUE(checker.is_active(1000.0));
    EXPECT_TRUE(checker.is_active(1e9));
}

TEST_F(SimulatorCheckerTest, AlwaysActiveRangeChecker)
{
    auto value = 5.0;
    auto checker = make_range_checker<double>(
        "always_range", [&value]() { return value; }, 0.0, 10.0, checker_activation_always{});

    EXPECT_TRUE(checker.check());
    value = 15.0;
    EXPECT_FALSE(checker.check());
}

TEST_F(SimulatorCheckerTest, AlwaysActiveEqualChecker)
{
    auto value = 42;
    auto checker =
        make_equal_checker<int>("always_equal", [&value]() { return value; }, 42, checker_activation_always{});

    EXPECT_TRUE(checker.check());
    value = 43;
    EXPECT_FALSE(checker.check());
}

TEST_F(SimulatorCheckerTest, AlwaysActiveMinChecker)
{
    auto value = 5.0;
    auto checker =
        make_min_checker<double>("always_min", [&value]() { return value; }, 0.0, checker_activation_always{});

    EXPECT_TRUE(checker.check());
    value = -1.0;
    EXPECT_FALSE(checker.check());
}

TEST_F(SimulatorCheckerTest, AlwaysActiveMaxChecker)
{
    auto value = 5.0;
    auto checker =
        make_max_checker<double>("always_max", [&value]() { return value; }, 10.0, checker_activation_always{});

    EXPECT_TRUE(checker.check());
    value = 11.0;
    EXPECT_FALSE(checker.check());
}

TEST_F(SimulatorCheckerTest, AlwaysActiveNearChecker)
{
    auto value = 5.001;
    auto checker =
        make_near_checker<double>("always_near", [&value]() { return value; }, 5.0, 0.01, checker_activation_always{});

    EXPECT_TRUE(checker.check());
    value = 5.1;
    EXPECT_FALSE(checker.check());
}

// User deactivation tests
TEST_F(SimulatorCheckerTest, UserDeactivationControl)
{
    auto value = 5.0;
    auto checker = make_range_checker<double>(
        "deactivation_test", [&value]() { return value; }, 0.0, 10.0, checker_activation_time_range{0.0, 10.0});

    // Initially active
    EXPECT_TRUE(checker.is_active(5.0));

    // Get deactivation control
    auto deactivation = checker.get_user_deactivation_control();
    EXPECT_FALSE(deactivation->is_deactivated());

    // Deactivate the checker
    deactivation->deactivate();
    EXPECT_TRUE(deactivation->is_deactivated());

    // Checker should no longer be active
    EXPECT_FALSE(checker.is_active(5.0));
}

TEST_F(SimulatorCheckerTest, UserDeactivationWithAlwaysActive)
{
    auto value = 5.0;
    auto checker = make_range_checker<double>(
        "always_deactivation_test", [&value]() { return value; }, 0.0, 10.0, checker_activation_always{});

    // Initially active at any time
    EXPECT_TRUE(checker.is_active(0.0));
    EXPECT_TRUE(checker.is_active(100.0));
    EXPECT_TRUE(checker.is_active(1000.0));

    // Deactivate
    auto deactivation = checker.get_user_deactivation_control();
    deactivation->deactivate();

    // Should no longer be active at any time
    EXPECT_FALSE(checker.is_active(0.0));
    EXPECT_FALSE(checker.is_active(100.0));
    EXPECT_FALSE(checker.is_active(1000.0));
}

TEST_F(SimulatorCheckerTest, UserDeactivationPersistence)
{
    auto value = 5.0;
    auto checker = make_range_checker<double>(
        "persistence_test", [&value]() { return value; }, 0.0, 10.0, checker_activation_time_range{0.0, 10.0});

    auto deactivation1 = checker.get_user_deactivation_control();
    auto deactivation2 = checker.get_user_deactivation_control();

    // Both should point to the same deactivation control
    EXPECT_FALSE(deactivation1->is_deactivated());
    EXPECT_FALSE(deactivation2->is_deactivated());

    deactivation1->deactivate();

    // Both should show deactivated
    EXPECT_TRUE(deactivation1->is_deactivated());
    EXPECT_TRUE(deactivation2->is_deactivated());
    EXPECT_FALSE(checker.is_active(5.0));
}

TEST_F(SimulatorCheckerTest, LambdaCheckerWithUserDeactivation)
{
    bool check_called = false;
    auto checker = lambda_checker(
        "lambda_deactivation",
        [&check_called]() {
            check_called = true;
            return true;
        },
        nullptr, 0.0, 10.0);

    // Initially active
    EXPECT_TRUE(checker.is_active(5.0));

    // Deactivate
    auto deactivation = checker.get_user_deactivation_control();
    deactivation->deactivate();

    // Should no longer be active
    EXPECT_FALSE(checker.is_active(5.0));
}

TEST_F(SimulatorCheckerTest, DeactivationDoesNotAffectCheckMethod)
{
    auto value = 5.0;
    auto checker = make_range_checker<double>(
        "check_method_test", [&value]() { return value; }, 0.0, 10.0, checker_activation_always{});

    // Verify check() still works when deactivated
    EXPECT_TRUE(checker.check());

    auto deactivation = checker.get_user_deactivation_control();
    deactivation->deactivate();

    // is_active should return false, but check() should still work
    EXPECT_FALSE(checker.is_active(0.0));
    EXPECT_TRUE(checker.check());  // check() method still functions

    value = 15.0;
    EXPECT_FALSE(checker.check());  // check() correctly returns false for out-of-range
}

TEST_F(SimulatorCheckerTest, MultipleCheckersIndependentDeactivation)
{
    auto value1 = 5.0;
    auto value2 = 7.0;

    auto checker1 =
        make_range_checker<double>("checker1", [&value1]() { return value1; }, 0.0, 10.0, checker_activation_always{});

    auto checker2 =
        make_range_checker<double>("checker2", [&value2]() { return value2; }, 0.0, 10.0, checker_activation_always{});

    // Both initially active
    EXPECT_TRUE(checker1.is_active(0.0));
    EXPECT_TRUE(checker2.is_active(0.0));

    // Deactivate only checker1
    auto deactivation1 = checker1.get_user_deactivation_control();
    deactivation1->deactivate();

    // Only checker1 should be deactivated
    EXPECT_FALSE(checker1.is_active(0.0));
    EXPECT_TRUE(checker2.is_active(0.0));
}

// Failure message tests
TEST_F(SimulatorCheckerTest, RangeCheckerFailureMessage)
{
    auto value = 15.0;
    auto checker =
        make_range_checker<double>("voltage", [&value]() { return value; }, 0.0, 10.0, checker_activation_always{});

    EXPECT_FALSE(checker.check());
    auto message = checker.get_failure_message();
    EXPECT_TRUE(message.find("voltage") != std::string::npos);
    EXPECT_TRUE(message.find("15") != std::string::npos);
    EXPECT_TRUE(message.find("outside range") != std::string::npos);
}

TEST_F(SimulatorCheckerTest, MinCheckerFailureMessage)
{
    auto value = -5.0;
    auto checker =
        make_min_checker<double>("temperature", [&value]() { return value; }, 0.0, checker_activation_always{});

    EXPECT_FALSE(checker.check());
    auto message = checker.get_failure_message();
    EXPECT_TRUE(message.find("temperature") != std::string::npos);
    EXPECT_TRUE(message.find("-5") != std::string::npos);
    EXPECT_TRUE(message.find("below minimum") != std::string::npos);
}

TEST_F(SimulatorCheckerTest, MaxCheckerFailureMessage)
{
    auto value = 15.0;
    auto checker = make_max_checker<double>("current", [&value]() { return value; }, 10.0, checker_activation_always{});

    EXPECT_FALSE(checker.check());
    auto message = checker.get_failure_message();
    EXPECT_TRUE(message.find("current") != std::string::npos);
    EXPECT_TRUE(message.find("15") != std::string::npos);
    EXPECT_TRUE(message.find("exceeds maximum") != std::string::npos);
}

TEST_F(SimulatorCheckerTest, EqualCheckerFailureMessage)
{
    auto value = 5;
    auto checker = make_equal_checker<int>("state", [&value]() { return value; }, 10, checker_activation_always{});

    EXPECT_FALSE(checker.check());
    auto message = checker.get_failure_message();
    EXPECT_TRUE(message.find("state") != std::string::npos);
    EXPECT_TRUE(message.find("5") != std::string::npos);
    EXPECT_TRUE(message.find("not equal") != std::string::npos);
}

TEST_F(SimulatorCheckerTest, NearCheckerFailureMessage)
{
    auto value = 5.5;
    auto checker =
        make_near_checker<double>("position", [&value]() { return value; }, 5.0, 0.1, checker_activation_always{});

    EXPECT_FALSE(checker.check());
    auto message = checker.get_failure_message();
    EXPECT_TRUE(message.find("position") != std::string::npos);
    EXPECT_TRUE(message.find("5.5") != std::string::npos);
    EXPECT_TRUE(message.find("not near") != std::string::npos);
    EXPECT_TRUE(message.find("0.1") != std::string::npos);
}

// Mixed activation types with different checker types
TEST_F(SimulatorCheckerTest, NearCheckerWithTimeRange)
{
    auto value = 5.001;
    auto checker = make_near_checker<double>(
        "near_timed", [&value]() { return value; }, 5.0, 0.01, checker_activation_time_range{1.0, 2.0});

    EXPECT_FALSE(checker.is_active(0.5));
    EXPECT_TRUE(checker.is_active(1.5));
    EXPECT_FALSE(checker.is_active(2.5));
    EXPECT_TRUE(checker.check());
}

TEST_F(SimulatorCheckerTest, NearCheckerWithExactTime)
{
    auto value = 5.001;
    auto checker = make_near_checker<double>(
        "near_exact", [&value]() { return value; }, 5.0, 0.01, checker_activation_exact_time{1.0, sampling_time});

    EXPECT_FALSE(checker.is_active(0.999));
    EXPECT_TRUE(checker.is_active(1.0));
    EXPECT_FALSE(checker.is_active(1.002));
    EXPECT_TRUE(checker.check());
}

// Default constructor test
TEST_F(SimulatorCheckerTest, CheckerTemplatedDefaultConstructor)
{
    auto value = 5.0;
    checker_type_range<double> range_checker(0.0, 10.0);
    auto templated_checker =
        checker_type_templated<checker_type_range<double>>("default_time", [&value]() { return value; }, range_checker);

    // Default should be active from t=0 onwards (start=0, end=-1)
    EXPECT_TRUE(templated_checker.is_active(0.0));
    EXPECT_TRUE(templated_checker.is_active(1.0));
    EXPECT_TRUE(templated_checker.is_active(100.0));
    EXPECT_TRUE(templated_checker.check());
}

// Unique pointer factory test
TEST_F(SimulatorCheckerTest, MakeUniqueChecker)
{
    auto value = 5.0;
    auto checker =
        make_range_checker<double>("unique_test", [&value]() { return value; }, 0.0, 10.0, checker_activation_always{});

    auto unique_checker = make_unique_checker(std::move(checker));
    EXPECT_TRUE(unique_checker->check());
    EXPECT_TRUE(unique_checker->is_active(0.0));
    EXPECT_EQ(unique_checker->get_name(), "unique_test");
}

// Complex scenario tests
TEST_F(SimulatorCheckerTest, ComplexScenarioMultipleCheckers)
{
    auto voltage = 5.0;
    auto current = 2.0;
    auto temperature = 25.0;

    auto voltage_checker = make_range_checker<double>(
        "voltage", [&voltage]() { return voltage; }, -12.0, 12.0, checker_activation_always{});

    auto current_checker = make_max_checker<double>(
        "current", [&current]() { return current; }, 10.0, checker_activation_time_range{0.0, 5.0});

    auto temp_checker = make_near_checker<double>(
        "temperature", [&temperature]() { return temperature; }, 25.0, 5.0, checker_activation_exact_time{2.0, 0.1});

    // Initial state - all values are good
    EXPECT_TRUE(voltage_checker.check());
    EXPECT_TRUE(current_checker.check());
    EXPECT_TRUE(temp_checker.check());

    // Check activation at different times
    EXPECT_TRUE(voltage_checker.is_active(3.0));  // Always active
    EXPECT_TRUE(current_checker.is_active(3.0));  // Active until t=5
    EXPECT_FALSE(temp_checker.is_active(3.0));    // Only active around t=2

    // Change values and verify
    voltage = 15.0;      // Out of range
    current = 12.0;      // Over max
    temperature = 35.0;  // Outside tolerance

    EXPECT_FALSE(voltage_checker.check());
    EXPECT_FALSE(current_checker.check());
    EXPECT_FALSE(temp_checker.check());
}

TEST_F(SimulatorCheckerTest, StateTransitionScenario)
{
    enum State { IDLE = 0, STARTING = 1, RUNNING = 2, STOPPING = 3 };
    State current_state = IDLE;

    // Check state transitions at specific times
    auto idle_check = make_equal_checker<int>(
        "idle_state", [&current_state]() { return static_cast<int>(current_state); }, static_cast<int>(IDLE),
        checker_activation_exact_time{0.0, 0.01});

    auto starting_check = make_equal_checker<int>(
        "starting_state", [&current_state]() { return static_cast<int>(current_state); }, static_cast<int>(STARTING),
        checker_activation_exact_time{1.0, 0.01});

    auto running_check = make_equal_checker<int>(
        "running_state", [&current_state]() { return static_cast<int>(current_state); }, static_cast<int>(RUNNING),
        checker_activation_time_range{2.0, 8.0});

    // Test state sequence
    EXPECT_TRUE(idle_check.is_active(0.0));
    EXPECT_TRUE(idle_check.check());

    current_state = STARTING;
    EXPECT_TRUE(starting_check.is_active(1.0));
    EXPECT_TRUE(starting_check.check());

    current_state = RUNNING;
    EXPECT_TRUE(running_check.is_active(5.0));
    EXPECT_TRUE(running_check.check());
}

// Edge case: negative time values
TEST_F(SimulatorCheckerTest, NegativeTimeValues)
{
    auto value = 5.0;
    // Use a range where start is negative but end is positive to test normal behavior
    auto checker = make_range_checker<double>(
        "negative_start", [&value]() { return value; }, 0.0, 10.0, checker_activation_time_range{-10.0, 5.0});

    EXPECT_FALSE(checker.is_active(-11.0));  // Before range
    EXPECT_TRUE(checker.is_active(-7.0));    // Within range (negative time)
    EXPECT_TRUE(checker.is_active(0.0));     // Within range (positive time)
    EXPECT_TRUE(checker.is_active(4.0));     // Within range
    EXPECT_FALSE(checker.is_active(6.0));    // After range
}

// Test for always-active behavior when both times are negative
TEST_F(SimulatorCheckerTest, BothNegativeTimesAlwaysActive)
{
    auto value = 5.0;
    auto checker = make_range_checker<double>(
        "both_negative", [&value]() { return value; }, 0.0, 10.0, checker_activation_time_range{-10.0, -5.0});

    // When both start and end are negative, checker is always active
    EXPECT_TRUE(checker.is_active(-11.0));  // Always active
    EXPECT_TRUE(checker.is_active(-7.0));   // Always active
    EXPECT_TRUE(checker.is_active(-4.0));   // Always active
    EXPECT_TRUE(checker.is_active(0.0));    // Always active
    EXPECT_TRUE(checker.is_active(100.0));  // Always active
}

// Type alias test
TEST_F(SimulatorCheckerTest, FloatingNearTypeAlias)
{
    auto value = 3.14159;
    checker_type_floating_near<double> pi_checker(3.14159, 0.00001);

    EXPECT_TRUE(pi_checker.check(value));
    EXPECT_FALSE(pi_checker.check(3.15));
}

}  // namespace