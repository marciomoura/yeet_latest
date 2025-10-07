#include "yeet_control/speed_voltage_relationship.hpp"

#include <gtest/gtest.h>

namespace {

using namespace yeet;
using namespace yeet::control;

class SpeedVoltageRelationshipTest : public ::testing::Test {
protected:
    const double sampling_time = 1e-3f;  // 1ms sampling time
    const real_t tolerance = 1e-6f;
};

TEST_F(SpeedVoltageRelationshipTest, InitialStateIsCorrect)
{
    speed_voltage_relationship model(sampling_time);

    EXPECT_FLOAT_EQ(model.get_voltage_magnitude_pu(), 0.0f);
    EXPECT_FLOAT_EQ(model.get_steady_state_voltage_pu(), 0.0f);
    EXPECT_FALSE(model.is_dynamic_mode());
}

TEST_F(SpeedVoltageRelationshipTest, DefaultConfigurationWorks)
{
    speed_voltage_relationship model(sampling_time);

    // Default: coefficient = 1.0, offset = 0.0, no dynamics
    model.update(0.5f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.5f, tolerance);

    model.update(1.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 1.0f, tolerance);
}

TEST_F(SpeedVoltageRelationshipTest, LinearRelationshipWithCoefficient)
{
    speed_voltage_relationship model(sampling_time);

    model.configure_voltage_speed_coefficient(0.8f);
    model.configure_voltage_offset_pu(0.0f);

    model.update(0.5f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.4f, tolerance);  // 0.8 * 0.5

    model.update(1.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.8f, tolerance);  // 0.8 * 1.0

    model.update(1.25f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 1.0f, tolerance);  // 0.8 * 1.25
}

TEST_F(SpeedVoltageRelationshipTest, LinearRelationshipWithOffset)
{
    speed_voltage_relationship model(sampling_time);

    model.configure_voltage_speed_coefficient(1.0f);
    model.configure_voltage_offset_pu(0.1f);

    model.update(0.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.1f, tolerance);  // 1.0 * 0.0 + 0.1

    model.update(0.5f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.6f, tolerance);  // 1.0 * 0.5 + 0.1

    model.update(1.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 1.1f, tolerance);  // 1.0 * 1.0 + 0.1
}

TEST_F(SpeedVoltageRelationshipTest, CombinedCoefficientAndOffset)
{
    speed_voltage_relationship model(sampling_time);

    model.configure_voltage_speed_coefficient(0.9f);
    model.configure_voltage_offset_pu(0.05f);

    model.update(0.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.05f, tolerance);  // 0.9 * 0.0 + 0.05

    model.update(1.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.95f, tolerance);  // 0.9 * 1.0 + 0.05

    model.update(1.11f);                                               // Should give exactly 1.0 pu
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 1.049f, tolerance);  // 0.9 * 1.11 + 0.05
}

TEST_F(SpeedVoltageRelationshipTest, VoltageLimitsAreEnforced)
{
    speed_voltage_relationship model(sampling_time);

    model.configure_voltage_speed_coefficient(1.0f);
    model.configure_voltage_offset_pu(0.0f);
    model.configure_voltage_limits_pu(0.1f, 1.2f);

    // Test upper limit
    model.update(2.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 1.2f, tolerance);

    // Test lower limit
    model.update(0.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.1f, tolerance);  // Clamped to minimum

    // Test within limits
    model.update(0.5f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.5f, tolerance);
}

TEST_F(SpeedVoltageRelationshipTest, SteadyStateVoltageMatchesAlgebraicResult)
{
    speed_voltage_relationship model(sampling_time);

    model.configure_voltage_speed_coefficient(0.85f);
    model.configure_voltage_offset_pu(0.08f);
    model.configure_voltage_limits_pu(0.0f, 1.5f);

    const real_t test_speed = 0.7f;
    const real_t expected = 0.85f * 0.7f + 0.08f;  // 0.595 + 0.08 = 0.675

    model.update(test_speed);

    EXPECT_NEAR(model.get_voltage_magnitude_pu(), expected, tolerance);
    EXPECT_NEAR(model.get_steady_state_voltage_pu(), expected, tolerance);
}

TEST_F(SpeedVoltageRelationshipTest, DynamicModeFirstOrderResponse)
{
    speed_voltage_relationship model(sampling_time);

    const real_t time_constant = 10e-3f;  // 10ms
    model.configure_voltage_speed_coefficient(1.0f);
    model.configure_voltage_offset_pu(0.0f);
    model.configure_time_constant(time_constant);

    EXPECT_TRUE(model.is_dynamic_mode());

    // Apply step input from 0 to 1 pu speed
    model.reset(0.0f);
    const real_t step_input = 1.0f;

    // Simulate for several time constants
    const int num_steps = static_cast<int>(5 * time_constant / sampling_time);

    for (int i = 0; i < num_steps; ++i) {
        model.update(step_input);

        // Steady-state should always be 1.0
        EXPECT_NEAR(model.get_steady_state_voltage_pu(), 1.0f, tolerance);

        // Dynamic response should be approaching 1.0
        EXPECT_LE(model.get_voltage_magnitude_pu(), 1.0f + tolerance);
    }

    // After 5 time constants, should be > 99% of final value
    EXPECT_GT(model.get_voltage_magnitude_pu(), 0.99f);
}

TEST_F(SpeedVoltageRelationshipTest, DynamicVsAlgebraicModeComparison)
{
    speed_voltage_relationship algebraic_model(sampling_time);
    speed_voltage_relationship dynamic_model(sampling_time);

    const real_t coefficient = 0.9f;
    const real_t offset = 0.05f;

    // Configure both models identically except for time constant
    algebraic_model.configure_voltage_speed_coefficient(coefficient);
    algebraic_model.configure_voltage_offset_pu(offset);
    algebraic_model.configure_time_constant(0.0f);  // Algebraic

    dynamic_model.configure_voltage_speed_coefficient(coefficient);
    dynamic_model.configure_voltage_offset_pu(offset);
    dynamic_model.configure_time_constant(5e-3f);  // 5ms time constant

    const real_t test_speed = 0.8f;
    const real_t expected_steady_state = coefficient * test_speed + offset;

    // Algebraic model gives immediate response
    algebraic_model.update(test_speed);
    EXPECT_NEAR(algebraic_model.get_voltage_magnitude_pu(), expected_steady_state, tolerance);

    // Dynamic model needs time to reach steady state
    dynamic_model.reset(0.0f);
    dynamic_model.update(test_speed);
    EXPECT_LT(dynamic_model.get_voltage_magnitude_pu(), expected_steady_state);

    // But steady-state voltage should be the same
    EXPECT_NEAR(dynamic_model.get_steady_state_voltage_pu(), expected_steady_state, tolerance);

    // Run dynamic model until settled
    for (int i = 0; i < 100; ++i) {
        dynamic_model.update(test_speed);
    }

    // Now both should give the same result
    EXPECT_NEAR(dynamic_model.get_voltage_magnitude_pu(), expected_steady_state, 1e-4f);
}

TEST_F(SpeedVoltageRelationshipTest, ResetMethodWorks)
{
    speed_voltage_relationship model(sampling_time);

    model.configure_voltage_speed_coefficient(1.0f);
    model.configure_time_constant(5e-3f);

    // Run model to some state
    model.update(1.0f);
    model.update(1.0f);
    model.update(1.0f);

    EXPECT_GT(model.get_voltage_magnitude_pu(), 0.1f);

    // Reset to specific value
    model.reset(0.5f);
    EXPECT_FLOAT_EQ(model.get_voltage_magnitude_pu(), 0.5f);

    // Reset to default (zero)
    model.reset();
    EXPECT_FLOAT_EQ(model.get_voltage_magnitude_pu(), 0.0f);
}

TEST_F(SpeedVoltageRelationshipTest, ResetRespectsVoltageLimits)
{
    speed_voltage_relationship model(sampling_time);

    model.configure_voltage_limits_pu(0.2f, 1.0f);

    // Try to reset to value outside limits
    model.reset(2.0f);  // Above upper limit
    EXPECT_FLOAT_EQ(model.get_voltage_magnitude_pu(), 1.0f);

    model.reset(0.1f);  // Below lower limit
    EXPECT_FLOAT_EQ(model.get_voltage_magnitude_pu(), 0.2f);

    model.reset(0.5f);  // Within limits
    EXPECT_FLOAT_EQ(model.get_voltage_magnitude_pu(), 0.5f);
}

TEST_F(SpeedVoltageRelationshipTest, ZeroSpeedHandling)
{
    speed_voltage_relationship model(sampling_time);

    model.configure_voltage_speed_coefficient(1.0f);
    model.configure_voltage_offset_pu(0.1f);

    model.update(0.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.1f, tolerance);  // Only offset

    // With no offset, should be zero (but respecting minimum limit)
    model.configure_voltage_offset_pu(0.0f);
    model.configure_voltage_limits_pu(0.0f, 2.0f);

    model.update(0.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 0.0f, tolerance);
}

TEST_F(SpeedVoltageRelationshipTest, HighSpeedOperation)
{
    speed_voltage_relationship model(sampling_time);

    model.configure_voltage_speed_coefficient(1.0f);
    model.configure_voltage_offset_pu(0.0f);
    model.configure_voltage_limits_pu(0.0f, 1.5f);

    // Test high speed operation
    model.update(1.2f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 1.2f, tolerance);

    // Test speed that would exceed voltage limit
    model.update(2.0f);
    EXPECT_NEAR(model.get_voltage_magnitude_pu(), 1.5f, tolerance);  // Clamped to max
}

}  // namespace