#include "yeet_control/integrator.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "yeet_control/angle.hpp"

namespace {

using namespace yeet::control;

TEST(IntegratorTest, IntegratesConstantInput) {
    // Verify that a constant input integrates correctly.
    integrator<double> integ;
    const double ts = 0.001;  // 1 ms sampling time
    integ.configure(ts);
    integ.reset();

    const size_t num_samples = 1000;  // simulate for 1 second
    const double input = 1.0;
    double expected = 0.0;

    for (size_t i = 0; i < num_samples; ++i) {
        expected += input * ts;
        double output = integ.update(input);
        EXPECT_NEAR(output, expected, 1e-6);
    }

    EXPECT_NEAR(integ.get_value(), expected, 1e-6);
}

TEST(IntegratorTest, SetValueAndReset) {
    // Verify that setting an initial value and then resetting works.
    integrator<double> integ;
    const double ts = 0.001;
    integ.configure(ts);

    integ.set_value(5.0);
    EXPECT_NEAR(integ.get_value(), 5.0, 1e-6);

    // Update with an input value and check the integration result.
    double output = integ.update(2.0);
    EXPECT_NEAR(output, 5.0 + 2.0 * ts, 1e-6);

    integ.reset();
    EXPECT_NEAR(integ.get_value(), 0.0, 1e-6);
}

TEST(IntegratorTest, AngleWrappingTest) {
    // Use the integrator to simulate angular integration.
    // For an angular velocity input of 2π * 50 rad/s (i.e. 50 rotations per second)
    // integrating over exactly 1 second should yield exactly 50 full rotations.
    // Both angle wrappers (zero_to_two_pi and minus_pi_to_pi) should wrap the result to 0.
    integrator<angle<zero_to_two_pi>> angle_integrator;
    const double ts = 0.001;  // sampling time: 1 ms
    angle_integrator.configure(ts);
    angle_integrator.reset();

    const double angular_velocity = 2.0 * pi<double> * 50.0;  // 50 rotations per second
    const size_t num_samples = 1000;                          // simulate for 1 second

    for (size_t i = 0; i < num_samples; ++i) {
        angle_integrator.update(angular_velocity);
    }

    double integrated_angle = angle_integrator.get_value();  // Should be approximately 2π * 50
    angle<zero_to_two_pi> angle_2pi(integrated_angle);
    EXPECT_NEAR(angle_2pi.get_radians(), 2.0 * pi<double>, 1e-6);
    EXPECT_NEAR(angle_2pi + angle<zero_to_two_pi>{1e-8}, 0.0f, 1e-6);  // Wrapping around 2π and zero

    angle<minus_pi_to_pi> wrapped_angle_minus(integrated_angle);
    EXPECT_NEAR(wrapped_angle_minus.get_radians(), 0.0, 1e-6);

    // Now simulate a non-integer number of rotations.
    // For example, simulate for 1.01 seconds.
    angle_integrator.reset();
    size_t new_samples = static_cast<size_t>(1.01 / ts);
    for (size_t i = 0; i < new_samples; ++i) {
        angle_integrator.update(angular_velocity);
    }
    integrated_angle = angle_integrator.get_value();  // ~2π * 50 * 1.01
    angle<zero_to_two_pi> wrapped_angle_zero2(integrated_angle);

    // Expected remainder after wrapping to [0, 2π)
    double expected_remainder = std::fmod(integrated_angle, 2.0 * pi<double>);
    EXPECT_NEAR(wrapped_angle_zero2.get_radians(), expected_remainder, 1e-6);
}

}  // namespace