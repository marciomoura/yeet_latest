#include "yeet_control/pi_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "yeet_control/integrator.hpp"
#include "yeet_control/reference_frame.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

TEST(PIControllerTest, ConfigureWithTi) {
    // Test tuning using the integral time constant method.
    const double ts = 0.01;
    const double kp = 2.0;
    const double ti = 4.0;  // Integral time constant.
    // With kp = 2 and ti = 4, Ki = kp / ti = 0.5
    pi_controller<double> controller(ts);
    controller.configure_with_ti(kp, ti);
    EXPECT_NEAR(controller.get_kp(), kp, 1e-6);
    EXPECT_NEAR(controller.get_ki(), 0.5, 1e-6);
}

TEST(PIControllerTest, OutputLimitClamping) {
    // Verify that output is clamped to the specified limits.
    const double ts = 0.01;
    const double kp = 5.0;
    const double ki = 10.0;
    pi_controller<double> controller(ts);
    controller.configure(kp, ki);

    // Limit output to [-1, 1]
    controller.set_output_limits(-1.0, 1.0);
    controller.reset();

    // Provide a large error repeatedly to build up the integral:
    double output;
    for (int i = 0; i < 20; ++i) {
        output = controller.update(1.0);
    }
    EXPECT_LE(output, 1.0);
    // Now try a negative error.
    controller.reset();
    for (int i = 0; i < 20; ++i) {
        output = controller.update(-1.0);
    }
    EXPECT_GE(output, -1.0);
}

TEST(PIControllerTest, ClosedLoopIntegrationPlant) {
    // Closed-loop simulation: the plant is a simple integrator and the PI controller
    // is used to drive the plant output to a setpoint.
    //
    // Plant model:
    //    x[k+1] = x[k] + Ts * u[k]
    // Controlled variable: x
    // Setpoint: r
    // Error: e = r - x
    //
    // The PI controller computes: u = Kp * e + Ki_disc * ∑e
    //
    // In steady state, the plant output should converge to the setpoint.
    const double ts = 0.01;              // 10 ms sampling time
    const double simulation_time = 5.0;  // Total simulation duration (seconds)
    const size_t n_steps = static_cast<size_t>(simulation_time / ts);

    // Set controller parameters.
    const double kp = 2.0;
    const double ti = 1.0;  // faster integration
    pi_controller<double> controller(ts);
    controller.configure_with_ti(kp, ti);
    // Optionally, set output limits to simulate actuator saturation.
    controller.set_output_limits(-10.0, 10.0);
    controller.reset();

    // Plant (integrator) representing the system.
    integrator<double> plant;
    plant.configure(ts);
    plant.reset();

    // Setpoint.
    const double r = 1.0;

    // Run simulation.
    double plant_output = plant.get_value();
    for (size_t i = 0; i < n_steps; ++i) {
        double error = r - plant_output;
        double u = controller.update(error);
        plant_output = plant.update(u);
    }

    // In steady state, the plant output should be near the setpoint.
    EXPECT_NEAR(plant_output, r, 0.01);
}

TEST(PIControllerTest, ControlInDQFrame) {
    // Set controller parameters.
    const double ts = 0.01;  // 10 ms sampling time
    const double kp = 2.0;
    const double ti = 1.0;  // faster integration
    pi_controller<dq<double>> controller(ts);
    controller.configure_with_ti(kp, ti);

    dq<double> output = controller.update(dq<double>{1.0, 1.0});

    EXPECT_DOUBLE_EQ(output[0], output[1]);
    EXPECT_GE(output[0], 0.0);
    EXPECT_GE(output[1], 0.0);
}

TEST(PIControllerTest, ControlInAlphaBetaFrame) {
    // Set controller parameters.
    const double ts = 0.01;  // 10 ms sampling time
    const double kp = 2.0;
    const double ti = 1.0;  // faster integration
    pi_controller<alphabeta<double>> controller(ts);
    controller.configure_with_ti(kp, ti);

    alphabeta<double> output = controller.update(alphabeta<double>{1.0, 1.0});

    EXPECT_DOUBLE_EQ(output[0], output[1]);
    EXPECT_GE(output[0], 0.0);
    EXPECT_GE(output[1], 0.0);
}

TEST(PIControllerTest, ControlInABCFrame) {
    // Set controller parameters.
    const double ts = 0.01;  // 10 ms sampling time
    const double kp = 2.0;
    const double ti = 1.0;  // faster integration
    pi_controller<abc<double>> controller(ts);
    controller.configure_with_ti(kp, ti);

    abc<double> output = controller.update(abc<double>{1.0, 1.0, 1.0});

    EXPECT_DOUBLE_EQ(output[0], output[1]);
    EXPECT_DOUBLE_EQ(output[0], output[2]);
    EXPECT_GE(output[0], 0.0);
    EXPECT_GE(output[1], 0.0);
}

TEST(PIControllerTest, ClosedLoopDQFrame) {
    // Closed-loop simulation in dq frame.
    // Plant model: state[k+1] = state[k] + Ts * u[k]
    const double ts = 0.01;
    const double simulation_time = 5.0;
    const size_t n_steps = static_cast<size_t>(simulation_time / ts);

    // PI controller parameters.
    const double kp = 2.0;
    const double ti = 1.0;  // Integral time constant.
    pi_controller<dq<double>> controller(ts);
    controller.configure_with_ti(kp, ti);
    controller.set_output_limits(-10.0, 10.0);
    controller.reset();

    // Desired setpoint in dq frame.
    dq<double> setpoint{1.0, 1.0};

    // Plant state initialization (assumed zero).
    integrator<dq<double>> plant;
    plant.configure(ts);

    for (size_t i = 0; i < n_steps; ++i) {
        dq<double> error = setpoint - plant.get_value();
        dq<double> control = controller.update(error);
        plant.update(control);
    }

    // In steady state, error should be small.
    dq<double> final_error = setpoint - plant.get_value();
    for (int i = 0; i < 2; i++) {
        EXPECT_NEAR(final_error[i], 0.0, 0.01);
    }
}

TEST(PIControllerTest, ClosedLoopAlphaBetaFrame) {
    // Closed-loop simulation in αβ frame.
    const double ts = 0.01;
    const double simulation_time = 5.0;
    const size_t n_steps = static_cast<size_t>(simulation_time / ts);

    const double kp = 2.0;
    const double ti = 1.0;
    pi_controller<alphabeta<double>> controller(ts);
    controller.configure_with_ti(kp, ti);
    controller.set_output_limits(-10.0, 10.0);
    controller.reset();

    alphabeta<double> setpoint{1.0, 1.0};
    integrator<alphabeta<double>> plant;
    plant.configure(ts);

    for (size_t i = 0; i < n_steps; ++i) {
        alphabeta<double> error = setpoint - plant.get_value();
        alphabeta<double> control = controller.update(error);
        plant.update(control);
    }

    alphabeta<double> final_error = setpoint - plant.get_value();
    for (int i = 0; i < 2; i++) {
        EXPECT_NEAR(final_error[i], 0.0, 0.01);
    }
}

TEST(PIControllerTest, ClosedLoopABCFrame) {
    const double ts = 0.01;
    const double simulation_time = 5.0;
    const size_t n_steps = static_cast<size_t>(simulation_time / ts);

    const double kp = 2.0;
    const double ti = 1.0;
    pi_controller<abc<double>> controller(ts);
    controller.configure_with_ti(kp, ti);
    controller.set_output_limits(-10.0, 10.0);
    controller.reset();

    abc<double> setpoint = {1.0, 1.0, 1.0};
    // Initialize plant state to zero.
    integrator<abc<double>> plant;
    plant.configure(ts);

    for (size_t i = 0; i < n_steps; ++i) {
        abc<double> error = setpoint - plant.get_value();
        abc<double> control = controller.update(error);
        plant.update(control);
    }

    abc<double> final_error = setpoint - plant.get_value();
    // Check all three phases.
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(final_error[i], 0.0, 0.01);
    }
}

TEST(PIControllerTest, IntegratorDisable) {
    // Test disabling the integrator functionality.
    const double ts = 0.01;
    const double kp = 2.0;
    const double ki = 5.0;
    pi_controller<double> controller(ts);
    controller.configure(kp, ki);

    controller.reset();

    // First, update with integrator enabled
    double error = 1.0;
    controller.update(error);
    double integral_after_first_update = controller.get_integral();

    // Disable integrator
    controller.enable_integrator(false);
    EXPECT_FALSE(controller.is_integrator_enabled());

    // Update again with same error
    controller.update(error);
    double integral_after_second_update = controller.get_integral();

    // Integral should not have changed
    EXPECT_FLOAT_EQ(integral_after_first_update, integral_after_second_update);

    // The output should be only the proportional term
    double output = controller.update(error);
    EXPECT_FLOAT_EQ(kp * error + controller.get_integral(), output);
    EXPECT_FLOAT_EQ(controller.get_integral(), integral_after_second_update);
}

TEST(PIControllerAntiWindupTest, AntiWindupPreventsIntegralWindup) {
    // Test that anti-windup prevents the integral from growing when output is saturated.
    const double ts = 0.01;
    const double kp = 2.0;
    const double ki = 10.0;  // High integral gain to cause windup
    pi_controller<double> controller(ts);
    controller.configure(kp, ki);

    // Set output limits to cause saturation
    controller.set_output_limits(-1.0, 1.0);

    // Set anti-windup gain (kc)
    const double kc = 1.0 / (kp / ki);  // A common choice is 1/Ti
    controller.set_antiwindup_gain(kc);

    controller.reset();

    // Apply large error to cause saturation
    double large_error = 10.0;
    double output;

    // Run for a few steps to let the system saturate
    for (int i = 0; i < 10; ++i) {
        output = controller.update(large_error);
        EXPECT_LE(output, 1.0);  // Output should be saturated
        EXPECT_GE(output, -1.0);
    }

    // Check that saturation was detected
    EXPECT_TRUE(controller.is_output_saturated());

    // The integral should have reached a steady state due to anti-windup
    double integral_after_saturation = controller.get_integral();

    // Continue applying the large error
    for (int i = 0; i < 10; ++i) {
        controller.update(large_error);
    }

    // Integral should not have grown significantly due to anti-windup
    double integral_final = controller.get_integral();
    EXPECT_LT(std::abs(integral_final - integral_after_saturation), 0.3f);
}

TEST(PIControllerAntiWindupTest, IntegratorClampingPreventsWindup) {
    // Test that integrator clamping prevents the integral from growing when output is saturated.
    const double ts = 0.01;
    const double kp = 2.0;
    const double ki = 10.0;  // High integral gain to cause windup
    pi_controller<double> controller(ts);
    controller.configure(kp, ki);

    // Set output limits to cause saturation
    controller.set_output_limits(-1.0, 1.0);

    // Enable integrator clamping
    controller.enable_integrator_clamping(true);

    controller.reset();

    // Apply large positive error to cause saturation at the upper limit
    double large_positive_error = 10.0;
    for (int i = 0; i < 10; ++i) {
        controller.update(large_positive_error);
    }

    EXPECT_TRUE(controller.is_output_saturated());
    double integral_at_positive_saturation = controller.get_integral();

    // Continue applying the large positive error
    for (int i = 0; i < 10; ++i) {
        controller.update(large_positive_error);
    }

    // Integral should not have changed
    EXPECT_FLOAT_EQ(controller.get_integral(), integral_at_positive_saturation);

    // Apply large negative error to cause saturation at the lower limit
    double large_negative_error = -10.0;
    for (int i = 0; i < 20; ++i) {  // More iterations to overcome the positive integral
        controller.update(large_negative_error);
    }

    EXPECT_TRUE(controller.is_output_saturated());
    double integral_at_negative_saturation = controller.get_integral();

    // Continue applying the large negative error
    for (int i = 0; i < 10; ++i) {
        controller.update(large_negative_error);
    }

    // Integral should not have changed
    EXPECT_FLOAT_EQ(controller.get_integral(), integral_at_negative_saturation);

    // Apply a small error that brings the controller out of saturation
    double small_error = -0.1;
    controller.update(small_error);

    // Integral should have changed now
    EXPECT_NE(controller.get_integral(), integral_at_negative_saturation);
}

}  // namespace
