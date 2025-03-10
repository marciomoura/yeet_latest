

#include "yeet_control/pi_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "yeet_control/integrator.hpp"
#include "yeet_control/reference_frame.hpp"

namespace {

using namespace yeet::control;

TEST(PIControllerTest, BasicUpdateAndReset) {
    // Use direct tuning method: configure() with kp and continuous ki.
    pi_controller<double> controller;
    const double ts = 0.01;  // 10 ms sampling time
    const double kp = 2.0;
    const double ki_cont = 3.0;             // continuous gain
    controller.configure(kp, ki_cont, ts);  // discrete ki = ki_cont * ts

    // Step input error = 1.0 repeatedly.
    controller.reset();
    double expected_integral = 0.0;
    const int n_steps = 50;
    for (int i = 0; i < n_steps; ++i) {
        expected_integral += 1.0;
        double expected_out = kp * 1.0 + (ki_cont * ts) * expected_integral;
        double output = controller.update(1.0);
        EXPECT_NEAR(output, expected_out, 1e-6);
    }
    // Reset and check that integral goes to zero.
    controller.reset();
    EXPECT_NEAR(controller.get_integral(), 0.0, 1e-6);
}

TEST(PIControllerTest, ConfigureWithTi) {
    // Test tuning using the integral time constant method.
    pi_controller<double> controller;
    const double ts = 0.01;
    const double kp = 2.0;
    const double ti = 4.0;  // Integral time constant.
    // With kp = 2 and ti = 4, Ki_cont = kp / ti = 0.5
    controller.configure_with_ti(kp, ti, ts);
    EXPECT_NEAR(controller.get_kp(), kp, 1e-6);
    EXPECT_NEAR(controller.get_ki_continuous(), 0.5, 1e-6);
    EXPECT_NEAR(controller.get_ki_discrete(), 0.5 * ts, 1e-6);
}

TEST(PIControllerTest, OutputLimitClamping) {
    // Verify that output is clamped to the specified limits.
    pi_controller<double> controller;
    const double ts = 0.01;
    const double kp = 5.0;
    const double ki_cont = 10.0;
    controller.configure(kp, ki_cont, ts);

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
    pi_controller<double> controller;
    controller.configure_with_ti(kp, ti, ts);
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
    pi_controller<dq<double>> controller;
    controller.configure_with_ti(kp, ti, ts);

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
    pi_controller<alphabeta<double>> controller;
    controller.configure_with_ti(kp, ti, ts);

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
    pi_controller<abc<double>> controller;
    controller.configure_with_ti(kp, ti, ts);

    abc<double> output = controller.update(abc<double>{1.0, 1.0, 1.0});

    EXPECT_DOUBLE_EQ(output[0], output[1]);
    EXPECT_DOUBLE_EQ(output[0], output[2]);
    EXPECT_GE(output[0], 0.0);
    EXPECT_GE(output[1], 0.0);
    EXPECT_GE(output[3], 0.0);
}

// Helper function to simulate plant integration in a coordinate frame.
// Assumes that the coordinate type supports elementwise addition and scalar multiplication.
template <typename Coord>
Coord integrate_state(const Coord &state, const Coord &control, double ts) {
    // For each component, update: new_state = state + ts * control.
    // Here, we assume that operator+ and operator* are defined.
    return state + control * ts;
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
    pi_controller<dq<double>> controller;
    controller.configure_with_ti(kp, ti, ts);
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
    pi_controller<alphabeta<double>> controller;
    controller.configure_with_ti(kp, ti, ts);
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
    pi_controller<abc<double>> controller;
    controller.configure_with_ti(kp, ti, ts);
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

}  // namespace
