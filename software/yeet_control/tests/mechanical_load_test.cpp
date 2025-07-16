#include "yeet_control/mechanical_load.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace yeet::control;

namespace {

static constexpr double kSamplingTime = 0.001;  // 1 ms

// Verifies that with no load and no damping, a constant torque produces constant acceleration.
TEST(MechanicalLoadTest, NoLoadAcceleration) {
    mechanical_load load(kSamplingTime);
    const double inertia_H = 2.0;  // s
    const double damping_D = 0.0;  // pu
    load.configure(inertia_H, damping_D);
    load.reset();

    const double torque_pu = 1.0;
    const double expected_acceleration = torque_pu / (2.0 * inertia_H);  // 1.0 / 4.0 = 0.25 pu/s

    double expected_speed_pu = 0.0;
    for (int i = 0; i < 1000; ++i) {
        double actual_speed_pu = load.update(torque_pu);
        expected_speed_pu += expected_acceleration * kSamplingTime;
        EXPECT_NEAR(actual_speed_pu, expected_speed_pu, 1e-9);
    }
}

// Verifies that damping reduces the net torque and thus the acceleration.
TEST(MechanicalLoadTest, AccelerationWithDamping) {
    mechanical_load load(kSamplingTime);
    const double inertia_H = 2.0;
    const double damping_D = 0.5;  // pu
    load.configure(inertia_H, damping_D);
    load.reset();

    const double torque_pu = 1.0;
    double actual_speed_pu = 0.0;

    // Simulate for a few steps and check that the speed is less than the no-damping case.
    for (int i = 0; i < 10; ++i) {
        actual_speed_pu = load.update(torque_pu);
    }

    const double no_damping_acceleration = torque_pu / (2.0 * inertia_H);
    const double no_damping_speed_at_10_steps = no_damping_acceleration * 10 * kSamplingTime;

    EXPECT_LT(actual_speed_pu, no_damping_speed_at_10_steps);
}

// Verifies that the system reaches a steady state when electromagnetic torque equals damping torque.
TEST(MechanicalLoadTest, SteadyStateSpeed) {
    mechanical_load load(kSamplingTime);
    const double inertia_H = 1.0;
    const double damping_D = 0.5;
    load.configure(inertia_H, damping_D);
    load.reset();

    const double torque_pu = 1.0;
    const double expected_steady_state_speed = torque_pu / damping_D;  // 1.0 / 0.5 = 2.0 pu

    // Simulate for a long time to reach steady state (e.g., 10 time constants)
    const int simulation_steps = static_cast<int>((10.0 * (2.0 * inertia_H / damping_D)) / kSamplingTime);
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(torque_pu);
    }

    EXPECT_NEAR(load.get_speed_pu(), expected_steady_state_speed, 1e-3);
}

// Verifies that applying a load torque correctly reduces the final speed.
TEST(MechanicalLoadTest, LoadTorqueApplication) {
    mechanical_load load(kSamplingTime);
    const double inertia_H = 1.0;
    const double damping_D = 0.5;
    load.configure(inertia_H, damping_D);
    load.reset();

    const double em_torque_pu = 1.0;
    const double load_torque_pu = 0.2;
    const double net_torque_for_ss = em_torque_pu - load_torque_pu;            // 0.8 pu
    const double expected_steady_state_speed = net_torque_for_ss / damping_D;  // 0.8 / 0.5 = 1.6 pu

    // Simulate for a long time to reach steady state
    const int simulation_steps = static_cast<int>((10.0 * (2.0 * inertia_H / damping_D)) / kSamplingTime);
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(em_torque_pu, load_torque_pu);
    }

    EXPECT_NEAR(load.get_speed_pu(), expected_steady_state_speed, 1e-3);
}

// Verifies that the reset functionality can set a specific initial speed.
TEST(MechanicalLoadTest, ResetToSpecificSpeed) {
    mechanical_load load(kSamplingTime);
    load.configure(1.0, 0.1);

    const double initial_speed_pu = 0.5;
    load.reset(initial_speed_pu);
    EXPECT_EQ(load.get_speed_pu(), initial_speed_pu);
}

// Verifies that speed remains constant if torques balance and there is no damping.
TEST(MechanicalLoadTest, ConstantSpeedWhenTorquesBalanceWithoutDamping) {
    mechanical_load load(kSamplingTime);
    load.configure(1.0, 0.0);  // No damping

    const double initial_speed_pu = 0.8;
    load.reset(initial_speed_pu);

    // Apply equal electromagnetic and load torques
    const double torque_pu = 0.5;
    for (int i = 0; i < 100; ++i) {
        load.update(torque_pu, torque_pu);
        EXPECT_EQ(load.get_speed_pu(), initial_speed_pu);
    }
}

// Verifies that speed decreases if torques balance but damping is present.
TEST(MechanicalLoadTest, SpeedDecreasesWhenTorquesBalanceWithDamping) {
    mechanical_load load(kSamplingTime);
    load.configure(1.0, 0.1);  // With damping

    const double initial_speed_pu = 1.0;
    load.reset(initial_speed_pu);

    // Apply equal electromagnetic and load torques
    const double torque_pu = 0.5;
    double new_speed = load.update(torque_pu, torque_pu);

    // With non-zero speed and damping, the damping torque will cause deceleration.
    EXPECT_LT(new_speed, initial_speed_pu);
}

}  // namespace