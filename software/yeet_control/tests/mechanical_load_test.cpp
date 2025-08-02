#include "yeet_control/mechanical_load.hpp"

#include <gtest/gtest.h>

using namespace yeet;
using namespace yeet::control;
using namespace yeet;

namespace {

static constexpr real_t kSamplingTime = 0.001f;  // 1 ms

// Verifies that with no load and no damping, a constant torque produces constant acceleration.
TEST(MechanicalLoadTest, NoLoadAcceleration) {
    mechanical_load load(kSamplingTime);
    const inertia_constant_t inertia_constant = inertia_constant_t{2.0f};                 // s
    const damping_coefficient_pu_t damping_coefficient = damping_coefficient_pu_t{0.0f};  // pu
    load.configure(inertia_constant, damping_coefficient);
    load.reset();

    const torque_pu_t torque_pu = torque_pu_t{1.0f};
    const real_t expected_acceleration =
        torque_pu.value() / (2.0f * inertia_constant.value());  // 1.0 / 4.0 = 0.25 pu/s

    real_t expected_speed_pu = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        speed_pu_t actual_speed_pu = load.update(torque_pu);
        expected_speed_pu += expected_acceleration * kSamplingTime;
        EXPECT_NEAR(actual_speed_pu.value(), expected_speed_pu, 1e-6f);
    }
}

// Verifies that damping reduces the net torque and thus the acceleration.
TEST(MechanicalLoadTest, AccelerationWithDamping) {
    mechanical_load load(kSamplingTime);
    const inertia_constant_t inertia_constant = inertia_constant_t{2.0f};
    const damping_coefficient_pu_t damping_coefficient = damping_coefficient_pu_t{0.5f};  // pu
    load.configure(inertia_constant, damping_coefficient);
    load.reset();

    const torque_pu_t torque_pu = torque_pu_t{1.0f};
    speed_pu_t actual_speed_pu = speed_pu_t{0.0f};

    // Simulate for a few steps and check that the speed is less than the no-damping case.
    for (int i = 0; i < 10; ++i) {
        actual_speed_pu = load.update(torque_pu);
    }

    const real_t no_damping_acceleration = torque_pu.value() / (2.0f * inertia_constant.value());
    const real_t no_damping_speed_at_10_steps = no_damping_acceleration * 10.0f * kSamplingTime;

    EXPECT_LT(actual_speed_pu.value(), no_damping_speed_at_10_steps);
}

// Verifies that the system reaches a steady state when electromagnetic torque equals damping torque.
TEST(MechanicalLoadTest, SteadyStateSpeed) {
    mechanical_load load(kSamplingTime);
    const inertia_constant_t inertia_constant = inertia_constant_t{1.0f};
    const damping_coefficient_pu_t damping_coefficient = damping_coefficient_pu_t{0.5f};
    load.configure(inertia_constant, damping_coefficient);
    load.reset();

    const torque_pu_t torque_pu = torque_pu_t{1.0f};
    const real_t expected_steady_state_speed = torque_pu.value() / damping_coefficient.value();  // 1.0 / 0.5 = 2.0 pu

    // Simulate for a long time to reach steady state (e.g., 10 time constants)
    const int simulation_steps =
        static_cast<int>((10.0f * (2.0f * inertia_constant.value() / damping_coefficient.value())) / kSamplingTime);
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(torque_pu);
    }

    EXPECT_NEAR(load.get_speed_pu().value(), expected_steady_state_speed, 1e-3f);
}

// Verifies that applying a load torque correctly reduces the final speed.
TEST(MechanicalLoadTest, LoadTorqueApplication) {
    mechanical_load load(kSamplingTime);
    const inertia_constant_t inertia_constant = inertia_constant_t{1.0f};
    const damping_coefficient_pu_t damping_coefficient = damping_coefficient_pu_t{0.5f};
    load.configure(inertia_constant, damping_coefficient);
    load.reset();

    const torque_pu_t em_torque_pu = torque_pu_t{1.0f};
    const torque_pu_t load_torque_pu = torque_pu_t{0.2f};
    const real_t net_torque_for_ss = em_torque_pu.value() - load_torque_pu.value();              // 0.8 pu
    const real_t expected_steady_state_speed = net_torque_for_ss / damping_coefficient.value();  // 0.8 / 0.5 = 1.6 pu

    // Simulate for a long time to reach steady state
    const int simulation_steps =
        static_cast<int>((10.0f * (2.0f * inertia_constant.value() / damping_coefficient.value())) / kSamplingTime);
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(em_torque_pu, load_torque_pu);
    }

    EXPECT_NEAR(load.get_speed_pu().value(), expected_steady_state_speed, 1e-3f);
}

// Verifies that the reset functionality can set a specific initial speed.
TEST(MechanicalLoadTest, ResetToSpecificSpeed) {
    mechanical_load load(kSamplingTime);
    load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f});

    const speed_pu_t initial_speed_pu = speed_pu_t{0.5f};
    load.reset(initial_speed_pu);
    EXPECT_FLOAT_EQ(load.get_speed_pu().value(), initial_speed_pu.value());
}

// Verifies that speed remains constant if torques balance and there is no damping.
TEST(MechanicalLoadTest, ConstantSpeedWhenTorquesBalanceWithoutDamping) {
    mechanical_load load(kSamplingTime);
    load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.0f});  // No damping

    const speed_pu_t initial_speed_pu = speed_pu_t{0.8f};
    load.reset(initial_speed_pu);

    // Apply equal electromagnetic and load torques
    const torque_pu_t torque_pu = torque_pu_t{0.5f};
    for (int i = 0; i < 100; ++i) {
        load.update(torque_pu, torque_pu);
        EXPECT_FLOAT_EQ(load.get_speed_pu().value(), initial_speed_pu.value());
    }
}

// Verifies that speed decreases if torques balance but damping is present.
TEST(MechanicalLoadTest, SpeedDecreasesWhenTorquesBalanceWithDamping) {
    mechanical_load load(kSamplingTime);
    load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f});  // With damping

    const speed_pu_t initial_speed_pu = speed_pu_t{1.0f};
    load.reset(initial_speed_pu);

    // Apply equal electromagnetic and load torques
    const torque_pu_t torque_pu = torque_pu_t{0.5f};
    speed_pu_t new_speed = load.update(torque_pu, torque_pu);

    // With non-zero speed and damping, the damping torque will cause deceleration.
    EXPECT_LT(new_speed.value(), initial_speed_pu.value());
}

// Test that configure method validates input parameters
TEST(MechanicalLoadTest, ConfigureValidatesParameters) {
    mechanical_load load(kSamplingTime);

    // Valid configuration should not crash
    EXPECT_NO_THROW(load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f}));
    EXPECT_NO_THROW(load.configure(inertia_constant_t{2.5f}, damping_coefficient_pu_t{0.0f}));
}

// Test that update method handles zero torques correctly
TEST(MechanicalLoadTest, UpdateWithZeroTorques) {
    mechanical_load load(kSamplingTime);
    load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f});

    const speed_pu_t initial_speed = speed_pu_t{1.0f};
    load.reset(initial_speed);

    // With zero electromagnetic and load torques, only damping should affect speed
    const speed_pu_t new_speed = load.update(torque_pu_t{0.0f}, torque_pu_t{0.0f});

    // Speed should decrease due to damping
    EXPECT_LT(new_speed.value(), initial_speed.value());
}

}  // namespace