#include "yeet_control/mechanical_load.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace yeet;
using namespace yeet::control;
using namespace yeet;

namespace {

static constexpr real_t kSamplingTime = 0.001f;  // 1 ms

// Verifies that with no load and no damping, a constant torque produces constant acceleration.
TEST(MechanicalLoadTest, NoLoadAcceleration)
{
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
TEST(MechanicalLoadTest, AccelerationWithDamping)
{
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
TEST(MechanicalLoadTest, SteadyStateSpeed)
{
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
TEST(MechanicalLoadTest, LoadTorqueApplication)
{
    mechanical_load load(kSamplingTime);
    const inertia_constant_t inertia_constant = inertia_constant_t{1.0f};
    const damping_coefficient_pu_t damping_coefficient = damping_coefficient_pu_t{0.5f};
    load.configure(inertia_constant, damping_coefficient);
    load.reset();

    const torque_pu_t em_torque_pu = torque_pu_t{1.0f};
    const torque_pu_t load_torque = torque_pu_t{0.2f};
    const real_t net_torque_for_ss = em_torque_pu.value() - load_torque.value();                 // 0.8 pu
    const real_t expected_steady_state_speed = net_torque_for_ss / damping_coefficient.value();  // 0.8 / 0.5 = 1.6 pu

    // Simulate for a long time to reach steady state
    const int simulation_steps =
        static_cast<int>((10.0f * (2.0f * inertia_constant.value() / damping_coefficient.value())) / kSamplingTime);
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(em_torque_pu, load_torque);
    }

    EXPECT_NEAR(load.get_speed_pu().value(), expected_steady_state_speed, 1e-3f);
}

// Verifies that the reset functionality can set a specific initial speed.
TEST(MechanicalLoadTest, ResetToSpecificSpeed)
{
    mechanical_load load(kSamplingTime);
    load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f});

    const speed_pu_t initial_speed_pu = speed_pu_t{0.5f};
    load.reset(initial_speed_pu);
    EXPECT_FLOAT_EQ(load.get_speed_pu().value(), initial_speed_pu.value());
}

// Verifies that speed remains constant if torques balance and there is no damping.
TEST(MechanicalLoadTest, ConstantSpeedWhenTorquesBalanceWithoutDamping)
{
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
TEST(MechanicalLoadTest, SpeedDecreasesWhenTorquesBalanceWithDamping)
{
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
TEST(MechanicalLoadTest, ConfigureValidatesParameters)
{
    mechanical_load load(kSamplingTime);

    // Valid configuration should not crash
    EXPECT_NO_THROW(load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f}));
    EXPECT_NO_THROW(load.configure(inertia_constant_t{2.5f}, damping_coefficient_pu_t{0.0f}));
}

// Test that update method handles zero torques correctly
TEST(MechanicalLoadTest, UpdateWithZeroTorques)
{
    mechanical_load load(kSamplingTime);
    load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f});

    const speed_pu_t initial_speed = speed_pu_t{1.0f};
    load.reset(initial_speed);

    // With zero electromagnetic and load torques, only damping should affect speed
    const speed_pu_t new_speed = load.update(torque_pu_t{0.0f}, torque_pu_t{0.0f});

    // Speed should decrease due to damping
    EXPECT_LT(new_speed.value(), initial_speed.value());
}

// ========== Speed-Dependent Load Tests ==========

// Test linear load: T_load = k1 * omega (typical for fans/blowers at low speed)
TEST(MechanicalLoadTest, LinearLoadSteadyState)
{
    mechanical_load load(kSamplingTime);
    const inertia_constant_t inertia_constant = inertia_constant_t{1.0f};
    const damping_coefficient_pu_t damping_coefficient = damping_coefficient_pu_t{0.0f};  // No damping
    const real_t k1 = 0.5f;  // Linear load coefficient [pu/pu]

    load.configure(inertia_constant, damping_coefficient, k1, 0.0f);
    load.reset();

    const torque_pu_t em_torque = torque_pu_t{1.0f};

    // Steady state: T_e = k1 * omega => omega_ss = T_e / k1 = 1.0 / 0.5 = 2.0 pu
    const real_t expected_steady_state_speed = em_torque.value() / k1;

    // Simulate to steady state (10 time constants)
    const int simulation_steps = static_cast<int>((10.0f * (2.0f * inertia_constant.value() / k1)) / kSamplingTime);
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(em_torque);
    }

    EXPECT_NEAR(load.get_speed_pu().value(), expected_steady_state_speed, 1e-2f);
}

// Test quadratic load: T_load = k2 * omega^2 (typical for centrifugal pumps/compressors)
TEST(MechanicalLoadTest, QuadraticLoadSteadyState)
{
    mechanical_load load(kSamplingTime);
    const inertia_constant_t inertia_constant = inertia_constant_t{1.0f};
    const damping_coefficient_pu_t damping_coefficient = damping_coefficient_pu_t{0.0f};  // No damping
    const real_t k2 = 0.25f;  // Quadratic load coefficient [pu/pu^2]

    load.configure(inertia_constant, damping_coefficient, 0.0f, k2);
    load.reset();

    const torque_pu_t em_torque = torque_pu_t{1.0f};

    // Steady state: T_e = k2 * omega^2 => omega_ss = sqrt(T_e / k2) = sqrt(1.0 / 0.25) = 2.0 pu
    const real_t expected_steady_state_speed = std::sqrt(em_torque.value() / k2);

    // Simulate to steady state (longer for quadratic load)
    const int simulation_steps = 100000;  // Extended simulation for quadratic convergence
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(em_torque);
    }

    EXPECT_NEAR(load.get_speed_pu().value(), expected_steady_state_speed, 5e-2f);
}

// Test combined constant + linear + quadratic load (realistic gas turbine compressor)
TEST(MechanicalLoadTest, CombinedSpeedDependentLoad)
{
    mechanical_load load(kSamplingTime);
    const inertia_constant_t inertia_constant = inertia_constant_t{2.0f};
    const damping_coefficient_pu_t damping_coefficient = damping_coefficient_pu_t{0.1f};  // Small damping
    const real_t k1 = 0.2f;                                                               // Linear load coefficient
    const real_t k2 = 0.1f;                                                               // Quadratic load coefficient

    load.configure(inertia_constant, damping_coefficient, k1, k2);
    load.reset();

    const torque_pu_t em_torque = torque_pu_t{0.8f};
    const torque_pu_t external_load = torque_pu_t{0.1f};  // Constant external load

    // Simulate to steady state
    const int simulation_steps = 200000;
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(em_torque, external_load);
    }

    // Steady state: T_e = T_external + k1*omega + k2*omega^2 + D*omega
    // At steady state: 0.8 = 0.1 + 0.2*omega + 0.1*omega^2 + 0.1*omega
    // => 0.1*omega^2 + 0.3*omega - 0.7 = 0
    // Solving: omega = (-0.3 + sqrt(0.09 + 0.28)) / 0.2 = (-0.3 + 0.608) / 0.2 ≈ 1.54 pu
    const real_t omega_ss = load.get_speed_pu().value();

    // Verify steady state equation is satisfied
    const real_t total_load =
        external_load.value() + k1 * omega_ss + k2 * omega_ss * omega_ss + damping_coefficient.value() * omega_ss;
    EXPECT_NEAR(total_load, em_torque.value(), 5e-2f);
}

// Test gas turbine soft-start scenario with increasing quadratic load
TEST(MechanicalLoadTest, GasTurbineSoftStartScenario)
{
    mechanical_load load(kSamplingTime);
    const inertia_constant_t inertia_constant = inertia_constant_t{3.0f};                  // Large inertia
    const damping_coefficient_pu_t damping_coefficient = damping_coefficient_pu_t{0.05f};  // Low damping
    const real_t k1 = 0.0f;                                                                // Minimal linear load
    const real_t k2 = 0.3f;  // Dominant quadratic load (compressor characteristic)

    load.configure(inertia_constant, damping_coefficient, k1, k2);
    load.reset();

    // Phase 1: Low-speed startup with constant torque
    const torque_pu_t startup_torque = torque_pu_t{0.2f};
    for (int i = 0; i < 5000; ++i) {
        load.update(startup_torque);
    }

    const real_t speed_after_startup = load.get_speed_pu().value();
    EXPECT_GT(speed_after_startup, 0.0f);
    EXPECT_LT(speed_after_startup, 1.0f);  // Should not reach high speed with low torque

    // Phase 2: Increase torque for acceleration (turbine starting to produce power)
    const torque_pu_t acceleration_torque = torque_pu_t{0.6f};
    for (int i = 0; i < 20000; ++i) {
        load.update(acceleration_torque);
    }

    const real_t speed_after_accel = load.get_speed_pu().value();
    EXPECT_GT(speed_after_accel, speed_after_startup);

    // With T_e = 0.6, steady state: 0.6 = 0.3*omega^2 + 0.05*omega
    // Quadratic dominates at higher speeds
    EXPECT_LT(speed_after_accel, 1.6f);  // Reasonable speed range
}

// Test that linear coefficient validation works
TEST(MechanicalLoadTest, LinearCoefficientValidation)
{
    mechanical_load load(kSamplingTime);
    EXPECT_NO_THROW(load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f}, 0.0f, 0.0f));
    EXPECT_NO_THROW(load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f}, 0.5f, 0.0f));
    EXPECT_NO_THROW(load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f}, 0.0f, 0.25f));
    EXPECT_NO_THROW(load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f}, 0.2f, 0.1f));
}

// Test backward compatibility: default speed-dependent coefficients are zero
TEST(MechanicalLoadTest, BackwardCompatibilityDefaultCoefficients)
{
    mechanical_load load(kSamplingTime);
    load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.1f});  // Old API without coefficients
    load.reset();

    const torque_pu_t em_torque = torque_pu_t{0.5f};
    const torque_pu_t external_load = torque_pu_t{0.2f};

    // Simulate to steady state (10 time constants: tau = 2H/D = 2*1.0/0.1 = 20s)
    const int simulation_steps = static_cast<int>((10.0f * 20.0f) / kSamplingTime);
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(em_torque, external_load);
    }

    // Steady state with only damping: T_e - T_load = D * omega
    // 0.5 - 0.2 = 0.1 * omega => omega = 3.0 pu
    const real_t expected_speed = (em_torque.value() - external_load.value()) / 0.1f;
    EXPECT_NEAR(load.get_speed_pu().value(), expected_speed, 1e-2f);
}

// Test that quadratic load prevents overspeed even with high torque
TEST(MechanicalLoadTest, QuadraticLoadPreventsOverspeed)
{
    mechanical_load load(kSamplingTime);
    const real_t k2 = 1.0f;  // Strong quadratic load

    load.configure(inertia_constant_t{1.0f}, damping_coefficient_pu_t{0.0f}, 0.0f, k2);
    load.reset();

    const torque_pu_t high_torque = torque_pu_t{2.0f};

    // Simulate
    const int simulation_steps = 100000;
    for (int i = 0; i < simulation_steps; ++i) {
        load.update(high_torque);
    }

    // Steady state: T_e = k2 * omega^2 => omega = sqrt(2.0 / 1.0) = 1.414 pu
    const real_t expected_speed = std::sqrt(high_torque.value() / k2);
    EXPECT_NEAR(load.get_speed_pu().value(), expected_speed, 5e-2f);
    EXPECT_LT(load.get_speed_pu().value(), 1.5f);  // Natural speed limiting
}

}  // namespace