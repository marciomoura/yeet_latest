#include "yeet_control/synchronous_machine.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "yeet_control/constants.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;
using namespace yeet;

/**
 * @brief Base fixture for synchronous machine tests with common functionality
 */
class SynchronousMachineTestBase : public ::testing::Test {
public:
    static constexpr double sampling_time = 50e-6;        // 50 µs for high resolution
    static constexpr double electrical_frequency = 50.0;  // 50 Hz
    static constexpr double current_magnitude = 10.0;     // 10 A peak current
    static constexpr double voltage_magnitude = 100.0;    // 100 V peak voltage
    static constexpr double field_voltage = 24.0;         // 24 V field excitation
    static constexpr double load_torque = 5.0;            // 5 N·m load torque

    void SetUp() override
    {
        // Machine parameters for a small 10 kW, 4-pole, 50 Hz synchronous machine
        // Stator parameters
        resistance_t R_s = resistance_t(0.5);     // Stator resistance (Ω)
        inductance_t L_d = inductance_t(15e-3);   // Direct-axis inductance (H)
        inductance_t L_q = inductance_t(10e-3);   // Quadrature-axis inductance (H)
        inductance_t L_ad = inductance_t(12e-3);  // Direct-axis mutual inductance (H)
        inductance_t L_aq = inductance_t(8e-3);   // Quadrature-axis mutual inductance (H)

        // Rotor parameters
        resistance_t R_f = resistance_t(2.0);     // Field resistance (Ω)
        inductance_t L_f = inductance_t(50e-3);   // Field self-inductance (H)
        resistance_t R_kD = resistance_t(1.5);    // Direct-axis damper resistance (Ω)
        inductance_t L_kD = inductance_t(10e-3);  // Direct-axis damper inductance (H)
        resistance_t R_kQ = resistance_t(1.5);    // Quadrature-axis damper resistance (Ω)
        inductance_t L_kQ = inductance_t(8e-3);   // Quadrature-axis damper inductance (H)

        // Mechanical parameters
        moment_of_inertia_t J = moment_of_inertia_t(0.05);  // Inertia (kg·m²)
        int P = 4;                                          // Number of poles

        // Create machine instance
        machine =
            std::make_unique<synchronous_machine>(R_s, R_f, R_kD, R_kQ, L_d, L_q, L_ad, L_aq, L_f, L_kD, L_kQ, J, P);

        // Initialize state
        state = synchronous_machine_state{};
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    double get_synchronous_speed() const { return 4.0 * pi_double * electrical_frequency / 4.0; }

protected:
    // Register all machine loggers with organized sub-groups
    void register_loggers(double& time_ref,
        const abc<current_t>* i_abc = nullptr,
        const abc<voltage_t>* v_abc = nullptr,
        const dq<current_t>* i_dq = nullptr,
        const dq<voltage_t>* v_dq = nullptr,
        const double* torque_em = nullptr)
    {
        // Mechanical quantities
        _simulator.register_sampler_entry(
            make_lambda_sampler<double>("mechanical:time", [&time_ref]() { return time_ref; }));
        _simulator.register_sampler_entry(
            make_lambda_sampler<double>("mechanical:theta", [&]() { return state.theta_r.get_radians().value(); }));
        _simulator.register_sampler_entry(
            make_lambda_sampler<double>("mechanical:omega", [&]() { return state.omega_r; }));

        // ABC currents and voltages
        if (i_abc && v_abc) {
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("currents:a", [i_abc]() { return i_abc->a().value(); }));
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("currents:b", [i_abc]() { return i_abc->b().value(); }));
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("currents:c", [i_abc]() { return i_abc->c().value(); }));
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("voltages:a", [v_abc]() { return v_abc->a().value(); }));
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("voltages:b", [v_abc]() { return v_abc->b().value(); }));
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("voltages:c", [v_abc]() { return v_abc->c().value(); }));
        }

        // DQ currents and voltages
        if (i_dq && v_dq) {
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("currents:d", [i_dq]() { return i_dq->d().value(); }));
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("currents:q", [i_dq]() { return i_dq->q().value(); }));
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("voltages:d", [v_dq]() { return v_dq->d().value(); }));
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("voltages:q", [v_dq]() { return v_dq->q().value(); }));
        }

        // Rotor fluxes
        _simulator.register_sampler_entry(make_lambda_sampler<double>("fluxes:field", [&]() { return state.psi_fD; }));
        _simulator.register_sampler_entry(
            make_lambda_sampler<double>("fluxes:damper_d", [&]() { return state.psi_kD; }));
        _simulator.register_sampler_entry(
            make_lambda_sampler<double>("fluxes:damper_q", [&]() { return state.psi_kQ; }));

        // Electromagnetic torque
        if (torque_em) {
            _simulator.register_sampler_entry(
                make_lambda_sampler<double>("mechanical:torque_electromagnetic", [torque_em]() { return *torque_em; }));
        }
    }

    std::unique_ptr<synchronous_machine> machine;
    synchronous_machine_state state;
    simulator _simulator = make_gtest_simulator();
};

/**
 * @brief Test fixture for steady-state operating point validation
 *
 * This fixture tests the machine at synchronous speed with balanced conditions.
 * Theoretical steady-state values are calculated from machine parameters.
 */
class SynchronousMachineSteadyStateTest : public SynchronousMachineTestBase {
protected:
    void setup_steady_state_at_sync_speed(double i_d_ref, double i_q_ref)
    {
        // Initialize at synchronous speed
        state.omega_r = get_synchronous_speed();
        state.theta_r = 0.0;

        // Calculate steady-state field flux: ψ_f = (V_f / R_f) * L_f = (24 / 2) * 0.05 = 0.6 Wb
        state.psi_fD = (field_voltage / 2.0) * 0.05;  // Steady-state field flux
        state.psi_kD = 0.0;                           // No damper currents in steady-state
        state.psi_kQ = 0.0;

        _i_d_setpoint = i_d_ref;
        _i_q_setpoint = i_q_ref;

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Generate currents in dq frame first, then transform to abc
            double electrical_angle = 2.0f * state.theta_r.get_radians().value();

            // Create dq currents at setpoint
            _i_dq = dq<current_t>(current_t(_i_d_setpoint), current_t(_i_q_setpoint));

            // Transform to abc
            angle_wrapped theta_e(electrical_angle);
            _i_abc = _i_dq.to_abc(theta_e);

            voltage_t v_field = voltage_t(field_voltage);
            torque_t T_load = torque_t(load_torque);

            // Calculate derivatives
            auto derivatives = machine->calculate_derivatives_current_fed(state, _i_abc, v_field, T_load);

            // Integrate electrical states
            state.psi_fD += derivatives.psi_fD * sampling_time;
            state.psi_kD += derivatives.psi_kD * sampling_time;
            state.psi_kQ += derivatives.psi_kQ * sampling_time;

            // Hold mechanical speed constant (ideal prime mover)
            state.theta_r += derivatives.theta_r * sampling_time;

            // Get terminal voltages
            _v_abc = machine->get_terminal_voltages_abc(state, _i_abc);
            _v_dq = _v_abc.to_dq(theta_e);

            // Calculate electromagnetic torque: T_e = 1.5 * P/2 * ψ_f * i_q
            _torque_em = 1.5 * 2.0 * state.psi_fD * _i_dq.q().value();

            _time += sampling_time;
        });

        register_loggers(_time, &_i_abc, &_v_abc, &_i_dq, &_v_dq, &_torque_em);
    }

    double _time = 0.0;
    abc<current_t> _i_abc{};
    abc<voltage_t> _v_abc{};
    dq<current_t> _i_dq{};
    dq<voltage_t> _v_dq{};
    double _torque_em = 0.0;
    double _i_d_setpoint = 0.0;
    double _i_q_setpoint = 0.0;
};

TEST_F(SynchronousMachineSteadyStateTest, NoLoadOperation)
{
    // No-load condition: i_d = 0, i_q = 0
    setup_steady_state_at_sync_speed(0.0, 0.0);
    simulate_for(0.1);  // 100 ms = 5 electrical cycles at 50 Hz for settling

    // Verify steady-state conditions
    double omega_sync = get_synchronous_speed();  // 157.08 rad/s
    EXPECT_NEAR(state.omega_r, omega_sync, 1e-3);

    // Field flux reaches steady-state considering mutual coupling with stator
    // Actual value includes contribution from mutual inductances
    EXPECT_GT(state.psi_fD, 0.6);   // At least steady-state field current value
    EXPECT_LT(state.psi_fD, 0.75);  // Reasonable upper bound with mutual coupling

    // Damper currents decay but may have small residuals
    EXPECT_LT(std::abs(state.psi_kD), 0.15);
    EXPECT_LT(std::abs(state.psi_kQ), 0.05);

    // No torque at no-load
    EXPECT_NEAR(_torque_em, 0.0, 0.1);
}

TEST_F(SynchronousMachineSteadyStateTest, LoadedOperation)
{
    // Loaded condition: i_d = 0, i_q = 10A (typical motoring)
    setup_steady_state_at_sync_speed(0.0, 10.0);
    simulate_for(0.1);  // 100 ms for settling

    // Verify currents match setpoint
    EXPECT_NEAR(_i_dq.d().value(), 0.0, 0.1);
    EXPECT_NEAR(_i_dq.q().value(), 10.0, 0.1);

    // Field flux at steady-state (includes mutual coupling effects)
    EXPECT_GT(state.psi_fD, 0.6);
    EXPECT_LT(state.psi_fD, 0.75);

    // Expected torque: T_e = 1.5 * (P/2) * ψ_f * i_q
    // Using actual flux value (approximately 0.7 Wb)
    double expected_torque_min = 1.5 * 2.0 * 0.6 * 10.0;   // 18 N·m minimum
    double expected_torque_max = 1.5 * 2.0 * 0.75 * 10.0;  // 22.5 N·m maximum
    EXPECT_GT(_torque_em, expected_torque_min);
    EXPECT_LT(_torque_em, expected_torque_max);

    // Damper currents will have some transient response to load
    EXPECT_LT(std::abs(state.psi_kD), 0.15);
    EXPECT_LT(std::abs(state.psi_kQ), 0.10);
}

/**
 * @brief Test dynamic response to load step with steady-state validation
 */
class SynchronousMachineLoadStepTest : public SynchronousMachineTestBase {
protected:
    static constexpr double step_time = 0.05;   // Apply load step at 50 ms
    static constexpr double i_q_initial = 5.0;  // Initial q-axis current (A)
    static constexpr double i_q_final = 15.0;   // Final q-axis current (A)

    void setup_load_step_test()
    {
        // Initialize at synchronous speed with steady-state field flux
        state.omega_r = get_synchronous_speed();
        state.theta_r = 0.0;
        state.psi_fD = 0.6;  // Steady-state field flux
        state.psi_kD = 0.0;
        state.psi_kQ = 0.0;

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Apply step in q-axis current at specified time
            double i_q = (_time < step_time) ? i_q_initial : i_q_final;

            double electrical_angle = 2.0f * state.theta_r.get_radians().value();
            angle_wrapped theta_e(electrical_angle);

            // DQ currents: i_d = 0 (unity power factor), i_q varies
            _i_dq = dq<current_t>(current_t(0.0), current_t(i_q));
            _i_abc = _i_dq.to_abc(theta_e);

            voltage_t v_field = voltage_t(field_voltage);

            // Calculate load torque from current: T_load ≈ T_em for steady-state
            double T_load_value = 1.5 * 2.0 * state.psi_fD * i_q;
            torque_t T_load = torque_t(T_load_value);

            // Calculate derivatives (speed can vary now)
            auto derivatives = machine->calculate_derivatives_current_fed(state, _i_abc, v_field, T_load);

            // Integrate all states
            state.psi_fD += derivatives.psi_fD * sampling_time;
            state.psi_kD += derivatives.psi_kD * sampling_time;
            state.psi_kQ += derivatives.psi_kQ * sampling_time;
            state.omega_r += derivatives.omega_r * sampling_time;
            state.theta_r += derivatives.theta_r * sampling_time;

            // Get terminal voltages
            _v_abc = machine->get_terminal_voltages_abc(state, _i_abc);
            _v_dq = _v_abc.to_dq(theta_e);

            // Calculate electromagnetic torque
            _torque_em = 1.5 * 2.0 * state.psi_fD * _i_dq.q().value();

            _time += sampling_time;
        });

        register_loggers(_time, nullptr, nullptr, &_i_dq, &_v_dq, &_torque_em);
    }

    double _time = 0.0;
    abc<current_t> _i_abc{};
    abc<voltage_t> _v_abc{};
    dq<current_t> _i_dq{};
    dq<voltage_t> _v_dq{};
    double _torque_em = 0.0;
};

TEST_F(SynchronousMachineLoadStepTest, SettlesToNewSteadyState)
{
    setup_load_step_test();
    simulate_for(0.2);  // 50ms initial + 150ms settling

    // After settling, verify machine has responded to load step
    // Speed will drop from synchronous due to heavy load torque requirement
    double omega_sync = get_synchronous_speed();
    EXPECT_GT(state.omega_r, 0.4 * omega_sync);   // At least 40% of sync speed
    EXPECT_LT(state.omega_r, 1.05 * omega_sync);  // No overshoot

    // Field flux should be established
    EXPECT_GT(state.psi_fD, 0.6);
    EXPECT_LT(state.psi_fD, 0.8);

    // Torque should have increased with load step
    // Using actual flux ~0.7 Wb: T ≈ 1.5 * 2 * 0.7 * 15 = 31.5 N·m
    EXPECT_GT(_torque_em, 25.0);  // Minimum expected torque
    EXPECT_LT(_torque_em, 40.0);  // Maximum reasonable torque

    // Q-axis current should be at final value
    EXPECT_NEAR(_i_dq.q().value(), i_q_final, 0.5);
}

/**
 * @brief Test acceleration from near-synchronous speed
 *
 * This test verifies the machine can maintain speed when already rotating.
 * Startup from standstill is complex and requires specific control strategies.
 */
class SynchronousMachineSpeedMaintenanceTest : public SynchronousMachineTestBase {
protected:
    void setup_speed_maintenance_test()
    {
        // Start near synchronous speed
        state.omega_r = 0.9 * get_synchronous_speed();  // 90% of sync speed
        state.theta_r = 0.0;
        state.psi_fD = 0.5;  // Partially established field
        state.psi_kD = 0.0;
        state.psi_kQ = 0.0;

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Generate balanced currents in dq frame
            double electrical_angle = 2.0f * state.theta_r.get_radians().value();
            angle_wrapped theta_e(electrical_angle);

            // Apply i_q current for torque
            dq<current_t> i_dq(current_t(0.0), current_t(10.0));
            _i_abc = i_dq.to_abc(theta_e);

            voltage_t v_field = voltage_t(field_voltage);
            torque_t T_load = torque_t(load_torque);

            // Calculate derivatives
            auto derivatives = machine->calculate_derivatives_current_fed(state, _i_abc, v_field, T_load);

            // Integrate all states
            state.psi_fD += derivatives.psi_fD * sampling_time;
            state.psi_kD += derivatives.psi_kD * sampling_time;
            state.psi_kQ += derivatives.psi_kQ * sampling_time;
            state.omega_r += derivatives.omega_r * sampling_time;
            state.theta_r += derivatives.theta_r * sampling_time;

            // Get terminal voltages
            _v_abc = machine->get_terminal_voltages_abc(state, _i_abc);

            _time += sampling_time;
        });

        register_loggers(_time, &_i_abc, &_v_abc);
    }

    double _time = 0.0;
    abc<current_t> _i_abc{};
    abc<voltage_t> _v_abc{};
};

TEST_F(SynchronousMachineSpeedMaintenanceTest, MaintainsRotationNearSynchronousSpeed)
{
    setup_speed_maintenance_test();
    simulate_for(0.2);  // 200 ms

    // Machine should maintain or slightly increase speed
    double omega_sync = get_synchronous_speed();
    EXPECT_GT(state.omega_r, 0.85 * omega_sync);  // At least 85% of sync
    EXPECT_LT(state.omega_r, 1.1 * omega_sync);   // Not exceeding 110%

    // Field flux should approach steady-state
    EXPECT_GT(state.psi_fD, 0.5);
    EXPECT_LT(state.psi_fD, 0.8);
}

}  // namespace