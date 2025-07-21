#include "yeet_control/three_phase_pll.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/types.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class ThreePhasePLLTest : public ::testing::Test {
protected:
    static constexpr yeet::real_t kSamplingTime = yeet::real_t{1e-4};       // 100 µs sampling time (10 kHz)
    static constexpr yeet::real_t kNominalFrequency = yeet::real_t{50.0};   // 50 Hz nominal frequency
    static constexpr yeet::real_t kFrequencyTolerance = yeet::real_t{0.1};  // 0.1 Hz
    static constexpr yeet::real_t kAngleTolerance = yeet::real_t{0.05};     // radians (~3 degrees)

    three_phase_pll pll{kSamplingTime};
    three_phase_waveform_generator waveform_generator{kSamplingTime};
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    void SetUp() override {
        // Default PLL configuration
        pll.configure_pi_controller(yeet::real_t{0.2}, yeet::real_t{0.05});
        pll.configure_nominal_frequency(kNominalFrequency);
        pll.configure_frequency_limits(yeet::real_t{30.0}, yeet::real_t{70.0});

        // Default waveform generator configuration
        waveform_generator.set_fundamental_positive_sequence_signal_amplitude(1.0);
        waveform_generator.set_signal_frequency(static_cast<double>(kNominalFrequency));

        // Simulator setup
        simulator.register_lambda_runner(static_cast<double>(kSamplingTime), [&]() {
            // Execute one simulation step
            waveform_generator.update();

            alphabeta<real_t> const pll_input = waveform_generator.get_signal_abc().to_alphabeta();

            pll.update(pll_input);
        });

        set_logger();
    }

    real_t get_estimation_error_magnitude() const {
        // Get the original input signal in the stationary frame
        alphabeta<real_t> const pll_input = waveform_generator.get_signal_abc().to_alphabeta();

        // Reconstruct the estimated signal in the stationary frame from the PLL's internal dq frame
        alphabeta<real_t> const estimated_alphabeta = pll.get_voltage_dq().to_alphabeta(pll.get_estimated_angle());

        // Return the magnitude of the error vector
        return (pll_input - estimated_alphabeta).magnitude();
    }

    void set_logger() {
        // Log signals in per-unit for comparable magnitudes
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "time", [&]() -> double { return simulator.get_current_simulation_time(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<>>(
            "waveform generator", [&]() -> alphabeta<> { return waveform_generator.get_signal_abc().to_alphabeta(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<angle<zero_to_two_pi>>(
            "waveform angle pu",
            [&]() -> angle<zero_to_two_pi> { return waveform_generator.get_fundamental_signal_angle().a().get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "waveform frequency",
            [&]() -> real_t { return waveform_generator.get_signal_frequency() / kNominalFrequency; }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "estimation error magnitude", [&]() -> real_t { return get_estimation_error_magnitude(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<dq<>>(
            "estimated dq", [&]() -> dq<> { return pll.get_voltage_dq().absolute(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<angle<zero_to_two_pi>>(
            "estimated angle pu", [&]() -> angle<zero_to_two_pi> { return pll.get_estimated_angle().get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "estimated frequency pu", [&]() -> real_t { return pll.get_estimated_frequency() / kNominalFrequency; }));

        simulator.register_sampler_entry(
            yeet::testing::make_lambda_sampler<bool>("is locked", [&]() -> bool { return pll.is_locked(); }));
    }
};

TEST_F(ThreePhasePLLTest, NominalFrequencyTracking) {
    waveform_generator.set_angle(angle<zero_to_two_pi>(pi / 2.0));  // 90 degrees phase shift

    // 1. Frequency estimation accuracy after settling
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "estimated frequency range", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.3}));

    // 2. Signal reconstruction accuracy (fundamental PLL requirement)
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "estimation error magnitude", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 1e-3f,
        yeet::testing::checker_activation_time_range{0.3}));

    // 3. Lock detection must be active after settling time
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "pll lock status", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.4}));
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "initial unlocked state", [&]() -> bool { return pll.is_locked(); }, false,
        yeet::testing::checker_activation_time_range{0.0, 0.1}));

    // 4. D-axis error should be near zero when locked (core PLL requirement)
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "D-axis error when locked", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d()); }, 0.00f, 0.01f,
        yeet::testing::checker_activation_time_range{0.3}));

    // 5. Q-axis should track input signal amplitude (unity for this test)
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "Q-axis amplitude tracking", [&]() -> real_t { return std::abs(pll.get_voltage_dq().q()); }, 0.99f, 1.01f,
        yeet::testing::checker_activation_time_range{0.3}));

    // Run the simulation for 1 second and check the results
    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(ThreePhasePLLTest, AmplitudeStep) {
    // Test immunity to amplitude variations (amplitude step from 1.0 to 0.2 pu)
    waveform_generator.set_amplitude_step(0.2, 1.0);

    // 1. Frequency should remain stable despite amplitude change
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "frequency stability during amplitude step", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.3}));

    // 2. Should achieve lock before amplitude change
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked before amplitude step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.5, 0.95}));

    // 3. Should maintain lock after amplitude change
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked after amplitude step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{1.1}));

    // 4. D-axis should reflect the new amplitude after step
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "q-axis amplitude after step", [&]() -> real_t { return std::abs(pll.get_voltage_dq().q()); }, 1.18f,
        1.22f,  // Should track new amplitude (0.2 pu)
        yeet::testing::checker_activation_time_range{1.1}));

    // 5. d-axis should remain small despite amplitude change
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "D-axis stability during amplitude step", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d()); }, 0.0f,
        0.01f, yeet::testing::checker_activation_time_range{1.1}));

    EXPECT_TRUE(simulator.simulate_for(2.0f));
}

TEST_F(ThreePhasePLLTest, ExtremeFrequencyConditions) {
    // Test PLL behavior at frequency limits (45 Hz - near minimum operational range)
    waveform_generator.set_angle(angle<zero_to_two_pi>(pi / 2.0));
    waveform_generator.set_signal_frequency(45.0);  // Near lower operational limit

    // 1. Should track low frequency within operational range
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "low frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency(); }, 44.5f, 45.5f,
        yeet::testing::checker_activation_time_range{0.4}));

    // 2. Should achieve lock even at extreme frequency
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "lock at extreme frequency", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.4}));

    // 3. D-axis regulation should work at extreme frequency
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "d-axis regulation at extreme frequency", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d()); }, 0.0f,
        0.02f, yeet::testing::checker_activation_time_range{0.4}));

    // 4. Signal reconstruction accuracy at extreme frequency
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "reconstruction at extreme frequency", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f,
        2e-3f, yeet::testing::checker_activation_time_range{0.4}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(ThreePhasePLLTest, HighFrequencyTracking) {
    // Test PLL behavior at higher frequency (60 Hz - typical US grid)
    waveform_generator.set_angle(angle<zero_to_two_pi>(pi / 2.0));
    waveform_generator.set_signal_frequency(60.0);  // Higher frequency operation

    // 1. Should track high frequency accurately
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "high frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency(); }, 59.8f, 60.2f,
        yeet::testing::checker_activation_time_range{0.4}));

    // 2. Should achieve lock at high frequency
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "lock at high frequency", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.4}));

    // 3. Performance metrics should be maintained
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "d-axis regulation at high frequency", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d()); }, 0.0f,
        0.015f, yeet::testing::checker_activation_time_range{0.4}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(ThreePhasePLLTest, FrequencyStep) {
    // Test tracking of frequency step from 50 Hz to 55 Hz
    waveform_generator.set_angle(angle<zero_to_two_pi>(pi / 2.0));
    waveform_generator.set_frequency_step(5.0f, 1.0f);

    const real_t target_frequency = kNominalFrequency + 5.0f;

    // 1. Initially should track nominal frequency
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "initial frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.3, 0.95}));

    // 2. Should be locked before frequency step
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked before step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.5, 0.95}));

    // 3. After step, should track new frequency (allowing for transient)
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "new frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency(); },
        target_frequency - kFrequencyTolerance, target_frequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{1.3}));

    // 4. Should regain lock after transient settles
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked after step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{1.4}));

    // 5. Signal reconstruction should be accurate after settling
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "reconstruction accuracy after step", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 1e-3f,
        yeet::testing::checker_activation_time_range{1.4}));

    // 6. D-axis error should be small when locked at new frequency
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "d-axis error at new frequency", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d()); }, 0.0f, 0.01f,
        yeet::testing::checker_activation_time_range{1.3}));

    EXPECT_TRUE(simulator.simulate_for(2.0f));
}

TEST_F(ThreePhasePLLTest, FrequencyRateOfChange) {
    // Test tracking during frequency ramp from 50 Hz to 55 Hz over 0.5 seconds
    waveform_generator.set_angle(angle<zero_to_two_pi>(pi / 2.0));
    waveform_generator.set_frequency_rate_of_change(5.0f, 1.0f, 1.5f);

    // 1. Initially should track nominal frequency
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "initial frequency during ramp", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.3, 0.95}));

    // 2. During ramp, frequency should track within reasonable bounds
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "frequency during ramp", [&]() -> real_t { return pll.get_estimated_frequency(); }, kNominalFrequency - 1.0f,
        kNominalFrequency + 6.0f,  // Allow wider bounds during ramp
        yeet::testing::checker_activation_time_range{1.0, 1.5}));

    // 3. After ramp completion, should settle to final frequency
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "final frequency after ramp", [&]() -> real_t { return pll.get_estimated_frequency(); }, 54.5f,
        55.5f,  // Final frequency should be 55 Hz ±0.5 Hz
        yeet::testing::checker_activation_time_range{1.7}));

    // 4. Should regain tight lock after ramp settles
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked after ramp", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{1.7}));

    // 5. Reconstruction accuracy should be maintained throughout
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "reconstruction accuracy during ramp", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f,
        5e-3f,  // Slightly relaxed during dynamic operation
        yeet::testing::checker_activation_time_range{1.0}));

    EXPECT_TRUE(simulator.simulate_for(2.0f));
}

TEST_F(ThreePhasePLLTest, AngleDisturbanceStep) {
    // Test phase disturbance rejection capability
    waveform_generator.set_angle(angle<zero_to_two_pi>(pi / 2.0));

    // 1. Should achieve lock before disturbance
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked before disturbance", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.5, 0.95}));

    // 2. Frequency should remain stable throughout (disturbance is phase-only)
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "frequency stability during disturbance", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency * 0.9f, kNominalFrequency * 1.1f, yeet::testing::checker_activation_time_range{0.3}));

    // 3. D-axis should show transient response but then settle
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "d-axis settling after disturbance", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d()); }, 0.0f,
        0.02f, yeet::testing::checker_activation_time_range{1.3}));

    // 4. Should regain lock after phase disturbance settles
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked after disturbance", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{1.4}));

    // 5. Signal reconstruction accuracy should be restored
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "reconstruction after disturbance", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 1e-3f,
        yeet::testing::checker_activation_time_range{1.4}));

    // Execute the test scenario
    EXPECT_TRUE(simulator.simulate_for(1.0f));
    angle<zero_to_two_pi> current_angle = waveform_generator.get_signal_abc().to_alphabeta().phase();
    waveform_generator.set_angle(current_angle + angle<zero_to_two_pi>(pi / 4.0));  // 45 degrees phase shift
    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(ThreePhasePLLTest, SignalWithNoise) {
    // Test noise rejection capability (5% noise)
    waveform_generator.set_angle(angle<zero_to_two_pi>(pi / 2.0));
    waveform_generator.enable_noise(0.01f);  // 5% noise level

    // 1. Should still achieve frequency lock despite noise
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "frequency tracking with noise", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - 0.01f, kNominalFrequency + 0.01f,  // Slightly relaxed tolerance for noise
        yeet::testing::checker_activation_time_range{0.4}));

    // 2. Should achieve lock despite noise (may take longer)
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "lock achievement with noise", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.5}));

    // 3. D-axis error should remain bounded despite noise
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "bounded d-axis error with noise", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d()); }, 0.0f,
        0.05f,  // Allow higher error due to noise
        yeet::testing::checker_activation_time_range{0.4}));

    // 4. Reconstruction should still be reasonable despite noise
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "reconstruction with noise", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f,
        0.05f,  // Relaxed due to noise impact on both amplitude and phase
        yeet::testing::checker_activation_time_range{0.4}));

    // 5. Q-axis amplitude should still track signal despite noise
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "Q-axis tracking with noise", [&]() -> real_t { return std::abs(pll.get_voltage_dq().q()); }, 0.9f,
        1.1f,  // Relaxed bounds for noisy conditions
        yeet::testing::checker_activation_time_range{0.4}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

}  // namespace