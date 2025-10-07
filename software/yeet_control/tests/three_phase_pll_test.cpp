#include "yeet_control/three_phase_pll.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class ThreePhasePLLTest : public ::testing::Test {
protected:
    static constexpr duration_t kSamplingTime = duration_t{1e-4};         // 100 µs sampling time (10 kHz)
    static constexpr frequency_t kNominalFrequency = frequency_t{50.0};   // 50 Hz nominal frequency
    static constexpr frequency_t kFrequencyTolerance = frequency_t{0.1};  // 0.1 Hz
    static constexpr real_t kAngleTolerance = real_t{0.05};               // radians (~3 degrees)

    three_phase_pll pll{kSamplingTime};
    three_phase_waveform_generator waveform_generator{kSamplingTime.value()};
    yeet::simulator simulator = yeet::make_gtest_simulator();

    void SetUp() override
    {
        // Default PLL configuration
        pll.configure_pi_controller(real_t{0.2}, duration_t{0.05});
        pll.configure_nominal_frequency(kNominalFrequency);
        pll.configure_frequency_limits(frequency_t{30.0}, frequency_t{70.0});

        // Default waveform generator configuration
        waveform_generator.set_fundamental_positive_sequence_signal_amplitude(1.0);
        waveform_generator.set_signal_frequency(static_cast<real_t>(kNominalFrequency.value()));

        // Simulator setup
        simulator.register_lambda_runner(static_cast<real_t>(kSamplingTime.value()), [&]() {
            // Execute one simulation step
            waveform_generator.update();

            alphabeta<real_t> const signal_ab = waveform_generator.get_signal_abc().to_alphabeta();
            alphabeta<voltage_pu_t> const pll_input{voltage_pu_t{signal_ab.alpha()}, voltage_pu_t{signal_ab.beta()}};

            pll.update(pll_input);
        });

        set_logger();
    }

    real_t get_estimation_error_magnitude() const
    {
        // Get the original input signal in the stationary frame
        alphabeta<real_t> const signal_ab = waveform_generator.get_signal_abc().to_alphabeta();
        alphabeta<real_t> const pll_input{signal_ab.alpha(), signal_ab.beta()};

        // Reconstruct the estimated signal in the stationary frame from the PLL's internal dq frame
        alphabeta<voltage_pu_t> const estimated_alphabeta =
            pll.get_voltage_dq().to_alphabeta(pll.get_estimated_angle());
        alphabeta<real_t> const estimated{estimated_alphabeta.alpha().value(), estimated_alphabeta.beta().value()};

        // Return the magnitude of the error vector
        return (pll_input - estimated).magnitude();
    }

    void set_logger()
    {
        // Log signals in per-unit for comparable magnitudes
        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
            "time", [&]() -> real_t { return simulator.get_current_simulation_time(); }));

        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<alphabeta<>>(
            "waveform generator", [&]() -> alphabeta<> { return waveform_generator.get_signal_abc().to_alphabeta(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<angle_pu_t>(
            "waveform angle pu", [&]() { return waveform_generator.get_fundamental_signal_angle().a().get_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("waveform frequency",
            [&]() -> real_t { return waveform_generator.get_signal_frequency() / kNominalFrequency.value(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
            "estimation error magnitude", [&]() -> real_t { return get_estimation_error_magnitude(); }));

        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<dq<>>("estimated dq",
            [&]() -> dq<> { return dq<>{pll.get_voltage_dq().d().value(), pll.get_voltage_dq().q().value()}; }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<angle_pu_t>(
            "estimated angle pu", [&]() { return pll.get_estimated_angle().get_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("estimated frequency pu",
            [&]() -> real_t { return pll.get_estimated_frequency().value() / kNominalFrequency.value(); }));

        simulator.register_sampler_entry(
            yeet::make_lambda_sampler<bool>("is locked", [&]() -> bool { return pll.is_locked(); }));
    }
};

TEST_F(ThreePhasePLLTest, NominalFrequencyTracking)
{
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));  // 90 degrees phase shift

    // 1. Frequency estimation accuracy after settling
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "estimated frequency range", [&]() -> real_t { return pll.get_estimated_frequency().value(); },
        (kNominalFrequency - kFrequencyTolerance).value(), (kNominalFrequency + kFrequencyTolerance).value(),
        yeet::checker_activation_time_range{0.3}));

    // 2. Signal reconstruction accuracy (fundamental PLL requirement)
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "estimation error magnitude", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 1e-3f,
        yeet::checker_activation_time_range{0.3}));

    // 3. Lock detection must be active after settling time
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "pll lock status", [&]() -> bool { return pll.is_locked(); }, true, yeet::checker_activation_time_range{0.4}));
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "initial unlocked state", [&]() -> bool { return pll.is_locked(); }, false,
        yeet::checker_activation_time_range{0.0, 0.1}));

    // 4. D-axis error should be near zero when locked (core PLL requirement)
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "D-axis error when locked", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d().value()); }, 0.00f,
        0.01f, yeet::checker_activation_time_range{0.3}));

    // 5. Q-axis should track input signal amplitude (unity for this test)
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "Q-axis amplitude tracking", [&]() -> real_t { return std::abs(pll.get_voltage_dq().q().value()); }, 0.99f,
        1.01f, yeet::checker_activation_time_range{0.3}));

    // Run the simulation for 1 second and check the results
    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(ThreePhasePLLTest, AmplitudeStep)
{
    // Test immunity to amplitude variations (amplitude step from 1.0 to 0.2 pu)
    waveform_generator.set_amplitude_step(0.2, 1.0);

    // 1. Frequency should remain stable despite amplitude change
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "frequency stability during amplitude step", [&]() -> real_t { return pll.get_estimated_frequency().value(); },
        (kNominalFrequency - kFrequencyTolerance).value(), (kNominalFrequency + kFrequencyTolerance).value(),
        yeet::checker_activation_time_range{0.3}));

    // 2. Should achieve lock before amplitude change
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "locked before amplitude step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{0.5, 0.95}));

    // 3. Should maintain lock after amplitude change
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "locked after amplitude step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{1.1}));

    // 4. D-axis should reflect the new amplitude after step
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "q-axis amplitude after step", [&]() -> real_t { return std::abs(pll.get_voltage_dq().q().value()); }, 1.18f,
        1.22f,  // Should track new amplitude (0.2 pu)
        yeet::checker_activation_time_range{1.1}));

    // 5. d-axis should remain small despite amplitude change
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "D-axis stability during amplitude step",
        [&]() -> real_t { return std::abs(pll.get_voltage_dq().d().value()); }, 0.0f, 0.01f,
        yeet::checker_activation_time_range{1.1}));

    EXPECT_TRUE(simulator.simulate_for(2.0f));
}

TEST_F(ThreePhasePLLTest, ExtremeFrequencyConditions)
{
    // Test PLL behavior at frequency limits (45 Hz - near minimum operational range)
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));
    waveform_generator.set_signal_frequency(45.0);  // Near lower operational limit

    // 1. Should track low frequency within operational range
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "low frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency().value(); }, 44.5f, 45.5f,
        yeet::checker_activation_time_range{0.4}));

    // 2. Should achieve lock even at extreme frequency
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "lock at extreme frequency", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{0.4}));

    // 3. D-axis regulation should work at extreme frequency
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "d-axis regulation at extreme frequency",
        [&]() -> real_t { return std::abs(pll.get_voltage_dq().d().value()); }, 0.0f, 0.02f,
        yeet::checker_activation_time_range{0.4}));

    // 4. Signal reconstruction accuracy at extreme frequency
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "reconstruction at extreme frequency", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f,
        2e-3f, yeet::checker_activation_time_range{0.4}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(ThreePhasePLLTest, HighFrequencyTracking)
{
    // Test PLL behavior at higher frequency (60 Hz - typical US grid)
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));
    waveform_generator.set_signal_frequency(60.0);  // Higher frequency operation

    // 1. Should track high frequency accurately
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "high frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency().value(); }, 59.8f, 60.2f,
        yeet::checker_activation_time_range{0.4}));

    // 2. Should achieve lock at high frequency
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "lock at high frequency", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{0.4}));

    // 3. Performance metrics should be maintained
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "d-axis regulation at high frequency", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d().value()); },
        0.0f, 0.015f, yeet::checker_activation_time_range{0.4}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(ThreePhasePLLTest, FrequencyStep)
{
    // Test tracking of frequency step from 50 Hz to 55 Hz
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));
    waveform_generator.set_frequency_step(5.0f, 1.0f);

    const real_t target_frequency = kNominalFrequency.value() + 5.0f;

    // 1. Initially should track nominal frequency
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "initial frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency().value(); },
        (kNominalFrequency - kFrequencyTolerance).value(), (kNominalFrequency + kFrequencyTolerance).value(),
        yeet::checker_activation_time_range{0.3, 0.95}));

    // 2. Should be locked before frequency step
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "locked before step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{0.5, 0.95}));

    // 3. After step, should track new frequency (allowing for transient)
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "new frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency().value(); },
        target_frequency - kFrequencyTolerance.value(), target_frequency + kFrequencyTolerance.value(),
        yeet::checker_activation_time_range{1.3}));

    // 4. Should regain lock after transient settles
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "locked after step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{1.4}));

    // 5. Signal reconstruction should be accurate after settling
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "reconstruction accuracy after step", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 1e-3f,
        yeet::checker_activation_time_range{1.4}));

    // 6. D-axis error should be small when locked at new frequency
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "d-axis error at new frequency", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d().value()); }, 0.0f,
        0.01f, yeet::checker_activation_time_range{1.3}));

    EXPECT_TRUE(simulator.simulate_for(2.0f));
}

TEST_F(ThreePhasePLLTest, FrequencyRateOfChange)
{
    // Test tracking during frequency ramp from 50 Hz to 55 Hz over 0.5 seconds
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));
    waveform_generator.set_frequency_rate_of_change(5.0f, 1.0f, 1.5f);

    // 1. Initially should track nominal frequency
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "initial frequency during ramp", [&]() -> real_t { return pll.get_estimated_frequency().value(); },
        (kNominalFrequency - kFrequencyTolerance).value(), (kNominalFrequency + kFrequencyTolerance).value(),
        yeet::checker_activation_time_range{0.3, 0.95}));

    // 2. During ramp, frequency should track within reasonable bounds
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "frequency during ramp", [&]() -> real_t { return pll.get_estimated_frequency().value(); },
        kNominalFrequency.value() - 1.0f,
        kNominalFrequency.value() + 6.0f,  // Allow wider bounds during ramp
        yeet::checker_activation_time_range{1.0, 1.5}));

    // 3. After ramp completion, should settle to final frequency
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "final frequency after ramp", [&]() -> real_t { return pll.get_estimated_frequency().value(); }, 54.5f,
        55.5f,  // Final frequency should be 55 Hz ±0.5 Hz
        yeet::checker_activation_time_range{1.7}));

    // 4. Should regain tight lock after ramp settles
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "locked after ramp", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{1.7}));

    // 5. Reconstruction accuracy should be maintained throughout
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "reconstruction accuracy during ramp", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f,
        5e-3f,  // Slightly relaxed during dynamic operation
        yeet::checker_activation_time_range{1.0}));

    EXPECT_TRUE(simulator.simulate_for(2.0f));
}

TEST_F(ThreePhasePLLTest, AngleDisturbanceStep)
{
    // Test phase disturbance rejection capability
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));

    // 1. Should achieve lock before disturbance
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "locked before disturbance", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{0.5, 0.95}));

    // 2. Frequency should remain stable throughout (disturbance is phase-only)
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "frequency stability during disturbance", [&]() -> real_t { return pll.get_estimated_frequency().value(); },
        kNominalFrequency.value() * 0.9f, kNominalFrequency.value() * 1.1f, yeet::checker_activation_time_range{0.3}));

    // 3. D-axis should show transient response but then settle
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "d-axis settling after disturbance", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d().value()); },
        0.0f, 0.02f, yeet::checker_activation_time_range{1.3}));

    // 4. Should regain lock after phase disturbance settles
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "locked after disturbance", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{1.4}));

    // 5. Signal reconstruction accuracy should be restored
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "reconstruction after disturbance", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 1e-3f,
        yeet::checker_activation_time_range{1.4}));

    // Execute the test scenario
    EXPECT_TRUE(simulator.simulate_for(1.0f));
    angle_wrapped current_angle = waveform_generator.get_signal_abc().to_alphabeta().phase();
    waveform_generator.set_angle(current_angle + angle_wrapped(pi / 4.0));  // 45 degrees phase shift
    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(ThreePhasePLLTest, SignalWithNoise)
{
    // Test noise rejection capability (5% noise)
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));
    waveform_generator.enable_noise(0.01f);  // 5% noise level

    // 1. Should still achieve frequency lock despite noise
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "frequency tracking with noise", [&]() -> real_t { return pll.get_estimated_frequency().value(); },
        kNominalFrequency.value() - 0.1f, kNominalFrequency.value() + 0.1f,  // Slightly relaxed tolerance for noise
        yeet::checker_activation_time_range{0.4}));

    // 2. Should achieve lock despite noise (may take longer)
    simulator.register_checker(yeet::make_equal_checker<bool>(
        "lock achievement with noise", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::checker_activation_time_range{0.5}));

    // 3. D-axis error should remain bounded despite noise
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "bounded d-axis error with noise", [&]() -> real_t { return std::abs(pll.get_voltage_dq().d().value()); }, 0.0f,
        0.05f,  // Allow higher error due to noise
        yeet::checker_activation_time_range{0.4}));

    // 4. Reconstruction should still be reasonable despite noise
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "reconstruction with noise", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f,
        0.05f,  // Relaxed due to noise impact on both amplitude and phase
        yeet::checker_activation_time_range{0.4}));

    // 5. Q-axis amplitude should still track signal despite noise
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "Q-axis tracking with noise", [&]() -> real_t { return std::abs(pll.get_voltage_dq().q().value()); }, 0.9f,
        1.1f,  // Relaxed bounds for noisy conditions
        yeet::checker_activation_time_range{0.4}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

}  // namespace