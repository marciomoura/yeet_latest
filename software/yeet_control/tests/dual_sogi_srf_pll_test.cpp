
#include "yeet_control/dual_sogi_srf_pll.hpp"

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

class DualSogiPhaseLockedLoopTest : public ::testing::Test {
protected:
    static constexpr duration_t kSamplingTime = duration_t{50e-6};        // 50 μs for good time resolution
    static constexpr frequency_t kNominalFrequency = frequency_t{50.0};   // 50 Hz nominal frequency
    static constexpr frequency_t kFrequencyTolerance = frequency_t{0.1};  // 0.1 Hz
    static constexpr real_t kAngleTolerance = real_t{0.05};               // radians (~3 degrees)

    dual_sogi_srf_pll pll{kSamplingTime};
    three_phase_waveform_generator waveform_generator{kSamplingTime.value()};
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    real_t pll_input_dc_value = {};
    alphabeta<voltage_pu_t> pll_input{};

    void SetUp() override
    {
        // Configure PLL for testing
        pll.configure_nominal_frequency(kNominalFrequency);
        pll.configure_pi_controller(real_t{0.2}, duration_t{0.05});
        pll.configure_frequency_limits(frequency_t{10.0f}, frequency_t{100.0f});
        pll.configure_sequence_extractor(1.414, duration_t{1.0 / 50.0},
            true);  // SOGI gain and DC rejection time constant
        pll.reset();

        // Configure the three-phase waveform generator for reference signal generation
        waveform_generator.set_fundamental_positive_sequence_signal_amplitude(real_t{1.0});
        waveform_generator.set_fundamental_negative_sequence_signal_amplitude(real_t{0.0});
        waveform_generator.set_signal_frequency(kNominalFrequency);
        waveform_generator.disable_noise();

        set_logger();

        simulator.register_lambda_runner(static_cast<double>(kSamplingTime.value()), [&]() {
            waveform_generator.update();

            abc<double> const signal = waveform_generator.get_signal_abc();
            alphabeta<double> signal_ab = signal.to_alphabeta();
            pll_input = alphabeta<voltage_pu_t>{voltage_pu_t{signal_ab.alpha() + pll_input_dc_value},
                voltage_pu_t{signal_ab.beta() + pll_input_dc_value}};

            // Update PLL with the reference signal
            pll.update(pll_input);
        });
    }

    real_t get_estimation_error_magnitude() const
    {
        // Calculate the magnitude of the error between the estimated and actual signal
        alphabeta<voltage_pu_t> estimated_signal = pll.get_positive_sequence();
        alphabeta<double> reference_signal = waveform_generator.get_signal_abc().to_alphabeta();

        alphabeta<real_t> error = alphabeta<real_t>{reference_signal.alpha() - estimated_signal.alpha().value(),
            reference_signal.beta() - estimated_signal.beta().value()};
        return std::sqrt(error.alpha() * error.alpha() + error.beta() * error.beta());
    }

    void set_logger()
    {
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "time", [&]() -> double { return simulator.get_current_simulation_time(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<>>(
            "waveform generator", [&]() -> alphabeta<> { return waveform_generator.get_signal_abc().to_alphabeta(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<angle_pu_t>(
            "waveform angle pu", [&]() { return waveform_generator.get_fundamental_signal_angle().a().get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>("waveform frequency",
            [&]() -> real_t { return waveform_generator.get_signal_frequency() / kNominalFrequency.value(); }));

        simulator.register_sampler_entry(
            yeet::testing::make_lambda_array_sampler<alphabeta<>>("pll input", [&]() -> alphabeta<> {
                return alphabeta<>{
                    pll.get_positive_sequence().alpha().value(), pll.get_positive_sequence().beta().value()};
            }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "estimation error magnitude", [&]() -> real_t { return get_estimation_error_magnitude(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "estimation error d-axis", [&]() -> real_t { return pll.get_estimation_error().value(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<angle_pu_t>(
            "estimated angle pu", [&]() { return pll.get_estimated_angle().get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>("estimated frequency pu",
            [&]() -> real_t { return pll.get_estimated_frequency().value() / kNominalFrequency.value(); }));

        simulator.register_sampler_entry(
            yeet::testing::make_lambda_sampler<bool>("is locked", [&]() -> bool { return pll.is_locked(); }));
    }
};

TEST_F(DualSogiPhaseLockedLoopTest, NominalFrequencyTracking)
{
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));  // 90 degrees phase shift

    // 1. Frequency estimation accuracy after settling
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "estimated frequency range", [&]() -> real_t { return pll.get_estimated_frequency().value(); },
        (kNominalFrequency - kFrequencyTolerance).value(), (kNominalFrequency + kFrequencyTolerance).value(),
        yeet::testing::checker_activation_time_range{0.4}));

    // 2. Signal reconstruction accuracy (fundamental PLL requirement)
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "estimation error magnitude", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 0.005f,
        yeet::testing::checker_activation_time_range{0.4}));

    // 3. Lock detection must be active after settling time
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "pll lock status", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.5}));
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "initial unlocked state", [&]() -> bool { return pll.is_locked(); }, false,
        yeet::testing::checker_activation_time_range{0.0, 0.1}));

    // 4. D-axis error should be near zero when locked (core PLL requirement)
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "d-axis error when locked", [&]() -> real_t { return std::abs(pll.get_estimation_error().value()); }, 0.00f,
        0.02f, yeet::testing::checker_activation_time_range{0.4}));

    // Run the simulation for 1 second and check the results
    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(DualSogiPhaseLockedLoopTest, NegativeSequenceRejection)
{
    // Test immunity to negative sequence components
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));
    waveform_generator.set_fundamental_negative_sequence_signal_amplitude(real_t{0.2});  // 20% negative sequence

    // 1. Frequency should remain stable despite negative sequence
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "frequency stability with negative sequence", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.4}));

    // 2. Should achieve lock despite negative sequence
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked with negative sequence", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.6}));

    // 3. D-axis error should be bounded despite negative sequence
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "d-axis error with negative sequence", [&]() -> real_t { return std::abs(pll.get_estimation_error().value()); },
        0.0f, 0.05f, yeet::testing::checker_activation_time_range{0.5}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(DualSogiPhaseLockedLoopTest, AmplitudeStep)
{
    // Test immunity to amplitude variations (amplitude step from 1.0 to 0.2 pu)
    waveform_generator.set_amplitude_step(real_t{0.2}, 0.5);

    // 1. Frequency should remain stable despite amplitude change
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "frequency stability during amplitude step", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.4}));

    // 2. Should achieve lock before amplitude change
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked before amplitude step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.4, 0.45}));

    // 3. Should maintain lock after amplitude change
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked after amplitude step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.7}));

    // 4. Q-axis should remain bounded despite amplitude change
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "D-axis stability during amplitude step",
        [&]() -> real_t { return std::abs(pll.get_estimation_error().value()); }, 0.0f, 0.02f,
        yeet::testing::checker_activation_time_range{0.7}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(DualSogiPhaseLockedLoopTest, FrequencyStep)
{
    // Test tracking of frequency step from 50 Hz to 51 Hz
    waveform_generator.set_angle(angle_wrapped(pi / 2.0));
    waveform_generator.set_frequency_step(real_t{1.0}, 0.5);

    const real_t target_frequency = kNominalFrequency + 1.0f;

    // 1. Initially should track nominal frequency
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "initial frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.3, 0.45}));

    // 2. Should be locked before frequency step
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked before step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.4, 0.45}));

    // 3. After step, should track new frequency (allowing for transient)
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "new frequency tracking", [&]() -> real_t { return pll.get_estimated_frequency(); },
        target_frequency - kFrequencyTolerance, target_frequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.8}));

    // 4. Should regain lock after transient settles
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked after step", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.9}));

    EXPECT_TRUE(simulator.simulate_for(1.5f));
}

TEST_F(DualSogiPhaseLockedLoopTest, HarmonicsAndNoise)
{
    // Test filtering capability with harmonics and noise
    waveform_generator.add_signal_harmonics(3, 0.1);   // Add 3rd harmonic with 10% amplitude
    waveform_generator.add_signal_harmonics(5, 0.05);  // Add 5th harmonic with 5% amplitude
    waveform_generator.add_signal_harmonics(7, 0.02);  // Add 7th harmonic with 2% amplitude
    waveform_generator.set_fundamental_negative_sequence_signal_amplitude(real_t{0.1});
    waveform_generator.enable_noise(0.05);  // Enable noise with 5% standard deviation

    // 1. Should still achieve frequency lock despite distortions
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "frequency tracking with harmonics and noise", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.6}));

    // 2. Should achieve lock despite distortions (may take longer)
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "lock achievement with distortions", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.7}));

    // 3. Q-axis error should remain bounded despite distortions
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "bounded d-axis error with distortions",
        [&]() -> real_t { return std::abs(pll.get_estimation_error().value()); }, 0.0f, 0.1f,
        yeet::testing::checker_activation_time_range{0.6}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(DualSogiPhaseLockedLoopTest, FrequencyRateOfChange)
{
    // Test tracking during frequency ramp from 50 Hz to 60 Hz over 0.8 seconds
    simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>("signal frequency reference",
        [&]() -> double { return static_cast<double>(waveform_generator.get_signal_frequency() / 50.0f); }));

    waveform_generator.set_frequency_rate_of_change(real_t{10.0}, 0.4, 1.2);

    // 1. Initially should track nominal frequency
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "initial frequency during ramp", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.3, 0.35}));

    // 2. During ramp, frequency should track within reasonable bounds
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "frequency during ramp", [&]() -> real_t { return pll.get_estimated_frequency(); }, kNominalFrequency - 2.0f,
        kNominalFrequency + 12.0f, yeet::testing::checker_activation_time_range{0.5, 1.1}));

    // 3. After ramp completion, should settle to final frequency
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "final frequency after ramp", [&]() -> real_t { return pll.get_estimated_frequency(); }, 59.5f, 60.5f,
        yeet::testing::checker_activation_time_range{1.4}));

    // 4. Should regain tight lock after ramp settles
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked after ramp", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{1.5}));

    EXPECT_TRUE(simulator.simulate_for(2.0f));
}

TEST_F(DualSogiPhaseLockedLoopTest, DCOffsetRejection)
{
    // Test DC offset rejection capability

    // 1. Should achieve lock before DC offset
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked before DC offset", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.4, 0.45}));

    // 2. Frequency should remain stable throughout (DC offset should be rejected)
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "frequency stability during DC offset", [&]() -> real_t { return pll.get_estimated_frequency(); },
        kNominalFrequency - kFrequencyTolerance, kNominalFrequency + kFrequencyTolerance,
        yeet::testing::checker_activation_time_range{0.3}));

    // 3. Should maintain lock after DC offset introduction
    simulator.register_checker(yeet::testing::make_equal_checker<bool>(
        "locked after DC offset", [&]() -> bool { return pll.is_locked(); }, true,
        yeet::testing::checker_activation_time_range{0.8}));

    // Execute the test scenario
    EXPECT_TRUE(simulator.simulate_for(0.5f));
    pll_input_dc_value = 0.05;  // Introduce a DC offset
    EXPECT_TRUE(simulator.simulate_for(0.8f));
}

}  // namespace
