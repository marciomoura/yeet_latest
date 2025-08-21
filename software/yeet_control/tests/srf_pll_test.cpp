#include "yeet_control/srf_pll.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class SrfPllTest : public ::testing::Test {
protected:
    static constexpr duration_t kSamplingTime = duration_t{1e-4};     // 100 µs sampling time (10 kHz)
    static constexpr frequency_t kTestFrequency = frequency_t{50.0};  // 50 Hz test frequency
    static constexpr real_t kTestAngularFrequency = 2.0 * pi * kTestFrequency.value();
    static constexpr real_t kFrequencyTolerance = 2.0 * pi * 0.1;  // 0.1 Hz tolerance in rad/s
    static constexpr real_t kAngleTolerance = 0.05;                // radians (~3 degrees)
    static constexpr real_t kAmplitude = 1.0;

    srf_pll pll{kSamplingTime};
    three_phase_waveform_generator waveform_generator{kSamplingTime.value()};
    yeet::simulator simulator = yeet::make_gtest_simulator();

    void SetUp() override
    {
        // Configure a stable PI controller for the tests
        pll.reset(frequency_pu_t{1.0});
        pll.configure_nominal_frequency(kTestFrequency);
        pll.configure_pi_controller(1, duration_t{0.1});

        // Default waveform generator configuration
        waveform_generator.set_fundamental_positive_sequence_signal_amplitude(kAmplitude);
        waveform_generator.set_signal_frequency(kTestFrequency.value());

        // Simulator setup
        simulator.register_lambda_runner(kSamplingTime.value(), [&]() {
            waveform_generator.update();

            const alphabeta<voltage_pu_t> signal_ab =
                to_dimension_alphabeta<voltage_pu_t>(waveform_generator.get_signal_abc().to_alphabeta());
            pll.update(signal_ab);
        });
        set_logger();
    }

    real_t get_angle_error() const
    {
        // Added a small term to force wrapping around 2pi
        return std::abs(0.01 - (waveform_generator.get_fundamental_signal_angle().a() - pll.get_estimated_angle() +
                                   angle_wrapped{0.01})
                                   .get_pu());
    }

    void set_logger()
    {
        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "time", [&]() -> double { return simulator.get_current_simulation_time(); }));

        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<alphabeta<>>(
            "waveform generator", [&]() -> alphabeta<> { return waveform_generator.get_signal_abc().to_alphabeta(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<angle_pu_t>(
            "waveform angle pu", [&]() { return waveform_generator.get_fundamental_signal_angle().a().get_pu(); }));

        simulator.register_sampler_entry(
            yeet::make_lambda_sampler<real_t>("angle error", [&]() -> real_t { return get_angle_error(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("waveform frequency",
            [&]() -> real_t { return waveform_generator.get_signal_frequency() / kTestFrequency.value(); }));

        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<dq<>>(
            "estimated dq", [&]() -> dq<> { return dq<>{pll.get_dq_vector().d(), pll.get_dq_vector().q()}; }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<angle_pu_t>(
            "estimated angle pu", [&]() { return pll.get_estimated_angle().get_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
            "estimated frequency pu", [&]() -> real_t { return pll.get_estimated_frequency_pu(); }));
    }
};

TEST_F(SrfPllTest, LocksToNominalFrequency)
{
    // Check if the PLL can lock onto a stable signal at the test frequency.

    // 1. Frequency estimation should converge to the test frequency.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "estimated frequency", [&]() { return pll.get_estimated_angular_frequency(); },
        kTestAngularFrequency - kFrequencyTolerance, kTestAngularFrequency + kFrequencyTolerance,
        yeet::checker_activation_time_range{0.3}));

    // 2. Angle estimation should track the source angle.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "angle error", [&]() { return get_angle_error(); }, 0.0, kAngleTolerance,
        yeet::checker_activation_time_range{0.3}));

    // 3. The q-axis component (error signal) should be driven to zero.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "q-axis error", [&]() { return pll.get_dq_vector().q(); }, -0.01, 0.01,
        yeet::checker_activation_time_range{0.3}));

    // 4. The d-axis component should represent the signal amplitude when locked.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "d-axis amplitude", [&]() { return pll.get_dq_vector().d(); }, kAmplitude - 0.01, kAmplitude + 0.01,
        yeet::checker_activation_time_range{0.3}));

    EXPECT_TRUE(simulator.simulate_for(0.5));
}

TEST_F(SrfPllTest, HandlesFrequencyStep)
{
    // Test the PLL's dynamic response to a sudden change in frequency.
    const real_t step_frequency = 60.0;
    const real_t step_angular_frequency = 2.0 * pi * step_frequency;
    waveform_generator.set_frequency_step(step_frequency - kTestFrequency.value(), 0.5);

    // 1. Check lock at the initial frequency.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "initial frequency", [&]() { return pll.get_estimated_angular_frequency(); },
        kTestAngularFrequency - kFrequencyTolerance, kTestAngularFrequency + kFrequencyTolerance,
        yeet::checker_activation_time_range{0.3, 0.45}));

    // 2. After the step, check lock at the new frequency.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "stepped frequency", [&]() { return pll.get_estimated_angular_frequency(); },
        step_angular_frequency - kFrequencyTolerance, step_angular_frequency + kFrequencyTolerance,
        yeet::checker_activation_time_range{0.8}));

    // 3. The q-axis error should be small after settling at the new frequency.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "q-axis error after step", [&]() { return pll.get_dq_vector().q(); }, -0.01, 0.01,
        yeet::checker_activation_time_range{0.8}));

    EXPECT_TRUE(simulator.simulate_for(1.0));
}

TEST_F(SrfPllTest, HandlesPhaseStep)
{
    // Test the PLL's ability to recover from a sudden phase jump.
    const real_t phase_jump = pi / 4.0;  // 45-degree jump

    // 1. Check that frequency remains stable before the jump.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "frequency before jump", [&]() { return pll.get_estimated_angular_frequency(); },
        kTestAngularFrequency - kFrequencyTolerance, kTestAngularFrequency + kFrequencyTolerance,
        yeet::checker_activation_time_range{0.3, 0.45}));

    // Simulate for 0.5s, then apply the phase jump.
    ASSERT_TRUE(simulator.simulate_for(0.5));
    waveform_generator.set_angle(waveform_generator.get_fundamental_signal_angle().a() + angle_wrapped{phase_jump});

    // 2. Check that the PLL re-locks and frequency returns to nominal.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "frequency after jump", [&]() { return pll.get_estimated_angular_frequency(); },
        kTestAngularFrequency - kFrequencyTolerance, kTestAngularFrequency + kFrequencyTolerance,
        yeet::checker_activation_time_range{0.8}));

    // 3. Check that the angle error is corrected after the jump.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "angle error after jump", [&]() { return get_angle_error(); }, 0.0, kAngleTolerance,
        yeet::checker_activation_time_range{0.8}));

    EXPECT_TRUE(simulator.simulate_for(0.5));
}

TEST_F(SrfPllTest, HandlesAmplitudeStep)
{
    // Test the PLL's immunity to amplitude changes.
    const real_t amplitude_step = 0.5;
    waveform_generator.set_amplitude_step(amplitude_step, 0.5);

    // 1. Frequency estimation should remain stable across the amplitude step.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "frequency stability", [&]() { return pll.get_estimated_angular_frequency(); },
        kTestAngularFrequency - kFrequencyTolerance, kTestAngularFrequency + kFrequencyTolerance,
        yeet::checker_activation_time_range{0.8}));

    // 2. The d-axis should track the new amplitude after the step.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "d-axis after step", [&]() { return pll.get_dq_vector().d(); }, kAmplitude + amplitude_step - 0.01,
        kAmplitude + amplitude_step + 0.01, yeet::checker_activation_time_range{0.8}));

    // 3. The q-axis error should remain small.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "q-axis error after step", [&]() { return pll.get_dq_vector().q(); }, -0.01, 0.01,
        yeet::checker_activation_time_range{0.8}));

    EXPECT_TRUE(simulator.simulate_for(1.0));
}

TEST_F(SrfPllTest, TracksFrequencyRamp)
{
    // Test if the PLL can follow a frequency ramp from near-zero to nominal.
    const frequency_t start_frequency = frequency_t{0.1};
    const real_t ramp_start_time = 0.1;
    const real_t ramp_duration = 2.0;
    const real_t ramp_end_time = ramp_start_time + ramp_duration;
    const real_t frequency_delta = kTestFrequency.value() - start_frequency.value();

    // Initialize PLL and generator at a very low frequency.
    pll.reset(frequency_pu_t{});
    waveform_generator.set_signal_frequency(start_frequency.value());

    // Configure the frequency ramp.
    waveform_generator.set_frequency_rate_of_change(frequency_delta, ramp_start_time, ramp_end_time);

    // Check that the frequency has settled at the nominal value after the ramp.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "final frequency", [&]() { return pll.get_estimated_angular_frequency(); },
        kTestAngularFrequency - kFrequencyTolerance, kTestAngularFrequency + kFrequencyTolerance,
        yeet::checker_activation_time_range{ramp_end_time + 0.3}));

    // Check that the angle error is small after settling.
    simulator.register_checker(yeet::make_range_checker<real_t>(
        "final angle error", [&]() { return get_angle_error(); }, 0.0, kAngleTolerance,
        yeet::checker_activation_time_range{ramp_end_time + 0.3}));

    EXPECT_TRUE(simulator.simulate_for(ramp_end_time + 0.5));
}

}  // namespace