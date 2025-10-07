#include "yeet_control/sogi_filter.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;
using namespace yeet;

class SogiFilterTest : public ::testing::Test {
public:
    static constexpr duration_t sampling_time = duration_t{50e-6};                  // 50 μs for good time resolution
    static constexpr frequency_t nominal_frequency = frequency_t{50.0};             // 50 Hz nominal frequency
    static constexpr frequency_t frequency_tolerance = frequency_t{0.1};            // Frequency tolerance for tests
    static constexpr voltage_pu_t estimation_error_tolerance = voltage_pu_t{1e-3};  // 1mV estimation error tolerance
    static constexpr voltage_pu_t dc_rejection_tolerance = voltage_pu_t{1e-3};      // DC rejection tolerance

    yeet::control::sogi_filter sogi_filter_instance{sampling_time};
    three_phase_waveform_generator waveform_generator{sampling_time};
    yeet::simulator simulator = yeet::make_gtest_simulator();

    frequency_t sogi_input_frequency = nominal_frequency;
    voltage_pu_t sogi_input_dc_value = voltage_pu_t{0};

    void SetUp() override
    {
        // Configure SOGI filter for testing
        sogi_filter_instance.reset();
        sogi_filter_instance.configure(real_t{1.414});                         // sqrt(2) gain
        sogi_filter_instance.configure_dc_rejection(1.0 / nominal_frequency);  // One cycle time constant

        // Configure the three-phase waveform generator for reference signal generation
        waveform_generator.set_fundamental_positive_sequence_signal_amplitude(real_t{1.0});
        waveform_generator.set_fundamental_negative_sequence_signal_amplitude(real_t{0.0});
        waveform_generator.set_signal_frequency(nominal_frequency);
        waveform_generator.disable_noise();

        set_logger();

        simulator.register_lambda_runner(static_cast<double>(sampling_time), [&]() {
            waveform_generator.update();

            abc<real_t> const signal = waveform_generator.get_signal_abc();
            voltage_pu_t const sogi_signal_input = voltage_pu_t{signal.to_alphabeta().alpha()} + sogi_input_dc_value;
            angular_frequency_t const sogi_omega = two_pi * sogi_input_frequency;

            // Update SOGI filter with the reference signal
            sogi_filter_instance.update(sogi_signal_input, sogi_omega);
        });
    }

    voltage_pu_t get_estimation_error_magnitude() const
    {
        // Calculate the magnitude of the error between the estimated and reference signal
        alphabeta<real_t> const reference_signal = waveform_generator.get_signal_abc().to_alphabeta();
        alphabeta<voltage_pu_t> const estimated_signal = sogi_filter_instance.get_output();

        alphabeta<voltage_pu_t> const error =
            alphabeta<voltage_pu_t>{voltage_pu_t{reference_signal.alpha()}, voltage_pu_t{reference_signal.beta()}} -
            estimated_signal;
        return error.magnitude();
    }

private:
    void set_logger()
    {
        // Log time
        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "time", [&]() -> double { return simulator.get_current_simulation_time(); }));

        // Log reference signals
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<alphabeta<>>(
            "waveform generator", [&]() -> alphabeta<> { return waveform_generator.get_signal_abc().to_alphabeta(); }));

        // Log SOGI filter output
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(
            "sogi filter", [&]() -> alphabeta<voltage_pu_t> { return sogi_filter_instance.get_output(); }));

        // Log DC offset estimation
        simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(
            "sogi dc offset", [&]() -> voltage_pu_t { return sogi_filter_instance.get_dc_offset(); }));

        // Log error magnitude
        simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>(
            "estimation error magnitude", [&]() -> voltage_pu_t { return get_estimation_error_magnitude(); }));

        // Log input signal with DC offset
        simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>("input signal", [&]() -> voltage_pu_t {
            return voltage_pu_t{waveform_generator.get_signal_abc().to_alphabeta().alpha()} + sogi_input_dc_value;
        }));
    }
};

TEST_F(SogiFilterTest, NominalSignalTracking)
{
    // 1. Estimation error magnitude should be within tolerance after settling
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "estimation error magnitude", [&]() -> voltage_pu_t { return get_estimation_error_magnitude(); },
        voltage_pu_t{0.0f}, voltage_pu_t{0.01}, yeet::checker_activation_time_range{0.2}));  // Allow settling time

    // 2. Amplitude should track the nominal value
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "amplitude", [&]() -> voltage_pu_t { return sogi_filter_instance.get_output().magnitude(); },
        voltage_pu_t{0.99f}, voltage_pu_t{1.01f}, yeet::checker_activation_time_range{0.2}));

    // 3. DC offset should remain near zero for pure AC signal
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "dc offset tracking", [&]() -> voltage_pu_t { return abs(sogi_filter_instance.get_dc_offset()); },
        voltage_pu_t{0.0f}, voltage_pu_t{0.01}, yeet::checker_activation_time_range{0.2}));

    EXPECT_TRUE(simulator.simulate_for(0.4f));
}

TEST_F(SogiFilterTest, AmplitudeStepResponse)
{
    waveform_generator.set_amplitude_step(real_t{0.2}, 0.5);  // Step to 20% amplitude at t=0.5s

    // 1. Before amplitude step, should track nominal amplitude accurately
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "amplitude before step", [&]() -> voltage_pu_t { return sogi_filter_instance.get_output().magnitude(); },
        voltage_pu_t{0.99f}, voltage_pu_t{1.01f}, yeet::checker_activation_time_range{0.3, 0.45}));

    // 2. After amplitude step, should track new amplitude
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "amplitude after step", [&]() -> voltage_pu_t { return sogi_filter_instance.get_output().magnitude(); },
        voltage_pu_t{1.18f}, voltage_pu_t{1.22f},  // 20% ±10%
        yeet::checker_activation_time_range{0.7}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(SogiFilterTest, FrequencyStepResponse)
{
    waveform_generator.set_frequency_step(real_t{5.0}, 0.5);  // Step to 55 Hz at t=0.5s

    // Update SOGI frequency tracking to match the step
    simulator.register_lambda_runner(static_cast<double>(sampling_time), [&]() {
        // Update frequency tracking after step occurs
        if (simulator.get_current_simulation_time() >= 0.5) {
            sogi_input_frequency = nominal_frequency + frequency_t{5.0f};  // Track the step
        }
    });

    // 1. Before frequency step, should have good tracking
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "tracking before frequency step", [&]() -> voltage_pu_t { return get_estimation_error_magnitude(); },
        voltage_pu_t{0.0f}, voltage_pu_t{0.01}, yeet::checker_activation_time_range{0.3, 0.45}));

    // 2. After frequency step and SOGI adaptation, should regain good tracking
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "tracking after frequency step", [&]() -> voltage_pu_t { return get_estimation_error_magnitude(); },
        voltage_pu_t{0.0f}, voltage_pu_t{0.01 * 5},  // Allow higher error due to frequency mismatch dynamics
        yeet::checker_activation_time_range{0.7}));

    // 3. Amplitude should remain stable despite frequency change
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "amplitude stability during frequency step",
        [&]() -> voltage_pu_t { return sogi_filter_instance.get_output().magnitude(); }, voltage_pu_t{0.9f},
        voltage_pu_t{1.1f}, yeet::checker_activation_time_range{0.1}));

    // 4. DC offset should remain stable
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "dc stability during frequency step",
        [&]() -> voltage_pu_t { return abs(sogi_filter_instance.get_dc_offset()); }, voltage_pu_t{0.0f},
        dc_rejection_tolerance* real_t{3}, yeet::checker_activation_time_range{0.1}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(SogiFilterTest, FrequencyMismatchBehavior)
{
    // Test behavior when SOGI frequency doesn't match signal frequency
    sogi_input_frequency = frequency_t{51.0};  // 1 Hz mismatch

    // 1. With frequency mismatch, estimation error will be higher but should be bounded
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "estimation error with frequency mismatch", [&]() -> voltage_pu_t { return get_estimation_error_magnitude(); },
        voltage_pu_t{0.0f}, voltage_pu_t{0.1f},  // Allow significant error due to frequency mismatch
        yeet::checker_activation_time_range{0.3}));

    // 2. Output magnitude should still be reasonable
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "output magnitude with mismatch",
        [&]() -> voltage_pu_t { return sogi_filter_instance.get_output().magnitude(); }, voltage_pu_t{0.5f},
        voltage_pu_t{1.5f},  // Wider bounds due to frequency mismatch effects
        yeet::checker_activation_time_range{0.3}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(SogiFilterTest, DCOffsetRejection)
{
    sogi_filter_instance.configure_dc_rejection(1.0f / frequency_t{50.0f});  // Faster DC rejection for testing

    // 1. Initially, DC offset should be near zero
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "initial dc offset", [&]() -> voltage_pu_t { return abs(sogi_filter_instance.get_dc_offset()); },
        voltage_pu_t{0.0f}, voltage_pu_t{0.005f}, yeet::checker_activation_time_range{0.2, 0.45}));

    // 2. Before DC injection, signal tracking should be good
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "tracking before dc injection", [&]() -> voltage_pu_t { return get_estimation_error_magnitude(); },
        voltage_pu_t{0.0f}, voltage_pu_t{0.01}, yeet::checker_activation_time_range{0.2, 0.45}));

    // First phase: establish baseline
    EXPECT_TRUE(simulator.simulate_for(0.5f));

    // Inject DC offset
    sogi_input_dc_value = voltage_pu_t{0.1f};  // 10% DC offset

    // 3. After DC injection, the DC estimator should converge to the injected value
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "dc offset estimation", [&]() -> voltage_pu_t { return sogi_filter_instance.get_dc_offset(); },
        voltage_pu_t{0.08f}, voltage_pu_t{0.12f},  // Should estimate the 0.1 DC offset within ±20%
        yeet::checker_activation_time_range{0.8}));

    // 4. Output amplitude should remain close to unity despite DC offset
    simulator.register_checker(yeet::make_range_checker<voltage_pu_t>(
        "amplitude with dc offset", [&]() -> voltage_pu_t { return sogi_filter_instance.get_output().magnitude(); },
        voltage_pu_t{0.95f}, voltage_pu_t{1.05f}, yeet::checker_activation_time_range{0.8}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));

    EXPECT_GT(sogi_filter_instance.get_dc_offset(), voltage_pu_t{0.05f});
}

}  // namespace
