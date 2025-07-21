#include "yeet_control/sogi_filter.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/types.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;
using namespace yeet;

class SogiFilterTest : public ::testing::Test {
public:
    static constexpr real_t sampling_time = real_t{50e-6};              // 10 μs for good time resolution
    static constexpr real_t nominal_frequency = real_t{50.0};           // 50 Hz nominal frequency
    static constexpr real_t frequency_tolerance = real_t{0.1};          // Frequency tolerance for tests
    static constexpr real_t estimation_error_tolerance = real_t{1e-3};  // 1mV estimation error tolerance
    static constexpr real_t dc_rejection_tolerance = real_t{1e-3};      // DC rejection tolerance

    sogi_filter sogi_filter{sampling_time};
    three_phase_waveform_generator waveform_generator{sampling_time};
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    real_t sogi_input_frequency = nominal_frequency;
    real_t sogi_input_dc_value = {};

    void SetUp() override {
        // Configure SOGI filter for testing
        sogi_filter.reset();
        sogi_filter.configure(real_t{1.414});                         // sqrt(2) gain
        sogi_filter.configure_dc_rejection(1.0 / nominal_frequency);  // One cycle time constant

        // Configure the three-phase waveform generator for reference signal generation
        waveform_generator.set_fundamental_positive_sequence_signal_amplitude(real_t{1.0});
        waveform_generator.set_fundamental_negative_sequence_signal_amplitude(real_t{0.0});
        waveform_generator.set_signal_frequency(nominal_frequency);
        waveform_generator.disable_noise();

        set_logger();

        simulator.register_lambda_runner(static_cast<double>(sampling_time), [&]() {
            waveform_generator.update();

            abc<real_t> const signal = waveform_generator.get_signal_abc();
            real_t const sogi_signal_input = signal.to_alphabeta().alpha() + sogi_input_dc_value;
            real_t const sogi_omega = two_pi * sogi_input_frequency;

            // Update SOGI filter with the reference signal
            sogi_filter.update(sogi_signal_input, sogi_omega);
        });
    }

    real_t get_estimation_error_magnitude() const {
        // Calculate the magnitude of the error between the estimated and reference signal
        alphabeta<real_t> const reference_signal = waveform_generator.get_signal_abc().to_alphabeta();
        alphabeta<real_t> const estimated_signal = sogi_filter.get_output();

        alphabeta<real_t> const error = reference_signal - estimated_signal;
        return error.magnitude();
    }

private:
    void set_logger() {
        // Log time
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "time", [&]() -> double { return simulator.get_current_simulation_time(); }));

        // Log reference signals
        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<>>(
            "waveform generator", [&]() -> alphabeta<> { return waveform_generator.get_signal_abc().to_alphabeta(); }));

        // Log SOGI filter output
        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<>>(
            "sogi filter", [&]() -> alphabeta<> { return sogi_filter.get_output(); }));

        // Log DC offset estimation
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "sogi dc offset", [&]() -> real_t { return sogi_filter.get_dc_offset(); }));

        // Log error magnitude
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "estimation error magnitude", [&]() -> real_t { return get_estimation_error_magnitude(); }));

        // Log input signal with DC offset
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>("input signal", [&]() -> real_t {
            return waveform_generator.get_signal_abc().to_alphabeta().alpha() + sogi_input_dc_value;
        }));
    }
};

TEST_F(SogiFilterTest, NominalSignalTracking) {
    // 1. Estimation error magnitude should be within tolerance after settling
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "estimation error magnitude", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 0.01,
        yeet::testing::checker_activation_time_range{0.2}));  // Allow settling time

    // 2. Amplitude should track the nominal value
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "amplitude", [&]() -> real_t { return sogi_filter.get_output().magnitude(); }, 0.99f, 1.01f,
        yeet::testing::checker_activation_time_range{0.2}));

    // 3. DC offset should remain near zero for pure AC signal
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "dc offset tracking", [&]() -> real_t { return std::abs(sogi_filter.get_dc_offset()); }, 0.0f, 0.01,
        yeet::testing::checker_activation_time_range{0.2}));

    EXPECT_TRUE(simulator.simulate_for(0.4f));
}

TEST_F(SogiFilterTest, AmplitudeStepResponse) {
    waveform_generator.set_amplitude_step(real_t{0.2}, 0.5);  // Step to 20% amplitude at t=0.5s

    // 1. Before amplitude step, should track nominal amplitude accurately
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "amplitude before step", [&]() -> real_t { return sogi_filter.get_output().magnitude(); }, 0.99f, 1.01f,
        yeet::testing::checker_activation_time_range{0.3, 0.45}));

    // 2. After amplitude step, should track new amplitude
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "amplitude after step", [&]() -> real_t { return sogi_filter.get_output().magnitude(); }, 1.18f,
        1.22f,  // 20% ±10%
        yeet::testing::checker_activation_time_range{0.7}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(SogiFilterTest, FrequencyStepResponse) {
    waveform_generator.set_frequency_step(real_t{5.0}, 0.5);  // Step to 55 Hz at t=0.5s

    // Update SOGI frequency tracking to match the step
    simulator.register_lambda_runner(static_cast<double>(sampling_time), [&]() {
        // Update frequency tracking after step occurs
        if (simulator.get_current_simulation_time() >= 0.5) {
            sogi_input_frequency = nominal_frequency + 5.0f;  // Track the step
        }
    });

    // 1. Before frequency step, should have good tracking
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "tracking before frequency step", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 0.01,
        yeet::testing::checker_activation_time_range{0.3, 0.45}));

    // 2. After frequency step and SOGI adaptation, should regain good tracking
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "tracking after frequency step", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f,
        0.01 * 5,  // Allow higher error due to frequency mismatch dynamics
        yeet::testing::checker_activation_time_range{0.7}));

    // 3. Amplitude should remain stable despite frequency change
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "amplitude stability during frequency step", [&]() -> real_t { return sogi_filter.get_output().magnitude(); },
        0.9f, 1.1f, yeet::testing::checker_activation_time_range{0.1}));

    // 4. DC offset should remain stable
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "dc stability during frequency step", [&]() -> real_t { return std::abs(sogi_filter.get_dc_offset()); }, 0.0f,
        dc_rejection_tolerance * 3, yeet::testing::checker_activation_time_range{0.1}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(SogiFilterTest, FrequencyMismatchBehavior) {
    // Test behavior when SOGI frequency doesn't match signal frequency
    sogi_input_frequency = real_t{51.0};  // 2 Hz mismatch

    // 1. With frequency mismatch, estimation error will be higher but should be bounded
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "estimation error with frequency mismatch", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f,
        0.1f,  // Allow significant error due to frequency mismatch
        yeet::testing::checker_activation_time_range{0.3}));

    // 2. Output magnitude should still be reasonable
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "output magnitude with mismatch", [&]() -> real_t { return sogi_filter.get_output().magnitude(); }, 0.5f,
        1.5f,  // Wider bounds due to frequency mismatch effects
        yeet::testing::checker_activation_time_range{0.3}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));
}

TEST_F(SogiFilterTest, DCOffsetRejection) {
    sogi_filter.configure_dc_rejection(1.0f / 50.0f);  // Faster DC rejection for testing

    // 1. Initially, DC offset should be near zero
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "initial dc offset", [&]() -> real_t { return std::abs(sogi_filter.get_dc_offset()); }, 0.0f, 0.005f,
        yeet::testing::checker_activation_time_range{0.2, 0.45}));

    // 2. Before DC injection, signal tracking should be good
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "tracking before dc injection", [&]() -> real_t { return get_estimation_error_magnitude(); }, 0.0f, 0.01,
        yeet::testing::checker_activation_time_range{0.2, 0.45}));

    // First phase: establish baseline
    EXPECT_TRUE(simulator.simulate_for(0.5f));

    // Inject DC offset
    sogi_input_dc_value = 0.1f;  // 10% DC offset

    // 3. After DC injection, the DC estimator should converge to the injected value
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "dc offset estimation", [&]() -> real_t { return sogi_filter.get_dc_offset(); }, 0.08f,
        0.12f,  // Should estimate the 0.1 DC offset within ±20%
        yeet::testing::checker_activation_time_range{0.8}));

    // 4. Output amplitude should remain close to unity despite DC offset
    simulator.register_checker(yeet::testing::make_range_checker<real_t>(
        "amplitude with dc offset", [&]() -> real_t { return sogi_filter.get_output().magnitude(); }, 0.95f, 1.05f,
        yeet::testing::checker_activation_time_range{0.8}));

    EXPECT_TRUE(simulator.simulate_for(1.0f));

    EXPECT_GT(sogi_filter.get_dc_offset(), 0.05f);
}

}  // namespace
