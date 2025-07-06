#include "yeet_control/first_order_low_pass_filter.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "gtest/gtest.h"
#include "yeet_control/constants.hpp"
#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::control;

class FirstOrderLowPassFilterTest : public ::testing::Test {
protected:
    // Test parameters
    static constexpr double sampling_freq = 100000.0;  // 100kHz sampling
    static constexpr double sampling_period = 1.0 / sampling_freq;
    static constexpr double cutoff_freq = 10.0;   // 10Hz cutoff
    static constexpr double test_duration = 1.0;  // 1 second
    static constexpr size_t num_samples = static_cast<size_t>(test_duration * sampling_freq);

    first_order_low_pass_filter<double> filter{sampling_period};

    void SetUp() override { filter.configure(cutoff_freq); }
};

TEST_F(FirstOrderLowPassFilterTest, MagnitudeResponseAtCutoff) {
    // At cutoff frequency, magnitude should be approximately -3dB
    const double magnitude_db = filter.get_magnitude_response_db(cutoff_freq);
    EXPECT_NEAR(magnitude_db, -3.0, 0.1);
}

TEST_F(FirstOrderLowPassFilterTest, MagnitudeResponseAtVariousFrequencies) {
    // Test frequencies: DC, below cutoff, at cutoff, above cutoff
    const std::vector<double> test_freqs = {0.1, 1.0, cutoff_freq, 100.0};
    const std::vector<double> expected_db = {0.0, -0.1, -3.0, -20.0};

    for (size_t i = 0; i < test_freqs.size(); ++i) {
        const double magnitude_db = filter.get_magnitude_response_db(test_freqs[i]);
        EXPECT_NEAR(magnitude_db, expected_db[i], 0.5) << "Failed at frequency " << test_freqs[i] << " Hz";
    }
}

TEST_F(FirstOrderLowPassFilterTest, PhaseResponseAtVariousFrequencies) {
    // At cutoff frequency, phase should be approximately -45 degrees
    EXPECT_NEAR(filter.get_phase_response_degrees(cutoff_freq), -45.0, 1.0);
    // A decade below cutoff, phase should be approximately 0 degrees
    EXPECT_NEAR(filter.get_phase_response_degrees(cutoff_freq / 100.0), 0.0, 1.0);  // Near DC
    // A decade above cutoff, phase should be approximately -90 degrees
    EXPECT_NEAR(filter.get_phase_response_degrees(cutoff_freq * 100.0), -90.0, 3.0);  // High frequency
}

TEST_F(FirstOrderLowPassFilterTest, SinusoidalResponseMagnitude) {
    const double test_freq = 5.0;  // Test at 5 Hz (below cutoff)
    const double input_amplitude = 1.0;

    // Run filter for several periods to reach steady state
    double max_output = 0.0;
    const double omega = 2.0 * pi<double> * test_freq;

    // Skip first second to avoid transient response
    for (size_t i = 0; i < 2 * num_samples; ++i) {
        const double t = i * sampling_period;
        const double input = input_amplitude * std::sin(omega * t);
        const double output = filter.update(input);

        if (i >= num_samples) {  // Only measure after transient
            max_output = std::max(max_output, std::abs(output));
        }
    }

    // Compare measured magnitude with theoretical
    const double measured_magnitude = max_output / input_amplitude;
    const double theoretical_magnitude = filter.get_magnitude_response(test_freq);
    EXPECT_NEAR(measured_magnitude, theoretical_magnitude, 0.001);
}

TEST_F(FirstOrderLowPassFilterTest, SinusoidalResponsePhase) {
    const double test_freq = 5.0;  // Test at 5 Hz
    const double input_amplitude = 1.0;
    const double omega = 2.0 * pi<double> * test_freq;

    std::vector<double> input_signal;
    std::vector<double> output_signal;

    // Generate signals
    for (size_t i = 0; i < 2 * num_samples; ++i) {
        const double t = i * sampling_period;
        const double input = input_amplitude * std::sin(omega * t);
        const double output = filter.update(input);

        if (i >= num_samples) {  // Store only steady-state response
            input_signal.push_back(input);
            output_signal.push_back(output);
        }
    }

    // Find zero crossings to measure phase difference
    size_t input_crossing = 0;
    size_t output_crossing = 0;

    for (size_t i = 1; i < input_signal.size(); ++i) {
        if (input_signal[i - 1] < 0 && input_signal[i] >= 0) {
            input_crossing = i;
            break;
        }
    }

    for (size_t i = 1; i < output_signal.size(); ++i) {
        if (output_signal[i - 1] < 0 && output_signal[i] >= 0) {
            output_crossing = i;
            break;
        }
    }

    // Calculate measured phase difference
    const double samples_per_period = sampling_freq / test_freq;
    const double measured_phase_deg = -360.0 * (output_crossing - input_crossing) / samples_per_period;
    const double theoretical_phase_deg = filter.get_phase_response_degrees(test_freq);

    EXPECT_NEAR(measured_phase_deg, theoretical_phase_deg, 0.01);
}

TEST_F(FirstOrderLowPassFilterTest, StepResponse) {
    // Filter parameters
    constexpr double cutoff_freq = 10.0;     // 10 Hz cutoff frequency
    constexpr double simulation_time = 5.0;  // Simulate for 5 seconds
    constexpr double step_input = 1.0;       // Step input magnitude

    // Create filter
    first_order_low_pass_filter<double> filter(sampling_period);
    filter.configure(cutoff_freq);

    // Simulation loop
    double current_time = 0.0;
    double output = 0.0;
    double rise_time = -1.0;
    double steady_state_value = 0.0;

    while (current_time < simulation_time) {
        // Simulate the filter
        output = filter.update(step_input);

        // Capture rise time (time to reach 63.2% of steady state)
        if (rise_time < 0.0 && output >= 0.632 * step_input) {
            rise_time = current_time;
        }

        // Capture steady-state value (average of last few samples)
        if (current_time >= simulation_time - 1.0) {
            steady_state_value += output;
        }

        // Increment time
        current_time += sampling_period;
    }

    // Compute average steady-state value
    steady_state_value /= (1.0 / sampling_period);

    // Check results
    ASSERT_NEAR(steady_state_value, step_input, 0.01);  // Steady-state should be close to 1
    ASSERT_GT(rise_time, 0.0);                          // Rise time should be positive
    ASSERT_LT(rise_time, 1.0);                          // Rise time should be less than 1 second (adjust as needed)
}

// Test configuration assert
TEST_F(FirstOrderLowPassFilterTest, ConfigurationAssert) {
    // Test death cases - these should trigger assertions in debug builds
#ifndef NDEBUG
    GTEST_FLAG_SET(death_test_style, "threadsafe");

    first_order_low_pass_filter<double> filter(1.0);

    // Negative cutoff frequency
    EXPECT_DEATH(filter.configure(-10.0), ".*");

    // Cutoff frequency above Nyquist
    EXPECT_DEATH(filter.configure(100000.0), ".*");
#else
    // In release builds, just verify the function doesn't crash
    first_order_low_pass_filter<double> filter(1.0);

    // These should not crash in release mode (assertions disabled)
    // but the behavior is undefined - just test that calls complete
    EXPECT_NO_THROW(filter.configure(-10.0));
    EXPECT_NO_THROW(filter.configure(100000.0));
#endif
}

// Test reset
TEST_F(FirstOrderLowPassFilterTest, Reset) {
    first_order_low_pass_filter<double> filter(sampling_period);
    filter.configure(cutoff_freq);

    // Run filter for a few samples
    filter.update(1.0);
    filter.update(2.0);
    filter.update(3.0);

    EXPECT_NE(filter.get_output(), 0.0);

    // Reset filter
    filter.reset();

    // Check that state variables are reset
    EXPECT_EQ(filter.get_output(), 0.0);
}

class FirstOrderLowPassFilterSimulatorTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-5;  // 100 kHz sampling rate
    static constexpr double time_constant = 0.1;   // 100ms time constant

    void SetUp() override {
        // Configure filter with specified time constant
        double fc = filter_dut.calculate_cutoff_frequency(time_constant);
        filter_dut.configure(fc);

        register_samplers();
    }

    void configure_test_for_constant_input() {
        // Register simulation updates
        simulator.register_lambda_runner(sampling_time, [&]() {
            // Augment input with white noise
            filter_input = input + noise_gen.generate();
            // Update filter
            output = filter_dut.update(filter_input);
        });
    }

    void configure_test_for_sinusoidal_input() {
        // Register simulation updates
        simulator.register_lambda_runner(sampling_time, [&]() {
            signal_gen->update();
            // Augment input with white noise
            filter_input = signal_gen->get_signal_abc()[0] + noise_gen.generate();
            // Update filter
            output = filter_dut.update(filter_input);
        });
    }

    bool simulate_for(double time) { return simulator.simulate_for(time); }

protected:
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    std::unique_ptr<three_phase_waveform_generator> signal_gen =
        std::make_unique<three_phase_waveform_generator>(sampling_time);

    double input{};
    double output{};
    yeet::control::first_order_low_pass_filter<double> filter_dut{sampling_time};
    white_noise_generator<double> noise_gen{0.0};

private:
    double filter_input{};

    void register_samplers() {
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("input", filter_input));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("output", output));
    }
};

TEST_F(FirstOrderLowPassFilterSimulatorTest, RiseTimeResponse) {
    configure_test_for_constant_input();

    // Step from 0 to 1 at t=0
    input = 1.0;

    // Check response at different time constants
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "one_tau", [&]() { return output; }, 0.632, 0.634,  // 63.2% of final value at 1τ
        yeet::testing::checker_activation_exact_time{time_constant, sampling_time}));

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "two_tau", [&]() { return output; }, 0.864, 0.866,  // 86.5% of final value at 2τ
        yeet::testing::checker_activation_exact_time{2.0 * time_constant, sampling_time}));

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "three_tau", [&]() { return output; }, 0.949, 0.951,  // 95% of final value at 3τ
        yeet::testing::checker_activation_exact_time{3.0 * time_constant, sampling_time}));

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "four_tau", [&]() { return output; }, 0.981, 0.983,  // 98.2% of final value at 4τ
        yeet::testing::checker_activation_exact_time{4.0 * time_constant, sampling_time}));

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "five_tau", [&]() { return output; }, 0.993, 0.994,  // 99.3% of final value at 5τ
        yeet::testing::checker_activation_exact_time{5.0 * time_constant, sampling_time}));

    EXPECT_TRUE(simulate_for(6.0 * time_constant));  // Simulate for 6 time constants
}

TEST_F(FirstOrderLowPassFilterSimulatorTest, FrequencyResponse) {
    configure_test_for_sinusoidal_input();

    // Test response at different frequencies relative to cutoff
    constexpr double test_duration = 0.1;
    const double fc = 50.0;

    filter_dut.configure(fc);

    // Test at cutoff frequency
    signal_gen->set_signal_frequency(fc);

    // At cutoff frequency, output magnitude should be -3dB (≈0.707)
    simulator.register_checker(yeet::testing::make_max_checker<double>(
        "cutoff_freq_magnitude_max", [&]() { return output; }, 0.72,  // Allow for some numerical error
        yeet::testing::checker_activation_time_range{0.02, 0.1}       // Check after settling
        ));

    simulator.register_checker(yeet::testing::make_min_checker<double>(
        "cutoff_freq_magnitude_min", [&]() { return output; }, -0.72,  // Allow for some numerical error
        yeet::testing::checker_activation_time_range{0.02, 0.1}        // Check after settling
        ));

    EXPECT_TRUE(simulate_for(test_duration));

    // Test at 10x cutoff frequency (stopband)
    // Attenuation should be of the order of -20dB, i.e. 0.1
    signal_gen->set_signal_frequency(fc * 10.0);

    simulator.register_checker(yeet::testing::make_max_checker<double>(
        "10x_cutoff_freq_magnitude_max", [&]() { return output; }, 0.11,  // Allow for some numerical error
        yeet::testing::checker_activation_time_range{0.12, 0.2}           // Check after settling
        ));

    simulator.register_checker(yeet::testing::make_min_checker<double>(
        "10x_cutoff_freq_magnitude_max", [&]() { return output; }, -0.11,  // Allow for some numerical error
        yeet::testing::checker_activation_time_range{0.12, 0.2}            // Check after settling
        ));

    EXPECT_TRUE(simulate_for(test_duration));

    // Test at 10x cutoff frequency (stopband)
    // Attenuation should be of the order of -40dB, i.e. 0.01
    signal_gen->set_signal_frequency(fc * 100.0);

    simulator.register_checker(yeet::testing::make_max_checker<double>(
        "100x_cutoff_freq_magnitude_max", [&]() { return output; }, 0.011,  // Allow for some numerical error
        yeet::testing::checker_activation_time_range{0.22, 0.3}             // Check after settling
        ));

    simulator.register_checker(yeet::testing::make_min_checker<double>(
        "100x_cutoff_freq_magnitude_max", [&]() { return output; }, -0.011,  // Allow for some numerical error
        yeet::testing::checker_activation_time_range{0.22, 0.3}              // Check after settling
        ));

    EXPECT_TRUE(simulate_for(test_duration));
}

TEST_F(FirstOrderLowPassFilterSimulatorTest, NoiseRejection) {
    configure_test_for_constant_input();
    // Add white noise to input with 10% standard deviation
    noise_gen.set_parameters(0.1);
    // Set filter input to 1.0
    input = 1.0;

    // Output should have less variance than input
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "noise_rejection", [&]() { return output; }, 0.8 * input, 1.2 * input,  // Tighter bounds than input noise
        yeet::testing::checker_activation_time_range{0.3, 1.0}));

    EXPECT_TRUE(simulate_for(1.0));
}