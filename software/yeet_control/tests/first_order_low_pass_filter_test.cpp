#include "yeet_control/first_order_low_pass_filter.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "gtest/gtest.h"
#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/limit.hpp"
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
    GTEST_FLAG_SET(death_test_style, "threadsafe");

    first_order_low_pass_filter<double> filter(1.0);

    // Negative cutoff frequency
    EXPECT_DEATH(filter.configure(-10.0), ".*");

    // Cutoff frequency above Nyquist
    EXPECT_DEATH(filter.configure(100000.0), ".*");
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
    static constexpr double sampling_time = 1e-3;

    void SetUp() override {
        set_logger();

        // Register DUT
        simulator.register_lambda_runner(sampling_time, [&]() { output = filter_dut.update(input); });
    }

    void simulate_for(double time) { simulator.simulate_for(time); }

protected:
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    double input{};
    double output{};

    yeet::control::first_order_low_pass_filter<double> filter_dut{sampling_time};

private:
    void set_logger() {
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("input", input));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("output", output));
    }
};

TEST_F(FirstOrderLowPassFilterSimulatorTest, StepResponse) {
    // Filter with a 1 second time-response
    double fc = filter_dut.calculate_cutoff_frequency(0.1);
    filter_dut.configure(fc);

    simulator.register_checker(yeet::make_range_checker<double>(
        "output_range", [&]() { return output; }, 1.1, 1.4, yeet::checker_activation_time_range{1.0, 2.0}));

    input = 0.0;
    simulate_for(1.0);

    input = 1.0;
    simulate_for(5.0);

    // Collect check reports
}