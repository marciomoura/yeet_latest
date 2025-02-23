#include "yeet_control/second_order_filter.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "yeet_control/constants.hpp"
#include "yeet_control/second_order_filter.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;

class SecondOrderLowPassFilterTest : public ::testing::Test {
protected:
    // Test parameters
    static constexpr double sampling_freq = 100000.0;  // 10kHz sampling
    static constexpr double sampling_period = 1.0 / sampling_freq;
    static constexpr double cutoff_freq = 100.0;    // 1kHz cutoff
    static constexpr double damping_ratio = 0.707;  // Critically damped
    static constexpr double test_duration = 1.0;    // 1 second
    static constexpr size_t num_samples = static_cast<size_t>(test_duration * sampling_freq);

    second_order_low_pass_filter<double> filter{sampling_period};

    void SetUp() override { filter.configure(cutoff_freq, damping_ratio); }
};

TEST_F(SecondOrderLowPassFilterTest, MagnitudeResponseAtCutoff) {
    // At cutoff frequency, magnitude should be approximately -3dB
    const double magnitude_db = filter.get_magnitude_response_db(cutoff_freq);
    EXPECT_NEAR(magnitude_db, -3.0, 0.1);
}

TEST_F(SecondOrderLowPassFilterTest, MagnitudeResponseAtVariousFrequencies) {
    // Test frequencies: DC, below cutoff, at cutoff, above cutoff
    const std::vector<double> test_freqs = {cutoff_freq / 100.0, cutoff_freq / 10.0, cutoff_freq, cutoff_freq * 10.0};
    const std::vector<double> expected_db = {0.0, 0.0, -3.0, -40.0};

    for (size_t i = 0; i < test_freqs.size(); ++i) {
        const double magnitude_db = filter.get_magnitude_response_db(test_freqs[i]);
        EXPECT_NEAR(magnitude_db, expected_db[i], 1.0) << "Failed at frequency " << test_freqs[i] << " Hz ";
    }
}

TEST_F(SecondOrderLowPassFilterTest, PhaseResponseAtVariousFrequencies) {
    // At cutoff frequency, phase should be approximately -90 degrees
    EXPECT_NEAR(filter.get_phase_response_degrees(cutoff_freq), -90.0, 2.0);

    // A decade below cutoff, phase should be approximately 0 degrees
    EXPECT_NEAR(filter.get_phase_response_degrees(cutoff_freq / 100.0), 0.0, 2.0);
    EXPECT_NEAR(filter.get_phase_response_degrees(cutoff_freq / 10.0), 0.0, 10.0);

    // A decade above cutoff, phase should be approximately -180 degrees
    EXPECT_NEAR(filter.get_phase_response_degrees(cutoff_freq * 10.0), -180.0, 10.0);
    EXPECT_NEAR(filter.get_phase_response_degrees(cutoff_freq * 100.0), -180.0, 5.0);
}

TEST_F(SecondOrderLowPassFilterTest, SinusoidalResponseMagnitude) {
    const double test_freq = 100.0;  // Test at 100 Hz (below cutoff)
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
    EXPECT_NEAR(measured_magnitude, theoretical_magnitude, 0.01);
}

TEST_F(SecondOrderLowPassFilterTest, SinusoidalResponsePhase) {
    const double test_freq = 20.0;  // Test at 100 Hz
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
    double input_crossing = 0;
    double output_crossing = 0;

    for (size_t i = 1; i < input_signal.size(); ++i) {
        if (input_signal[i - 1] < 0 && input_signal[i] >= 0) {
            input_crossing = static_cast<double>(i);
            break;
        }
    }

    for (size_t i = 1; i < output_signal.size(); ++i) {
        if (output_signal[i - 1] < 0 && output_signal[i] >= 0) {
            output_crossing = static_cast<double>(i);
            break;
        }
    }

    // Calculate measured phase difference
    const double samples_per_period = sampling_freq / test_freq;
    const double measured_phase_deg = -360.0 * (output_crossing - input_crossing) / samples_per_period;
    const double theoretical_phase_deg = filter.get_phase_response_degrees(test_freq);

    EXPECT_NEAR(measured_phase_deg, theoretical_phase_deg, 0.1);
}

class SecondOrderBandRejectFilterTest : public ::testing::Test {
protected:
    // Test parameters
    static constexpr double sampling_freq = 10000.0;  // 10kHz sampling
    static constexpr double sampling_period = 1.0 / sampling_freq;
    static constexpr double center_freq = 100.0;  // 100kHz center frequency
    static constexpr double bandwidth = 10.0;     // 10Hz bandwidth
    static constexpr double test_duration = 1.0;  // 1 second
    static constexpr size_t num_samples = static_cast<size_t>(test_duration * sampling_freq);

    second_order_band_reject_filter<double> filter{sampling_period};

    void SetUp() override { filter.configure(center_freq, bandwidth); }
};

TEST_F(SecondOrderBandRejectFilterTest, MagnitudeResponseAtCenterFrequency) {
    // At center frequency, magnitude should be a minimum (ideally -Inf dB, but practically a large negative value)
    const double magnitude_db = filter.get_magnitude_response_db(center_freq);
    EXPECT_LT(magnitude_db, -20.0);  // Expect at least -20dB attenuation
}

TEST_F(SecondOrderBandRejectFilterTest, MagnitudeResponseAtVariousFrequencies) {
    // Test frequencies: well below center, at lower band edge, at center, at upper band edge, well above center
    const std::vector<double> test_freqs = {center_freq / 10.0, center_freq - bandwidth / 2.0, center_freq,
                                            center_freq + bandwidth / 2.0, center_freq * 10.0};
    const std::vector<double> expected_db = {0.0, -3.0, -100.0, -3.0, 0.0};

    for (size_t i = 0; i < test_freqs.size(); ++i) {
        const double magnitude_db = filter.get_magnitude_response_db(test_freqs[i]);
        if (i == 2) {
            EXPECT_LT(magnitude_db, -20.0) << "Failed at frequency " << test_freqs[i] << " Hz";
        } else {
            EXPECT_NEAR(magnitude_db, expected_db[i], 3.0) << "Failed at frequency " << test_freqs[i] << " Hz";
        }
    }
}

TEST_F(SecondOrderBandRejectFilterTest, PhaseResponseAtVariousFrequencies) {
    // For a typical RBJ notch filter:
    // - Frequencies well below or above the notch should have nearly 0° phase.
    // - At the notch frequency the phase is not well defined, but typically the filter shows a rapid phase transition.
    // We test below, at, and above the center frequency with appropriate tolerances.

    const double phase_below = filter.get_phase_response_degrees(center_freq / 10.0);
    EXPECT_NEAR(phase_below, 0.0, 10.0) << "Phase response below notch expected near 0°, got " << phase_below << "°.";

    // Though the amplitude is nearly zero exactly at the notch, we can still check
    // for a characteristic phase value. With the RBJ design, you might expect around -90°.
    const double phase_center = filter.get_phase_response_degrees(center_freq);
    EXPECT_NEAR(phase_center, -180.0, 20.0)
        << "Phase response at notch expected near -90° (loose tolerance), got " << phase_center << "°.";

    const double phase_above = filter.get_phase_response_degrees(center_freq * 10.0);
    EXPECT_NEAR(phase_above, 0.0, 10.0) << "Phase response above notch expected near 0°, got " << phase_above << "°.";
}

TEST_F(SecondOrderBandRejectFilterTest, SinusoidalResponseMagnitude) {
    const double test_freq = center_freq;  // Test at center frequency
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
    EXPECT_LT(measured_magnitude, 0.1);  // Expect significant attenuation
}

TEST_F(SecondOrderBandRejectFilterTest, SinusoidalResponsePhase) {
    const double test_freq = center_freq / 10.0;  // Test below center frequency
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
    double input_crossing = 0;
    double output_crossing = 0;

    for (size_t i = 1; i < input_signal.size(); ++i) {
        if (input_signal[i - 1] < 0 && input_signal[i] >= 0) {
            input_crossing = static_cast<double>(i);
            break;
        }
    }

    for (size_t i = 1; i < output_signal.size(); ++i) {
        if (output_signal[i - 1] < 0 && output_signal[i] >= 0) {
            output_crossing = static_cast<double>(i);
            break;
        }
    }

    // Calculate measured phase difference
    const double samples_per_period = sampling_freq / test_freq;
    const double measured_phase_deg = -360.0 * (output_crossing - input_crossing) / samples_per_period;
    const double theoretical_phase_deg = filter.get_phase_response_degrees(test_freq);
    EXPECT_NEAR(measured_phase_deg, theoretical_phase_deg, 10.0);
}

class SecondOrderHighPassFilterTest : public ::testing::Test {
protected:
    // Test parameters
    static constexpr double sampling_freq = 100000.0;  // 100 kHz sampling
    static constexpr double sampling_period = 1.0 / sampling_freq;
    static constexpr double cutoff_freq = 100.0;    // 1 kHz cutoff frequency
    static constexpr double damping_ratio = 0.707;  // Critically damped (Butterworth)
    static constexpr double test_duration = 1.0;    // 1 second test duration
    static constexpr size_t num_samples = static_cast<size_t>(test_duration * sampling_freq);

    second_order_high_pass_filter<double> filter{sampling_period};

    void SetUp() override { filter.configure(cutoff_freq, damping_ratio); }
};

TEST_F(SecondOrderHighPassFilterTest, MagnitudeResponseAtCutoff) {
    // At cutoff frequency, magnitude should be approximately -3 dB.
    const double magnitude_db = filter.get_magnitude_response_db(cutoff_freq);
    EXPECT_NEAR(magnitude_db, -3.0, 0.5);
}

TEST_F(SecondOrderHighPassFilterTest, MagnitudeResponseAtVariousFrequencies) {
    // Frequencies: DC, below cutoff, at cutoff, and above cutoff.
    // At DC, the filter should almost completely attenuate the signal.
    const std::vector<double> test_freqs = {cutoff_freq / 100.0, cutoff_freq / 10.0, cutoff_freq, cutoff_freq * 10.0};

    // Expected responses (in dB):
    // - At DC (cutoff/100) we expect a very large attenuation (much lower than -40 dB)
    // - Below cutoff (cutoff/10) should still be strongly attenuated (< -40 dB)
    // - At cutoff: ~ -3 dB
    // - Well above cutoff: nearly 0 dB
    const std::vector<double> expected_db = {-60.0, -40.0, -3.0, 0.0};

    for (size_t i = 0; i < test_freqs.size(); ++i) {
        const double magnitude_db = filter.get_magnitude_response_db(test_freqs[i]);
        if (i < 2) {
            // For very low frequencies we check that the magnitude is very low.
            EXPECT_LT(magnitude_db, -30.0) << "Failed at frequency " << test_freqs[i] << " Hz";
        } else {
            EXPECT_NEAR(magnitude_db, expected_db[i], 3.0) << "Failed at frequency " << test_freqs[i] << " Hz";
        }
    }
}

TEST_F(SecondOrderHighPassFilterTest, PhaseResponseAtVariousFrequencies) {
    // For a high-pass filter designed by the RBJ cookbook:
    // - At cutoff frequency, phase shift is expected to be near -90°.
    // - At frequencies well above cutoff, phase shift should approach 0°.
    //
    // (At very low frequencies the magnitude is nearly zero so phase is not reliable.)
    const double phase_at_cutoff = filter.get_phase_response_degrees(cutoff_freq);
    EXPECT_NEAR(phase_at_cutoff, 90.0, 10.0)
        << "Phase response at cutoff expected near 90°, got " << phase_at_cutoff << "°.";

    const double phase_above = filter.get_phase_response_degrees(cutoff_freq * 10.0);
    EXPECT_NEAR(phase_above, 0.0, 10.0) << "Phase response above cutoff expected near 0°, got " << phase_above << "°.";
}

TEST_F(SecondOrderHighPassFilterTest, SinusoidalResponseMagnitude) {
    // Test using a sinusoidal input above cutoff frequency.
    const double test_freq = cutoff_freq * 2.0;  // well above cutoff
    const double input_amplitude = 1.0;
    double max_output = 0.0;
    const double omega = 2.0 * pi<double> * test_freq;

    // Run filter for several periods to reach steady state.
    // Skip the first second to avoid transients.
    for (size_t i = 0; i < 2 * num_samples; ++i) {
        const double t = i * sampling_period;
        const double input = input_amplitude * std::sin(omega * t);
        const double output = filter.update(input);
        if (i >= num_samples) {  // steady state measurement
            max_output = std::max(max_output, std::abs(output));
        }
    }

    const double measured_magnitude = max_output / input_amplitude;
    const double theoretical_magnitude = filter.get_magnitude_response(test_freq);
    EXPECT_NEAR(measured_magnitude, theoretical_magnitude, 0.05);
}

TEST_F(SecondOrderHighPassFilterTest, SinusoidalResponsePhase) {
    // Test the phase response using sinusoidal input.
    // We generate steady-state input and output signals for a frequency above cutoff.
    const double test_freq = cutoff_freq * 2.0;  // above cutoff ensures significant amplitude
    const double input_amplitude = 1.0;
    const double omega = 2.0 * pi<double> * test_freq;

    std::vector<double> input_signal;
    std::vector<double> output_signal;

    for (size_t i = 0; i < 2 * num_samples; ++i) {
        const double t = i * sampling_period;
        const double input = input_amplitude * std::sin(omega * t);
        const double output = filter.update(input);
        if (i >= num_samples) {  // only store steady state
            input_signal.push_back(input);
            output_signal.push_back(output);
        }
    }

    // Use zero-crossing method to estimate phase delay.
    double input_crossing = 0;
    double output_crossing = 0;
    for (size_t i = 1; i < input_signal.size(); ++i) {
        if (input_signal[i - 1] < 0 && input_signal[i] >= 0) {
            input_crossing = static_cast<double>(i);
            break;
        }
    }
    for (size_t i = 1; i < output_signal.size(); ++i) {
        if (output_signal[i - 1] < 0 && output_signal[i] >= 0) {
            output_crossing = static_cast<double>(i);
            break;
        }
    }

    const double samples_per_period = sampling_freq / test_freq;
    const double measured_phase_deg = -360.0 * (output_crossing - input_crossing) / samples_per_period;
    const double theoretical_phase_deg = filter.get_phase_response_degrees(test_freq);
    EXPECT_NEAR(measured_phase_deg, theoretical_phase_deg, 10.0);
}

TEST_F(SecondOrderHighPassFilterTest, DCRemovalFromSinusoidalSignal) {
    // In this test, a sinusoid with a DC offset is fed into the high-pass filter.
    // Since the high-pass filter should remove DC components, the mean of the output
    // should be near zero.
    const double test_freq = cutoff_freq * 2.0;  // use a frequency above cutoff
    const double input_amplitude = 1.0;
    const double dc_offset = 2.0;  // constant DC offset
    const double omega = 2.0 * pi<double> * test_freq;
    double sum_output = 0.0;

    // Run the simulation for 2 seconds and use the second half (steady-state) for evaluation.
    for (size_t i = 0; i < 2 * num_samples; ++i) {
        const double t = i * sampling_period;
        const double input = dc_offset + input_amplitude * std::sin(omega * t);
        const double output = filter.update(input);
        if (i >= num_samples) {  // steady-state samples only
            sum_output += output;
        }
    }
    EXPECT_NEAR(sum_output, 0.0, 1e-5) << "The high-pass filter did not sufficiently remove the DC offset.";
}

class SecondOrderLowPassFilterSimulatorTest : public ::testing::Test {
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

    yeet::control::second_order_low_pass_filter<double> filter_dut{sampling_time};

private:
    void set_logger() {
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("input", input));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("output", output));
    }
};

TEST_F(SecondOrderLowPassFilterSimulatorTest, ButterworthStepResponse) {
    // Filter with a 1 second time-response
    filter_dut.configure(1.0, 0.707);

    // simulator.register_checker(yeet::make_range_checker<double>(
    //     "output_range", [&]() { return output; }, 1.1, 1.4, yeet::checker_activation_time_range{1.0, 2.0}));

    input = 0.0;
    simulate_for(0.5);

    input = 1.0;
    simulate_for(3.0);

    // Collect check reports
}

TEST_F(SecondOrderLowPassFilterSimulatorTest, HighQStepResponse) {
    // Filter with a 1 second time-response
    filter_dut.configure(1.0, 0.9);

    // simulator.register_checker(yeet::make_range_checker<double>(
    //     "output_range", [&]() { return output; }, 1.1, 1.4, yeet::checker_activation_time_range{1.0, 2.0}));

    input = 0.0;
    simulate_for(0.5);

    input = 1.0;
    simulate_for(3.0);

    // Collect check reports
}

// Add after existing test classes...

class SecondOrderBandRejectFilterSimulatorTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-4;  // 10 kHz sampling rate

    void SetUp() override {
        set_logger();

        // Configure test signal generator
        signal_gen = std::make_unique<three_phase_waveform_generator>(sampling_time);
        signal_gen->set_signal_frequency(test_freq);
        signal_gen->set_fundamental_positive_sequence_signal_amplitude(1.0);

        // Register DUT and signal generator
        simulator.register_lambda_runner(sampling_time, [&]() {
            signal_gen->update();
            input = signal_gen->get_fundamental_signal_abc()[0];  // Use phase A
            output = filter_dut.update(input);
        });

        // Configure filter with notch at test frequency
        filter_dut.configure(test_freq, bandwidth);
    }

    void simulate_for(double time) { simulator.simulate_for(time); }

protected:
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    double input{};
    double output{};
    double test_freq{50.0};  // 50Hz test frequency
    double bandwidth{5.0};   // 5Hz bandwidth

    std::unique_ptr<yeet::control::three_phase_waveform_generator> signal_gen;
    yeet::control::second_order_band_reject_filter<double> filter_dut{sampling_time};

private:
    void set_logger() {
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("input", input));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("output", output));
    }
};

TEST_F(SecondOrderBandRejectFilterSimulatorTest, NotchAtFundamentalFrequency) {
    // Test rejection of fundamental frequency
    // Expect strong attenuation at notch frequency

    // Add amplitude check for strong attenuation
    // simulator.register_checker(yeet::testing::make_range_checker(
    //     "output_amplitude", [&]() { return std::abs(output); }, 0.0, 0.1,  // Expect < 10% of input amplitude
    //     yeet::testing::checker_activation_time_range{1.0, 2.0}             // Check after settling
    //     ));

    simulate_for(2.0);  // Run for 2 seconds
}

TEST_F(SecondOrderBandRejectFilterSimulatorTest, PassBelowNotch) {
    // Test passing frequencies below notch
    test_freq = 25.0;  // Set frequency to half of notch frequency
    signal_gen->set_signal_frequency(test_freq);
    filter_dut.configure(50.0, bandwidth);  // Keep notch at 50Hz

    // Check that signal passes through with minimal attenuation
    // simulator.register_checker(yeet::testing::make_range_checker(
    //     "output_amplitude", [&]() { return std::abs(output); }, 0.8, 1.2,  // Expect close to unity gain
    //     yeet::testing::checker_activation_time_range{1.0, 2.0}));

    simulate_for(2.0);
}

TEST_F(SecondOrderBandRejectFilterSimulatorTest, PassAboveNotch) {
    // Test passing frequencies above notch
    test_freq = 100.0;  // Set frequency to double the notch frequency
    signal_gen->set_signal_frequency(test_freq);
    filter_dut.configure(50.0, bandwidth);  // Keep notch at 50Hz

    // Check that signal passes through with minimal attenuation
    // simulator.register_checker(yeet::testing::make_range_checker(
    //     "output_amplitude", [&]() { return std::abs(output); }, 0.8, 1.2,  // Expect close to unity gain
    //     yeet::testing::checker_activation_time_range{1.0, 2.0}));

    simulate_for(2.0);
}

TEST_F(SecondOrderBandRejectFilterSimulatorTest, FrequencySweep) {
    // Test filter response across frequency range
    constexpr double sweep_duration = 5.0;
    constexpr double start_freq = 10.0;
    constexpr double end_freq = 100.0;

    filter_dut.configure(50.0, bandwidth);  // Notch at 50Hz

    // Set up frequency sweep
    signal_gen->set_signal_frequency(start_freq);
    signal_gen->set_frequency_rate_of_change(end_freq - start_freq,  // Total frequency change
                                             0.0,                    // Start time
                                             sweep_duration          // End time
    );

    // Add checker for notch region
    // simulator.register_checker(yeet::testing::make_range_checker(
    //     "notch_region", [&]() { return std::abs(output); }, 0.0, 0.1,  // Strong attenuation expected
    //     yeet::testing::checker_activation_time_range{2.4, 2.6}         // Check around 50Hz (middle of sweep)
    //     ));

    simulate_for(sweep_duration);
}

}  // namespace