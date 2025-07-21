#include "yeet_control/second_order_filter.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "yeet_control/constants.hpp"
#include "yeet_control/second_order_filter.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
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
    const double omega = 2.0 * pi * test_freq;

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
    const double omega = 2.0 * pi * test_freq;

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
    const double omega = 2.0 * pi * test_freq;

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
    const double omega = 2.0 * pi * test_freq;

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
    const double omega = 2.0 * pi * test_freq;

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
    const double omega = 2.0 * pi * test_freq;

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
    const double omega = 2.0 * pi * test_freq;
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
    static constexpr double sampling_time = 1e-4;  // 10 kHz sampling rate
    static constexpr double cutoff_freq = 10.0;    // 10 Hz cutoff frequency

    void SetUp() override {
        // Create signal generator but DO NOT register the update yet.
        signal_gen = std::make_unique<three_phase_waveform_generator>(sampling_time);
        signal_gen->set_fundamental_positive_sequence_signal_amplitude(1.0);

        register_samplers();
    }

    // Use constant input (step) update
    void configure_test_for_constant_input() {
        simulator.register_lambda_runner(sampling_time, [&]() {
            // For step responses, use the externally set 'input'
            filter_input = input + noise_gen.generate();
            output = filter_dut.update(filter_input);
        });
    }

    // Use sinusoidal input generated by the three-phase signal generator.
    void configure_test_for_sinusoidal_input() {
        simulator.register_lambda_runner(sampling_time, [&]() {
            signal_gen->update();
            // Route the generated sinusoid (phase A) as filter input
            filter_input = signal_gen->get_signal_abc()[0] + noise_gen.generate();
            output = filter_dut.update(filter_input);
        });
    }

    bool simulate_for(double duration) { return simulator.simulate_for(duration); }

protected:
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();
    std::unique_ptr<three_phase_waveform_generator> signal_gen;

    double input{};  // For constant/step input tests
    double output{};
    double filter_input{};  // Value routed to the filter
    yeet::control::second_order_low_pass_filter<double> filter_dut{sampling_time};
    white_noise_generator<double> noise_gen{0.0};

private:
    void register_samplers() {
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("input", filter_input));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("output", output));
    }
};

//---------------------------------------------------------------------
// Example Test: Butterworth Step Response (constant input)
//---------------------------------------------------------------------
TEST_F(SecondOrderLowPassFilterSimulatorTest, ButterworthStepResponse) {
    // Configure as Butterworth filter (damping ratio ≈ 0.707)
    filter_dut.configure(cutoff_freq, 0.707);

    // Use constant (step) input: set input to 1.0
    input = 1.0;

    // Use constant input update function
    configure_test_for_constant_input();

    // Check response characteristics
    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "settling_band", [&]() { return output; }, 0.95, 1.05,  // 5% settling band
        yeet::testing::checker_activation_time_range{0.4, 0.5}  // Check after 4-5 time constants
        ));
    // Check that overshoot is no more than 5%
    simulator.register_checker(yeet::testing::make_max_checker<double>(
        "max_overshoot", [&]() { return output; }, 1.05, yeet::testing::checker_activation_time_range{0.0, 0.5}));

    EXPECT_TRUE(simulate_for(0.5));  // Simulate for 0.5 seconds
}

//---------------------------------------------------------------------
// Example Test: Frequency Response (sinusoidal input)
//---------------------------------------------------------------------
TEST_F(SecondOrderLowPassFilterSimulatorTest, FrequencyResponse) {
    configure_test_for_sinusoidal_input();

    // Test response at different frequencies relative to cutoff
    constexpr double test_duration = 0.1;
    const double fc = 50.0;

    filter_dut.configure(fc, 0.707);

    // Test at cutoff frequency
    signal_gen->set_signal_frequency(fc);

    // The filter is designed as a Butterworth filter, and by definition, a Butterworth filter (whether first or second
    // order) has a -3dB magnitude at its cutoff frequency. In other words, at the cutoff frequency (ω=ωc), the filter’s
    // gain is 1/√2 (~0.707), which corresponds to -3dB.
    // This is true regardless of the order for a Butterworth prototype, since the magnitude response is given by:
    // |H(jω)| = 1/√(1 + (ω/ωc)^(2n))
    // So when ω = ωc, |H(jω)| = 1/√(1+1) = 1/√2, i.e. -3dB, even for a second-order filter.

    // At cutoff frequency, output magnitude should be -3dB (≈0.707)
    simulator.register_checker(yeet::testing::make_max_checker<double>(
        "cutoff_freq_magnitude_max", [&]() { return output; }, 0.708,  // Allow for some numerical error
        yeet::testing::checker_activation_time_range{0.05, 0.1}        // Check after settling
        ));

    simulator.register_checker(yeet::testing::make_min_checker<double>(
        "cutoff_freq_magnitude_min", [&]() { return output; }, -0.708,  // Allow for some numerical error
        yeet::testing::checker_activation_time_range{0.05, 0.1}         // Check after settling
        ));

    EXPECT_TRUE(simulate_for(test_duration));

    // Test at 10x cutoff frequency (stopband)
    // Attenuation should be of the order of -40dB, i.e. 0.1
    signal_gen->set_signal_frequency(fc * 10.0);

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "10x_cutoff_freq_magnitude", [&]() { return output; }, -0.010, 0.010,  // Allow for some numerical error
        yeet::testing::checker_activation_time_range{0.15, 0.2}                // Check after settling
        ));

    EXPECT_TRUE(simulate_for(test_duration));
}

//---------------------------------------------------------------------
// Example Test: Noise Rejection
//---------------------------------------------------------------------
TEST_F(SecondOrderLowPassFilterSimulatorTest, NoiseRejection) {
    // Configure as Butterworth filter
    filter_dut.configure(cutoff_freq, 0.707);

    // Use constant input update function
    configure_test_for_constant_input();

    // Add significant noise (20% standard deviation)
    noise_gen.set_parameters(0.2);
    input = 1.0;  // Step input of 1.0

    simulator.register_checker(yeet::testing::make_range_checker<double>(
        "noise_rejection", [&]() { return output; }, 0.85,
        1.15,  // Expect output variation to be less than the input noise variations
        yeet::testing::checker_activation_time_range{0.3, 1.0}));

    EXPECT_TRUE(simulate_for(1.0));
}

class SecondOrderBandRejectFilterSimulatorTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 1e-4;  // 10 kHz sampling rate
    static constexpr double center_freq = 100.0;   // Reject 100 Hz
    static constexpr double bandwidth = 10.0;      // Notch width

    void SetUp() override {
        // Create a signal generator, though not used in the composite signal test.
        signal_gen = std::make_unique<three_phase_waveform_generator>(sampling_time);
        signal_gen->set_fundamental_positive_sequence_signal_amplitude(1.0);
        register_samplers();
    }

    // In composite tests we override the input by directly computing the sum.
    void configure_test_for_composite_input() {
        simulator.register_lambda_runner(sampling_time, [&]() {
            // Get simulation time (assume simulator.get_current_simulation_time() exists)
            double t = simulator.get_current_simulation_time();
            // Composite signal: 50 Hz fundamental (amplitude 1.0) and 100 Hz harmonic (amplitude 0.5)
            filter_input = std::sin(2.0 * pi * 50.0 * t) + 0.5 * std::sin(2.0 * pi * 100.0 * t) + noise_gen.generate();
            output = filter_dut.update(filter_input);
        });
    }

    // Use sinusoidal input from the generator (for the frequency sweep test)
    void configure_test_for_sinusoidal_input() {
        simulator.register_lambda_runner(sampling_time, [&]() {
            signal_gen->update();
            // Route the generated sinusoid (phase A) as filter input
            filter_input = signal_gen->get_signal_abc()[0] + noise_gen.generate();
            output = filter_dut.update(filter_input);
        });
    }

    bool simulate_for(double duration) { return simulator.simulate_for(duration); }

protected:
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();
    std::unique_ptr<three_phase_waveform_generator> signal_gen;

    double input{};  // Not used by composite test
    double output{};
    double filter_input{};  // Value routed to the filter
    second_order_band_reject_filter<double> filter_dut{sampling_time};
    white_noise_generator<double> noise_gen{0.0};

private:
    void register_samplers() {
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("input", filter_input));
        simulator.register_sampler_entry(yeet::testing::make_reference_sampler("output", output));
    }
};

//---------------------------------------------------------------------
// Test 1: Composite Signal Rejection
//---------------------------------------------------------------------
// This test feeds a composite signal containing a 50 Hz fundamental (passband)
// and a 100 Hz harmonic (which falls at the notch). The filter is tuned to reject 100 Hz,
// so the output is expected to retain mainly the 50 Hz component.
TEST_F(SecondOrderBandRejectFilterSimulatorTest, CompositeSignalRejection) {
    // Configure the band-reject filter; center frequency = 100 Hz
    filter_dut.configure(center_freq, bandwidth);

    // Use composite input (generated in the lambda runner)
    configure_test_for_composite_input();

    // Run simulation for enough time to reach steady state
    constexpr double test_duration = 0.2;  // 200 ms
    EXPECT_TRUE(simulate_for(test_duration));

    // In the steady state, the output should be nearly a clean 50 Hz sinusoid.
    // For a pure 50 Hz sinusoidal signal, the amplitude is ~1.0.
    // The 100 Hz harmonic should be heavily attenuated by the notch.
    // We check that the max amplitude at the output is close to 1.0.
    simulator.register_checker(yeet::testing::make_max_checker<double>(
        "composite_output_max", [&]() { return output; }, 1.02,  // Allow small numerical error
        yeet::testing::checker_activation_time_range{0.15, 0.2}));
    simulator.register_checker(yeet::testing::make_min_checker<double>(
        "composite_output_min", [&]() { return output; }, -1.02,
        yeet::testing::checker_activation_time_range{0.15, 0.2}));

    EXPECT_TRUE(simulate_for(test_duration));
}

//---------------------------------------------------------------------
// Test 2: Frequency Sweep Response
//---------------------------------------------------------------------
// In this test the input frequency is swept from 50 Hz to 200 Hz.
// The filter (tuned around 100 Hz) should show minimal attenuation
// at 50 Hz and 200 Hz and a deep notch around 100 Hz.
TEST_F(SecondOrderBandRejectFilterSimulatorTest, FrequencySweepResponse) {
    // Configure filter with center frequency 100 Hz and bandwidth 10 Hz.
    filter_dut.configure(center_freq, bandwidth);

    // Use sinusoidal input from the generator.
    configure_test_for_sinusoidal_input();

    double simulation_time = 5.0;

    // Frequency sweep: Start at 50 Hz, go to 150 Hz over 5 seconds.
    signal_gen->set_signal_frequency(50.0);
    signal_gen->set_frequency_rate_of_change(100.0, 0.0, simulation_time);  // Increase frequency by 150 Hz over 5 s.

    // Set up a lambda checker, which will be called at each simulation step.
    simulator.register_checker(yeet::testing::lambda_checker(
        "signal_mangitude",
        [&]() {
            // Check that the output is within the expected range.
            auto magnitude = filter_dut.get_magnitude_response(signal_gen->get_signal_frequency());

            // At this range the filter may not reach steady-state in the current setup.
            // Therefore these hardcoded entries have been selected to allow for some transient.
            if (magnitude < 0.1) {
                return std::abs(output) <= 0.2;
            }

            if (magnitude < 0.3) {
                return std::abs(output) <= 0.4;
            }

            if (magnitude < 0.5) {
                return std::abs(output) <= 0.6;
            }

            return std::abs(output) <= (magnitude * 1.1);
        },
        [&]() {
            std::string message{};
            message += "Frequency: " + std::to_string(signal_gen->get_signal_frequency()) + " Hz, ";
            message += "Output: " + std::to_string(output) + ", ";
            message +=
                "Magnitude: " + std::to_string(filter_dut.get_magnitude_response(signal_gen->get_signal_frequency()));
            return message;
        },
        0.0, simulation_time));

    EXPECT_TRUE(simulate_for(simulation_time));
}

}  // namespace