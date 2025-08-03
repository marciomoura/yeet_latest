#include "yeet_control/sogi_filter_sequence_extractor.hpp"

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

class SogiFilterSequenceExtractorTest : public ::testing::Test {
public:
    static constexpr duration_t sampling_time = duration_t{50e-6};        // 50 μs for good time resolution
    static constexpr frequency_t nominal_frequency = frequency_t{50.0};   // 50 Hz nominal frequency
    static constexpr frequency_t frequency_tolerance = frequency_t{0.1};  // Frequency tolerance for tests
    static constexpr voltage_pu_t sequence_extraction_tolerance =
        voltage_pu_t{5e-3};                                                     // 5mV sequence extraction tolerance
    static constexpr voltage_pu_t dc_rejection_tolerance = voltage_pu_t{1e-3};  // DC rejection tolerance
    static constexpr real_t phase_error_tolerance = real_t{0.05};               // Phase error tolerance (~3 degrees)

    sogi_filter_sequence_extractor sogi_filter{sampling_time};
    three_phase_waveform_generator waveform_generator{sampling_time};
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    frequency_t sogi_input_frequency = nominal_frequency;
    voltage_pu_t sogi_input_dc_value = voltage_pu_t{0};

    void SetUp() override
    {
        // Configure the three-phase waveform generator for reference signal generation
        waveform_generator.set_fundamental_positive_sequence_signal_amplitude(real_t{1.0});
        waveform_generator.set_fundamental_negative_sequence_signal_amplitude(real_t{0.0});
        waveform_generator.set_signal_frequency(nominal_frequency);
        waveform_generator.disable_noise();

        set_logger();

        simulator.register_lambda_runner(static_cast<double>(sampling_time), [&]() {
            waveform_generator.update();

            abc<real_t> const signal = waveform_generator.get_signal_abc();
            auto const sogi_signal_input = alphabeta<voltage_pu_t>{voltage_pu_t{signal.to_alphabeta().alpha()},
                                               voltage_pu_t{signal.to_alphabeta().beta()}} +
                                           alphabeta<voltage_pu_t>{sogi_input_dc_value, sogi_input_dc_value};
            angular_frequency_t const sogi_omega = two_pi * sogi_input_frequency;

            // Update sequence extractor with the reference signal
            sogi_filter.update(sogi_signal_input, sogi_omega);
        });
    }

    voltage_pu_t get_positive_sequence_extraction_error_magnitude() const
    {
        // Calculate the magnitude of error between reference positive sequence and extracted sequence
        alphabeta<real_t> const reference_positive_sequence =
            waveform_generator.get_fundamental_positive_sequence_signal_abc().to_alphabeta();
        alphabeta<voltage_pu_t> const extracted_positive_sequence = sogi_filter.get_positive_sequence();

        alphabeta<voltage_pu_t> const error = alphabeta<voltage_pu_t>{voltage_pu_t{reference_positive_sequence.alpha()},
                                                  voltage_pu_t{reference_positive_sequence.beta()}} -
                                              extracted_positive_sequence;
        return error.magnitude();
    }

private:
    void set_logger()
    {
        // Log time
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "time", [&]() -> double { return simulator.get_current_simulation_time(); }));

        // Log reference signals
        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<>>(
            "waveform generator", [&]() -> alphabeta<> { return waveform_generator.get_signal_abc().to_alphabeta(); }));

        // Log extracted positive sequence
        simulator.register_sampler_entry(
            yeet::testing::make_lambda_array_sampler<alphabeta<voltage_pu_t>>("sogi filter positive sequence",
                [&]() -> alphabeta<voltage_pu_t> { return sogi_filter.get_positive_sequence(); }));

        // Log DC offset estimates
        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(
            "sogi filter dc offset", [&]() -> alphabeta<voltage_pu_t> { return sogi_filter.get_dc_offset(); }));

        // Log extraction errors
        simulator.register_sampler_entry(
            yeet::testing::make_lambda_sampler<voltage_pu_t>("positive sequence extraction error magnitude",
                [&]() -> voltage_pu_t { return get_positive_sequence_extraction_error_magnitude(); }));

        // Log individual SOGI outputs for diagnostics
        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(
            "alpha sogi output", [&]() -> alphabeta<voltage_pu_t> { return sogi_filter.get_alpha_sogi_output(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(
            "beta sogi output", [&]() -> alphabeta<voltage_pu_t> { return sogi_filter.get_beta_sogi_output(); }));
    }
};

TEST_F(SogiFilterSequenceExtractorTest, NominalPositiveSequenceExtraction)
{
    // 1. Positive sequence extraction accuracy after settling
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "positive sequence extraction error",
        [&]() -> voltage_pu_t { return get_positive_sequence_extraction_error_magnitude(); }, voltage_pu_t{0.0f},
        sequence_extraction_tolerance, yeet::testing::checker_activation_time_range{0.2}));  // Allow settling time

    // 2. DC offset should remain near zero for pure AC signal
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "alpha dc offset", [&]() -> voltage_pu_t { return abs(sogi_filter.get_dc_offset().alpha()); },
        voltage_pu_t{0.0f}, dc_rejection_tolerance, yeet::testing::checker_activation_time_range{0.1}));

    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "beta dc offset", [&]() -> voltage_pu_t { return abs(sogi_filter.get_dc_offset().beta()); }, voltage_pu_t{0.0f},
        dc_rejection_tolerance, yeet::testing::checker_activation_time_range{0.1}));

    EXPECT_TRUE(simulator.simulate_for(1.0));
}

TEST_F(SogiFilterSequenceExtractorTest, NegativeSequenceRejection)
{
    // Add 20% negative sequence component to test rejection capability
    waveform_generator.set_fundamental_negative_sequence_signal_amplitude(real_t{0.2});

    // 1. Positive sequence extraction should still be accurate despite negative sequence
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "positive sequence extraction with negative sequence",
        [&]() -> voltage_pu_t { return get_positive_sequence_extraction_error_magnitude(); }, voltage_pu_t{0.0f},
        sequence_extraction_tolerance* real_t{5},             // Allow higher error due to negative sequence
        yeet::testing::checker_activation_time_range{0.5}));  // Longer settling due to complexity

    // 2. Extracted amplitude should track the positive sequence amplitude (1.0 pu)
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "positive sequence amplitude with negative sequence",
        [&]() -> voltage_pu_t { return sogi_filter.get_positive_sequence().magnitude(); }, voltage_pu_t{0.95f},
        voltage_pu_t{1.05f},  // Should extract ~1.0 pu positive sequence
        yeet::testing::checker_activation_time_range{0.5}));

    // 3. DC offsets should remain bounded
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "alpha dc stability with negative sequence",
        [&]() -> voltage_pu_t { return abs(sogi_filter.get_dc_offset().alpha()); }, voltage_pu_t{0.0f},
        dc_rejection_tolerance* real_t{3}, yeet::testing::checker_activation_time_range{0.3}));

    EXPECT_TRUE(simulator.simulate_for(1.0));
}

TEST_F(SogiFilterSequenceExtractorTest, AmplitudeStepResponse)
{
    waveform_generator.set_amplitude_step(real_t{0.2}, 0.5);  // Step to 20% amplitude at t=0.5s

    // 1. Before amplitude step, should track nominal amplitude
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "amplitude before step", [&]() -> voltage_pu_t { return sogi_filter.get_positive_sequence().magnitude(); },
        voltage_pu_t{0.95f}, voltage_pu_t{1.05f}, yeet::testing::checker_activation_time_range{0.3, 0.45}));

    // 2. After amplitude step, should track new amplitude
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "amplitude after step", [&]() -> voltage_pu_t { return sogi_filter.get_positive_sequence().magnitude(); },
        voltage_pu_t{1.18f}, voltage_pu_t{1.22f},  // Should track the new 0.2 amplitude
        yeet::testing::checker_activation_time_range{0.6}));

    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "extraction error during amplitude",
        [&]() -> voltage_pu_t { return get_positive_sequence_extraction_error_magnitude(); }, voltage_pu_t{0.0f},
        sequence_extraction_tolerance* real_t{10},  // Allow higher error during transient
        yeet::testing::checker_activation_time_range{0.6}));

    // 3. DC offset should remain stable throughout
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "dc stability during amplitude step",
        [&]() -> voltage_pu_t {
            return max(abs(sogi_filter.get_dc_offset().alpha()), abs(sogi_filter.get_dc_offset().beta()));
        },
        voltage_pu_t{0.0f}, dc_rejection_tolerance* real_t{5}, yeet::testing::checker_activation_time_range{0.6}));

    EXPECT_TRUE(simulator.simulate_for(1.0));
}

TEST_F(SogiFilterSequenceExtractorTest, FrequencyStepResponse)
{
    waveform_generator.set_frequency_step(real_t{5.0}, 0.5);  // Step to 55 Hz at t=0.5s

    // Update SOGI frequency tracking to match the step
    simulator.register_lambda_runner(static_cast<double>(sampling_time), [&]() {
        if (simulator.get_current_simulation_time() >= 0.5) {
            sogi_input_frequency = nominal_frequency + frequency_t{5.0f};  // Track the frequency step
        }
    });

    // 1. Before frequency step, should have good extraction
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "extraction before frequency step",
        [&]() -> voltage_pu_t { return get_positive_sequence_extraction_error_magnitude(); }, voltage_pu_t{0.0f},
        sequence_extraction_tolerance, yeet::testing::checker_activation_time_range{0.3, 0.45}));

    // 2. After frequency step and SOGI adaptation, should regain good extraction
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "extraction after frequency step",
        [&]() -> voltage_pu_t { return get_positive_sequence_extraction_error_magnitude(); }, voltage_pu_t{0.0f},
        sequence_extraction_tolerance* real_t{8},  // Allow higher error due to frequency dynamics
        yeet::testing::checker_activation_time_range{0.8}));

    // 3. Amplitude should remain stable despite frequency change
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "amplitude stability during frequency step",
        [&]() -> voltage_pu_t { return sogi_filter.get_positive_sequence().magnitude(); }, voltage_pu_t{0.9f},
        voltage_pu_t{1.1f}, yeet::testing::checker_activation_time_range{0.2}));

    // 4. DC offset should remain stable
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "dc stability during frequency step",
        [&]() -> voltage_pu_t {
            return max(abs(sogi_filter.get_dc_offset().alpha()), abs(sogi_filter.get_dc_offset().beta()));
        },
        voltage_pu_t{0.0f}, dc_rejection_tolerance* real_t{5}, yeet::testing::checker_activation_time_range{0.2}));

    EXPECT_TRUE(simulator.simulate_for(1.0));
}

TEST_F(SogiFilterSequenceExtractorTest, FrequencyMismatchBehavior)
{
    // Test behavior when SOGI frequency doesn't match signal frequency
    sogi_input_frequency = frequency_t{52.0};  // 2 Hz mismatch

    // 1. With frequency mismatch, extraction error will be higher but bounded
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "extraction error with frequency mismatch",
        [&]() -> voltage_pu_t { return get_positive_sequence_extraction_error_magnitude(); }, voltage_pu_t{0.0f},
        voltage_pu_t{0.1f},  // Allow significant error due to frequency mismatch
        yeet::testing::checker_activation_time_range{0.5}));

    // 2. Output magnitude should still be reasonable
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "output magnitude with frequency mismatch",
        [&]() -> voltage_pu_t { return sogi_filter.get_positive_sequence().magnitude(); }, voltage_pu_t{0.5f},
        voltage_pu_t{1.5f},  // Wider bounds due to frequency mismatch effects
        yeet::testing::checker_activation_time_range{0.5}));

    // 3. DC offset should remain bounded despite frequency mismatch
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "dc offset with frequency mismatch",
        [&]() -> voltage_pu_t {
            return max(abs(sogi_filter.get_dc_offset().alpha()), abs(sogi_filter.get_dc_offset().beta()));
        },
        voltage_pu_t{0.0f}, dc_rejection_tolerance* real_t{10}, yeet::testing::checker_activation_time_range{0.5}));

    EXPECT_TRUE(simulator.simulate_for(1.0));
}

TEST_F(SogiFilterSequenceExtractorTest, DCOffsetRejection)
{
    // Test DC offset rejection on both alpha and beta components

    // 1. Initially, DC offsets should be near zero
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "initial alpha dc offset", [&]() -> voltage_pu_t { return abs(sogi_filter.get_dc_offset().alpha()); },
        voltage_pu_t{0.0f}, dc_rejection_tolerance, yeet::testing::checker_activation_time_range{0.1, 0.15}));

    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "initial beta dc offset", [&]() -> voltage_pu_t { return abs(sogi_filter.get_dc_offset().beta()); },
        voltage_pu_t{0.0f}, dc_rejection_tolerance, yeet::testing::checker_activation_time_range{0.1, 0.15}));

    EXPECT_TRUE(simulator.simulate_for(0.2));

    // Inject DC offset on both components
    sogi_input_dc_value = voltage_pu_t{0.1f};  // 10% DC offset on both alpha and beta

    // 2. After DC injection, the DC estimators should converge to the injected value
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "alpha dc offset estimation", [&]() -> voltage_pu_t { return sogi_filter.get_dc_offset().alpha(); },
        voltage_pu_t{0.08f}, voltage_pu_t{0.12f},  // Should estimate the 0.1 DC offset within ±20%
        yeet::testing::checker_activation_time_range{0.6}));

    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "beta dc offset estimation", [&]() -> voltage_pu_t { return sogi_filter.get_dc_offset().beta(); },
        voltage_pu_t{0.08f}, voltage_pu_t{0.12f},  // Should estimate the 0.1 DC offset within ±20%
        yeet::testing::checker_activation_time_range{0.6}));

    // 3. After DC rejection settles, positive sequence extraction should be restored
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "extraction after dc rejection",
        [&]() -> voltage_pu_t { return get_positive_sequence_extraction_error_magnitude(); }, voltage_pu_t{0.0f},
        sequence_extraction_tolerance* real_t{5},  // Allow some degradation due to DC transient
        yeet::testing::checker_activation_time_range{0.8}));

    // 4. Output amplitude should remain close to unity despite DC offset
    simulator.register_checker(yeet::testing::make_range_checker<voltage_pu_t>(
        "amplitude with dc offset", [&]() -> voltage_pu_t { return sogi_filter.get_positive_sequence().magnitude(); },
        voltage_pu_t{0.9f}, voltage_pu_t{1.1f}, yeet::testing::checker_activation_time_range{0.8}));

    EXPECT_TRUE(simulator.simulate_for(1.0));
}

}  // namespace
