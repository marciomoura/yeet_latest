#include "yeet_control/hardware_filter_compensator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

struct TestParams {
    std::string name;
    double signal_frequency;
};

class HardwareFilterCompensatorTestBase : public ::testing::Test {
public:
    static constexpr double sampling_time = 50e-6;  // 50 μs (20 kHz fs)
    static constexpr double fc1 = 796.0;            // 796 Hz
    static constexpr double fc2 = 159.0;            // 159 Hz

    void setup_runner()
    {
        // Configure LPFs for each phase (two stages each)
        _lpf1_a.configure(fc1);
        _lpf1_b.configure(fc1);
        _lpf1_c.configure(fc1);

        _lpf2_a.configure(fc2);
        _lpf2_b.configure(fc2);
        _lpf2_c.configure(fc2);

        // Configure compensator
        _compensator.configure(frequency_t{static_cast<real_t>(fc1)}, frequency_t{static_cast<real_t>(fc2)}, 2);

        setup_telemetry();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            _waveform_gen.update();
            _reference_abc = _waveform_gen.get_signal_abc();

            // Apply filters (Stage 1)
            abc<double> stage1_abc = {_lpf1_a.update(_reference_abc.a()), _lpf1_b.update(_reference_abc.b()),
                _lpf1_c.update(_reference_abc.c())};

            // Apply filters (Stage 2)
            _lagged_abc = {
                _lpf2_a.update(stage1_abc.a()), _lpf2_b.update(stage1_abc.b()), _lpf2_c.update(stage1_abc.c())};

            // Apply compensation
            _compensated_abc = abc<double>(_compensator.update(
                abc<real_t>(_lagged_abc), frequency_t{static_cast<real_t>(_waveform_gen.get_signal_frequency())}));

            // Calculate phase error for telemetry/checkers
            const alphabeta<double> ab_ref = _reference_abc.to_alphabeta();
            const alphabeta<double> ab_comp = _compensated_abc.to_alphabeta();
            _phase_error = angle_wrapped{static_cast<real_t>(std::atan2(ab_comp.beta(), ab_comp.alpha()) -
                                                             std::atan2(ab_ref.beta(), ab_ref.alpha()))}
                               .get_radians();
        });
    }

    void simulate_for(double duration) { _simulator.simulate_for(duration); }

private:
    void setup_telemetry()
    {
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("reference", [&]() { return _reference_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("reference_after_filter", [&]() { return _lagged_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("reference_compensated", [&]() { return _compensated_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("frequency", [&]() { return _waveform_gen.get_signal_frequency(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("phase_error", [&]() { return _phase_error; }));
    }

protected:
    yeet::simulator _simulator = yeet::make_gtest_simulator();
    three_phase_waveform_generator _waveform_gen{sampling_time};

    first_order_low_pass_filter<double> _lpf1_a{sampling_time};
    first_order_low_pass_filter<double> _lpf1_b{sampling_time};
    first_order_low_pass_filter<double> _lpf1_c{sampling_time};

    first_order_low_pass_filter<double> _lpf2_a{sampling_time};
    first_order_low_pass_filter<double> _lpf2_b{sampling_time};
    first_order_low_pass_filter<double> _lpf2_c{sampling_time};

    hardware_filter_compensator _compensator;

    abc<double> _reference_abc{};
    abc<double> _lagged_abc{};
    abc<double> _compensated_abc{};
    double _phase_error{0.0};
};

class HardwareFilterCompensatorPlotTest : public HardwareFilterCompensatorTestBase,
                                          public ::testing::WithParamInterface<TestParams> {
public:
    void SetUp() override
    {
        const double f_sig = GetParam().signal_frequency;

        // Configure waveform generator
        _waveform_gen.set_signal_frequency(f_sig);
        _waveform_gen.set_fundamental_positive_sequence_signal_amplitude(1.0);
        _waveform_gen.disable_noise();

        setup_runner();
    }
};

TEST_P(HardwareFilterCompensatorPlotTest, CompareReferenceVsCompensated)
{
    // Simulate for 5 cycles to allow settlement and see waveform
    const double f_sig = GetParam().signal_frequency;
    const double duration = 5.0 / f_sig;

    // Phase alignment checker (allow small tolerance for discretization)
    _simulator.register_checker(yeet::make_near_checker<double>(
        "phase_alignment_steady_state", [&]() { return _phase_error; }, 0.0, 0.02,
        yeet::checker_activation_time_range{duration * 0.5, duration}));

    simulate_for(duration);
}

INSTANTIATE_TEST_SUITE_P(Frequencies,
    HardwareFilterCompensatorPlotTest,
    ::testing::Values(TestParams{"f10Hz", 10.0}, TestParams{"f50Hz", 50.0}, TestParams{"f100Hz", 100.0}),
    [](const ::testing::TestParamInfo<TestParams>& info) { return info.param.name; });

class HardwareFilterCompensatorRampPlotTest : public HardwareFilterCompensatorTestBase {
public:
    void SetUp() override
    {
        // Start at 0 Hz
        _waveform_gen.set_signal_frequency(0.0);
        _waveform_gen.set_fundamental_positive_sequence_signal_amplitude(1.0);
        _waveform_gen.disable_noise();

        // Ramp frequency from 0 to 50 Hz in 20 seconds
        _waveform_gen.set_frequency_rate_of_change(50.0, 0.0, 20.0);

        setup_runner();
    }
};

TEST_F(HardwareFilterCompensatorRampPlotTest, FrequencyRamp)
{
    // Check points during ramp (5s, 10s, 15s, 20s)
    _simulator.register_checker(yeet::make_near_checker<double>(
        "phase_alignment_5s", [&]() { return _phase_error; }, 0.0, 0.02,
        yeet::checker_activation_exact_time{5.0, sampling_time}));

    _simulator.register_checker(yeet::make_near_checker<double>(
        "phase_alignment_10s", [&]() { return _phase_error; }, 0.0, 0.02,
        yeet::checker_activation_exact_time{10.0, sampling_time}));

    _simulator.register_checker(yeet::make_near_checker<double>(
        "phase_alignment_15s", [&]() { return _phase_error; }, 0.0, 0.02,
        yeet::checker_activation_exact_time{15.0, sampling_time}));

    _simulator.register_checker(yeet::make_near_checker<double>(
        "phase_alignment_20s", [&]() { return _phase_error; }, 0.0, 0.02,
        yeet::checker_activation_exact_time{20.0, sampling_time}));

    // Simulate for 20 seconds
    simulate_for(20.0);
}

class HardwareFilterCompensatorSingleFilterPlotTest : public HardwareFilterCompensatorTestBase {
public:
    void SetUp() override
    {
        // 1 filter at 500 Hz
        const double fc_single = 500.0;

        _waveform_gen.set_signal_frequency(50.0);
        _waveform_gen.set_fundamental_positive_sequence_signal_amplitude(1.0);
        _waveform_gen.disable_noise();

        _lpf1_a.configure(fc_single);
        _lpf1_b.configure(fc_single);
        _lpf1_c.configure(fc_single);

        // Stage 2 is bypassed by configuring compensator with 1 filter
        _compensator.configure(frequency_t{static_cast<real_t>(fc_single)}, frequency_t{0.0f}, 1);

        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("reference", [&]() { return _reference_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("reference_after_filter", [&]() { return _lagged_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("reference_compensated", [&]() { return _compensated_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("phase_error", [&]() { return _phase_error; }));

        _simulator.register_lambda_runner(sampling_time, [&]() {
            _waveform_gen.update();
            _reference_abc = _waveform_gen.get_signal_abc();

            // Apply filter (Stage 1 only for lag)
            _lagged_abc = {_lpf1_a.update(_reference_abc.a()), _lpf1_b.update(_reference_abc.b()),
                _lpf1_c.update(_reference_abc.c())};

            // Apply compensation
            _compensated_abc = abc<double>(_compensator.update(
                abc<real_t>(_lagged_abc), frequency_t{static_cast<real_t>(_waveform_gen.get_signal_frequency())}));

            const alphabeta<double> ab_ref = _reference_abc.to_alphabeta();
            const alphabeta<double> ab_comp = _compensated_abc.to_alphabeta();
            _phase_error = angle_wrapped{static_cast<real_t>(std::atan2(ab_comp.beta(), ab_comp.alpha()) -
                                                             std::atan2(ab_ref.beta(), ab_ref.alpha()))}
                               .get_radians();
        });
    }
};

TEST_F(HardwareFilterCompensatorSingleFilterPlotTest, OneFilterCompensation)
{
    // Check phase alignment after 10 cycles
    const double duration = 10.0 / 50.0;
    _simulator.register_checker(yeet::make_near_checker<double>(
        "phase_alignment_1filter", [&]() { return _phase_error; }, 0.0, 0.01,
        yeet::checker_activation_time_range{duration * 0.8, duration}));

    simulate_for(duration);
}

TEST(HardwareFilterCompensatorTest, RobustConfiguration)
{
    hardware_filter_compensator compensator;
    abc<real_t> input{1.0f, -0.5f, -0.5f};

    // Case 1: Invalid f1, requested 1
    compensator.configure(frequency_t{0.0f}, frequency_t{100.0f}, 1);
    abc<real_t> output = compensator.update(input, frequency_t{50.0f});
    EXPECT_NEAR(output.a(), input.a(), 1e-6);  // Should be bypassed

    // Case 2: Invalid f1, valid f2, requested 2
    compensator.configure(frequency_t{0.0f}, frequency_t{100.0f}, 2);
    output = compensator.update(input, frequency_t{100.0f});
    // Should only have f2 compensation (atan(100/100) = 45 deg)
    real_t phase_45 = std::atan(1.0f);
    EXPECT_NEAR(output.a(), input.to_alphabeta().rotate_counter_clockwise(angle_wrapped{phase_45}).to_abc().a(), 1e-6);

    // Case 3: Valid f1, invalid f2, requested 2
    compensator.configure(frequency_t{100.0f}, frequency_t{0.0f}, 2);
    output = compensator.update(input, frequency_t{100.0f});
    // Should only have f1 compensation
    EXPECT_NEAR(output.a(), input.to_alphabeta().rotate_counter_clockwise(angle_wrapped{phase_45}).to_abc().a(), 1e-6);

    // Case 4: Negative frequencies (should be treated as invalid/0)
    compensator.configure(frequency_t{-50.0f}, frequency_t{-100.0f}, 2);
    output = compensator.update(input, frequency_t{50.0f});
    EXPECT_NEAR(output.a(), input.a(), 1e-6);  // Bypassed

    // Case 5: Wrong filter count (num_filters = 0)
    compensator.configure(frequency_t{50.0f}, frequency_t{100.0f}, 0);
    output = compensator.update(input, frequency_t{50.0f});
    EXPECT_NEAR(output.a(), input.a(), 1e-6);  // Bypassed

    // Case 6: Wrong filter count (num_filters = 3, clamped to 2 based on internal implementation or just tested for
    // safety) Based on src, it will try run both if valid.
    compensator.configure(frequency_t{50.0f}, frequency_t{100.0f}, 3);
    output = compensator.update(input, frequency_t{50.0f});
    real_t phase_expected = std::atan(50.0f / 50.0f) + std::atan(50.0f / 100.0f);
    EXPECT_NEAR(
        output.a(), input.to_alphabeta().rotate_counter_clockwise(angle_wrapped{phase_expected}).to_abc().a(), 1e-6);
}

}  // namespace
