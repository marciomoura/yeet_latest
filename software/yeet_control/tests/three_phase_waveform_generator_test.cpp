#include "yeet_control/three_phase_waveform_generator.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class ThreePhaseWaveformGeneratorTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 25e-6;
    static constexpr std::size_t default_max_error_counter = 5;

    static constexpr double default_tolerance = 1e-4f;

    void SetUp() override {
        set_logger();

        // Register DUT
        _simulator.register_lambda_runner(sampling_time, [&]() { _generator_dut.update(); });

        // Register Checker
        _simulator.register_lambda_runner(sampling_time, [&]() {
            if (_checker_callback) {
                _checker_callback();
            }

            set_iterator_next_sample();
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    double get_current_simulation_time() const { return _simulator.get_current_simulation_time(); }

    three_phase_waveform_generator& get_waveform_generator() { return _generator_dut; }

    void set_checker_callback(std::function<void()> callback) { _checker_callback = callback; }

    constexpr abc<angle<zero_to_two_pi>> get_default_phase_shift() {
        return {angle<zero_to_two_pi>{}, -angle<zero_to_two_pi>(2.0 * pi / 3.0), angle<zero_to_two_pi>(2.0 * pi / 3.0)};
    }

    static abc<double> generate_three_phase_signal(std::size_t harmonic, abc<double> amplitude,
                                                   angle<zero_to_two_pi> current_angle,
                                                   abc<angle<zero_to_two_pi>> phase_shift) {
        return {amplitude[0] * std::sin(current_angle.get_radians() + static_cast<double>(harmonic) * phase_shift[0]),
                amplitude[1] * std::sin(current_angle.get_radians() + static_cast<double>(harmonic) * phase_shift[1]),
                amplitude[2] * std::sin(current_angle.get_radians() + static_cast<double>(harmonic) * phase_shift[2])};
    }

    void evaluate_pure_fundamental_waveform(double tolerance = default_tolerance,
                                            std::size_t max_error_counter = default_max_error_counter) {
        abc<double> const expected_signal = *_generate_harmonics_signal_iterator[1];

        abc<double> const generator_signal = _generator_dut.get_fundamental_signal_abc();

        for (std::size_t phase = 0; phase < 3; ++phase) {
            EXPECT_FLOAT_EQ(_generator_dut.get_fundamental_signal_abc()[phase], generator_signal[phase]);

            EXPECT_NEAR(generator_signal[phase], expected_signal[phase], tolerance)
                << " at time " << get_current_simulation_time();
        }
    }

    void evaluate_full_waveform(double tolerance = default_tolerance,
                                std::size_t max_error_counter = default_max_error_counter) {
        abc<double> const expected_full_signal = get_current_full_signal();

        abc<double> const generator_signal = _generator_dut.get_signal_abc();

        for (std::size_t phase = 0; phase < 3; ++phase) {
            EXPECT_NEAR(generator_signal[phase], expected_full_signal[phase], tolerance);
        }

        for (auto& [harmonic, iterator] : _generate_harmonics_signal_iterator) {
            abc<double> const generator_harmonic_signal_abc = _generator_dut.get_harmonic_signal_abc(harmonic);

            abc<double> const expected_harmonic_signal_abc = get_current_signal(harmonic);

            for (std::size_t phase = 0; phase < 3; ++phase) {
                EXPECT_NEAR(generator_harmonic_signal_abc[phase], expected_harmonic_signal_abc[phase], tolerance);
            }
        }
    }

    void add_lambda_sampler(const std::string& name, std::function<double()> lambda) {
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler(name, lambda));
    }

    static angle<zero_to_two_pi> get_next_angle(std::size_t harmonic, angle<zero_to_two_pi> current_angle,
                                                double signal_frequency, double sampling_time) {
        return current_angle +
               angle<zero_to_two_pi>(static_cast<double>(harmonic) * 2.0 * pi * signal_frequency * sampling_time);
    }

    angle<zero_to_two_pi> generate_abc_signal_reference(std::size_t harmonic, angle<zero_to_two_pi> initial_angle,
                                                        double simulation_time, abc<double> amplitude,
                                                        abc<angle<zero_to_two_pi>> phase_shift,
                                                        double signal_frequency) {
        std::size_t number_of_samples = static_cast<std::size_t>(simulation_time / sampling_time);

        angle<zero_to_two_pi> angle = initial_angle;

        for (std::size_t sample = 0; sample < number_of_samples; ++sample) {
            _generate_harmonics_signal[harmonic].push_back(
                generate_three_phase_signal(harmonic, amplitude, angle, phase_shift));

            angle = get_next_angle(harmonic, angle, signal_frequency, sampling_time);
        }

        _generate_harmonics_signal_iterator[harmonic] = _generate_harmonics_signal[harmonic].begin();

        return angle;
    }

    angle<zero_to_two_pi> insert_sample_abc_signal_reference(std::size_t harmonic, angle<zero_to_two_pi> signal_angle,
                                                             abc<double> amplitude,
                                                             abc<angle<zero_to_two_pi>> phase_shift,
                                                             double signal_frequency) {
        _generate_harmonics_signal[harmonic].push_back(
            generate_three_phase_signal(harmonic, amplitude, signal_angle, phase_shift));

        _generate_harmonics_signal_iterator[harmonic] = _generate_harmonics_signal[harmonic].begin();

        return get_next_angle(harmonic, signal_angle, signal_frequency, sampling_time);
    }

    void set_iterator_next_sample() {
        for (auto& [harmonics, signal] : _generate_harmonics_signal_iterator) {
            ++signal;
        }
    }

    abc<double> get_current_full_signal() const {
        abc<double> full_signal{};

        for (auto const& [harmonics, signal] : _generate_harmonics_signal_iterator) {
            full_signal += *signal;
        }

        return full_signal;
    }

    abc<double> get_current_signal(std::size_t harmonic) const {
        return *_generate_harmonics_signal_iterator.at(harmonic);
    }

private:
    void set_logger() {
        _simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<double>>(
            "v_{generator}", [&]() -> abc<double> { return _generator_dut.get_signal_abc(); }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<double>>(
            "v_{reference}", [&]() -> abc<double> { return get_current_full_signal(); }));
    }

    std::map<std::size_t, std::vector<abc<double>>> _generate_harmonics_signal{};

    std::map<std::size_t, std::vector<abc<double>>::iterator> _generate_harmonics_signal_iterator{};

    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();

    std::function<void()> _checker_callback;

    three_phase_waveform_generator _generator_dut{sampling_time};
};

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalVoltage) {
    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};

    double const simulation_time = 0.2f;

    generate_abc_signal_reference(1, angle<zero_to_two_pi>{}, simulation_time, amplitude, get_default_phase_shift(),
                                  50.0f);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(); });

    simulate_for(simulation_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalVoltageSetSymmetricFundamentalPositiveSignalAmplitude) {
    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};

    double const simulation_time = 0.1f;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();

    waveform_generator.set_fundamental_positive_sequence_signal_amplitude(2.0f);

    generate_abc_signal_reference(1, angle<zero_to_two_pi>{}, simulation_time, 2.0 * amplitude,
                                  get_default_phase_shift(), 50.0f);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(); });

    simulate_for(simulation_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalVoltageSetAssymetricFundamentalPositiveSignalAmplitude) {
    abc<double> const amplitude = {2.0f, 1.0f, 3.0f};

    double const simulation_time = 0.1f;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();

    waveform_generator.set_fundamental_positive_sequence_signal_amplitude(amplitude);

    generate_abc_signal_reference(1, angle<zero_to_two_pi>{}, simulation_time, amplitude, get_default_phase_shift(),
                                  50.0f);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(); });

    simulate_for(simulation_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalVoltageSetFundamentalNegativeSignalAmplitude) {
    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};

    double const simulation_time = 0.1f;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();

    waveform_generator.set_fundamental_positive_sequence_signal_amplitude({0.0f, 0.0f, 0.0f});
    waveform_generator.set_fundamental_negative_sequence_signal_amplitude(amplitude);

    generate_abc_signal_reference(1, angle<zero_to_two_pi>{}, simulation_time, amplitude, -get_default_phase_shift(),
                                  50.0f);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(); });

    simulate_for(simulation_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalSymmetricStepAmplitude) {
    double const step_time = 0.2f;
    double const total_time = 0.5f;

    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};
    double const amplitude_step = 0.2f;
    abc<double> const amplitude_with_step = amplitude + abc<double>{amplitude_step, amplitude_step, amplitude_step};

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();
    waveform_generator.set_amplitude_step(amplitude_step, step_time);

    angle<zero_to_two_pi> const last_angle = generate_abc_signal_reference(1, angle<zero_to_two_pi>{}, step_time,
                                                                           amplitude, get_default_phase_shift(), 50.0f);
    generate_abc_signal_reference(1, last_angle, total_time - step_time + 0.1, amplitude_with_step,
                                  get_default_phase_shift(), 50.0f);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(); });

    simulate_for(total_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalAsymmetricStepAmplitude) {
    double const step_time = 0.1f;
    double const total_time = 0.2f;

    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};
    abc<double> const amplitude_step = {0.2f, 0.4f, 0.3f};
    abc<double> const amplitude_with_step = amplitude + amplitude_step;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();
    waveform_generator.set_amplitude_step(amplitude_step, step_time);

    angle<zero_to_two_pi> const last_angle = generate_abc_signal_reference(1, angle<zero_to_two_pi>{}, step_time,
                                                                           amplitude, get_default_phase_shift(), 50.0f);
    generate_abc_signal_reference(1, last_angle, total_time - step_time, amplitude_with_step, get_default_phase_shift(),
                                  50.0f);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(); });

    simulate_for(total_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, FundamentalRateOfChangeAmplitude) {
    static constexpr double tolerance = 1e-3f;

    double const amplitude_step = 0.5f;
    double const initial_amplitude = 1.0f;

    double const time_start = 0.1f;
    double const time_delta = 0.1f;
    double const total_time = 0.3f;

    assert(total_time >= time_start + time_delta);

    double expected_amplitude = initial_amplitude;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();
    waveform_generator.set_amplitude_rate_of_change({amplitude_step, amplitude_step, amplitude_step}, time_start,
                                                    time_start + time_delta);

    angle<zero_to_two_pi> last_angle = generate_abc_signal_reference(
        1, angle<zero_to_two_pi>{}, time_start, {initial_amplitude, initial_amplitude, initial_amplitude},
        get_default_phase_shift(), 50.0f);

    double simulation_time = time_start;
    while (simulation_time < time_start + time_delta) {
        expected_amplitude += amplitude_step * sampling_time / time_delta;

        last_angle = insert_sample_abc_signal_reference(1, last_angle,
                                                        {expected_amplitude, expected_amplitude, expected_amplitude},
                                                        get_default_phase_shift(), 50.0f);

        simulation_time += sampling_time;
    }

    generate_abc_signal_reference(1, last_angle, total_time - time_delta - time_start + sampling_time,
                                  {expected_amplitude, expected_amplitude, expected_amplitude},
                                  get_default_phase_shift(), 50.0f);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(tolerance); });

    simulate_for(total_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalStepAngle) {
    double const step_time = 0.1f;
    double const total_time = 0.2f;

    angle<zero_to_two_pi> const step_value = angle<zero_to_two_pi>(pi / 2.0f);

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();

    waveform_generator.set_angle_step(step_value, step_time);

    angle<zero_to_two_pi> last_angle = generate_abc_signal_reference(
        1, angle<zero_to_two_pi>{}, step_time, {1.0f, 1.0f, 1.0f}, get_default_phase_shift(), 50.0f);

    generate_abc_signal_reference(1, last_angle + step_value, total_time - step_time, {1.0f, 1.0f, 1.0f},
                                  get_default_phase_shift(), 50.0f);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(); });

    simulate_for(total_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalStepFrequency) {
    double const step_time = 0.1f;
    double const total_time = 0.2f;

    double const frequency_step = 25.0f;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();
    waveform_generator.set_frequency_step(frequency_step, step_time);

    angle<zero_to_two_pi> last_angle = generate_abc_signal_reference(
        1, angle<zero_to_two_pi>{}, step_time, {1.0f, 1.0f, 1.0f}, get_default_phase_shift(), 50.0f);

    generate_abc_signal_reference(1, last_angle, total_time - step_time, {1.0f, 1.0f, 1.0f}, get_default_phase_shift(),
                                  50.0f + frequency_step);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(); });

    simulate_for(total_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalVoltageWithNoise) {
    static constexpr double step_time = 0.1f;
    static constexpr double total_time = 0.2f;

    // tolerance with noise is a bit higher, but still small enough to detect gross errors
    static constexpr double tolerance_with_noise = 0.3f;
    static constexpr double tolerance_without_noise = 1e-4f;

    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();

    waveform_generator.enable_noise(0.05f);

    generate_abc_signal_reference(1, angle<zero_to_two_pi>{}, total_time, amplitude, get_default_phase_shift(), 50.0f);

    // Add tolerance signals
    add_lambda_sampler("v_{a,tol,max}", [&]() -> double {
        if (this->get_current_simulation_time() <= step_time)
            return this->get_current_full_signal()[0] + tolerance_with_noise;
        else
            return this->get_current_full_signal()[0] + tolerance_without_noise;
    });

    add_lambda_sampler("v_{a,tol,min}", [&]() -> double {
        if (this->get_current_simulation_time() <= step_time)
            return this->get_current_full_signal()[0] - tolerance_with_noise;
        else
            return this->get_current_full_signal()[0] - tolerance_without_noise;
    });

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(tolerance_with_noise); });

    simulate_for(step_time);

    waveform_generator.disable_noise();

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(tolerance_without_noise); });

    simulate_for(total_time - step_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalRateOfChangeFrequency) {
    static constexpr double tolerance = 1e-2f;

    double const time_start = 0.1f;
    double const time_delta = 0.1f;
    double const total_time = 0.3f;

    double const initial_frequency = 50.0f;
    double const frequency_delta = 25.0f;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();
    waveform_generator.set_frequency_rate_of_change(frequency_delta, time_start, time_start + time_delta);

    angle<zero_to_two_pi> last_angle = generate_abc_signal_reference(
        1, angle<zero_to_two_pi>{}, time_start, {1.0f, 1.0f, 1.0f}, get_default_phase_shift(), initial_frequency);

    double expected_frequency = initial_frequency;

    double simulation_time = time_start;

    while (simulation_time < time_start + time_delta) {
        last_angle = insert_sample_abc_signal_reference(1, last_angle, {1.0f, 1.0f, 1.0f}, get_default_phase_shift(),
                                                        expected_frequency);

        expected_frequency += frequency_delta * sampling_time / time_delta;

        simulation_time += sampling_time;
    }

    add_lambda_sampler("error", [&]() -> double {
        return this->get_current_full_signal()[0] - waveform_generator.get_fundamental_signal_abc()[0];
    });

    add_lambda_sampler("signal_frequency",
                       [&]() -> double { return waveform_generator.get_signal_frequency() / initial_frequency; });

    generate_abc_signal_reference(1, last_angle, total_time - time_start - time_delta, {1.0f, 1.0f, 1.0f},
                                  get_default_phase_shift(), initial_frequency + frequency_delta);

    set_checker_callback([&]() { evaluate_pure_fundamental_waveform(tolerance); });

    simulate_for(total_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, PureFundamentalVoltageWithHarmonics) {
    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};
    abc<double> const amplitude_harmoics = {0.1f, 0.1f, 0.1f};

    double const simulation_time = 0.2f;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();

    waveform_generator.add_signal_harmonics(5, 0.1f);
    waveform_generator.add_signal_harmonics(6, 0.1f);
    waveform_generator.add_signal_harmonics(7, 0.1f);

    generate_abc_signal_reference(1, angle<zero_to_two_pi>{}, simulation_time, amplitude, get_default_phase_shift(),
                                  50.0f);
    generate_abc_signal_reference(5, angle<zero_to_two_pi>{}, simulation_time, amplitude_harmoics,
                                  get_default_phase_shift(), 50.0f);
    generate_abc_signal_reference(6, angle<zero_to_two_pi>{}, simulation_time, amplitude_harmoics,
                                  get_default_phase_shift(), 50.0f);
    generate_abc_signal_reference(7, angle<zero_to_two_pi>{}, simulation_time, amplitude_harmoics,
                                  get_default_phase_shift(), 50.0f);

    set_checker_callback([&]() { evaluate_full_waveform(); });

    simulate_for(simulation_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, StepAmplitudeWithHarmonics) {
    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};
    abc<double> const amplitude_step = {0.2f, 0.4f, 0.3f};
    abc<double> const amplitude_with_step = amplitude + amplitude_step;
    abc<double> const amplitude_harmoics = {0.1f, 0.1f, 0.1f};

    double const step_time = 0.1f;
    double const total_time = 0.2f;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();

    waveform_generator.set_amplitude_step(amplitude_step, step_time);
    waveform_generator.add_signal_harmonics(5, 0.1f);
    waveform_generator.add_signal_harmonics(6, 0.1f);
    waveform_generator.add_signal_harmonics(7, 0.1f);

    add_lambda_sampler("v_{a,5th}", [&]() -> double { return get_current_signal(5)[0]; });
    add_lambda_sampler("v_{b,5th}", [&]() -> double { return get_current_signal(5)[1]; });
    add_lambda_sampler("v_{c,5th}", [&]() -> double { return get_current_signal(5)[2]; });

    add_lambda_sampler("v_{a,6th}", [&]() -> double { return get_current_signal(6)[0]; });
    add_lambda_sampler("v_{b,6th}", [&]() -> double { return get_current_signal(6)[1]; });
    add_lambda_sampler("v_{c,6th}", [&]() -> double { return get_current_signal(6)[2]; });

    add_lambda_sampler("v_{a,7th}", [&]() -> double { return get_current_signal(7)[0]; });
    add_lambda_sampler("v_{b,7th}", [&]() -> double { return get_current_signal(7)[1]; });
    add_lambda_sampler("v_{c,7th}", [&]() -> double { return get_current_signal(7)[2]; });

    angle<zero_to_two_pi> const final_angle_fundamental = generate_abc_signal_reference(
        1, angle<zero_to_two_pi>{}, step_time, amplitude, get_default_phase_shift(), 50.0f);
    generate_abc_signal_reference(1, final_angle_fundamental, total_time - step_time, amplitude_with_step,
                                  get_default_phase_shift(), 50.0f);

    generate_abc_signal_reference(5, angle<zero_to_two_pi>{}, total_time, amplitude_harmoics, get_default_phase_shift(),
                                  50.0f);
    generate_abc_signal_reference(6, angle<zero_to_two_pi>{}, total_time, amplitude_harmoics, get_default_phase_shift(),
                                  50.0f);
    generate_abc_signal_reference(7, angle<zero_to_two_pi>{}, total_time, amplitude_harmoics, get_default_phase_shift(),
                                  50.0f);

    set_checker_callback([&]() { evaluate_full_waveform(); });

    simulate_for(total_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, StepFrequencyWithHarmonics) {
    double const step_time = 0.1f;
    double const total_time = 0.2f;

    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};
    abc<double> const amplitude_harmoics = {0.1f, 0.1f, 0.1f};

    double const frequency_step = 25.0f;

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();

    waveform_generator.set_frequency_step(frequency_step, step_time);

    waveform_generator.add_signal_harmonics(5, 0.1f);

    add_lambda_sampler("v_{a,exp,5th}", [&]() -> double { return get_current_signal(5)[0]; });
    add_lambda_sampler("v_{b,exp,5th}", [&]() -> double { return get_current_signal(5)[1]; });
    add_lambda_sampler("v_{c,exp,5th}", [&]() -> double { return get_current_signal(5)[2]; });

    add_lambda_sampler("v_{a,gen,5th}", [&]() -> double { return waveform_generator.get_harmonic_signal_abc(5)[0]; });
    add_lambda_sampler("v_{b,gen,5th}", [&]() -> double { return waveform_generator.get_harmonic_signal_abc(5)[1]; });
    add_lambda_sampler("v_{c,gen,5th}", [&]() -> double { return waveform_generator.get_harmonic_signal_abc(5)[2]; });

    angle<zero_to_two_pi> const final_angle_fundamental = generate_abc_signal_reference(
        1, angle<zero_to_two_pi>{}, step_time, amplitude, get_default_phase_shift(), 50.0f);

    generate_abc_signal_reference(1, final_angle_fundamental, total_time - step_time, amplitude,
                                  get_default_phase_shift(), 50.0f + frequency_step);

    angle<zero_to_two_pi> const final_angle_fifth = generate_abc_signal_reference(
        5, angle<zero_to_two_pi>{}, step_time, amplitude_harmoics, get_default_phase_shift(), 50.0f);

    generate_abc_signal_reference(5, final_angle_fifth, total_time - step_time, amplitude_harmoics,
                                  get_default_phase_shift(), 50.0f + frequency_step);

    set_checker_callback([&]() { evaluate_full_waveform(); });

    simulate_for(total_time);
}

TEST_F(ThreePhaseWaveformGeneratorTest, StepAngleWithHarmonics) {
    abc<double> const amplitude = {1.0f, 1.0f, 1.0f};
    abc<double> const amplitude_harmoics = {0.1f, 0.1f, 0.1f};

    double const step_time = 0.1f;
    double const total_time = 0.2f;

    angle<zero_to_two_pi> const angle_step = angle<zero_to_two_pi>(pi / 2.0);

    three_phase_waveform_generator& waveform_generator = get_waveform_generator();

    waveform_generator.set_angle_step(angle_step, step_time);
    waveform_generator.add_signal_harmonics(5, 0.1f);

    add_lambda_sampler("v_{a,5th}", [&]() -> double { return get_current_signal(5)[0]; });
    add_lambda_sampler("v_{b,5th}", [&]() -> double { return get_current_signal(5)[1]; });
    add_lambda_sampler("v_{c,5th}", [&]() -> double { return get_current_signal(5)[2]; });

    angle<zero_to_two_pi> const final_angle_fundamental = generate_abc_signal_reference(
        1, angle<zero_to_two_pi>{}, step_time, amplitude, get_default_phase_shift(), 50.0f);

    generate_abc_signal_reference(1, final_angle_fundamental + angle_step, total_time - step_time, amplitude,
                                  get_default_phase_shift(), 50.0f);

    angle<zero_to_two_pi> const final_angle_fifth = generate_abc_signal_reference(
        5, angle<zero_to_two_pi>{}, step_time, amplitude_harmoics, get_default_phase_shift(), 50.0f);

    generate_abc_signal_reference(5, final_angle_fifth + angle_step * 5.0, total_time - step_time, amplitude_harmoics,
                                  get_default_phase_shift(), 50.0f);

    set_checker_callback([&]() { evaluate_full_waveform(); });

    simulate_for(total_time);
}

}  // namespace
