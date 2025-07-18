#include <gtest/gtest.h>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class SixPulseGeneratorPlotTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 50e-6;         // 50 μs for good pulse resolution
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz
    static constexpr double pulse_width_deg = 30.0;        // 15 degree pulse width
    static constexpr double firing_delay_deg = 30.0;       // 30 degree firing delay (alpha)

    void SetUp() override {
        // Configure the three-phase waveform generator for reference angle generation
        _waveform_generator.set_fundamental_positive_sequence_signal_amplitude(1.0);
        _waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _waveform_generator.set_signal_frequency(fundamental_frequency);
        _waveform_generator.disable_noise();

        set_logger();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Update waveform generator to get reference angles
            _waveform_generator.update();

            // Get the phase A angle as reference (synchronized with grid)
            angle<zero_to_two_pi> reference_angle = _waveform_generator.get_fundamental_signal_angle().a();

            // Update six-pulse generator with different firing delays
            _pulses_no_delay = _six_pulse_gen.update(reference_angle, angle<zero_to_two_pi>(0.0), true);
            _pulses_with_delay =
                _six_pulse_gen.update(reference_angle, angle<zero_to_two_pi>(deg_to_rad(firing_delay_deg)), true);

            // Store current reference angle and three-phase voltages for plotting
            _current_reference_angle = reference_angle;
            _current_three_phase_voltages = _waveform_generator.get_fundamental_signal_abc();
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

private:
    void set_logger() {
        // Log reference angle in degrees
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "reference_angle_deg", [&]() -> double { return _current_reference_angle.get_pu(); }));

        // Log three-phase reference voltages
        _simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<double>>(
            "three_phase_voltages", [&]() -> abc<double> { return _current_three_phase_voltages; }));

        // Log six pulses without delay
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p1_no_delay", [&]() -> double { return _pulses_no_delay.p1 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p2_no_delay", [&]() -> double { return _pulses_no_delay.p2 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p3_no_delay", [&]() -> double { return _pulses_no_delay.p3 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p4_no_delay", [&]() -> double { return _pulses_no_delay.p4 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p5_no_delay", [&]() -> double { return _pulses_no_delay.p5 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p6_no_delay", [&]() -> double { return _pulses_no_delay.p6 ? 1.0 : 0.0; }));

        // Log six pulses with delay
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p1_with_delay", [&]() -> double { return _pulses_with_delay.p1 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p2_with_delay", [&]() -> double { return _pulses_with_delay.p2 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p3_with_delay", [&]() -> double { return _pulses_with_delay.p3 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p4_with_delay", [&]() -> double { return _pulses_with_delay.p4 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p5_with_delay", [&]() -> double { return _pulses_with_delay.p5 ? 1.0 : 0.0; }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "pulse_p6_with_delay", [&]() -> double { return _pulses_with_delay.p6 ? 1.0 : 0.0; }));
    }

    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();
    three_phase_waveform_generator _waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen{six_pulse_generator::reference_type::phase,
                                       angle<zero_to_two_pi>(deg_to_rad(pulse_width_deg))};  // 5 degree pulse width

    // Current state variables
    angle<zero_to_two_pi> _current_reference_angle{0.0};
    abc<double> _current_three_phase_voltages{0.0, 0.0, 0.0};
    six_pulse_generator::pulses _pulses_no_delay{};
    six_pulse_generator::pulses _pulses_with_delay{};
};

TEST_F(SixPulseGeneratorPlotTest, SixPulseGeneratorBehaviorComparison) {
    // Simulate for 3 complete cycles to see pulse behavior clearly across multiple periods
    double simulation_time = 3.0 / fundamental_frequency;  // 3 cycles at 50 Hz = 60 ms

    simulate_for(simulation_time);

    // The test passes if we complete simulation without errors
    // Visual verification should be done through the generated plots
    EXPECT_TRUE(true);
}

}  // namespace
