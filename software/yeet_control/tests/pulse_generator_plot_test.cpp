#include <gtest/gtest.h>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/pulse_generator.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class PulseGeneratorPlotTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 100e-6;        // 100 μs for good pulse resolution
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz
    static constexpr double pulse_width_deg = 180.0;       // 60 degree pulse width
    static constexpr double firing_offset_deg = 0.0;       // 30 degree firing offset
    static constexpr double firing_delay_deg = 90.0;       // 30 degree firing delay

    void SetUp() override
    {
        // Configure the three-phase waveform generator for reference angle generation
        _waveform_generator.set_fundamental_positive_sequence_signal_amplitude(1.0);
        _waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _waveform_generator.set_signal_frequency(fundamental_frequency);
        _waveform_generator.disable_noise();

        // Configure the pulse generator
        _pulse_gen = std::make_unique<pulse_generator>(angle_wrapped(deg_to_rad(firing_offset_deg)),  // firing_offset
            angle_wrapped(deg_to_rad(pulse_width_deg))                                                // pulse_width
        );

        set_logger();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Update waveform generator to get reference angles
            _waveform_generator.update();

            // Get the phase A angle as reference (synchronized with grid)
            angle_wrapped reference_angle = _waveform_generator.get_fundamental_signal_angle().a();

            // Update pulse generator with and without firing delay
            _pulse_no_delay = _pulse_gen->update(reference_angle, angle_wrapped(0.0), true);
            _pulse_with_delay = _pulse_gen->update(reference_angle, angle_wrapped(deg_to_rad(firing_delay_deg)), true);

            // Store current reference angle for plotting
            _current_reference_angle = reference_angle;
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

private:
    void set_logger()
    {
        // Log reference angle in degrees
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "reference_angle_deg", [&]() -> double { return _current_reference_angle.get_pu(); }));

        // Log reference sine wave (normalized -1 to 1)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "reference_sine", [&]() -> double { return std::sin(_current_reference_angle.get_radians()); }));

        // Log pulse without delay (0 or 1)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_no_delay", [&]() -> double { return static_cast<double>(_pulse_no_delay); }));

        // Log pulse with delay (0 or 1)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_with_delay", [&]() -> double { return static_cast<double>(_pulse_with_delay); }));

        // // Log expected firing window start (no delay)
        // _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        //     "expected_window_start_no_delay", [&]() -> double { return firing_offset_deg; }));

        // Log expected firing window end (no delay)
        // _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        //     "expected_window_end_no_delay", [&]() -> double { return firing_offset_deg + pulse_width_deg; }));

        // Log expected firing window start (with delay)
        // _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        //     "expected_window_start_with_delay", [&]() -> double { return firing_offset_deg + firing_delay_deg; }));

        // Log expected firing window end (with delay)
        // _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
        //     "expected_window_end_with_delay",
        //     [&]() -> double { return firing_offset_deg + firing_delay_deg + pulse_width_deg; }));
    }

    yeet::simulator _simulator = yeet::make_gtest_simulator();
    three_phase_waveform_generator _waveform_generator{sampling_time};
    std::unique_ptr<pulse_generator> _pulse_gen;

    // Current state variables
    angle_wrapped _current_reference_angle{0.0};
    bool _pulse_no_delay{false};
    bool _pulse_with_delay{false};
};

TEST_F(PulseGeneratorPlotTest, SinglePulseGeneratorBehavior)
{
    // Simulate for 2 complete cycles to see pulse behavior clearly
    double simulation_time = 2.0 / fundamental_frequency;  // 2 cycles at 50 Hz = 40 ms

    simulate_for(simulation_time);

    // The test passes if we complete simulation without errors
    // Visual verification should be done through the generated plots
    EXPECT_TRUE(true);
}

}  // namespace
