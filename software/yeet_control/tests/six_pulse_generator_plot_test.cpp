#include <gtest/gtest.h>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

//==============================================================================
// Scenario 1: Normal Mode - External Reference Angle with Firing Angle
//==============================================================================

class SixPulseGeneratorNormalModePlotTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 50e-6;         // 50 μs for good pulse resolution
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz
    static constexpr double pulse_width_deg = 30.0;        // 30 degree pulse width
    static constexpr double firing_delay_deg = 30.0;       // 30 degree firing delay (alpha)

    void SetUp() override
    {
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
            angle_wrapped reference_angle = _waveform_generator.get_fundamental_signal_angle().a();

            // Update six-pulse generator with different firing angles
            _six_pulse_gen.update(reference_angle, angle_wrapped(0.0), true);
            _pulses_no_delay = _six_pulse_gen.get_pulses();

            _six_pulse_gen.update(reference_angle, angle_wrapped(deg_to_rad(firing_delay_deg)), true);
            _pulses_with_delay = _six_pulse_gen.get_pulses();

            // Store current sector information for plotting
            _current_sector = _six_pulse_gen.get_sector();

            // Store current reference angle and three-phase voltages for plotting
            _current_reference_angle = reference_angle;
            _current_three_phase_voltages = _waveform_generator.get_fundamental_signal_abc();
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

private:
    void set_logger()
    {
        // Log reference angle in degrees
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "reference_angle_deg", [&]() -> double { return _current_reference_angle.get_pu(); }));

        // Log three-phase reference voltages
        _simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<double>>(
            "three_phase_voltages", [&]() -> abc<double> { return _current_three_phase_voltages; }));

        // Log six pulses without delay
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p1_no_delay", [&]() -> double { return _pulses_no_delay.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p2_no_delay", [&]() -> double { return _pulses_no_delay.p2 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p3_no_delay", [&]() -> double { return _pulses_no_delay.p3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p4_no_delay", [&]() -> double { return _pulses_no_delay.p4 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p5_no_delay", [&]() -> double { return _pulses_no_delay.p5 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p6_no_delay", [&]() -> double { return _pulses_no_delay.p6 ? 1.0 : 0.0; }));

        // Log six pulses with delay
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p1_with_delay", [&]() -> double { return _pulses_with_delay.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p2_with_delay", [&]() -> double { return _pulses_with_delay.p2 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p3_with_delay", [&]() -> double { return _pulses_with_delay.p3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p4_with_delay", [&]() -> double { return _pulses_with_delay.p4 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p5_with_delay", [&]() -> double { return _pulses_with_delay.p5 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_p6_with_delay", [&]() -> double { return _pulses_with_delay.p6 ? 1.0 : 0.0; }));

        // Log sector information
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "sector", [&]() -> double { return static_cast<double>(static_cast<int>(_current_sector)) / 6.0; }));
    }

    yeet::simulator _simulator = yeet::make_gtest_simulator();
    three_phase_waveform_generator _waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};

    // Current state variables
    angle_wrapped _current_reference_angle{0.0};
    abc<double> _current_three_phase_voltages{0.0, 0.0, 0.0};
    six_pulse_generator::pulses _pulses_no_delay{};
    six_pulse_generator::pulses _pulses_with_delay{};
    six_pulse_generator::pulse_sector_t _current_sector{six_pulse_generator::pulse_sector_t::none};
};

TEST_F(SixPulseGeneratorNormalModePlotTest, NormalModePulseGeneration)
{
    // Simulate for 3 complete cycles to see pulse behavior clearly across multiple periods
    double simulation_time = 3.0 / 50.0;  // 3 cycles at 50 Hz = 60 ms

    simulate_for(simulation_time);

    // The test passes if we complete simulation without errors
    // Visual verification should be done through the generated plots
    EXPECT_TRUE(true);
}

//==============================================================================
// Scenario 2: Virtual Reference Mode - Internal Frequency Integration
//==============================================================================

class SixPulseGeneratorVirtualRefPlotTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 50e-6;     // 50 μs
    static constexpr double virtual_frequency = 50.0;  // 50 Hz virtual reference
    static constexpr double pulse_width_deg = 30.0;    // 30 degree pulse width
    static constexpr double firing_angle_deg = 45.0;   // 45 degree firing angle

    void SetUp() override
    {
        // Configure virtual reference mode
        _six_pulse_gen.configure_virtual_reference(frequency_t{virtual_frequency}, us(sampling_time * 1e6));
        _six_pulse_gen.enable_virtual_reference(true);
        _six_pulse_gen.reset_virtual_reference();

        set_logger();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Update using virtual reference mode (single argument overload)
            _six_pulse_gen.update(angle_wrapped(deg_to_rad(firing_angle_deg)), true);
            _pulses = _six_pulse_gen.get_pulses();

            // Store virtual reference angle for plotting
            _current_virtual_angle = _six_pulse_gen.get_virtual_reference_angle();
            _current_sector = _six_pulse_gen.get_sector();
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

private:
    void set_logger()
    {
        // Log virtual reference angle in degrees
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "virtual_ref_angle_pu", [&]() -> double { return _current_virtual_angle.get_pu(); }));

        // Log all six pulses
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulse_p1", [&]() -> double { return _pulses.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulse_p2", [&]() -> double { return _pulses.p2 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulse_p3", [&]() -> double { return _pulses.p3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulse_p4", [&]() -> double { return _pulses.p4 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulse_p5", [&]() -> double { return _pulses.p5 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulse_p6", [&]() -> double { return _pulses.p6 ? 1.0 : 0.0; }));

        // Log sector information
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "sector", [&]() -> double { return static_cast<double>(static_cast<int>(_current_sector)) / 6.0; }));
    }

    yeet::simulator _simulator = yeet::make_gtest_simulator();
    six_pulse_generator _six_pulse_gen{
        six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(pulse_width_deg))};

    // Current state variables
    angle_wrapped _current_virtual_angle{0.0};
    six_pulse_generator::pulses _pulses{};
    six_pulse_generator::pulse_sector_t _current_sector{six_pulse_generator::pulse_sector_t::none};
};

TEST_F(SixPulseGeneratorVirtualRefPlotTest, VirtualReferencePulseGeneration)
{
    // Simulate for 3 complete cycles
    double simulation_time = 3.0 / virtual_frequency;

    simulate_for(simulation_time);

    EXPECT_TRUE(true);
}

//==============================================================================
// Scenario 3: Variable Firing Angle with Virtual Reference
//==============================================================================

class SixPulseGeneratorVariableFiringAnglePlotTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 50e-6;
    static constexpr double virtual_frequency = 50.0;
    static constexpr double pulse_width_deg = 30.0;

    void SetUp() override
    {
        // Configure virtual reference
        _six_pulse_gen.configure_virtual_reference(frequency_t{virtual_frequency}, us(sampling_time * 1e6));
        _six_pulse_gen.enable_virtual_reference(true);
        _six_pulse_gen.reset_virtual_reference();
        _six_pulse_gen.set_pulse_width(angle_wrapped(deg_to_rad(60.0f)));

        set_logger();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Get current virtual angle
            angle_wrapped current_angle = _six_pulse_gen.get_virtual_reference_angle();

            // Generate pulses with different firing angles for comparison
            _six_pulse_gen.update(current_angle, angle_wrapped(deg_to_rad(0.0)), true);
            _pulses_alpha_0 = _six_pulse_gen.get_pulses();

            _six_pulse_gen.update(current_angle, angle_wrapped(deg_to_rad(30.0)), true);
            _pulses_alpha_30 = _six_pulse_gen.get_pulses();

            _six_pulse_gen.update(current_angle, angle_wrapped(deg_to_rad(60.0)), true);
            _pulses_alpha_60 = _six_pulse_gen.get_pulses();

            _six_pulse_gen.update(current_angle, angle_wrapped(deg_to_rad(90.0)), true);
            _pulses_alpha_90 = _six_pulse_gen.get_pulses();

            // Update virtual reference for next cycle (only call update once per cycle)
            _six_pulse_gen.update(angle_wrapped(deg_to_rad(0.0)), true);

            _current_virtual_angle = _six_pulse_gen.get_virtual_reference_angle();
            _current_sector = _six_pulse_gen.get_sector();
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

private:
    void set_logger()
    {
        // Log virtual reference angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "virtual_ref_angle_pu", [&]() -> double { return _current_virtual_angle.get_pu(); }));

        // Log P1 at different firing angles
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "p1_alpha_0", [&]() -> double { return _pulses_alpha_0.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "p1_alpha_30", [&]() -> double { return _pulses_alpha_30.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "p1_alpha_60", [&]() -> double { return _pulses_alpha_60.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "p1_alpha_90", [&]() -> double { return _pulses_alpha_90.p1 ? 1.0 : 0.0; }));

        // Log P4 (opposite phase) at different firing angles
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "p4_alpha_0", [&]() -> double { return _pulses_alpha_0.p4 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "p4_alpha_30", [&]() -> double { return _pulses_alpha_30.p4 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "p4_alpha_60", [&]() -> double { return _pulses_alpha_60.p4 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "p4_alpha_90", [&]() -> double { return _pulses_alpha_90.p4 ? 1.0 : 0.0; }));

        // Log sector
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "sector", [&]() -> double { return static_cast<double>(static_cast<int>(_current_sector)) / 6.0; }));
    }

    yeet::simulator _simulator = yeet::make_gtest_simulator();
    six_pulse_generator _six_pulse_gen{
        six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(pulse_width_deg))};

    // Current state variables
    angle_wrapped _current_virtual_angle{0.0};
    six_pulse_generator::pulses _pulses_alpha_0{};
    six_pulse_generator::pulses _pulses_alpha_30{};
    six_pulse_generator::pulses _pulses_alpha_60{};
    six_pulse_generator::pulses _pulses_alpha_90{};
    six_pulse_generator::pulse_sector_t _current_sector{six_pulse_generator::pulse_sector_t::none};
};

TEST_F(SixPulseGeneratorVariableFiringAnglePlotTest, VariableFiringAngleComparison)
{
    // Simulate for 2 complete cycles
    double simulation_time = 2.0 / virtual_frequency;

    simulate_for(simulation_time);

    EXPECT_TRUE(true);
}

//==============================================================================
// Scenario 5: Pulse Masking Tests - Shows all 6 outputs with different masks
//==============================================================================

struct PulseMaskTestCase {
    std::string name;
    six_pulse_generator::pulse_mask mask;
    double frequency_hz{50.0};  // Virtual reference frequency
};

class SixPulseGeneratorPulseTrainMaskingTest : public ::testing::TestWithParam<PulseMaskTestCase> {
public:
    static constexpr double sampling_time = 50e-6;
    static constexpr double pulse_width_deg = 30.0;
    static constexpr double firing_angle_deg = 0.0;

    void SetUp() override
    {
        const auto& test_case = GetParam();
        _frequency_hz = test_case.frequency_hz;

        // Configure virtual reference for consistent testing
        _six_pulse_gen.configure_virtual_reference(
            frequency_t{static_cast<real_t>(_frequency_hz)}, us(sampling_time * 1e6));
        _six_pulse_gen.enable_virtual_reference(true);
        _six_pulse_gen.reset_virtual_reference();

        // Configure mask from test parameter
        _six_pulse_gen.configure_pulse_mask(test_case.mask);
        _six_pulse_gen.enable_pulse_mask(true);

        // Store mask for validation
        _mask = test_case.mask;

        set_logger();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Update using virtual reference mode with mask active
            _six_pulse_gen.update(angle_wrapped(deg_to_rad(firing_angle_deg)), true);
            _pulses = _six_pulse_gen.get_pulses();

            // Track if any enabled pulse has fired (for validation)
            if (_mask.p1 && _pulses.p1) _p1_fired = true;
            if (_mask.p2 && _pulses.p2) _p2_fired = true;
            if (_mask.p3 && _pulses.p3) _p3_fired = true;
            if (_mask.p4 && _pulses.p4) _p4_fired = true;
            if (_mask.p5 && _pulses.p5) _p5_fired = true;
            if (_mask.p6 && _pulses.p6) _p6_fired = true;

            // Check masked pulses are always zero
            if (!_mask.p1 && _pulses.p1) _masked_pulse_fired = true;
            if (!_mask.p2 && _pulses.p2) _masked_pulse_fired = true;
            if (!_mask.p3 && _pulses.p3) _masked_pulse_fired = true;
            if (!_mask.p4 && _pulses.p4) _masked_pulse_fired = true;
            if (!_mask.p5 && _pulses.p5) _masked_pulse_fired = true;
            if (!_mask.p6 && _pulses.p6) _masked_pulse_fired = true;

            // Store virtual reference angle for plotting
            _current_virtual_angle = _six_pulse_gen.get_virtual_reference_angle();
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    double get_frequency() const { return _frequency_hz; }
    six_pulse_generator::pulse_mask get_mask() const { return _mask; }
    bool masked_pulse_fired() const { return _masked_pulse_fired; }
    bool enabled_p1_fired() const { return !_mask.p1 || _p1_fired; }
    bool enabled_p2_fired() const { return !_mask.p2 || _p2_fired; }
    bool enabled_p3_fired() const { return !_mask.p3 || _p3_fired; }
    bool enabled_p4_fired() const { return !_mask.p4 || _p4_fired; }
    bool enabled_p5_fired() const { return !_mask.p5 || _p5_fired; }
    bool enabled_p6_fired() const { return !_mask.p6 || _p6_fired; }

private:
    void set_logger()
    {
        // Log virtual reference angle (normalized 0-1)
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("angle_pu", [&]() -> double { return _current_virtual_angle.get_pu(); }));

        // Log all six pulse outputs
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p1", [&]() -> double { return _pulses.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p2", [&]() -> double { return _pulses.p2 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p3", [&]() -> double { return _pulses.p3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p4", [&]() -> double { return _pulses.p4 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p5", [&]() -> double { return _pulses.p5 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p6", [&]() -> double { return _pulses.p6 ? 1.0 : 0.0; }));
    }

    yeet::simulator _simulator = yeet::make_gtest_simulator();
    six_pulse_generator _six_pulse_gen{
        six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(pulse_width_deg))};

    angle_wrapped _current_virtual_angle{0.0};
    six_pulse_generator::pulses _pulses{};
    six_pulse_generator::pulse_mask _mask{};
    double _frequency_hz{50.0};

    // Validation flags
    bool _p1_fired{false};
    bool _p2_fired{false};
    bool _p3_fired{false};
    bool _p4_fired{false};
    bool _p5_fired{false};
    bool _p6_fired{false};
    bool _masked_pulse_fired{false};
};

TEST_P(SixPulseGeneratorPulseTrainMaskingTest, MaskedPulseGeneration)
{
    // Simulate for 3 complete cycles at the configured frequency
    double simulation_time = 3.0 / get_frequency();
    simulate_for(simulation_time);

    // Verify: No masked pulse should ever fire
    EXPECT_FALSE(masked_pulse_fired()) << "A masked pulse fired when it should have been blocked";

    // Verify: All enabled pulses should have fired at least once during 3 cycles
    EXPECT_TRUE(enabled_p1_fired()) << "Enabled P1 never fired during simulation";
    EXPECT_TRUE(enabled_p2_fired()) << "Enabled P2 never fired during simulation";
    EXPECT_TRUE(enabled_p3_fired()) << "Enabled P3 never fired during simulation";
    EXPECT_TRUE(enabled_p4_fired()) << "Enabled P4 never fired during simulation";
    EXPECT_TRUE(enabled_p5_fired()) << "Enabled P5 never fired during simulation";
    EXPECT_TRUE(enabled_p6_fired()) << "Enabled P6 never fired during simulation";
}

//==============================================================================
// Scenario 6: Double Pulse Mode - Shows 120-degree conduction with overlaps
//==============================================================================

class SixPulseGeneratorDoublePulsePlotTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 50e-6;
    static constexpr double virtual_frequency = 50.0;
    static constexpr double pulse_width_deg = 120.0;
    static constexpr double firing_angle_deg = 0.0;

    void SetUp() override
    {
        _six_pulse_gen.configure_virtual_reference(frequency_t{virtual_frequency}, duration_t{sampling_time});
        _six_pulse_gen.enable_virtual_reference(true);
        _six_pulse_gen.reset_virtual_reference();
        _six_pulse_gen.set_pulse_width(angle_wrapped(deg_to_rad(pulse_width_deg)));

        set_logger();

        _simulator.configure_sampler_sampling_rate(sampling_time);

        _simulator.register_lambda_runner(sampling_time, [&]() {
            _six_pulse_gen.update(angle_wrapped::from_degrees(firing_angle_deg), true);
            _pulses = _six_pulse_gen.get_pulses();
            _current_angle = _six_pulse_gen.get_virtual_reference_angle();
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

private:
    void set_logger()
    {
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "virtual_angle_deg", [&]() -> double { return _current_angle.get_pu(); }));

        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p1", [&]() -> double { return _pulses.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p2", [&]() -> double { return _pulses.p2 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p3", [&]() -> double { return _pulses.p3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p4", [&]() -> double { return _pulses.p4 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p5", [&]() -> double { return _pulses.p5 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("p6", [&]() -> double { return _pulses.p6 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "any_thyristor_fired", [&]() -> double { return _six_pulse_gen.get_any_thyristor_fired(); }));
    }

    yeet::simulator _simulator = yeet::make_gtest_simulator();
    six_pulse_generator _six_pulse_gen{six_pulse_generator::reference_type::line};
    angle_wrapped _current_angle{0.0};
    six_pulse_generator::pulses _pulses{};
};

TEST_F(SixPulseGeneratorDoublePulsePlotTest, DoublePulseConduction)
{
    // Simulate for 2 cycles to see the 120-degree blocks clearly
    simulate_for(2.0 / virtual_frequency);
    EXPECT_TRUE(true);
}

INSTANTIATE_TEST_SUITE_P(PulseMaskCases,
    SixPulseGeneratorPulseTrainMaskingTest,
    ::testing::Values(
        // Only P1 enabled at 50 Hz
        PulseMaskTestCase{"OnlyP1_50Hz", {true, false, false, false, false, false}, 50.0},
        // Only P1 enabled at 10 Hz (slower)
        PulseMaskTestCase{"OnlyP1_10Hz", {true, false, false, false, false, false}, 10.0},
        // Only P2 enabled at 50 Hz
        PulseMaskTestCase{"OnlyP2_50Hz", {false, true, false, false, false, false}, 50.0},
        // Only P3 enabled at 60 Hz
        PulseMaskTestCase{"OnlyP3_60Hz", {false, false, true, false, false, false}, 60.0},
        // Only P4 enabled at 50 Hz
        PulseMaskTestCase{"OnlyP4_50Hz", {false, false, false, true, false, false}, 50.0},
        // Odd pulses only (P1, P3, P5) at 50 Hz
        PulseMaskTestCase{"OddOnly_50Hz", {true, false, true, false, true, false}, 50.0},
        // Even pulses only (P2, P4, P6) at 25 Hz
        PulseMaskTestCase{"EvenOnly_25Hz", {false, true, false, true, false, true}, 25.0},
        // P1 and P4 (opposite pair) at 50 Hz
        PulseMaskTestCase{"P1andP4_50Hz", {true, false, false, true, false, false}, 50.0},
        // All pulses enabled at 50 Hz
        PulseMaskTestCase{"AllEnabled_50Hz", {true, true, true, true, true, true}, 50.0},
        // All pulses enabled at 100 Hz (faster)
        PulseMaskTestCase{"AllEnabled_100Hz", {true, true, true, true, true, true}, 100.0}),
    [](const ::testing::TestParamInfo<PulseMaskTestCase>& info) { return info.param.name; });

}  // namespace
