#include <gtest/gtest.h>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/thyristor_rectifier.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class ThyristorBridgePlotTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 10e-6;         // 10 μs for good resolution
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz grid
    static constexpr double v_ac_peak = 100.0;             // 100V peak line-to-neutral for testing
    static constexpr double pulse_width_deg = 120.0;       // 120 degree pulse width (standard for 6-pulse)

    // Grid-side RL impedance parameters
    static constexpr double grid_resistance = 0.1;   // 0.1 Ohm per phase
    static constexpr double grid_inductance = 1e-3;  // 1 mH per phase

    // DC load parameters
    static constexpr double load_resistance = 1.0;    // 1 Ohm
    static constexpr double load_inductance = 10e-3;  // 10 mH
    static constexpr double v_dc_back_emf = 0.0;      // No back-EMF (resistive load)

    void SetUp() override
    {
        // Configure three-phase voltage source (ideal balanced grid)
        _waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_ac_peak);
        _waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _waveform_generator.set_signal_frequency(fundamental_frequency);
        _waveform_generator.disable_noise();

        set_logger();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Update three-phase voltage source
            _waveform_generator.update();

            // Get AC voltages from source
            _v_ac_source_abc = _waveform_generator.get_signal_abc();
            angle_wrapped reference_angle = _waveform_generator.get_fundamental_signal_angle().a();

            // Generate gate pulses with current firing angle
            _pulses = _six_pulse_gen.update(reference_angle, _current_firing_angle, true);

            // Update complete rectifier system (grid + bridge + DC load)
            _rectifier.update(_v_ac_source_abc, _pulses, v_dc_back_emf);

            // Get states from rectifier
            _i_ac_abc = _rectifier.get_grid_currents();
            _i_dc = _rectifier.get_dc_current();
            _v_dc_output = _rectifier.get_dc_voltage();

            // Store current reference angle and sector
            _current_reference_angle = reference_angle;
            _current_sector = _six_pulse_gen.get_sector();

            // Accumulate voltage for averaging
            _v_dc_sum += _v_dc_output;
            _v_dc_sample_count++;
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    void set_firing_angle(double angle_degrees) { _current_firing_angle = angle_wrapped(deg_to_rad(angle_degrees)); }

    double get_average_dc_voltage() const
    {
        if (_v_dc_sample_count == 0) return 0.0;
        return _v_dc_sum / static_cast<double>(_v_dc_sample_count);
    }

    double get_theoretical_dc_voltage(double firing_angle_deg) const
    {
        // For 6-pulse bridge: V_dc_avg = (3/pi) * V_LL_peak * cos(alpha)
        // where V_LL_peak = sqrt(3) * V_LN_peak (line-to-line peak voltage)
        // The bridge operates on line-to-line voltages, not line-to-neutral
        double alpha_rad = deg_to_rad(firing_angle_deg);
        double v_ll_peak = std::sqrt(3.0) * v_ac_peak;  // Convert line-to-neutral to line-to-line
        return (3.0 / pi) * v_ll_peak * std::cos(alpha_rad);
    }

    void reset_voltage_averaging()
    {
        _v_dc_sum = 0.0;
        _v_dc_sample_count = 0;
    }

protected:
private:
    void set_logger()
    {
        // Log reference angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "reference_angle_deg", [&]() -> double { return _current_reference_angle.get_degrees(); }));

        // Log firing angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "firing_angle_deg", [&]() -> double { return _current_firing_angle.get_degrees(); }));

        // Log three-phase AC source voltages (line-to-neutral)
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("v_ac_source_a", [&]() -> double { return _v_ac_source_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("v_ac_source_b", [&]() -> double { return _v_ac_source_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("v_ac_source_c", [&]() -> double { return _v_ac_source_abc.c(); }));

        // Log three-phase line-to-line voltages
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_ac_line_ab", [&]() -> double { return _v_ac_source_abc.a() - _v_ac_source_abc.b(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_ac_line_bc", [&]() -> double { return _v_ac_source_abc.b() - _v_ac_source_abc.c(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_ac_line_ca", [&]() -> double { return _v_ac_source_abc.c() - _v_ac_source_abc.a(); }));

        // Log three-phase AC load currents
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_ac_a", [&]() -> double { return _i_ac_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_ac_b", [&]() -> double { return _i_ac_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_ac_c", [&]() -> double { return _i_ac_abc.c(); }));

        // Log gate pulses
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_t1", [&]() -> double { return _pulses.p1 ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_t2", [&]() -> double { return _pulses.p2 ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_t3", [&]() -> double { return _pulses.p3 ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_t4", [&]() -> double { return _pulses.p4 ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_t5", [&]() -> double { return _pulses.p5 ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "pulse_t6", [&]() -> double { return _pulses.p6 ? v_ac_peak * 1.0 : 0.0; }));

        // Log thyristor conduction states
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("thyristor_t1_on",
            [&]() -> double { return _rectifier.get_conduction_states()[0] ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("thyristor_t2_on",
            [&]() -> double { return _rectifier.get_conduction_states()[1] ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("thyristor_t3_on",
            [&]() -> double { return _rectifier.get_conduction_states()[2] ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("thyristor_t4_on",
            [&]() -> double { return _rectifier.get_conduction_states()[3] ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("thyristor_t5_on",
            [&]() -> double { return _rectifier.get_conduction_states()[4] ? v_ac_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("thyristor_t6_on",
            [&]() -> double { return _rectifier.get_conduction_states()[5] ? v_ac_peak * 1.0 : 0.0; }));

        // Log DC voltage and current
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("v_dc_output", [&]() -> double { return _v_dc_output; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("i_dc", [&]() -> double { return _i_dc; }));

        // Log average voltages (theoretical and measured running average)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("v_dc_avg_theoretical",
            [&]() -> double { return get_theoretical_dc_voltage(_current_firing_angle.get_degrees()); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_dc_avg_measured", [&]() -> double { return get_average_dc_voltage(); }));

        // Log sector
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "sector", [&]() -> double { return static_cast<double>(static_cast<int>(_current_sector)); }));
    }

    yeet::simulator _simulator = yeet::make_gtest_simulator();
    three_phase_waveform_generator _waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};
    thyristor_rectifier _rectifier{grid_resistance, grid_inductance, load_resistance, load_inductance, sampling_time};

    // Current state variables
    angle_wrapped _current_reference_angle{0.0};
    angle_wrapped _current_firing_angle{0.0};
    abc<double> _v_ac_source_abc{0.0, 0.0, 0.0};
    abc<double> _i_ac_abc{0.0, 0.0, 0.0};
    six_pulse_generator::pulses _pulses{};
    six_pulse_generator::pulse_sector_t _current_sector{six_pulse_generator::pulse_sector_t::none};
    double _v_dc_output{0.0};
    double _i_dc{0.0};

    // Voltage averaging for validation
    double _v_dc_sum{0.0};
    size_t _v_dc_sample_count{0};
};

// Test 1: Natural commutation point (firing angle = 0°)
TEST_F(ThyristorBridgePlotTest, NaturalCommutationAlphaZero)
{
    set_firing_angle(0.0);  // Natural commutation point

    // Simulate for 3 grid cycles to observe steady-state behavior
    double simulation_time = 3.0 / fundamental_frequency;  // 60 ms @ 50 Hz
    simulate_for(simulation_time);

    // Validate average DC voltage against theoretical value
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_theoretical = get_theoretical_dc_voltage(0.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_theoretical) / v_dc_avg_theoretical * 100.0;

    // Allow 5% error tolerance (accounts for transients, commutation overlap, numerical effects)
    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_theoretical, v_dc_avg_theoretical * 0.05)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Theoretical V_dc_avg = " << v_dc_avg_theoretical << " V, "
        << "Error = " << error_percent << " %";
}

// Test 2: Delayed firing (firing angle = 30°)
TEST_F(ThyristorBridgePlotTest, DelayedFiringAlpha30)
{
    set_firing_angle(30.0);  // 30° firing delay

    // Simulate for 3 grid cycles
    double simulation_time = 3.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Validate average DC voltage against theoretical value
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_theoretical = get_theoretical_dc_voltage(30.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_theoretical) / v_dc_avg_theoretical * 100.0;

    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_theoretical, v_dc_avg_theoretical * 0.05)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Theoretical V_dc_avg = " << v_dc_avg_theoretical << " V, "
        << "Error = " << error_percent << " %";
}

// Test 3: High firing angle (firing angle = 60°)
TEST_F(ThyristorBridgePlotTest, HighFiringAngleAlpha60)
{
    set_firing_angle(60.0);  // 60° firing delay

    // Simulate for 3 grid cycles
    double simulation_time = 3.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Validate average DC voltage against theoretical value
    // Higher tolerance needed due to commutation overlap and inductive voltage drop
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_theoretical = get_theoretical_dc_voltage(60.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_theoretical) / v_dc_avg_theoretical * 100.0;

    // 20% tolerance accounts for commutation overlap angle (μ) at high firing angles
    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_theoretical, v_dc_avg_theoretical * 0.20)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Theoretical V_dc_avg = " << v_dc_avg_theoretical << " V, "
        << "Error = " << error_percent << " % (higher due to commutation overlap)";
}

// Test 4: Extended simulation showing steady-state operation
TEST_F(ThyristorBridgePlotTest, ExtendedOperationAlpha45)
{
    set_firing_angle(45.0);  // Mid-range firing angle

    // Simulate for 10 grid cycles to show consistent operation
    double simulation_time = 10.0 / fundamental_frequency;  // 200 ms @ 50 Hz
    simulate_for(simulation_time);

    // Validate average DC voltage with extended simulation for better accuracy
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_theoretical = get_theoretical_dc_voltage(45.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_theoretical) / v_dc_avg_theoretical * 100.0;

    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_theoretical, v_dc_avg_theoretical * 0.05)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Theoretical V_dc_avg = " << v_dc_avg_theoretical << " V, "
        << "Error = " << error_percent << " %";
}

// Test 5: Commutation behavior verification
TEST_F(ThyristorBridgePlotTest, CommutationVerificationAlpha15)
{
    set_firing_angle(15.0);  // Small firing delay for clear commutation observation

    // Simulate for 2 grid cycles
    double simulation_time = 2.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Validate average DC voltage against theoretical value
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_theoretical = get_theoretical_dc_voltage(15.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_theoretical) / v_dc_avg_theoretical * 100.0;

    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_theoretical, v_dc_avg_theoretical * 0.05)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Theoretical V_dc_avg = " << v_dc_avg_theoretical << " V, "
        << "Error = " << error_percent << " %";
}

}  // namespace
