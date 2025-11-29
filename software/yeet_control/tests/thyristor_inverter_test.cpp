#include "yeet_control/thyristor_inverter.hpp"

#include <gtest/gtest.h>

#include <iostream>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class ThyristorInverterTest : public ::testing::Test {
public:
    static constexpr duration_t sampling_time{10e-6};      // 10 μs
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz machine
    static constexpr double v_ac_peak = 100.0;             // 100V peak Back-EMF
    static constexpr double pulse_width_deg = 120.0;

    // DC Link parameters
    static constexpr resistance_t dc_link_resistance{1.0};
    static constexpr inductance_t dc_link_inductance{10e-3};  // 10 mH

    // AC Machine parameters
    static constexpr resistance_t ac_resistance{0.05};    // 0.05 Ohm
    static constexpr inductance_t ac_inductance{0.5e-3};  // 0.5 mH

    void SetUp() override
    {
        // Configure three-phase voltage source (Machine Back-EMF)
        _waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_ac_peak);
        _waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _waveform_generator.set_signal_frequency(fundamental_frequency);
        _waveform_generator.disable_noise();

        set_logger();

        _simulator.register_lambda_runner(sampling_time.value(), [&]() {
            // Update Back-EMF
            _waveform_generator.update();
            auto v_abc_raw = _waveform_generator.get_signal_abc();
            _v_ac_emf_abc =
                abc<voltage_t>{voltage_t{v_abc_raw.a()}, voltage_t{v_abc_raw.b()}, voltage_t{v_abc_raw.c()}};
            angle_wrapped reference_angle = _waveform_generator.get_fundamental_signal_angle().a();

            // Generate gate pulses
            // Note: For inverter operation, firing angle is typically > 90 degrees (e.g., 150)
            auto gen_pulses = _six_pulse_gen.update(reference_angle, _current_firing_angle, true);
            _pulses.t1 = gen_pulses.p1;
            _pulses.t2 = gen_pulses.p2;
            _pulses.t3 = gen_pulses.p3;
            _pulses.t4 = gen_pulses.p4;
            _pulses.t5 = gen_pulses.p5;
            _pulses.t6 = gen_pulses.p6;

            // Update Inverter
            _inverter.update(_v_dc_source, _pulses, _v_ac_emf_abc);

            // Get states
            _i_ac_abc = _inverter.get_ac_currents();
            _i_dc = _inverter.get_dc_current();
            _v_bridge_dc = _inverter.get_bridge_dc_voltage();

            _current_reference_angle = reference_angle;

            // Accumulate voltage for averaging
            _v_bridge_sum += _v_bridge_dc.value();
            _v_bridge_sample_count++;
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    void set_firing_angle(double angle_degrees) { _current_firing_angle = angle_wrapped(deg_to_rad(angle_degrees)); }
    void set_dc_source_voltage(double voltage) { _v_dc_source = voltage_t{static_cast<real_t>(voltage)}; }

    double get_average_bridge_voltage() const
    {
        if (_v_bridge_sample_count == 0) return 0.0;
        return _v_bridge_sum / static_cast<double>(_v_bridge_sample_count);
    }

    double get_theoretical_bridge_voltage(double firing_angle_deg) const
    {
        // V_dc_avg = (3/pi) * V_LL_peak * cos(alpha)
        double alpha_rad = deg_to_rad(firing_angle_deg);
        double v_ll_peak = std::sqrt(3.0) * v_ac_peak;
        return (3.0 / pi) * v_ll_peak * std::cos(alpha_rad);
    }

    double get_expected_dc_current(double firing_angle_deg) const
    {
        // Steady state: V_source - V_bridge_avg - I*R = 0
        // I = (V_source - V_bridge_avg) / R
        double v_bridge = get_theoretical_bridge_voltage(firing_angle_deg);
        // Note: v_bridge is negative for inverter mode
        return (_v_dc_source.value() - v_bridge) / dc_link_resistance.value();
    }

protected:
    yeet::simulator _simulator = yeet::make_gtest_simulator();
    three_phase_waveform_generator _waveform_generator{sampling_time.value()};
    six_pulse_generator _six_pulse_gen{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};
    thyristor_inverter _inverter{dc_link_resistance, dc_link_inductance, ac_resistance, ac_inductance, sampling_time};

    // State variables
    voltage_t _v_dc_source{200.0};
    angle_wrapped _current_reference_angle{0.0};
    angle_wrapped _current_firing_angle{0.0};
    abc<voltage_t> _v_ac_emf_abc{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
    abc<current_t> _i_ac_abc{current_t{0.0}, current_t{0.0}, current_t{0.0}};
    thyristor_pulses _pulses{};
    voltage_t _v_bridge_dc{0.0};
    current_t _i_dc{0.0};

    double _v_bridge_sum{0.0};
    size_t _v_bridge_sample_count{0};

private:
    void set_logger()
    {
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "firing_angle_deg", [&]() -> double { return _current_firing_angle.get_degrees(); }));

        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("v_bridge_dc", [&]() -> double { return _v_bridge_dc.value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_dc", [&]() -> double { return _i_dc.value(); }));

        _simulator.register_sampler_entry(
            yeet::make_lambda_array_sampler<abc<current_t>>("i_abc", [&]() -> abc<current_t> { return _i_ac_abc; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_ac_b", [&]() -> double { return _i_ac_abc.b().value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_ac_c", [&]() -> double { return _i_ac_abc.c().value(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_avg_measured", [&]() -> double { return get_average_bridge_voltage(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("v_bridge_avg_theoretical",
            [&]() -> double { return get_theoretical_bridge_voltage(_current_firing_angle.get_degrees()); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_phase_a", [&]() -> double { return _inverter.get_ac_phase_voltages()[0].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_phase_b", [&]() -> double { return _inverter.get_ac_phase_voltages()[1].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_phase_c", [&]() -> double { return _inverter.get_ac_phase_voltages()[2].value(); }));

        // Log actual terminal line-to-line voltages at bridge (shows voltage notch during commutation)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_ab", [&]() -> double { return _inverter.get_ac_line_voltages()[0].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_bc", [&]() -> double { return _inverter.get_ac_line_voltages()[1].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_ca", [&]() -> double { return _inverter.get_ac_line_voltages()[2].value(); }));
    }
};

// Test 1: Inverter Operation (Alpha = 150 degrees)
TEST_F(ThyristorInverterTest, InverterOperationAlpha150)
{
    set_firing_angle(150.0);

    // Set DC source voltage to simulate inverter mode (Motoring)
    // V_bridge approx -140V (Counter-EMF).
    // V_source = 160V -> Net +20V driving current.
    set_dc_source_voltage(160.0);

    // Initialize DC current to ensure conduction starts (avoid start-up paradox)
    // If I=0, rails are undefined/low, and firing might be blocked.
    // By setting I > 0, we simulate steady-state operation.
    _inverter.set_dc_current(current_t{0.0});

    // Simulate for 5 grid cycles
    double simulation_time = 5.0 / fundamental_frequency;
    simulate_for(simulation_time);

    double v_bridge_avg = get_average_bridge_voltage();
    double v_bridge_expected = get_theoretical_bridge_voltage(150.0);
    double i_dc_final = _inverter.get_dc_current().value();

    // Check that bridge voltage is negative
    EXPECT_LT(v_bridge_avg, -50.0) << "Bridge voltage should be significantly negative in inverter mode";

    // Check accuracy (allow 15% due to commutation and ripple)
    EXPECT_NEAR(v_bridge_avg, v_bridge_expected, std::abs(v_bridge_expected) * 0.15);

    // Verify current is positive and reasonable
    // Loop: V_source + V_bridge - I*R = 0
    // I = (V_source + V_bridge) / R
    // Use theoretical voltage for expectation to avoid transient skewing
    double expected_i = (160.0 + v_bridge_expected) / dc_link_resistance.value();
    EXPECT_GT(i_dc_final, 10.0);
    EXPECT_NEAR(i_dc_final, expected_i, expected_i * 0.25);
}

}  // namespace
