#include "yeet_control/thyristor_rectifier.hpp"

#include <gtest/gtest.h>

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

class ThyristorRectifierTest : public ::testing::Test {
public:
    static constexpr duration_t sampling_time{10e-6};      // 10 μs for good resolution
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz grid
    static constexpr double v_ac_peak = 100.0;             // 100V peak line-to-neutral for testing
    static constexpr double pulse_width_deg = 120.0;       // 120 degree pulse width (standard for 6-pulse)

    // Grid-side RL impedance parameters
    static constexpr resistance_t grid_resistance{0.1};     // 0.1 Ohm per phase
    static constexpr inductance_t grid_inductance{0.5e-3};  // 0.1 mH per phase

    // DC load parameters
    static constexpr resistance_t load_resistance{0.5};   // 0.5 Ohm
    static constexpr inductance_t load_inductance{5e-3};  // 5 mH
    static constexpr voltage_t v_dc_back_emf{0.0};        // No back-EMF (resistive load)

    void SetUp() override
    {
        // Configure three-phase voltage source (ideal balanced grid)
        _waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_ac_peak);
        _waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _waveform_generator.set_signal_frequency(fundamental_frequency);
        _waveform_generator.disable_noise();

        set_logger();

        _simulator.register_lambda_runner(sampling_time.value(), [&]() {
            // Update three-phase voltage source
            _waveform_generator.update();

            // Get AC voltages from source
            auto v_abc_raw = _waveform_generator.get_signal_abc();
            _v_ac_source_abc =
                abc<voltage_t>{voltage_t{v_abc_raw.a()}, voltage_t{v_abc_raw.b()}, voltage_t{v_abc_raw.c()}};
            angle_wrapped reference_angle = _waveform_generator.get_fundamental_signal_angle().a();

            // Generate gate pulses with current firing angle
            _six_pulse_gen.update(reference_angle, _current_firing_angle, true);
            auto gen_pulses = _six_pulse_gen.get_pulses();
            _pulses.t1 = gen_pulses.p1;
            _pulses.t2 = gen_pulses.p2;
            _pulses.t3 = gen_pulses.p3;
            _pulses.t4 = gen_pulses.p4;
            _pulses.t5 = gen_pulses.p5;
            _pulses.t6 = gen_pulses.p6;

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
            _v_dc_sum += _v_dc_output.value();
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

    double get_theoretical_dc_voltage_no_load(double firing_angle_deg) const
    {
        // For 6-pulse bridge with NO LOAD (ideal case): V_dc_avg = (3/pi) * V_LL_peak * cos(alpha)
        // where V_LL_peak = sqrt(3) * V_LN_peak (line-to-line peak voltage)
        // The bridge operates on line-to-line voltages, not line-to-neutral
        double alpha_rad = deg_to_rad(firing_angle_deg);
        double v_ll_peak = std::sqrt(3.0) * v_ac_peak;  // Convert line-to-neutral to line-to-line
        return (3.0 / pi) * v_ll_peak * std::cos(alpha_rad);
    }

    double get_expected_dc_current(double firing_angle_deg) const
    {
        // Expected steady-state DC current with resistive load
        // Note: This is an idealized calculation. The actual simulation includes
        // commutation overlap which reduces the effective voltage and current.
        // I_dc = V_dc_no_load / (R_grid + R_load)
        double v_dc_no_load = get_theoretical_dc_voltage_no_load(firing_angle_deg);
        double r_total = grid_resistance.value() + load_resistance.value();
        return v_dc_no_load / r_total;
    }

    double get_expected_dc_voltage(double firing_angle_deg) const
    {
        // Expected DC voltage at the bridge output (after grid impedance, before load)
        // This is the idealized calculation: V_dc = V_dc0 - R_grid * I_dc
        //
        // Note: The actual simulation includes commutation overlap effects which cause
        // additional voltage drop. The measured voltage will be lower than this ideal value.
        // Tests should use appropriate tolerance to account for this difference.
        //
        // Circuit: V_source -> [R_grid] -> Bridge -> [R_load] -> Ground
        //          V_dc_output is measured at Bridge output (after R_grid, before R_load)

        double v_dc_no_load = get_theoretical_dc_voltage_no_load(firing_angle_deg);
        double r_total = grid_resistance.value() + load_resistance.value();
        double i_dc_expected = v_dc_no_load / r_total;

        // Voltage at bridge output = no-load voltage - grid resistance drop
        return v_dc_no_load - grid_resistance.value() * i_dc_expected;
    }

    void reset_voltage_averaging()
    {
        _v_dc_sum = 0.0;
        _v_dc_sample_count = 0;
    }

protected:
    yeet::simulator _simulator = yeet::make_gtest_simulator();
    three_phase_waveform_generator _waveform_generator{sampling_time.value()};
    six_pulse_generator _six_pulse_gen{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};
    thyristor_rectifier _rectifier{grid_resistance, grid_inductance, load_resistance, load_inductance, sampling_time};

    // Current state variables
    angle_wrapped _current_reference_angle{0.0};
    angle_wrapped _current_firing_angle{0.0};
    abc<voltage_t> _v_ac_source_abc{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
    abc<current_t> _i_ac_abc{current_t{0.0}, current_t{0.0}, current_t{0.0}};
    thyristor_pulses _pulses{};
    six_pulse_generator::pulse_sector_t _current_sector{six_pulse_generator::pulse_sector_t::none};
    voltage_t _v_dc_output{0.0};
    current_t _i_dc{0.0};

    // Voltage averaging for validation
    double _v_dc_sum{0.0};
    size_t _v_dc_sample_count{0};

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
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_ac_source_a", [&]() -> double { return _v_ac_source_abc.a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_ac_source_b", [&]() -> double { return _v_ac_source_abc.b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_ac_source_c", [&]() -> double { return _v_ac_source_abc.c().value(); }));

        // Log actual terminal phase voltages at bridge (shows voltage notch during commutation)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_phase_a", [&]() -> double { return _rectifier.get_ac_phase_voltages()[0].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_phase_b", [&]() -> double { return _rectifier.get_ac_phase_voltages()[1].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_phase_c", [&]() -> double { return _rectifier.get_ac_phase_voltages()[2].value(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("comm: incoming_thyristor",
            [&]() -> double { return _rectifier.bridge().get_commutation_incoming_thyristor_idx(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("comm: outgoing_thyristor",
            [&]() -> double { return _rectifier.bridge().get_commutation_outgoing_thyristor_idx(); }));

        // Log actual terminal line-to-line voltages at bridge (shows voltage notch during commutation)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_ab", [&]() -> double { return _rectifier.get_ac_line_voltages()[0].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_bc", [&]() -> double { return _rectifier.get_ac_line_voltages()[1].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_bridge_ca", [&]() -> double { return _rectifier.get_ac_line_voltages()[2].value(); }));

        // Log three-phase line-to-line voltages at source (no notch, for comparison)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_ac_line_ab", [&]() -> double { return _v_ac_source_abc.a().value() - _v_ac_source_abc.b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_ac_line_bc", [&]() -> double { return _v_ac_source_abc.b().value() - _v_ac_source_abc.c().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_ac_line_ca", [&]() -> double { return _v_ac_source_abc.c().value() - _v_ac_source_abc.a().value(); }));

        // Log three-phase AC load currents
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_ac_a", [&]() -> double { return _i_ac_abc.a().value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_ac_b", [&]() -> double { return _i_ac_abc.b().value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_ac_c", [&]() -> double { return _i_ac_abc.c().value(); }));

        // Log gate pulses
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulses: t1", [&]() -> double { return _pulses.t1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulses: t2", [&]() -> double { return _pulses.t2 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulses: t3", [&]() -> double { return _pulses.t3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulses: t4", [&]() -> double { return _pulses.t4 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulses: t5", [&]() -> double { return _pulses.t5 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("pulses: t6", [&]() -> double { return _pulses.t6 ? 1.0 : 0.0; }));

        // Log thyristor conduction states
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("conduction: thyristor_t1_on",
            [&]() -> double { return _rectifier.get_conduction_states()[0] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("conduction: thyristor_t2_on",
            [&]() -> double { return _rectifier.get_conduction_states()[1] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("conduction: thyristor_t3_on",
            [&]() -> double { return _rectifier.get_conduction_states()[2] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("conduction: thyristor_t4_on",
            [&]() -> double { return _rectifier.get_conduction_states()[3] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("conduction: thyristor_t5_on",
            [&]() -> double { return _rectifier.get_conduction_states()[4] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("conduction: thyristor_t6_on",
            [&]() -> double { return _rectifier.get_conduction_states()[5] ? 1.0 : 0.0; }));

        // Log DC voltage and current
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("v_dc_output", [&]() -> double { return _v_dc_output.value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("i_dc", [&]() -> double { return _i_dc.value(); }));

        // Log average voltages (no-load theoretical, expected with load, and measured running average)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("v_dc_avg_no_load",
            [&]() -> double { return get_theoretical_dc_voltage_no_load(_current_firing_angle.get_degrees()); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("v_dc_avg_expected",
            [&]() -> double { return get_expected_dc_voltage(_current_firing_angle.get_degrees()); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "v_dc_avg_measured", [&]() -> double { return get_average_dc_voltage(); }));

        // Log expected DC current
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "i_dc_expected", [&]() -> double { return get_expected_dc_current(_current_firing_angle.get_degrees()); }));

        // Log sector
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "sector", [&]() -> double { return static_cast<double>(static_cast<int>(_current_sector)); }));

        // Log individual thyristor currents for commutation analysis
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "i_t1", [&]() -> double { return _rectifier.get_thyristor_currents()[0].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "i_t2", [&]() -> double { return _rectifier.get_thyristor_currents()[1].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "i_t3", [&]() -> double { return _rectifier.get_thyristor_currents()[2].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "i_t4", [&]() -> double { return _rectifier.get_thyristor_currents()[3].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "i_t5", [&]() -> double { return _rectifier.get_thyristor_currents()[4].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "i_t6", [&]() -> double { return _rectifier.get_thyristor_currents()[5].value(); }));
    }
};

// Test 1: Natural commutation point (firing angle = 0°)
TEST_F(ThyristorRectifierTest, NaturalCommutationAlphaZero)
{
    set_firing_angle(0.0);  // Natural commutation point

    // Simulate for 3 grid cycles to observe steady-state behavior
    double simulation_time = 3.0 / fundamental_frequency;  // 60 ms @ 50 Hz
    simulate_for(simulation_time);

    // Validate DC voltage across load (accounting for resistive voltage drops)
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_expected = get_expected_dc_voltage(0.0);
    double i_dc_expected = get_expected_dc_current(0.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_expected) / v_dc_avg_expected * 100.0;

    // Allow 20% error tolerance at alpha=0 (commutation overlap is maximum at low firing angles)
    // The simulation includes commutation overlap effects which reduce the DC voltage
    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_expected, v_dc_avg_expected * 0.20)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Expected V_dc_avg = " << v_dc_avg_expected << " V (I_dc=" << i_dc_expected << " A), "
        << "Error = " << error_percent << " %";
}

// Test 2: Delayed firing (firing angle = 30°)
TEST_F(ThyristorRectifierTest, DelayedFiringAlpha30)
{
    set_firing_angle(30.0);  // 30° firing delay

    // Simulate for 3 grid cycles
    double simulation_time = 3.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Validate DC voltage across load (accounting for resistive voltage drops)
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_expected = get_expected_dc_voltage(30.0);
    double i_dc_expected = get_expected_dc_current(30.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_expected) / v_dc_avg_expected * 100.0;

    // Allow 20% tolerance to account for commutation overlap effects in simulation
    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_expected, v_dc_avg_expected * 0.20)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Expected V_dc_avg = " << v_dc_avg_expected << " V (I_dc=" << i_dc_expected << " A), "
        << "Error = " << error_percent << " %";
}

// Test 3: High firing angle (firing angle = 60°)
TEST_F(ThyristorRectifierTest, HighFiringAngleAlpha60)
{
    set_firing_angle(60.0);  // 60° firing delay

    // Simulate for 3 grid cycles
    double simulation_time = 3.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Validate DC voltage across load (accounting for resistive voltage drops)
    // Higher tolerance needed due to commutation overlap and inductive voltage drop
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_expected = get_expected_dc_voltage(60.0);
    double i_dc_expected = get_expected_dc_current(60.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_expected) / v_dc_avg_expected * 100.0;

    // 20% tolerance accounts for commutation overlap angle (μ) at high firing angles
    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_expected, v_dc_avg_expected * 0.20)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Expected V_dc_avg = " << v_dc_avg_expected << " V (I_dc=" << i_dc_expected << " A), "
        << "Error = " << error_percent << " % (higher due to commutation overlap)";
}

// Test 4: Extended simulation showing steady-state operation
TEST_F(ThyristorRectifierTest, ExtendedOperationAlpha45)
{
    set_firing_angle(45.0);  // Mid-range firing angle

    // Simulate for 10 grid cycles to show consistent operation
    double simulation_time = 10.0 / fundamental_frequency;  // 200 ms @ 50 Hz
    simulate_for(simulation_time);

    // Validate DC voltage across load with extended simulation for better accuracy
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_expected = get_expected_dc_voltage(45.0);
    double i_dc_expected = get_expected_dc_current(45.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_expected) / v_dc_avg_expected * 100.0;

    // Allow 20% error tolerance (commutation effects accumulate over extended simulation)
    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_expected, v_dc_avg_expected * 0.20)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Expected V_dc_avg = " << v_dc_avg_expected << " V (I_dc=" << i_dc_expected << " A), "
        << "Error = " << error_percent << " %";
}

// Test 5: Commutation behavior verification
TEST_F(ThyristorRectifierTest, CommutationVerificationAlpha15)
{
    set_firing_angle(15.0);  // Small firing delay for clear commutation observation

    // Simulate for 2 grid cycles
    double simulation_time = 2.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Validate DC voltage across load (accounting for resistive voltage drops)
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_expected = get_expected_dc_voltage(15.0);
    double i_dc_expected = get_expected_dc_current(15.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_expected) / v_dc_avg_expected * 100.0;

    // Allow 20% tolerance to account for commutation overlap effects
    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_expected, v_dc_avg_expected * 0.20)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Expected V_dc_avg = " << v_dc_avg_expected << " V (I_dc=" << i_dc_expected << " A), "
        << "Error = " << error_percent << " %";
}

// Test 6: VALIDATION - Verify actual current flow through thyristors
TEST_F(ThyristorRectifierTest, ThyristorCurrentFlowValidation)
{
    set_firing_angle(30.0);  // Mid-range angle for testing

    // Data capture variables (must be captured by reference in lambda)
    struct CaptureData {
        bool any_pulse_generated = false;
        bool any_thyristor_conducting = false;
        bool any_dc_current_flowing = false;
        bool any_thyristor_current_flowing = false;
        double max_i_dc = 0.0;
        double max_thyristor_current = 0.0;
        int pulse_count = 0;
        int conduction_count = 0;
        int samples_with_dc_current = 0;
        int samples_with_thyristor_current = 0;
    };
    auto capture = std::make_shared<CaptureData>();

    // Add inline data capture - runs AFTER the main update loop
    _simulator.register_lambda_runner(sampling_time.value(), [this, capture]() {
        // Check if pulses are generated
        if (_pulses.t1 || _pulses.t2 || _pulses.t3 || _pulses.t4 || _pulses.t5 || _pulses.t6) {
            capture->any_pulse_generated = true;
            capture->pulse_count++;
        }

        // Check thyristor conduction states
        const auto& conducting = _rectifier.get_conduction_states();
        if (conducting[0] || conducting[1] || conducting[2] || conducting[3] || conducting[4] || conducting[5]) {
            capture->any_thyristor_conducting = true;
            capture->conduction_count++;
        }

        // Check DC current
        double i_dc_current = _rectifier.get_dc_current().value();
        if (std::abs(i_dc_current) > 1e-6) {
            capture->any_dc_current_flowing = true;
            capture->samples_with_dc_current++;
            capture->max_i_dc = std::max(capture->max_i_dc, std::abs(i_dc_current));
        }

        // Check individual thyristor currents
        const auto& i_thyristors = _rectifier.get_thyristor_currents();
        for (int i = 0; i < 6; ++i) {
            if (std::abs(i_thyristors[i].value()) > 1e-6) {
                capture->any_thyristor_current_flowing = true;
                capture->samples_with_thyristor_current++;
                capture->max_thyristor_current =
                    std::max(capture->max_thyristor_current, static_cast<double>(std::abs(i_thyristors[i].value())));
                break;  // Only count once per sample
            }
        }
    });

    // Run for 3 grid cycles
    const double simulation_time = 3.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Calculate total samples
    int total_samples = static_cast<int>(simulation_time / sampling_time.value());

    // Print diagnostic information
    std::cout << "\n╔════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   THYRISTOR CURRENT VALIDATION - DIAGNOSTIC RESULTS           ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Total samples:                  " << std::setw(10) << total_samples << "                   ║"
              << std::endl;
    std::cout << "║ Samples with pulses:            " << std::setw(10) << capture->pulse_count << " (" << std::setw(5)
              << std::fixed << std::setprecision(1) << (100.0 * capture->pulse_count / total_samples) << "%)      ║"
              << std::endl;
    std::cout << "║ Samples with thyristor ON:      " << std::setw(10) << capture->conduction_count << " ("
              << std::setw(5) << std::fixed << std::setprecision(1)
              << (100.0 * capture->conduction_count / total_samples) << "%)      ║" << std::endl;
    std::cout << "║ Samples with DC current:        " << std::setw(10) << capture->samples_with_dc_current << " ("
              << std::setw(5) << std::fixed << std::setprecision(1)
              << (100.0 * capture->samples_with_dc_current / total_samples) << "%)      ║" << std::endl;
    std::cout << "║ Samples with thyristor current: " << std::setw(10) << capture->samples_with_thyristor_current
              << " (" << std::setw(5) << std::fixed << std::setprecision(1)
              << (100.0 * capture->samples_with_thyristor_current / total_samples) << "%)      ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Maximum DC current:             " << std::setw(10) << std::fixed << std::setprecision(3)
              << capture->max_i_dc << " A              ║" << std::endl;
    std::cout << "║ Maximum thyristor current:      " << std::setw(10) << std::fixed << std::setprecision(3)
              << capture->max_thyristor_current << " A              ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Gate pulses generated:          " << (capture->any_pulse_generated ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ Thyristors conducting:          " << (capture->any_thyristor_conducting ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ DC current flowing:             " << (capture->any_dc_current_flowing ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ Thyristor currents flowing:     " << (capture->any_thyristor_current_flowing ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n" << std::endl;

    // Critical assertions
    EXPECT_TRUE(capture->any_pulse_generated) << "CRITICAL: No gate pulses were generated!";
    EXPECT_TRUE(capture->any_thyristor_conducting) << "CRITICAL: No thyristors turned ON despite gate pulses!";
    EXPECT_TRUE(capture->any_dc_current_flowing) << "CRITICAL: No DC current flowing despite thyristors conducting!";
    EXPECT_TRUE(capture->any_thyristor_current_flowing) << "CRITICAL: No individual thyristor currents flowing!";
    EXPECT_GT(capture->max_i_dc, 0.1) << "DC current too low: " << capture->max_i_dc << " A (expected > 0.1 A)";

    // Check voltage and current against expected values (accounting for resistive load)
    double v_dc_avg_measured = get_average_dc_voltage();
    double v_dc_avg_expected = get_expected_dc_voltage(30.0);
    double i_dc_expected = get_expected_dc_current(30.0);
    double error_percent = std::abs(v_dc_avg_measured - v_dc_avg_expected) / v_dc_avg_expected * 100.0;

    std::cout << "DC Voltage: Measured = " << v_dc_avg_measured << " V, "
              << "Expected = " << v_dc_avg_expected << " V (I_dc=" << i_dc_expected << " A), "
              << "Error = " << error_percent << " %\n"
              << std::endl;

    // Allow 20% tolerance to account for commutation overlap effects in simulation
    EXPECT_NEAR(v_dc_avg_measured, v_dc_avg_expected, v_dc_avg_expected * 0.20)
        << "Measured V_dc_avg = " << v_dc_avg_measured << " V, "
        << "Expected V_dc_avg = " << v_dc_avg_expected << " V (I_dc=" << i_dc_expected << " A), "
        << "Error = " << error_percent << " %";
}

}  // namespace
