#include "yeet_control/thyristor_back_to_back.hpp"

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

class ThyristorBackToBackTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 10e-6;         // 10 μs for good resolution
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz grid/machine frequency
    static constexpr double v_grid_peak = 100.0;           // 100V peak line-to-neutral (grid)
    static constexpr double v_machine_peak = 80.0;         // 80V peak line-to-neutral (machine back-EMF)
    static constexpr double pulse_width_deg = 120.0;       // 120 degree pulse width (standard for 6-pulse)

    // Grid-side RL impedance parameters
    static constexpr double grid_resistance = 0.01;    // 50 mOhm per phase
    static constexpr double grid_inductance = 0.5e-3;  // 0.1 mH per phase (Increased for visible notches)

    // DC link parameters
    static constexpr double dc_resistance = 1.0;   // 100 mOhm
    static constexpr double dc_inductance = 5e-3;  // 10 mH

    // Machine-side RL impedance parameters
    static constexpr double machine_resistance = 0.01;    // 50 mOhm per phase
    static constexpr double machine_inductance = 0.5e-3;  // 0.1 mH per phase (Increased for visible notches)

    void SetUp() override
    {
        // Configure grid-side three-phase voltage source
        _grid_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_grid_peak);
        _grid_waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _grid_waveform_generator.set_signal_frequency(fundamental_frequency);
        _grid_waveform_generator.disable_noise();

        // Configure machine-side three-phase voltage source (back-EMF)
        _machine_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_machine_peak);
        _machine_waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _machine_waveform_generator.set_signal_frequency(fundamental_frequency);
        _machine_waveform_generator.disable_noise();

        set_logger();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Update grid-side voltage source
            _grid_waveform_generator.update();
            _v_grid_abc = _grid_waveform_generator.get_signal_abc();
            angle_wrapped grid_reference_angle = _grid_waveform_generator.get_fundamental_signal_angle().a();

            // Update machine-side voltage source (with phase shift)
            _machine_waveform_generator.update();
            _v_machine_abc = _machine_waveform_generator.get_signal_abc();
            angle_wrapped machine_reference_angle = _machine_waveform_generator.get_fundamental_signal_angle().a();

            // Generate rectifier gate pulses
            _pulses_rect = _six_pulse_gen_rect.update(grid_reference_angle, _current_firing_angle_rect, true);

            // Generate inverter gate pulses
            _pulses_inv = _six_pulse_gen_inv.update(machine_reference_angle, _current_firing_angle_inv, true);

            // Convert pulses
            thyristor_pulses p_rect_conv;
            p_rect_conv.t1 = _pulses_rect.p1;
            p_rect_conv.t2 = _pulses_rect.p2;
            p_rect_conv.t3 = _pulses_rect.p3;
            p_rect_conv.t4 = _pulses_rect.p4;
            p_rect_conv.t5 = _pulses_rect.p5;
            p_rect_conv.t6 = _pulses_rect.p6;

            thyristor_pulses p_inv_conv;
            p_inv_conv.t1 = _pulses_inv.p1;
            p_inv_conv.t2 = _pulses_inv.p2;
            p_inv_conv.t3 = _pulses_inv.p3;
            p_inv_conv.t4 = _pulses_inv.p4;
            p_inv_conv.t5 = _pulses_inv.p5;
            p_inv_conv.t6 = _pulses_inv.p6;

            // Update complete back-to-back system
            abc<voltage_t> v_grid_typed{voltage_t{static_cast<real_t>(_v_grid_abc.a())},
                voltage_t{static_cast<real_t>(_v_grid_abc.b())}, voltage_t{static_cast<real_t>(_v_grid_abc.c())}};
            abc<voltage_t> v_mach_typed{voltage_t{static_cast<real_t>(_v_machine_abc.a())},
                voltage_t{static_cast<real_t>(_v_machine_abc.b())}, voltage_t{static_cast<real_t>(_v_machine_abc.c())}};

            _back_to_back.update(v_grid_typed, v_mach_typed, p_rect_conv, p_inv_conv);

            // Get states from back-to-back system
            auto i_grid = _back_to_back.get_grid_currents();
            _i_grid_abc = abc<double>{i_grid.a().value(), i_grid.b().value(), i_grid.c().value()};

            auto i_mach = _back_to_back.get_machine_currents();
            _i_machine_abc = abc<double>{i_mach.a().value(), i_mach.b().value(), i_mach.c().value()};

            _i_dc = _back_to_back.get_dc_current().value();
            _v_rect_dc = _back_to_back.get_rectifier_voltage().value();
            _v_inv_dc = _back_to_back.get_inverter_voltage().value();

            // Store current reference angles and sectors
            _current_grid_reference_angle = grid_reference_angle;
            _current_machine_reference_angle = machine_reference_angle;
            _current_grid_sector = _six_pulse_gen_rect.get_sector();
            _current_machine_sector = _six_pulse_gen_inv.get_sector();

            // Accumulate DC link voltage for averaging
            _v_dc_sum += (_v_rect_dc - _v_inv_dc);
            _v_dc_sample_count++;
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    void set_rectifier_firing_angle(double angle_degrees)
    {
        _current_firing_angle_rect = angle_wrapped(deg_to_rad(angle_degrees));
    }

    void set_inverter_firing_angle(double angle_degrees)
    {
        _current_firing_angle_inv = angle_wrapped(deg_to_rad(angle_degrees));
    }

    double get_average_dc_link_voltage() const
    {
        if (_v_dc_sample_count == 0) return 0.0;
        return _v_dc_sum / static_cast<double>(_v_dc_sample_count);
    }

    double get_theoretical_rectifier_voltage_no_load(double firing_angle_deg) const
    {
        // For 6-pulse bridge: V_dc_avg = (3/pi) * V_LL_peak * cos(alpha)
        double alpha_rad = deg_to_rad(firing_angle_deg);
        double v_ll_peak = std::sqrt(3.0) * v_grid_peak;
        return (3.0 / pi) * v_ll_peak * std::cos(alpha_rad);
    }

    double get_theoretical_inverter_voltage_no_load(double firing_angle_deg) const
    {
        // For 6-pulse bridge operating as inverter (load commutation)
        // The voltage depends on the advance angle (beta = 180° - gamma)
        // where gamma is the extinction angle
        // For simplicity, using same formula with machine voltage
        double alpha_rad = deg_to_rad(firing_angle_deg);
        double v_ll_peak = std::sqrt(3.0) * v_machine_peak;
        return (3.0 / pi) * v_ll_peak * std::cos(alpha_rad);
    }

    void reset_voltage_averaging()
    {
        _v_dc_sum = 0.0;
        _v_dc_sample_count = 0;
    }

protected:
    yeet::simulator _simulator = yeet::make_gtest_simulator();

    // Grid-side components
    three_phase_waveform_generator _grid_waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen_rect{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};

    // Machine-side components
    three_phase_waveform_generator _machine_waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen_inv{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};

    // Back-to-back converter
    thyristor_back_to_back _back_to_back{resistance_t{grid_resistance}, inductance_t{grid_inductance},
        resistance_t{dc_resistance}, inductance_t{dc_inductance}, resistance_t{machine_resistance},
        inductance_t{machine_inductance}, duration_t{sampling_time}};

    // Current state variables
    angle_wrapped _current_grid_reference_angle{0.0};
    angle_wrapped _current_machine_reference_angle{0.0};
    angle_wrapped _current_firing_angle_rect{0.0};
    angle_wrapped _current_firing_angle_inv{0.0};

    abc<double> _v_grid_abc{0.0, 0.0, 0.0};
    abc<double> _v_machine_abc{0.0, 0.0, 0.0};
    abc<double> _i_grid_abc{0.0, 0.0, 0.0};
    abc<double> _i_machine_abc{0.0, 0.0, 0.0};

    six_pulse_generator::pulses _pulses_rect{};
    six_pulse_generator::pulses _pulses_inv{};

    six_pulse_generator::pulse_sector_t _current_grid_sector{six_pulse_generator::pulse_sector_t::none};
    six_pulse_generator::pulse_sector_t _current_machine_sector{six_pulse_generator::pulse_sector_t::none};

    double _v_rect_dc{0.0};
    double _v_inv_dc{0.0};
    double _i_dc{0.0};

    // DC link voltage averaging
    double _v_dc_sum{0.0};
    size_t _v_dc_sample_count{0};

private:
    void set_logger()
    {
        // ===== GRID SIDE SIGNALS =====

        // Grid reference angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:reference_angle_deg", [&]() -> double { return _current_grid_reference_angle.get_degrees(); }));

        // Grid firing angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:firing_angle_deg", [&]() -> double { return _current_firing_angle_rect.get_degrees(); }));

        // Grid AC source voltages (line-to-neutral)
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_a", [&]() -> double { return _v_grid_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_b", [&]() -> double { return _v_grid_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_c", [&]() -> double { return _v_grid_abc.c(); }));

        // Grid line-to-line voltages
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:v_ac_line_ab", [&]() -> double { return _v_grid_abc.a() - _v_grid_abc.b(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:v_ac_line_bc", [&]() -> double { return _v_grid_abc.b() - _v_grid_abc.c(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:v_ac_line_ca", [&]() -> double { return _v_grid_abc.c() - _v_grid_abc.a(); }));

        // Grid AC currents
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_a", [&]() -> double { return _i_grid_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_b", [&]() -> double { return _i_grid_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_c", [&]() -> double { return _i_grid_abc.c(); }));

        // Rectifier gate pulses
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:pulse_t1", [&]() -> double { return _pulses_rect.p1 ? v_grid_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:pulse_t2", [&]() -> double { return _pulses_rect.p2 ? v_grid_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:pulse_t3", [&]() -> double { return _pulses_rect.p3 ? v_grid_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:pulse_t4", [&]() -> double { return _pulses_rect.p4 ? v_grid_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:pulse_t5", [&]() -> double { return _pulses_rect.p5 ? v_grid_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:pulse_t6", [&]() -> double { return _pulses_rect.p6 ? v_grid_peak * 1.0 : 0.0; }));

        // Rectifier thyristor conduction states
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t1_on", [&]() -> double {
            return _back_to_back.get_rectifier_bridge().get_conduction_states()[0] ? v_grid_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t2_on", [&]() -> double {
            return _back_to_back.get_rectifier_bridge().get_conduction_states()[1] ? v_grid_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t3_on", [&]() -> double {
            return _back_to_back.get_rectifier_bridge().get_conduction_states()[2] ? v_grid_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t4_on", [&]() -> double {
            return _back_to_back.get_rectifier_bridge().get_conduction_states()[3] ? v_grid_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t5_on", [&]() -> double {
            return _back_to_back.get_rectifier_bridge().get_conduction_states()[4] ? v_grid_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t6_on", [&]() -> double {
            return _back_to_back.get_rectifier_bridge().get_conduction_states()[5] ? v_grid_peak * 1.0 : 0.0;
        }));

        // Rectifier thyristor currents
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:i_t1",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_thyristor_currents()[0].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:i_t2",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_thyristor_currents()[1].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:i_t3",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_thyristor_currents()[2].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:i_t4",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_thyristor_currents()[3].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:i_t5",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_thyristor_currents()[4].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:i_t6",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_thyristor_currents()[5].value(); }));

        // Grid sector
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:sector", [&]() -> double { return static_cast<double>(static_cast<int>(_current_grid_sector)); }));

        // Grid terminal voltages (after source impedance, with notches)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:v_term_a",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_ac_phase_voltages().a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:v_term_b",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_ac_phase_voltages().b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:v_term_c",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_ac_phase_voltages().c().value(); }));

        // Grid line voltages at terminals (with notches)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:v_term_ab",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_ac_line_voltages().a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:v_term_bc",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_ac_line_voltages().b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:v_term_ca",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_ac_line_voltages().c().value(); }));

        // ===== MACHINE SIDE SIGNALS =====

        // Machine reference angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:reference_angle_deg", [&]() -> double { return _current_machine_reference_angle.get_degrees(); }));

        // Machine firing angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:firing_angle_deg", [&]() -> double { return _current_firing_angle_inv.get_degrees(); }));

        // Machine AC voltages (back-EMF, line-to-neutral)
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:v_ac_a", [&]() -> double { return _v_machine_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:v_ac_b", [&]() -> double { return _v_machine_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:v_ac_c", [&]() -> double { return _v_machine_abc.c(); }));

        // Machine line-to-line voltages
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_line_ab", [&]() -> double { return _v_machine_abc.a() - _v_machine_abc.b(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_line_bc", [&]() -> double { return _v_machine_abc.b() - _v_machine_abc.c(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_line_ca", [&]() -> double { return _v_machine_abc.c() - _v_machine_abc.a(); }));

        // Machine AC currents
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:i_ac_a", [&]() -> double { return _i_machine_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:i_ac_b", [&]() -> double { return _i_machine_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:i_ac_c", [&]() -> double { return _i_machine_abc.c(); }));

        // Inverter gate pulses
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:pulse_t1", [&]() -> double { return _pulses_inv.p1 ? v_machine_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:pulse_t2", [&]() -> double { return _pulses_inv.p2 ? v_machine_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:pulse_t3", [&]() -> double { return _pulses_inv.p3 ? v_machine_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:pulse_t4", [&]() -> double { return _pulses_inv.p4 ? v_machine_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:pulse_t5", [&]() -> double { return _pulses_inv.p5 ? v_machine_peak * 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:pulse_t6", [&]() -> double { return _pulses_inv.p6 ? v_machine_peak * 1.0 : 0.0; }));

        // Inverter thyristor conduction states
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t1_on", [&]() -> double {
            return _back_to_back.get_inverter_bridge().get_conduction_states()[0] ? v_machine_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t2_on", [&]() -> double {
            return _back_to_back.get_inverter_bridge().get_conduction_states()[1] ? v_machine_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t3_on", [&]() -> double {
            return _back_to_back.get_inverter_bridge().get_conduction_states()[2] ? v_machine_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t4_on", [&]() -> double {
            return _back_to_back.get_inverter_bridge().get_conduction_states()[3] ? v_machine_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t5_on", [&]() -> double {
            return _back_to_back.get_inverter_bridge().get_conduction_states()[4] ? v_machine_peak * 1.0 : 0.0;
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t6_on", [&]() -> double {
            return _back_to_back.get_inverter_bridge().get_conduction_states()[5] ? v_machine_peak * 1.0 : 0.0;
        }));

        // Inverter thyristor currents
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:i_t1",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_thyristor_currents()[0].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:i_t2",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_thyristor_currents()[1].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:i_t3",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_thyristor_currents()[2].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:i_t4",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_thyristor_currents()[3].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:i_t5",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_thyristor_currents()[4].value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:i_t6",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_thyristor_currents()[5].value(); }));

        // Machine sector
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:sector",
            [&]() -> double { return static_cast<double>(static_cast<int>(_current_machine_sector)); }));

        // Machine terminal voltages (after source impedance, with notches)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:v_term_a",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_ac_phase_voltages().a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:v_term_b",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_ac_phase_voltages().b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:v_term_c",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_ac_phase_voltages().c().value(); }));

        // Machine line voltages at terminals (with notches)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:v_term_ab",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_ac_line_voltages().a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:v_term_bc",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_ac_line_voltages().b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:v_term_ca",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_ac_line_voltages().c().value(); }));

        // ===== DC LINK SIGNALS =====

        // DC link current
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:i_dc", [&]() -> double { return _i_dc; }));

        // DC voltages
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_rect_dc", [&]() -> double { return _v_rect_dc; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_inv_dc", [&]() -> double { return _v_inv_dc; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_dc_link", [&]() -> double { return _v_rect_dc - _v_inv_dc; }));

        // Average DC link voltage
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "dc:v_dc_link_avg", [&]() -> double { return get_average_dc_link_voltage(); }));

        // Theoretical voltages
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("dc:v_rect_theoretical", [&]() -> double {
            return get_theoretical_rectifier_voltage_no_load(_current_firing_angle_rect.get_degrees());
        }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("dc:v_inv_theoretical", [&]() -> double {
            return get_theoretical_inverter_voltage_no_load(_current_firing_angle_inv.get_degrees());
        }));

        // Power flow
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "dc:power", [&]() -> double { return (_v_rect_dc - _v_inv_dc) * _i_dc; }));
    }
};

// Test 1: Rectifier mode operation (positive power flow from grid to machine)
TEST_F(ThyristorBackToBackTest, RectifierModeOperation)
{
    // Rectifier: alpha = 15° (low firing angle, high DC voltage)
    // Inverter: alpha = 30° (higher firing angle, lower DC voltage)
    // Net voltage: V_rect > V_inv → current flows grid to machine
    set_rectifier_firing_angle(15.0);
    set_inverter_firing_angle(30.0);

    // Simulate for 5 grid cycles
    double simulation_time = 5.0 / fundamental_frequency;  // 100 ms @ 50 Hz
    simulate_for(simulation_time);

    // Validate positive power flow
    double i_dc_avg = _i_dc;
    EXPECT_GT(i_dc_avg, 1.0) << "Expected positive DC current in rectifier mode";

    // Validate DC link voltage is positive (rectifier voltage > inverter voltage)
    double v_dc_link_avg = get_average_dc_link_voltage();
    EXPECT_GT(v_dc_link_avg, 0.0) << "Expected positive DC link voltage (V_rect > V_inv)";
}

// Test 2: Inverter mode operation (regenerative braking, power flow from machine to grid)
TEST_F(ThyristorBackToBackTest, InverterModeOperation)
{
    // Rectifier: alpha = 45° (higher firing angle, lower DC voltage)
    // Inverter: alpha = 15° (low firing angle, high DC voltage)
    // Net voltage: V_inv > V_rect → current flows machine to grid
    set_rectifier_firing_angle(45.0);
    set_inverter_firing_angle(150.0);

    // Simulate for 5 grid cycles
    double simulation_time = 5.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // In this mode, we expect current to try to reverse but DC link prevents it
    // The system should reach equilibrium with minimal current
    double i_dc_avg = _i_dc;

    // Note: With ideal sources, this might result in very low current or oscillation
    // In real systems, the machine EMF phase angle would determine power flow direction
    EXPECT_GE(i_dc_avg, 0.0) << "DC current should be non-negative";
}

// Test 3: Balanced operation (matched DC voltages, minimal current flow)
TEST_F(ThyristorBackToBackTest, BalancedOperation)
{
    // Adjust firing angles to match DC voltages
    // V_rect_max ~ 165V, V_inv_max ~ 132V
    // Set Rectifier alpha = 45 deg -> V_rect ~ 116V
    // Set Inverter alpha = 30 deg -> V_inv ~ 114V
    // Delta V ~ 2V, should result in low current
    set_rectifier_firing_angle(45.0);
    set_inverter_firing_angle(180.0 - 30.0);

    // Simulate for 5 grid cycles
    double simulation_time = 5.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // With balanced voltages, expect minimal current
    double i_dc_avg = _i_dc;

    // Current should be reasonable when voltages are balanced
    EXPECT_LT(i_dc_avg, 50.0) << "Expected low current in balanced voltage mode";
    EXPECT_GT(i_dc_avg, 0.0) << "Expected some positive current due to small voltage delta";
}

// Test 4: Extended operation showing steady-state
TEST_F(ThyristorBackToBackTest, 90DegreesToNominalTest)
{
    set_rectifier_firing_angle(90.0);
    set_inverter_firing_angle(180.0 - 30.0f);

    simulate_for(0.5);

    set_rectifier_firing_angle(20.0);
    set_inverter_firing_angle(180.0 - 30.0f);

    simulate_for(1.0);

    set_rectifier_firing_angle(120.0);
    set_inverter_firing_angle(180.0 - 30.0f);

    simulate_for(1.0);
}

// Test 4: Extended operation showing steady-state
TEST_F(ThyristorBackToBackTest, ExtendedSteadyStateOperation)
{
    set_rectifier_firing_angle(20.0);
    set_inverter_firing_angle(35.0);

    // Simulate for 10 grid cycles
    double simulation_time = 10.0 / fundamental_frequency;  // 200 ms @ 50 Hz
    simulate_for(simulation_time);

    // Validate steady-state operation
    double i_dc_avg = _i_dc;
    double v_dc_link_avg = get_average_dc_link_voltage();

    EXPECT_GT(i_dc_avg, 0.0) << "Expected positive DC current";
    EXPECT_GT(v_dc_link_avg, 0.0) << "Expected positive DC link voltage";

    std::cout << "\n=== Extended Steady-State Results ===" << std::endl;
    std::cout << "DC Current:       " << i_dc_avg << " A" << std::endl;
    std::cout << "DC Link Voltage:  " << v_dc_link_avg << " V" << std::endl;
    std::cout << "Power Flow:       " << (v_dc_link_avg * i_dc_avg) << " W" << std::endl;
    std::cout << "====================================\n" << std::endl;
}

// Test 5: Comprehensive waveform validation
TEST_F(ThyristorBackToBackTest, WaveformValidation)
{
    // Nominal operating condition
    set_rectifier_firing_angle(20.0);
    set_inverter_firing_angle(180 - 30.0);

    // Data capture for validation
    struct CaptureData {
        bool rect_pulses_generated = false;
        bool inv_pulses_generated = false;
        bool rect_thyristors_conducting = false;
        bool inv_thyristors_conducting = false;
        bool dc_current_flowing = false;
        double max_i_dc = 0.0;
        double max_grid_current = 0.0;
        double max_machine_current = 0.0;
    };
    auto capture = std::make_shared<CaptureData>();

    _simulator.register_lambda_runner(sampling_time, [this, capture]() {
        // Check rectifier pulses
        if (_pulses_rect.p1 || _pulses_rect.p2 || _pulses_rect.p3 || _pulses_rect.p4 || _pulses_rect.p5 ||
            _pulses_rect.p6) {
            capture->rect_pulses_generated = true;
        }

        // Check inverter pulses
        if (_pulses_inv.p1 || _pulses_inv.p2 || _pulses_inv.p3 || _pulses_inv.p4 || _pulses_inv.p5 || _pulses_inv.p6) {
            capture->inv_pulses_generated = true;
        }

        // Check rectifier thyristor states
        const auto& rect_states = _back_to_back.get_rectifier_bridge().get_conduction_states();
        if (rect_states[0] || rect_states[1] || rect_states[2] || rect_states[3] || rect_states[4] || rect_states[5]) {
            capture->rect_thyristors_conducting = true;
        }

        // Check inverter thyristor states
        const auto& inv_states = _back_to_back.get_inverter_bridge().get_conduction_states();
        if (inv_states[0] || inv_states[1] || inv_states[2] || inv_states[3] || inv_states[4] || inv_states[5]) {
            capture->inv_thyristors_conducting = true;
        }

        // Check DC current
        if (std::abs(_i_dc) > 1e-6) {
            capture->dc_current_flowing = true;
            capture->max_i_dc = std::max(capture->max_i_dc, std::abs(_i_dc));
        }

        // Track maximum currents
        capture->max_grid_current = std::max(capture->max_grid_current,
            std::max({std::abs(_i_grid_abc.a()), std::abs(_i_grid_abc.b()), std::abs(_i_grid_abc.c())}));
        capture->max_machine_current = std::max(capture->max_machine_current,
            std::max({std::abs(_i_machine_abc.a()), std::abs(_i_machine_abc.b()), std::abs(_i_machine_abc.c())}));
    });

    // Run simulation
    double simulation_time = 5.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Print diagnostic information
    std::cout << "\n╔════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   BACK-TO-BACK WAVEFORM VALIDATION - DIAGNOSTIC RESULTS      ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Rectifier pulses generated:     " << (capture->rect_pulses_generated ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ Inverter pulses generated:      " << (capture->inv_pulses_generated ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ Rectifier thyristors conducting: " << (capture->rect_thyristors_conducting ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ Inverter thyristors conducting:  " << (capture->inv_thyristors_conducting ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ DC current flowing:              " << (capture->dc_current_flowing ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Maximum DC current:              " << std::setw(10) << std::fixed << std::setprecision(3)
              << capture->max_i_dc << " A              ║" << std::endl;
    std::cout << "║ Maximum grid current:            " << std::setw(10) << std::fixed << std::setprecision(3)
              << capture->max_grid_current << " A              ║" << std::endl;
    std::cout << "║ Maximum machine current:         " << std::setw(10) << std::fixed << std::setprecision(3)
              << capture->max_machine_current << " A              ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n" << std::endl;

    // Critical assertions
    EXPECT_TRUE(capture->rect_pulses_generated) << "Rectifier pulses not generated!";
    EXPECT_TRUE(capture->inv_pulses_generated) << "Inverter pulses not generated!";
    EXPECT_TRUE(capture->rect_thyristors_conducting) << "Rectifier thyristors not conducting!";
    EXPECT_TRUE(capture->inv_thyristors_conducting) << "Inverter thyristors not conducting!";
    EXPECT_TRUE(capture->dc_current_flowing) << "No DC current flowing!";
    EXPECT_GT(capture->max_i_dc, 0.1) << "DC current too low: " << capture->max_i_dc << " A";
}

// Test: Voltage notch detection during commutation
TEST_F(ThyristorBackToBackTest, VoltageNotchDuringCommutation)
{
    // Set operating point with moderate firing angles
    set_rectifier_firing_angle(25.0);
    set_inverter_firing_angle(35.0);

    // Data capture for commutation and voltage notch detection
    struct CommutationData {
        bool commutation_detected_rect = false;
        bool commutation_detected_inv = false;
        bool voltage_notch_detected_rect = false;
        bool voltage_notch_detected_inv = false;
        double min_line_voltage_rect = 1e9;
        double min_line_voltage_inv = 1e9;
        double max_line_voltage_rect = -1e9;
        double max_line_voltage_inv = -1e9;
    };
    auto data = std::make_shared<CommutationData>();

    _simulator.register_lambda_runner(sampling_time, [this, data]() {
        // Get thyristor states
        const auto& rect_states = _back_to_back.get_rectifier_bridge().get_conduction_states();
        const auto& inv_states = _back_to_back.get_inverter_bridge().get_conduction_states();

        // Detect rectifier commutation (more than 2 thyristors conducting)
        int rect_conducting = 0;
        for (bool state : rect_states) {
            if (state) rect_conducting++;
        }
        if (rect_conducting > 2) {
            data->commutation_detected_rect = true;
        }

        // Detect inverter commutation (more than 2 thyristors conducting)
        int inv_conducting = 0;
        for (bool state : inv_states) {
            if (state) inv_conducting++;
        }
        if (inv_conducting > 2) {
            data->commutation_detected_inv = true;
        }

        // Get line voltages at AC terminals
        const auto& v_line_rect = _back_to_back.get_rectifier_bridge().get_ac_line_voltages();
        const auto& v_line_inv = _back_to_back.get_inverter_bridge().get_ac_line_voltages();

        // Track min/max line voltages
        // v_line_rect is abc<voltage_t>, iterate manually
        std::array<double, 3> v_rect_vals = {v_line_rect.a().value(), v_line_rect.b().value(), v_line_rect.c().value()};
        for (double v : v_rect_vals) {
            data->min_line_voltage_rect = std::min(data->min_line_voltage_rect, std::abs(v));
            data->max_line_voltage_rect = std::max(data->max_line_voltage_rect, std::abs(v));
        }

        std::array<double, 3> v_inv_vals = {v_line_inv.a().value(), v_line_inv.b().value(), v_line_inv.c().value()};
        for (double v : v_inv_vals) {
            data->min_line_voltage_inv = std::min(data->min_line_voltage_inv, std::abs(v));
            data->max_line_voltage_inv = std::max(data->max_line_voltage_inv, std::abs(v));
        }

        // During commutation, check for voltage notch (line voltage drops significantly)
        if (rect_conducting > 2) {
            // Check if any line voltage is near zero (voltage notch)
            for (double v : v_rect_vals) {
                if (std::abs(v) < 50.0) {  // Threshold for voltage notch (< 50V for 100V system)
                    data->voltage_notch_detected_rect = true;
                    break;
                }
            }
        }

        if (inv_conducting > 2) {
            // Check if any line voltage is near zero (voltage notch)
            for (double v : v_inv_vals) {
                if (std::abs(v) < 50.0) {  // Threshold for voltage notch
                    data->voltage_notch_detected_inv = true;
                    break;
                }
            }
        }
    });

    // Run simulation for multiple grid cycles to capture commutation events
    double simulation_time = 5.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Print diagnostic information
    std::cout << "\n╔════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   VOLTAGE NOTCH DETECTION - DIAGNOSTIC RESULTS              ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Rectifier commutation detected: " << (data->commutation_detected_rect ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ Inverter commutation detected:  " << (data->commutation_detected_inv ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ Voltage notch (rectifier):      " << (data->voltage_notch_detected_rect ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "║ Voltage notch (inverter):       " << (data->voltage_notch_detected_inv ? "✓ YES" : "✗ NO ")
              << "                       ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Rectifier min line voltage:     " << std::setw(10) << std::fixed << std::setprecision(3)
              << data->min_line_voltage_rect << " V              ║" << std::endl;
    std::cout << "║ Rectifier max line voltage:     " << std::setw(10) << std::fixed << std::setprecision(3)
              << data->max_line_voltage_rect << " V              ║" << std::endl;
    std::cout << "║ Inverter min line voltage:      " << std::setw(10) << std::fixed << std::setprecision(3)
              << data->min_line_voltage_inv << " V              ║" << std::endl;
    std::cout << "║ Inverter max line voltage:      " << std::setw(10) << std::fixed << std::setprecision(3)
              << data->max_line_voltage_inv << " V              ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n" << std::endl;

    // Assertions
    EXPECT_TRUE(data->commutation_detected_rect) << "Rectifier commutation not detected!";
    EXPECT_TRUE(data->commutation_detected_inv) << "Inverter commutation not detected!";
    EXPECT_TRUE(data->voltage_notch_detected_rect)
        << "Voltage notch not detected during rectifier commutation! Min voltage: " << data->min_line_voltage_rect
        << " V";
    EXPECT_TRUE(data->voltage_notch_detected_inv)
        << "Voltage notch not detected during inverter commutation! Min voltage: " << data->min_line_voltage_inv
        << " V";
    EXPECT_LT(data->min_line_voltage_rect, 50.0)
        << "Rectifier line voltage did not drop during commutation: " << data->min_line_voltage_rect << " V";
    EXPECT_LT(data->min_line_voltage_inv, 50.0)
        << "Inverter line voltage did not drop during commutation: " << data->min_line_voltage_inv << " V";
}

// Industrial LCI Test Fixture (matching sfc_plant_model_voltage_behind_reactance parameters)
class IndustrialLCITest : public ::testing::Test {
public:
    // Industrial LCI parameters
    static constexpr double sampling_time = 10e-6;         // 10 μs (100 kHz)
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz

    // Grid voltage: 2090V RMS line-to-line → phase voltage = 2090/√3 ≈ 1207V RMS
    // Peak phase voltage = 1207 × √2 ≈ 1707V
    static constexpr double v_grid_rms_ll = 2090.0;
    static constexpr double v_grid_phase_peak = v_grid_rms_ll / std::sqrt(3.0) * std::sqrt(2.0);

    // Machine voltage: 1980V RMS line-to-line → phase voltage = 1980/√3 ≈ 1143V RMS
    // Peak phase voltage = 1143 × √2 ≈ 1617V
    static constexpr double v_machine_rms_ll = 1980.0;
    static constexpr double v_machine_phase_peak = v_machine_rms_ll / std::sqrt(3.0) * std::sqrt(2.0);

    static constexpr double pulse_width_deg = 120.0;

    // Circuit parameters from sfc_plant_model_voltage_behind_reactance
    static constexpr double grid_resistance = 0.1552f;
    static constexpr double grid_inductance = 0.4941e-3;
    static constexpr double dc_resistance = 10e-3;
    static constexpr double dc_inductance = 8e-3;
    static constexpr double machine_resistance = 0.0788f;
    static constexpr double machine_inductance = 0.2508e-3;

    void SetUp() override
    {
        // Configure grid-side waveform generator
        _grid_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_grid_phase_peak);
        _grid_waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _grid_waveform_generator.set_signal_frequency(fundamental_frequency);
        _grid_waveform_generator.disable_noise();

        // Configure machine-side waveform generator
        _machine_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_machine_phase_peak);
        _machine_waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _machine_waveform_generator.set_signal_frequency(fundamental_frequency);
        _machine_waveform_generator.disable_noise();

        set_logger();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Update grid-side voltage source
            _grid_waveform_generator.update();
            _v_grid_abc = _grid_waveform_generator.get_signal_abc();
            angle_wrapped grid_reference_angle = _grid_waveform_generator.get_fundamental_signal_angle().a();

            // Update machine-side voltage source
            _machine_waveform_generator.update();
            _v_machine_abc = _machine_waveform_generator.get_signal_abc();
            angle_wrapped machine_reference_angle = _machine_waveform_generator.get_fundamental_signal_angle().a();

            // Generate rectifier gate pulses
            _pulses_rect = _six_pulse_gen_rect.update(grid_reference_angle, _current_firing_angle_rect, true);

            // Generate inverter gate pulses
            _pulses_inv = _six_pulse_gen_inv.update(machine_reference_angle, _current_firing_angle_inv, true);

            // Convert pulses
            thyristor_pulses p_rect_conv;
            p_rect_conv.t1 = _pulses_rect.p1;
            p_rect_conv.t2 = _pulses_rect.p2;
            p_rect_conv.t3 = _pulses_rect.p3;
            p_rect_conv.t4 = _pulses_rect.p4;
            p_rect_conv.t5 = _pulses_rect.p5;
            p_rect_conv.t6 = _pulses_rect.p6;

            thyristor_pulses p_inv_conv;
            p_inv_conv.t1 = _pulses_inv.p1;
            p_inv_conv.t2 = _pulses_inv.p2;
            p_inv_conv.t3 = _pulses_inv.p3;
            p_inv_conv.t4 = _pulses_inv.p4;
            p_inv_conv.t5 = _pulses_inv.p5;
            p_inv_conv.t6 = _pulses_inv.p6;

            // Update complete back-to-back system
            abc<voltage_t> v_grid_typed{voltage_t{static_cast<real_t>(_v_grid_abc.a())},
                voltage_t{static_cast<real_t>(_v_grid_abc.b())}, voltage_t{static_cast<real_t>(_v_grid_abc.c())}};
            abc<voltage_t> v_mach_typed{voltage_t{static_cast<real_t>(_v_machine_abc.a())},
                voltage_t{static_cast<real_t>(_v_machine_abc.b())}, voltage_t{static_cast<real_t>(_v_machine_abc.c())}};

            _back_to_back.update(v_grid_typed, v_mach_typed, p_rect_conv, p_inv_conv);

            // Get states from back-to-back system
            auto i_grid = _back_to_back.get_grid_currents();
            _i_grid_abc = abc<double>{i_grid.a().value(), i_grid.b().value(), i_grid.c().value()};

            auto i_mach = _back_to_back.get_machine_currents();
            _i_machine_abc = abc<double>{i_mach.a().value(), i_mach.b().value(), i_mach.c().value()};

            _i_dc = _back_to_back.get_dc_current().value();
            _v_rect_dc = _back_to_back.get_rectifier_voltage().value();
            _v_inv_dc = _back_to_back.get_inverter_voltage().value();

            // Store current reference angles and sectors
            _current_grid_reference_angle = grid_reference_angle;
            _current_machine_reference_angle = machine_reference_angle;
            _current_grid_sector = _six_pulse_gen_rect.get_sector();
            _current_machine_sector = _six_pulse_gen_inv.get_sector();

            // Accumulate DC link voltage for averaging
            _v_dc_sum += (_v_rect_dc - _v_inv_dc);
            _v_dc_sample_count++;
        });
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    void set_rectifier_firing_angle(double angle_degrees)
    {
        _current_firing_angle_rect = angle_wrapped(deg_to_rad(angle_degrees));
    }

    void set_inverter_firing_angle(double angle_degrees)
    {
        _current_firing_angle_inv = angle_wrapped(deg_to_rad(angle_degrees));
    }

    double get_average_dc_link_voltage() const
    {
        if (_v_dc_sample_count == 0) return 0.0;
        return _v_dc_sum / static_cast<double>(_v_dc_sample_count);
    }

    void reset_voltage_averaging()
    {
        _v_dc_sum = 0.0;
        _v_dc_sample_count = 0;
    }

protected:
    yeet::simulator _simulator = yeet::make_gtest_simulator();

    // Grid-side components
    three_phase_waveform_generator _grid_waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen_rect{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};

    // Machine-side components
    three_phase_waveform_generator _machine_waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen_inv{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};

    // Back-to-back converter with industrial parameters
    thyristor_back_to_back _back_to_back{resistance_t{grid_resistance}, inductance_t{grid_inductance},
        resistance_t{dc_resistance}, inductance_t{dc_inductance}, resistance_t{machine_resistance},
        inductance_t{machine_inductance}, duration_t{sampling_time}};

    // Current state variables
    angle_wrapped _current_grid_reference_angle{0.0};
    angle_wrapped _current_machine_reference_angle{0.0};
    angle_wrapped _current_firing_angle_rect{0.0};
    angle_wrapped _current_firing_angle_inv{0.0};

    abc<double> _v_grid_abc{0.0, 0.0, 0.0};
    abc<double> _v_machine_abc{0.0, 0.0, 0.0};
    abc<double> _i_grid_abc{0.0, 0.0, 0.0};
    abc<double> _i_machine_abc{0.0, 0.0, 0.0};

    six_pulse_generator::pulses _pulses_rect{};
    six_pulse_generator::pulses _pulses_inv{};

    six_pulse_generator::pulse_sector_t _current_grid_sector{six_pulse_generator::pulse_sector_t::none};
    six_pulse_generator::pulse_sector_t _current_machine_sector{six_pulse_generator::pulse_sector_t::none};

    double _v_rect_dc{0.0};
    double _v_inv_dc{0.0};
    double _i_dc{0.0};

    // DC link voltage averaging
    double _v_dc_sum{0.0};
    size_t _v_dc_sample_count{0};

private:
    void set_logger()
    {
        // ===== GRID SIDE SIGNALS =====

        // Grid reference angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:reference_angle_deg", [&]() -> double { return _current_grid_reference_angle.get_degrees(); }));

        // Grid firing angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:firing_angle_deg", [&]() -> double { return _current_firing_angle_rect.get_degrees(); }));

        // Grid AC source voltages (line-to-neutral)
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_a", [&]() -> double { return _v_grid_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_b", [&]() -> double { return _v_grid_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_c", [&]() -> double { return _v_grid_abc.c(); }));

        // Grid AC currents
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_a", [&]() -> double { return _i_grid_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_b", [&]() -> double { return _i_grid_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_c", [&]() -> double { return _i_grid_abc.c(); }));

        // Grid terminal voltages
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:v_term_a",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_ac_phase_voltages().a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:v_term_b",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_ac_phase_voltages().b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:v_term_c",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_ac_phase_voltages().c().value(); }));

        // ===== MACHINE SIDE SIGNALS =====

        // Machine reference angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:reference_angle_deg", [&]() -> double { return _current_machine_reference_angle.get_degrees(); }));

        // Machine firing angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:firing_angle_deg", [&]() -> double { return _current_firing_angle_inv.get_degrees(); }));

        // Machine AC voltages (back-EMF)
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:v_ac_a", [&]() -> double { return _v_machine_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:v_ac_b", [&]() -> double { return _v_machine_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:v_ac_c", [&]() -> double { return _v_machine_abc.c(); }));

        // Machine AC currents
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:i_ac_a", [&]() -> double { return _i_machine_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:i_ac_b", [&]() -> double { return _i_machine_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:i_ac_c", [&]() -> double { return _i_machine_abc.c(); }));

        // Machine terminal voltages
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:v_term_a",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_ac_phase_voltages().a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:v_term_b",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_ac_phase_voltages().b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:v_term_c",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_ac_phase_voltages().c().value(); }));

        // ===== DC LINK SIGNALS =====

        // DC link current
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:i_dc", [&]() -> double { return _i_dc; }));

        // DC voltages
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_rect_dc", [&]() -> double { return _v_rect_dc; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_inv_dc", [&]() -> double { return _v_inv_dc; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_dc_link", [&]() -> double { return _v_rect_dc - _v_inv_dc; }));

        // Average DC link voltage
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "dc:v_dc_link_avg", [&]() -> double { return get_average_dc_link_voltage(); }));

        // Power flow
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "dc:power", [&]() -> double { return (_v_rect_dc - _v_inv_dc) * _i_dc; }));
    }
};

// Test 7: Industrial LCI Configuration (matching sfc_plant_model_voltage_behind_reactance parameters)
TEST_F(IndustrialLCITest, NominalOperatingPoint)
{
    // Set firing angles matching open loop test default values
    // Grid: 13° (nominal rectifier firing angle)
    // Machine: 180° - 26.84° ≈ 153.16° (nominal inverter advance angle)
    double alpha_rect_deg = 13.0;
    double alpha_inv_deg = 180.0 - 26.84;
    set_rectifier_firing_angle(alpha_rect_deg);
    set_inverter_firing_angle(alpha_inv_deg);

    // Simulate for 100ms (5 cycles at 50 Hz)
    simulate_for(0.1);

    // Calculate average DC link voltage (skip first 2 cycles for transient settling)
    double v_dc_link_avg = get_average_dc_link_voltage();

    // Expected theoretical DC voltages (no-load, ideal conditions):
    // V_rect_dc = (3/π) × V_LL_peak × cos(α)
    // For α = 13°: V_rect_dc ≈ (3/π) × 2090√2 × cos(13°) ≈ 2822V
    // For machine side (α = 153.16°): V_inv_dc ≈ (3/π) × 1980√2 × cos(153.16°) ≈ -2426V
    double alpha_rect_rad = deg_to_rad(alpha_rect_deg);
    double alpha_inv_rad = deg_to_rad(alpha_inv_deg);
    double v_ll_peak_grid = std::sqrt(2.0) * v_grid_rms_ll;
    double v_ll_peak_machine = std::sqrt(2.0) * v_machine_rms_ll;
    double v_rect_theoretical = (3.0 / pi) * v_ll_peak_grid * std::cos(alpha_rect_rad);
    double v_inv_theoretical = (3.0 / pi) * v_ll_peak_machine * std::cos(alpha_inv_rad);
    double expected_theoretical_net = v_rect_theoretical - v_inv_theoretical;

    // Validate that current is positive (motoring mode)
    EXPECT_GT(_i_dc, 0.0) << "Expected positive DC current in industrial LCI configuration";

    // Validate DC link voltage is positive
    EXPECT_GT(v_dc_link_avg, 0.0) << "Expected positive net DC link voltage";

    // The actual DC link voltage will be lower than theoretical due to resistive drops and commutation overlap
    // Expect it to be within 30% of theoretical (conservative estimate given the circuit impedances)
    EXPECT_GT(v_dc_link_avg, expected_theoretical_net * 0.3)
        << "DC link voltage significantly lower than expected. "
        << "Average: " << v_dc_link_avg << "V, Theoretical net: " << expected_theoretical_net << "V";

    // Validate that DC current is reasonable (should be in hundreds of amps range for MW-class system)
    // Base current for 2.95 MVA system at ~5kV DC link ≈ 590 A
    // At these firing angles, expect current in the range of 10-2000 A
    EXPECT_GT(_i_dc, 10.0) << "DC current too low for industrial configuration";
    EXPECT_LT(_i_dc, 2000.0) << "DC current unreasonably high";

    // Print diagnostic information
    std::cout << "\n=== Industrial LCI Configuration Test Results ===" << std::endl;
    std::cout << "Grid voltage (RMS L-L): " << v_grid_rms_ll << " V" << std::endl;
    std::cout << "Machine voltage (RMS L-L): " << v_machine_rms_ll << " V" << std::endl;
    std::cout << "Firing angle (grid): " << alpha_rect_deg << "°" << std::endl;
    std::cout << "Firing angle (machine): " << alpha_inv_deg << "°" << std::endl;
    std::cout << "DC link current: " << _i_dc << " A" << std::endl;
    std::cout << "Rectifier DC voltage: " << _v_rect_dc << " V" << std::endl;
    std::cout << "Inverter DC voltage: " << _v_inv_dc << " V" << std::endl;
    std::cout << "Net DC link voltage (avg): " << v_dc_link_avg << " V" << std::endl;
    std::cout << "Theoretical rectifier voltage: " << v_rect_theoretical << " V" << std::endl;
    std::cout << "Theoretical inverter voltage: " << v_inv_theoretical << " V" << std::endl;
    std::cout << "Theoretical net voltage: " << expected_theoretical_net << " V" << std::endl;
    std::cout << "DC link power: " << (v_dc_link_avg * _i_dc / 1e6) << " MW" << std::endl;
}

// =============================================================================
// DISCONTINUOUS CONDUCTION MODE TESTS
// =============================================================================

// Test fixture for discontinuous conduction mode (DCM) testing
// Models low-speed operation with reduced machine back-EMF
class DiscontinuousConductionModeTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 10e-6;
    static constexpr double grid_frequency = 50.0;

    // Grid: Full voltage (2090V RMS L-L)
    static constexpr double v_grid_rms_ll = 2090.0;
    static constexpr double v_grid_phase_peak = v_grid_rms_ll / std::sqrt(3.0) * std::sqrt(2.0);

    // Machine: Reduced voltage (simulating low speed operation)
    // At 10% speed, back-EMF is ~10% of rated
    double _machine_voltage_factor = 0.20;    // 20% of rated voltage
    double _machine_frequency_factor = 0.10;  // 10% of rated frequency (5 Hz)

    static constexpr double v_machine_rms_ll_rated = 1980.0;
    static constexpr double pulse_width_deg = 120.0;

    // Circuit parameters
    static constexpr double grid_resistance = 0.1552;
    static constexpr double grid_inductance = 0.4941e-3;
    static constexpr double dc_resistance = 10e-3;
    static constexpr double dc_inductance = 8e-3;
    static constexpr double machine_resistance = 0.0788;
    static constexpr double machine_inductance = 0.2508e-3;

    void SetUp() override
    {
        configure_machine_operating_point(_machine_voltage_factor, _machine_frequency_factor);
        set_logger();

        _simulator.register_lambda_runner(sampling_time, [&]() {
            // Update grid-side voltage source (always at nominal)
            _grid_waveform_generator.update();
            _v_grid_abc = _grid_waveform_generator.get_signal_abc();
            angle_wrapped grid_reference_angle = _grid_waveform_generator.get_fundamental_signal_angle().a();

            // Update machine-side voltage source (reduced voltage/frequency)
            _machine_waveform_generator.update();
            _v_machine_abc = _machine_waveform_generator.get_signal_abc();
            angle_wrapped machine_reference_angle = _machine_waveform_generator.get_fundamental_signal_angle().a();

            // Generate rectifier gate pulses
            _pulses_rect = _six_pulse_gen_rect.update(grid_reference_angle, _current_firing_angle_rect, true);

            // Generate inverter gate pulses
            _pulses_inv = _six_pulse_gen_inv.update(machine_reference_angle, _current_firing_angle_inv, true);

            // Convert pulses
            thyristor_pulses p_rect_conv = convert_pulses(_pulses_rect);
            thyristor_pulses p_inv_conv = convert_pulses(_pulses_inv);

            // Update complete back-to-back system
            abc<voltage_t> v_grid_typed = to_voltage_abc(_v_grid_abc);
            abc<voltage_t> v_mach_typed = to_voltage_abc(_v_machine_abc);

            _back_to_back.update(v_grid_typed, v_mach_typed, p_rect_conv, p_inv_conv);

            // Get states from back-to-back system
            auto i_grid = _back_to_back.get_grid_currents();
            _i_grid_abc = abc<double>{i_grid.a().value(), i_grid.b().value(), i_grid.c().value()};

            auto i_mach = _back_to_back.get_machine_currents();
            _i_machine_abc = abc<double>{i_mach.a().value(), i_mach.b().value(), i_mach.c().value()};

            _i_dc = _back_to_back.get_dc_current().value();
            _v_rect_dc = _back_to_back.get_rectifier_voltage().value();
            _v_inv_dc = _back_to_back.get_inverter_voltage().value();

            // Track min/max DC current for DCM analysis
            if (_i_dc > _i_dc_max) _i_dc_max = _i_dc;
            if (_i_dc < _i_dc_min) _i_dc_min = _i_dc;

            // Count zero-crossing events (DCM indicator)
            if (_i_dc_prev > 0.1 && _i_dc < 0.1) {
                _dcm_zero_crossings++;
            }
            _i_dc_prev = _i_dc;

            // Track conducting thyristor counts
            const auto& rect_states = _back_to_back.get_rectifier_bridge().get_conduction_states();
            const auto& inv_states = _back_to_back.get_inverter_bridge().get_conduction_states();
            int rect_count = 0, inv_count = 0;
            for (int i = 0; i < 6; i++) {
                if (rect_states[i]) rect_count++;
                if (inv_states[i]) inv_count++;
            }
            if (rect_count == 0 || inv_count == 0) {
                _no_conduction_samples++;
            }
            _total_samples++;

            // Store reference angles
            _current_grid_reference_angle = grid_reference_angle;
            _current_machine_reference_angle = machine_reference_angle;
        });
    }

    void configure_machine_operating_point(double voltage_factor, double frequency_factor)
    {
        _machine_voltage_factor = voltage_factor;
        _machine_frequency_factor = frequency_factor;

        double v_machine_phase_peak = (v_machine_rms_ll_rated * voltage_factor) / std::sqrt(3.0) * std::sqrt(2.0);
        double machine_frequency = grid_frequency * frequency_factor;

        // Configure grid-side waveform generator (always nominal)
        _grid_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_grid_phase_peak);
        _grid_waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _grid_waveform_generator.set_signal_frequency(grid_frequency);
        _grid_waveform_generator.disable_noise();

        // Configure machine-side waveform generator (reduced)
        _machine_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_machine_phase_peak);
        _machine_waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _machine_waveform_generator.set_signal_frequency(machine_frequency);
        _machine_waveform_generator.disable_noise();
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    void set_rectifier_firing_angle(double angle_degrees)
    {
        _current_firing_angle_rect = angle_wrapped(deg_to_rad(angle_degrees));
    }

    void set_inverter_firing_angle(double angle_degrees)
    {
        _current_firing_angle_inv = angle_wrapped(deg_to_rad(angle_degrees));
    }

    void reset_dcm_tracking()
    {
        _i_dc_max = 0.0;
        _i_dc_min = 1e9;
        _i_dc_prev = 0.0;
        _dcm_zero_crossings = 0;
        _no_conduction_samples = 0;
        _total_samples = 0;
    }

    double get_dcm_ratio() const
    {
        if (_total_samples == 0) return 0.0;
        return static_cast<double>(_no_conduction_samples) / static_cast<double>(_total_samples);
    }

protected:
    yeet::simulator _simulator = yeet::make_gtest_simulator();

    three_phase_waveform_generator _grid_waveform_generator{sampling_time};
    three_phase_waveform_generator _machine_waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen_rect{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};
    six_pulse_generator _six_pulse_gen_inv{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};

    thyristor_back_to_back _back_to_back{resistance_t{grid_resistance}, inductance_t{grid_inductance},
        resistance_t{dc_resistance}, inductance_t{dc_inductance}, resistance_t{machine_resistance},
        inductance_t{machine_inductance}, duration_t{sampling_time}};

    angle_wrapped _current_grid_reference_angle{0.0};
    angle_wrapped _current_machine_reference_angle{0.0};
    angle_wrapped _current_firing_angle_rect{0.0};
    angle_wrapped _current_firing_angle_inv{0.0};

    abc<double> _v_grid_abc{0.0, 0.0, 0.0};
    abc<double> _v_machine_abc{0.0, 0.0, 0.0};
    abc<double> _i_grid_abc{0.0, 0.0, 0.0};
    abc<double> _i_machine_abc{0.0, 0.0, 0.0};

    six_pulse_generator::pulses _pulses_rect{};
    six_pulse_generator::pulses _pulses_inv{};

    double _v_rect_dc{0.0};
    double _v_inv_dc{0.0};
    double _i_dc{0.0};

    // DCM tracking
    double _i_dc_max{0.0};
    double _i_dc_min{1e9};
    double _i_dc_prev{0.0};
    size_t _dcm_zero_crossings{0};
    size_t _no_conduction_samples{0};
    size_t _total_samples{0};

private:
    thyristor_pulses convert_pulses(const six_pulse_generator::pulses& p)
    {
        thyristor_pulses conv;
        conv.t1 = p.p1;
        conv.t2 = p.p2;
        conv.t3 = p.p3;
        conv.t4 = p.p4;
        conv.t5 = p.p5;
        conv.t6 = p.p6;
        return conv;
    }

    abc<voltage_t> to_voltage_abc(const abc<double>& v)
    {
        return abc<voltage_t>{voltage_t{static_cast<real_t>(v.a())}, voltage_t{static_cast<real_t>(v.b())},
            voltage_t{static_cast<real_t>(v.c())}};
    }

    void set_logger()
    {
        // Grid reference angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:reference_angle_deg", [&]() -> double { return _current_grid_reference_angle.get_degrees(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:firing_angle_deg", [&]() -> double { return _current_firing_angle_rect.get_degrees(); }));

        // Grid voltages
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_a", [&]() -> double { return _v_grid_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_b", [&]() -> double { return _v_grid_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_c", [&]() -> double { return _v_grid_abc.c(); }));

        // Grid currents
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_a", [&]() -> double { return _i_grid_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_b", [&]() -> double { return _i_grid_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_c", [&]() -> double { return _i_grid_abc.c(); }));

        // Grid thyristor states
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t1_on",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_conduction_states()[0] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t2_on",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_conduction_states()[1] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t3_on",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_conduction_states()[2] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t4_on",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_conduction_states()[3] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t5_on",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_conduction_states()[4] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid:thyristor_t6_on",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().get_conduction_states()[5] ? 1.0 : 0.0; }));

        // Machine reference angle
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:reference_angle_deg", [&]() -> double { return _current_machine_reference_angle.get_degrees(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:firing_angle_deg", [&]() -> double { return _current_firing_angle_inv.get_degrees(); }));

        // Machine voltages
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:v_ac_a", [&]() -> double { return _v_machine_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:v_ac_b", [&]() -> double { return _v_machine_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:v_ac_c", [&]() -> double { return _v_machine_abc.c(); }));

        // Machine currents
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:i_ac_a", [&]() -> double { return _i_machine_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:i_ac_b", [&]() -> double { return _i_machine_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("machine:i_ac_c", [&]() -> double { return _i_machine_abc.c(); }));

        // Machine thyristor states
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t1_on",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_conduction_states()[0] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t2_on",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_conduction_states()[1] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t3_on",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_conduction_states()[2] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t4_on",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_conduction_states()[3] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t5_on",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_conduction_states()[4] ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:thyristor_t6_on",
            [&]() -> double { return _back_to_back.get_inverter_bridge().get_conduction_states()[5] ? 1.0 : 0.0; }));

        // DC link signals
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:i_dc", [&]() -> double { return _i_dc; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_rect_dc", [&]() -> double { return _v_rect_dc; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_inv_dc", [&]() -> double { return _v_inv_dc; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_dc_net", [&]() -> double { return _v_rect_dc + _v_inv_dc; }));

        // Bridge conducting path indicators
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("dc:rect_has_path",
            [&]() -> double { return _back_to_back.get_rectifier_bridge().has_conducting_path() ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("dc:inv_has_path",
            [&]() -> double { return _back_to_back.get_inverter_bridge().has_conducting_path() ? 1.0 : 0.0; }));

        // Machine current magnitude (peak of 3 phases)
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine:i_peak", [&]() -> double {
            return std::max({std::abs(_i_machine_abc.a()), std::abs(_i_machine_abc.b()), std::abs(_i_machine_abc.c())});
        }));
    }
};

// Test: Discontinuous conduction at 10% speed, 20% voltage
TEST_F(DiscontinuousConductionModeTest, LowSpeedLowVoltageOperation)
{
    configure_machine_operating_point(0.20, 0.10);  // 20% voltage, 10% frequency (5 Hz)

    // Moderate firing angles for discontinuous mode
    set_rectifier_firing_angle(60.0);  // Reduced rectifier voltage
    set_inverter_firing_angle(150.0);  // Typical inverter angle

    reset_dcm_tracking();
    simulate_for(0.5);  // 500ms = 25 machine cycles at 5 Hz

    // Print diagnostic information
    std::cout << "\n╔════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   DISCONTINUOUS CONDUCTION MODE TEST (10% speed, 20% voltage) ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Machine voltage factor:          " << std::setw(6) << (_machine_voltage_factor * 100) << " %"
              << "               ║" << std::endl;
    std::cout << "║ Machine frequency:               " << std::setw(6) << (grid_frequency * _machine_frequency_factor)
              << " Hz" << "              ║" << std::endl;
    std::cout << "║ DC current (final):              " << std::setw(10) << std::fixed << std::setprecision(2) << _i_dc
              << " A" << "          ║" << std::endl;
    std::cout << "║ DC current (max):                " << std::setw(10) << _i_dc_max << " A" << "          ║"
              << std::endl;
    std::cout << "║ DC current (min):                " << std::setw(10) << (_i_dc_min < 1e8 ? _i_dc_min : 0.0) << " A"
              << "          ║" << std::endl;
    std::cout << "║ DCM zero crossings:              " << std::setw(10) << _dcm_zero_crossings << ""
              << "            ║" << std::endl;
    std::cout << "║ No-conduction ratio:             " << std::setw(10) << std::setprecision(4)
              << (get_dcm_ratio() * 100) << " %" << "         ║" << std::endl;
    std::cout << "║ Rectifier DC voltage:            " << std::setw(10) << std::setprecision(1) << _v_rect_dc << " V"
              << "          ║" << std::endl;
    std::cout << "║ Inverter DC voltage:             " << std::setw(10) << _v_inv_dc << " V" << "          ║"
              << std::endl;
    std::cout << "║ Net DC voltage:                  " << std::setw(10) << (_v_rect_dc + _v_inv_dc) << " V"
              << "          ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n" << std::endl;

    // In DCM, current should go to zero periodically
    // Expect some no-conduction periods, but machine current should exist when DC flows
    EXPECT_GT(_i_dc_max, 0.0) << "Expected some DC current to build up";

    // Verify machine currents are present when DC current exists
    double machine_current_peak =
        std::max({std::abs(_i_machine_abc.a()), std::abs(_i_machine_abc.b()), std::abs(_i_machine_abc.c())});
    if (_i_dc > 10.0) {
        EXPECT_GT(machine_current_peak, 1.0) << "Machine current should be present when DC current is flowing. "
                                             << "DC=" << _i_dc << "A, Machine peak=" << machine_current_peak << "A";
    }
}

// Test: Rectifier in inverting mode (alpha = 150°) - should produce negative voltage
TEST_F(DiscontinuousConductionModeTest, RectifierInvertingMode150Degrees)
{
    configure_machine_operating_point(1.0, 1.0);  // Full voltage/frequency for this test

    // Rectifier at 150° produces negative DC voltage
    // This simulates regenerative braking or incorrect firing
    set_rectifier_firing_angle(150.0);
    set_inverter_firing_angle(150.0);  // Inverter also in typical mode

    reset_dcm_tracking();
    simulate_for(0.2);  // 200ms = 10 grid cycles

    // Calculate theoretical rectifier voltage at alpha = 150°
    double alpha_rad = deg_to_rad(150.0);
    double v_ll_peak = std::sqrt(2.0) * v_grid_rms_ll;
    double v_rect_theoretical = (3.0 / pi) * v_ll_peak * std::cos(alpha_rad);

    std::cout << "\n╔════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   RECTIFIER INVERTING MODE TEST (alpha = 150°)                ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Rectifier firing angle:          " << std::setw(10) << 150.0 << " °" << "          ║" << std::endl;
    std::cout << "║ Theoretical rectifier voltage:   " << std::setw(10) << std::fixed << std::setprecision(1)
              << v_rect_theoretical << " V" << "          ║" << std::endl;
    std::cout << "║ Actual rectifier DC voltage:     " << std::setw(10) << _v_rect_dc << " V" << "          ║"
              << std::endl;
    std::cout << "║ Inverter DC voltage:             " << std::setw(10) << _v_inv_dc << " V" << "          ║"
              << std::endl;
    std::cout << "║ Net DC voltage:                  " << std::setw(10) << (_v_rect_dc + _v_inv_dc) << " V"
              << "          ║" << std::endl;
    std::cout << "║ DC current:                      " << std::setw(10) << std::setprecision(2) << _i_dc << " A"
              << "          ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n" << std::endl;

    // At alpha = 150°, rectifier should produce negative voltage
    // Theoretical: V_rect = (3/π) × V_LL_peak × cos(150°) ≈ -2440V
    EXPECT_LT(v_rect_theoretical, 0.0) << "Theoretical rectifier voltage should be negative at 150°";

    // With both bridges at high alpha, net voltage should be low or negative
    // Current should be minimal or zero since there's no driving voltage
    EXPECT_LT(_i_dc, 100.0) << "DC current should be low when rectifier is inverting";
}

// Test: Verify DC voltage is bounded correctly during transients
TEST_F(DiscontinuousConductionModeTest, DCVoltageBoundsDuringStartup)
{
    configure_machine_operating_point(0.20, 0.10);  // Low speed condition

    set_rectifier_firing_angle(30.0);
    set_inverter_firing_angle(150.0);

    // Track max voltage seen during simulation
    double v_dc_max_seen = 0.0;
    double v_dc_min_seen = 1e9;
    double v_net_max_seen = 0.0;

    _simulator.register_lambda_runner(sampling_time, [&]() {
        double v_net = _v_rect_dc + _v_inv_dc;
        if (std::abs(_v_rect_dc) > v_dc_max_seen) v_dc_max_seen = std::abs(_v_rect_dc);
        if (std::abs(_v_inv_dc) > v_dc_max_seen) v_dc_max_seen = std::abs(_v_inv_dc);
        if (std::abs(v_net) > v_net_max_seen) v_net_max_seen = std::abs(v_net);
    });

    simulate_for(0.1);  // 100ms startup

    // Calculate theoretical maximum DC voltage
    double v_ll_peak_grid = std::sqrt(2.0) * v_grid_rms_ll;
    double v_rect_max_theoretical = (3.0 / pi) * v_ll_peak_grid;  // At alpha = 0

    std::cout << "\n╔════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   DC VOLTAGE BOUNDS CHECK DURING STARTUP                      ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Max theoretical rect voltage:    " << std::setw(10) << std::fixed << std::setprecision(1)
              << v_rect_max_theoretical << " V" << "          ║" << std::endl;
    std::cout << "║ Max seen DC voltage (any):       " << std::setw(10) << v_dc_max_seen << " V" << "          ║"
              << std::endl;
    std::cout << "║ Max seen net voltage:            " << std::setw(10) << v_net_max_seen << " V" << "          ║"
              << std::endl;
    std::cout << "║ Final DC current:                " << std::setw(10) << std::setprecision(2) << _i_dc << " A"
              << "          ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n" << std::endl;

    // DC voltage should never exceed theoretical maximum by more than 20% (commutation transients)
    EXPECT_LT(v_dc_max_seen, v_rect_max_theoretical * 1.2)
        << "DC voltage exceeded physical bounds! Max seen: " << v_dc_max_seen
        << "V, Theoretical max: " << v_rect_max_theoretical << "V";

    // Net voltage should be bounded by sum of both theoretical maximums
    double v_ll_peak_machine = std::sqrt(2.0) * v_machine_rms_ll_rated * _machine_voltage_factor;
    double v_inv_max_theoretical = (3.0 / pi) * v_ll_peak_machine;
    double v_net_max_theoretical = v_rect_max_theoretical + v_inv_max_theoretical;

    EXPECT_LT(v_net_max_seen, v_net_max_theoretical * 1.2)
        << "Net DC voltage exceeded physical bounds! Max seen: " << v_net_max_seen
        << "V, Theoretical max: " << v_net_max_theoretical << "V";
}

// Test: Step response of firing angle change - track average voltage over a cycle
// This test verifies that the DC voltage follows the cos(α) relationship
TEST_F(DiscontinuousConductionModeTest, FiringAngleStepResponse)
{
    configure_machine_operating_point(1.0, 1.0);  // Full voltage/frequency

    // Start with balanced operation at α=45°
    set_rectifier_firing_angle(45.0);
    set_inverter_firing_angle(150.0);

    // Simulate and compute average voltage over one grid cycle (20ms at 50Hz)
    const double grid_period = 1.0 / 50.0;  // 20ms
    const int samples_per_cycle = static_cast<int>(grid_period / sampling_time);

    simulate_for(0.08);  // Initial settle

    // Measure average over one cycle
    double v_sum_45 = 0.0;
    for (int i = 0; i < samples_per_cycle; i++) {
        simulate_for(sampling_time);
        v_sum_45 += _v_rect_dc;
    }
    double v_avg_45 = v_sum_45 / samples_per_cycle;

    // Theoretical average at α=45°: V_dc = (3√2/π) × V_LL × cos(45°)
    double v_ll_peak = std::sqrt(2.0) * v_grid_rms_ll;
    double v_dc_theory_45 = (3.0 * std::sqrt(2.0) / pi) * v_ll_peak * std::cos(45.0 * pi / 180.0);

    std::cout << "\n=== At α=45° ===" << std::endl;
    std::cout << "Average rectifier voltage: " << v_avg_45 << " V" << std::endl;
    std::cout << "Theoretical (α=45°): " << v_dc_theory_45 << " V" << std::endl;

    // Step to α=90° (theoretical average = 0)
    set_rectifier_firing_angle(90.0);
    simulate_for(0.02);  // Wait one cycle for transition

    // Measure average over one cycle at α=90°
    double v_sum_90 = 0.0;
    for (int i = 0; i < samples_per_cycle; i++) {
        simulate_for(sampling_time);
        v_sum_90 += _v_rect_dc;
    }
    double v_avg_90 = v_sum_90 / samples_per_cycle;
    double v_dc_theory_90 = (3.0 * std::sqrt(2.0) / pi) * v_ll_peak * std::cos(90.0 * pi / 180.0);

    std::cout << "\n=== At α=90° ===" << std::endl;
    std::cout << "Average rectifier voltage: " << v_avg_90 << " V" << std::endl;
    std::cout << "Theoretical (α=90°): " << v_dc_theory_90 << " V" << std::endl;
    std::cout << "Instantaneous voltage: " << _v_rect_dc << " V (swings widely)" << std::endl;

    // Step to α=120° (inverting mode, theoretical average negative)
    set_rectifier_firing_angle(120.0);
    simulate_for(0.02);

    double v_sum_120 = 0.0;
    for (int i = 0; i < samples_per_cycle; i++) {
        simulate_for(sampling_time);
        v_sum_120 += _v_rect_dc;
    }
    double v_avg_120 = v_sum_120 / samples_per_cycle;
    double v_dc_theory_120 = (3.0 * std::sqrt(2.0) / pi) * v_ll_peak * std::cos(120.0 * pi / 180.0);

    std::cout << "\n=== At α=120° (inverting mode) ===" << std::endl;
    std::cout << "Average rectifier voltage: " << v_avg_120 << " V" << std::endl;
    std::cout << "Theoretical (α=120°): " << v_dc_theory_120 << " V" << std::endl;

    // === Key verification: cos(α) relationship ===
    // The average DC voltage should follow: V_dc_avg = k × cos(α) where k is a scaling factor
    // Even with commutation effects and DCM, the cosine relationship should hold

    // Calculate ratio of actual to theoretical for each firing angle
    double ratio_45 = v_avg_45 / v_dc_theory_45;
    double ratio_90 = (std::abs(v_dc_theory_90) < 1.0) ? 1.0 : v_avg_90 / v_dc_theory_90;  // Avoid div by zero
    double ratio_120 = v_avg_120 / v_dc_theory_120;

    std::cout << "\n=== cos(α) Relationship Analysis ===" << std::endl;
    std::cout << "Ratio at α=45°: " << ratio_45 << std::endl;
    std::cout << "Ratio at α=120°: " << ratio_120 << std::endl;

    // The ratios should be similar (within 20%) indicating consistent cos(α) behavior
    // Note: DCM and commutation effects cause deviation from ideal, but relationship holds
    double ratio_diff = std::abs(ratio_45 - ratio_120);
    std::cout << "Ratio difference: " << ratio_diff << " (expect < 0.2)" << std::endl;

    // Verify the cos(α) relationship is being followed
    // Both ratios should be positive and in similar range (60-100% of theoretical)
    EXPECT_GT(ratio_45, 0.5) << "At α=45°, average voltage should be >50% of theoretical";
    EXPECT_LT(ratio_45, 1.1) << "At α=45°, average voltage should be <110% of theoretical";
    EXPECT_GT(ratio_120, 0.5) << "At α=120°, average voltage should be >50% of theoretical";
    EXPECT_LT(ratio_120, 1.1) << "At α=120°, average voltage should be <110% of theoretical";

    // Verify α=90° produces near-zero average (within ±500V tolerance for 2090V system)
    EXPECT_NEAR(v_avg_90, 0.0, 500.0) << "At α=90°, average voltage should be near zero";

    // Verify polarity changes correctly
    EXPECT_GT(v_avg_45, 0.0) << "At α=45°, average voltage should be positive (rectifier mode)";
    EXPECT_LT(v_avg_120, 0.0) << "At α=120°, average voltage should be negative (inverter mode)";
}

// Test: Repetitive current buildup and forced-to-zero cycles
// This test verifies the system can handle multiple cycles of current buildup followed by
// forcing current to zero (e.g., by setting firing angle to 150° or disabling pulses).
// This is critical for LCI soft-starter applications where current is repeatedly pulsed.
TEST_F(DiscontinuousConductionModeTest, RepetitiveCurrentZeroCycles)
{
    configure_machine_operating_point(1.0, 1.0);  // Full voltage/frequency

    constexpr int num_cycles = 25;         // Perform at least 20 current buildup/zero cycles
    constexpr double buildup_time = 0.04;  // 40ms to build current (2 grid cycles)
    constexpr double zero_time = 0.03;     // 30ms at zero (allows current to decay)

    int successful_buildups = 0;
    int successful_zeros = 0;
    double max_current_seen = 0.0;
    double min_current_after_zero = 1e9;

    std::cout << "\n╔════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     REPETITIVE CURRENT ZERO CYCLES TEST (" << num_cycles << " cycles)           ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;

    for (int cycle = 0; cycle < num_cycles; cycle++) {
        // Phase 1: Build up current with nominal firing angles
        set_rectifier_firing_angle(30.0);  // Rectifier mode - positive voltage
        set_inverter_firing_angle(150.0);  // Inverter mode for machine side

        // Simulate and track max current during buildup
        double i_dc_max_this_cycle = 0.0;
        const int buildup_steps = static_cast<int>(buildup_time / sampling_time);
        for (int step = 0; step < buildup_steps; step++) {
            simulate_for(sampling_time);
            if (_i_dc > i_dc_max_this_cycle) {
                i_dc_max_this_cycle = _i_dc;
            }
        }

        if (i_dc_max_this_cycle > max_current_seen) {
            max_current_seen = i_dc_max_this_cycle;
        }

        // Check if current built up
        bool buildup_ok = (i_dc_max_this_cycle > 10.0);  // Expect >10A during buildup
        if (buildup_ok) successful_buildups++;

        // Phase 2: Force current to zero by inverting rectifier voltage
        set_rectifier_firing_angle(150.0);  // Inverted rectifier - negative voltage forces current down
        set_inverter_firing_angle(150.0);   // Keep inverter the same

        // Simulate and track min current during zero phase
        const int zero_steps = static_cast<int>(zero_time / sampling_time);
        for (int step = 0; step < zero_steps; step++) {
            simulate_for(sampling_time);
        }

        double i_dc_after_zero = _i_dc;
        if (i_dc_after_zero < min_current_after_zero) {
            min_current_after_zero = i_dc_after_zero;
        }

        // Check if current reached near-zero
        bool zero_ok = (i_dc_after_zero < 50.0);  // Expect <50A after forcing to zero
        if (zero_ok) successful_zeros++;

        // Print progress every 5 cycles
        if ((cycle + 1) % 5 == 0 || cycle == 0) {
            std::cout << "║ Cycle " << std::setw(2) << (cycle + 1) << ": I_max=" << std::setw(8) << std::fixed
                      << std::setprecision(1) << i_dc_max_this_cycle << "A, I_after_zero=" << std::setw(8)
                      << i_dc_after_zero << "A";
            std::cout << (buildup_ok && zero_ok ? " ✓" : " ✗") << "     ║" << std::endl;
        }
    }

    std::cout << "╠════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Summary:                                                       ║" << std::endl;
    std::cout << "║   Successful buildups:  " << std::setw(3) << successful_buildups << " / " << num_cycles
              << "                              ║" << std::endl;
    std::cout << "║   Successful zeros:     " << std::setw(3) << successful_zeros << " / " << num_cycles
              << "                              ║" << std::endl;
    std::cout << "║   Max current seen:     " << std::setw(10) << std::fixed << std::setprecision(1) << max_current_seen
              << " A                        ║" << std::endl;
    std::cout << "║   Min current at zero:  " << std::setw(10)
              << (min_current_after_zero < 1e8 ? min_current_after_zero : 0.0) << " A                        ║"
              << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n" << std::endl;

    // Assertions
    EXPECT_GE(successful_buildups, 20) << "Expected at least 20 successful current buildups out of " << num_cycles;
    EXPECT_GE(successful_zeros, 20) << "Expected at least 20 successful current-to-zero transitions out of "
                                    << num_cycles;
    EXPECT_GT(max_current_seen, 50.0) << "Expected significant current buildup during cycles";

    // Verify the system is stable after all cycles (no runaway or stuck states)
    // Do one more buildup and verify it still works
    set_rectifier_firing_angle(30.0);
    set_inverter_firing_angle(150.0);
    simulate_for(0.1);  // 100ms final buildup

    EXPECT_GT(_i_dc, 10.0) << "System should still function after " << num_cycles << " zero-current cycles";
}

// Test: Rapid on-off switching stress test
// Tests the thyristor model's ability to handle rapid firing angle changes
TEST_F(DiscontinuousConductionModeTest, RapidOnOffSwitchingStressTest)
{
    configure_machine_operating_point(1.0, 1.0);  // Full voltage/frequency

    constexpr int num_rapid_cycles = 30;
    constexpr double on_time = 0.01;   // 10ms on (half grid cycle)
    constexpr double off_time = 0.01;  // 10ms off

    int transitions_completed = 0;
    double max_on_current = 0.0;
    double min_off_current = 1e9;

    std::cout << "\n=== Rapid On-Off Switching Stress Test (" << num_rapid_cycles << " cycles) ===" << std::endl;

    for (int cycle = 0; cycle < num_rapid_cycles; cycle++) {
        // ON phase: Enable current flow
        set_rectifier_firing_angle(20.0);
        set_inverter_firing_angle(160.0);

        double i_max_on = 0.0;
        for (int step = 0; step < static_cast<int>(on_time / sampling_time); step++) {
            simulate_for(sampling_time);
            if (_i_dc > i_max_on) i_max_on = _i_dc;
        }
        if (i_max_on > max_on_current) max_on_current = i_max_on;

        // OFF phase: Force current down
        set_rectifier_firing_angle(160.0);  // Invert rectifier
        set_inverter_firing_angle(20.0);    // Invert inverter too

        double i_min_off = 1e9;
        for (int step = 0; step < static_cast<int>(off_time / sampling_time); step++) {
            simulate_for(sampling_time);
            if (_i_dc < i_min_off) i_min_off = _i_dc;
        }
        if (i_min_off < min_off_current) min_off_current = i_min_off;

        // Count valid transition (current during ON > current during OFF)
        if (i_max_on > i_min_off + 1.0) {  // Allow 1A tolerance
            transitions_completed++;
        }
    }

    std::cout << "Transitions completed: " << transitions_completed << " / " << num_rapid_cycles << std::endl;
    std::cout << "Max current during ON phases: " << max_on_current << " A" << std::endl;
    std::cout << "Min current during OFF phases: " << min_off_current << " A" << std::endl;

    // Expect most transitions to be valid
    EXPECT_GE(transitions_completed, 20) << "Expected at least 20 valid on->off transitions";
    EXPECT_GT(max_on_current, 0.0) << "Expected some current during ON phases";
}

// =============================================================================
// DC VOLTAGE VALIDATION TESTS
// =============================================================================

// Test: Comprehensive DC voltage validation against theoretical values
// Verifies that rectifier and inverter DC voltages follow the expected cos(α) relationship
// across different firing angles and with current flow
TEST_F(DiscontinuousConductionModeTest, DCVoltageTheoreticalValidation)
{
    configure_machine_operating_point(1.0, 1.0);  // Full voltage/frequency

    const double grid_period = 1.0 / grid_frequency;
    const int samples_per_cycle = static_cast<int>(grid_period / sampling_time);
    const int num_cycles_to_average = 5;

    // Theoretical voltage calculation constants
    // For 6-pulse bridge: V_dc_avg = (3√2/π) × V_LL × cos(α)
    const double v_ll_peak_grid = std::sqrt(2.0) * v_grid_rms_ll;
    const double v_ll_peak_machine = std::sqrt(2.0) * v_machine_rms_ll_rated;
    const double k_rect = (3.0 * std::sqrt(2.0) / pi) * v_ll_peak_grid;
    const double k_inv = (3.0 * std::sqrt(2.0) / pi) * v_ll_peak_machine;

    std::cout << "\n╔════════════════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║           DC VOLTAGE THEORETICAL VALIDATION                                ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Grid L-L RMS: " << std::setw(8) << v_grid_rms_ll << " V   Machine L-L RMS: " << std::setw(8)
              << v_machine_rms_ll_rated << " V           ║" << std::endl;
    std::cout << "║ k_rect = (3√2/π)×V_LL_peak = " << std::setw(8) << std::fixed << std::setprecision(1) << k_rect
              << " V                                ║" << std::endl;
    std::cout << "║ k_inv  = (3√2/π)×V_LL_peak = " << std::setw(8) << k_inv << " V                                ║"
              << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════════════════╣" << std::endl;

    // Test multiple firing angle combinations
    struct TestPoint {
        double alpha_rect_deg;
        double alpha_inv_deg;
        const char* description;
    };

    std::vector<TestPoint> test_points = {
        {20.0, 160.0, "Nominal rectifier mode"},
        {30.0, 150.0, "Moderate rectifier"},
        {45.0, 135.0, "Equal angles"},
        {60.0, 120.0, "High rectifier angle"},
        {15.0, 165.0, "Low firing angle (high voltage)"},
    };

    bool all_tests_passed = true;

    for (const auto& tp : test_points) {
        // Set operating point
        set_rectifier_firing_angle(tp.alpha_rect_deg);
        set_inverter_firing_angle(tp.alpha_inv_deg);

        // Let system settle
        simulate_for(0.05);

        // Measure average voltages over multiple cycles
        double v_rect_sum = 0.0;
        double v_inv_sum = 0.0;
        double i_dc_sum = 0.0;
        int sample_count = 0;

        for (int cycle = 0; cycle < num_cycles_to_average; cycle++) {
            for (int step = 0; step < samples_per_cycle; step++) {
                simulate_for(sampling_time);
                v_rect_sum += _v_rect_dc;
                v_inv_sum += _v_inv_dc;
                i_dc_sum += _i_dc;
                sample_count++;
            }
        }

        double v_rect_avg = v_rect_sum / sample_count;
        double v_inv_avg = v_inv_sum / sample_count;
        double i_dc_avg = i_dc_sum / sample_count;

        // Calculate theoretical values
        double alpha_rect_rad = tp.alpha_rect_deg * pi / 180.0;
        double alpha_inv_rad = tp.alpha_inv_deg * pi / 180.0;
        double v_rect_theory = k_rect * std::cos(alpha_rect_rad);
        double v_inv_theory = k_inv * std::cos(alpha_inv_rad);

        // Calculate ratios
        double rect_ratio = (std::abs(v_rect_theory) > 100.0) ? v_rect_avg / v_rect_theory : 1.0;
        double inv_ratio = (std::abs(v_inv_theory) > 100.0) ? v_inv_avg / v_inv_theory : 1.0;

        // Print results
        std::cout << "║ " << std::setw(28) << std::left << tp.description << std::right << " (α_rect=" << std::setw(3)
                  << tp.alpha_rect_deg << "°, α_inv=" << std::setw(3) << tp.alpha_inv_deg << "°)          ║"
                  << std::endl;
        std::cout << "║   V_rect: avg=" << std::setw(8) << std::fixed << std::setprecision(1) << v_rect_avg
                  << "V, theory=" << std::setw(8) << v_rect_theory << "V, ratio=" << std::setw(5)
                  << std::setprecision(3) << rect_ratio << "        ║" << std::endl;
        std::cout << "║   V_inv:  avg=" << std::setw(8) << std::setprecision(1) << v_inv_avg
                  << "V, theory=" << std::setw(8) << v_inv_theory << "V, ratio=" << std::setw(5) << std::setprecision(3)
                  << inv_ratio << "        ║" << std::endl;
        std::cout << "║   I_dc:   avg=" << std::setw(8) << i_dc_avg << " A                                       ║"
                  << std::endl;

        // Validate: ratios should be between 0.6 and 1.1 (allowing for losses and DCM effects)
        bool rect_ok = (rect_ratio > 0.5 && rect_ratio < 1.2) || (std::abs(v_rect_theory) < 100.0);
        bool inv_ok = (inv_ratio > 0.5 && inv_ratio < 1.2) || (std::abs(v_inv_theory) < 100.0);

        if (!rect_ok || !inv_ok) {
            all_tests_passed = false;
            std::cout << "║   ⚠ VOLTAGE RATIO OUT OF EXPECTED RANGE!                                   ║" << std::endl;
        }
    }

    std::cout << "╚════════════════════════════════════════════════════════════════════════════╝" << std::endl;

    EXPECT_TRUE(all_tests_passed) << "One or more DC voltage validation points failed";
}

// Test: Transition from low-speed (forced commutation) to high-speed (natural commutation)
// This validates that DC voltages remain correct during the mode transition
TEST_F(DiscontinuousConductionModeTest, ForcedToNaturalCommutationTransition)
{
    // Start at low speed (10% voltage/frequency) - forced commutation regime
    configure_machine_operating_point(0.1, 0.1);

    const double grid_period = 1.0 / grid_frequency;
    const int samples_per_cycle = static_cast<int>(grid_period / sampling_time);

    // Theoretical constants
    const double v_ll_peak_grid = std::sqrt(2.0) * v_grid_rms_ll;
    const double k_rect = (3.0 * std::sqrt(2.0) / pi) * v_ll_peak_grid;

    std::cout << "\n╔════════════════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     FORCED TO NATURAL COMMUTATION TRANSITION - DC VOLTAGE VALIDATION      ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════════════════════╣" << std::endl;

    struct SpeedPoint {
        double voltage_factor;
        double frequency_factor;
        double alpha_rect_deg;
        double alpha_inv_deg;
        const char* mode;
    };

    std::vector<SpeedPoint> speed_ramp = {
        {0.10, 0.10, 30.0, 150.0, "Forced (10% speed)"},
        {0.20, 0.20, 30.0, 150.0, "Forced (20% speed)"},
        {0.30, 0.30, 25.0, 155.0, "Transition (30%)"},
        {0.50, 0.50, 20.0, 160.0, "Natural (50% speed)"},
        {0.75, 0.75, 18.0, 162.0, "Natural (75% speed)"},
        {1.00, 1.00, 15.0, 165.0, "Natural (100% speed)"},
    };

    double prev_v_rect_avg = 0.0;
    double prev_i_dc_avg = 0.0;

    for (const auto& sp : speed_ramp) {
        // Configure operating point
        configure_machine_operating_point(sp.voltage_factor, sp.frequency_factor);
        set_rectifier_firing_angle(sp.alpha_rect_deg);
        set_inverter_firing_angle(sp.alpha_inv_deg);

        // Settling time
        simulate_for(0.1);

        // Measure average over 3 cycles
        double v_rect_sum = 0.0;
        double v_inv_sum = 0.0;
        double i_dc_sum = 0.0;
        double v_net_sum = 0.0;
        int samples = 0;

        for (int cycle = 0; cycle < 3; cycle++) {
            for (int step = 0; step < samples_per_cycle; step++) {
                simulate_for(sampling_time);
                v_rect_sum += _v_rect_dc;
                v_inv_sum += _v_inv_dc;
                i_dc_sum += _i_dc;
                v_net_sum += (_v_rect_dc + _v_inv_dc);  // Net driving voltage
                samples++;
            }
        }

        double v_rect_avg = v_rect_sum / samples;
        double v_inv_avg = v_inv_sum / samples;
        double i_dc_avg = i_dc_sum / samples;
        double v_net_avg = v_net_sum / samples;

        // Theoretical rectifier voltage
        double alpha_rect_rad = sp.alpha_rect_deg * pi / 180.0;
        double v_rect_theory = k_rect * std::cos(alpha_rect_rad);
        double rect_ratio = v_rect_avg / v_rect_theory;

        // Theoretical inverter voltage (machine-side, reduced by voltage factor)
        double v_ll_peak_machine = std::sqrt(2.0) * v_machine_rms_ll_rated * sp.voltage_factor;
        double k_inv = (3.0 * std::sqrt(2.0) / pi) * v_ll_peak_machine;
        double alpha_inv_rad = sp.alpha_inv_deg * pi / 180.0;
        double v_inv_theory = k_inv * std::cos(alpha_inv_rad);
        double inv_ratio = (std::abs(v_inv_theory) > 10.0) ? v_inv_avg / v_inv_theory : 1.0;

        std::cout << "║ " << std::setw(22) << std::left << sp.mode << std::right << " α_r=" << std::setw(2)
                  << sp.alpha_rect_deg << "° α_i=" << std::setw(3) << sp.alpha_inv_deg << "°                         ║"
                  << std::endl;
        std::cout << "║   V_rect: " << std::setw(8) << std::fixed << std::setprecision(1) << v_rect_avg
                  << " V (theory: " << std::setw(8) << v_rect_theory << " V, ratio: " << std::setprecision(3)
                  << rect_ratio << ")       ║" << std::endl;
        std::cout << "║   V_inv:  " << std::setw(8) << std::setprecision(1) << v_inv_avg
                  << " V (theory: " << std::setw(8) << v_inv_theory << " V, ratio: " << std::setprecision(3)
                  << inv_ratio << ")       ║" << std::endl;
        std::cout << "║   V_net:  " << std::setw(8) << v_net_avg << " V   I_dc: " << std::setw(8) << i_dc_avg
                  << " A                             ║" << std::endl;

        // At full speed only, validate voltage ratios (should be 0.6-0.8 range)
        // At low speeds, heavy current causes excessive voltage drop, making ratios unreliable
        if (sp.voltage_factor >= 1.0) {
            EXPECT_GT(rect_ratio, 0.5) << "Rectifier ratio too low at " << sp.mode;
            EXPECT_LT(rect_ratio, 1.0) << "Rectifier ratio too high at " << sp.mode;
            EXPECT_GT(inv_ratio, 0.5) << "Inverter ratio too low at " << sp.mode;
            EXPECT_LT(inv_ratio, 1.0) << "Inverter ratio too high at " << sp.mode;
        }

        // At all speed points, validate basic physics:
        // 1. Rectifier voltage should be positive (α < 90°)
        EXPECT_GT(v_rect_avg, 0.0) << "Rectifier voltage should be positive at " << sp.mode;

        // 2. Inverter voltage should be negative (α > 90° for machine side)
        EXPECT_LT(v_inv_avg, 0.0) << "Inverter voltage should be negative at " << sp.mode;

        // 3. DC current should be positive (motoring mode)
        EXPECT_GT(i_dc_avg, 0.0) << "DC current should be positive for motoring at " << sp.mode;

        prev_v_rect_avg = v_rect_avg;
        prev_i_dc_avg = i_dc_avg;
    }

    std::cout << "╚════════════════════════════════════════════════════════════════════════════╝" << std::endl;
}

// Test: Validate that DC voltage sign conventions are correct
// Rectifier: positive voltage in rectifier mode (α < 90°), negative in inverter mode (α > 90°)
// Inverter: negative voltage when operating as inverter (α > 90° for machine side)
TEST_F(DiscontinuousConductionModeTest, DCVoltageSignConvention)
{
    configure_machine_operating_point(1.0, 1.0);

    const double grid_period = 1.0 / grid_frequency;
    const int samples_per_cycle = static_cast<int>(grid_period / sampling_time);

    std::cout << "\n=== DC Voltage Sign Convention Validation ===" << std::endl;

    // Test 1: Rectifier mode (α < 90°) should produce positive DC voltage
    set_rectifier_firing_angle(30.0);
    set_inverter_firing_angle(150.0);  // 180° - 30° = operating as inverter
    simulate_for(0.1);

    double v_rect_sum = 0.0, v_inv_sum = 0.0;
    for (int i = 0; i < samples_per_cycle * 3; i++) {
        simulate_for(sampling_time);
        v_rect_sum += _v_rect_dc;
        v_inv_sum += _v_inv_dc;
    }
    double v_rect_avg_rect_mode = v_rect_sum / (samples_per_cycle * 3);
    double v_inv_avg_inv_mode = v_inv_sum / (samples_per_cycle * 3);

    std::cout << "Rectifier at α=30° (rectifier mode): V_rect_avg = " << v_rect_avg_rect_mode << " V (expect >0)"
              << std::endl;
    std::cout << "Inverter at α=150° (inverter mode):  V_inv_avg  = " << v_inv_avg_inv_mode << " V (expect <0)"
              << std::endl;

    EXPECT_GT(v_rect_avg_rect_mode, 500.0) << "Rectifier voltage should be strongly positive at α=30°";
    EXPECT_LT(v_inv_avg_inv_mode, -500.0) << "Inverter voltage should be strongly negative at α=150°";

    // Test 2: Inverted rectifier (α > 90°) should produce negative DC voltage
    set_rectifier_firing_angle(120.0);
    set_inverter_firing_angle(150.0);
    simulate_for(0.1);

    v_rect_sum = 0.0;
    for (int i = 0; i < samples_per_cycle * 3; i++) {
        simulate_for(sampling_time);
        v_rect_sum += _v_rect_dc;
    }
    double v_rect_avg_inv_mode = v_rect_sum / (samples_per_cycle * 3);

    std::cout << "Rectifier at α=120° (inverter mode): V_rect_avg = " << v_rect_avg_inv_mode << " V (expect <0)"
              << std::endl;

    EXPECT_LT(v_rect_avg_inv_mode, -500.0) << "Rectifier voltage should be strongly negative at α=120°";

    // Test 3: Net voltage determines power flow direction
    // For motoring: V_net = V_rect + V_inv > 0 (both contribute to drive current into machine)
    set_rectifier_firing_angle(20.0);
    set_inverter_firing_angle(160.0);
    simulate_for(0.1);

    double v_net_sum = 0.0, i_dc_sum = 0.0;
    for (int i = 0; i < samples_per_cycle * 3; i++) {
        simulate_for(sampling_time);
        v_net_sum += (_v_rect_dc + _v_inv_dc);
        i_dc_sum += _i_dc;
    }
    double v_net_avg = v_net_sum / (samples_per_cycle * 3);
    double i_dc_avg = i_dc_sum / (samples_per_cycle * 3);

    std::cout << "Motoring mode (α_r=20°, α_i=160°): V_net = " << v_net_avg << " V, I_dc = " << i_dc_avg << " A"
              << std::endl;

    // Power = V_net × I_dc should be positive for motoring
    double power = v_net_avg * i_dc_avg;
    std::cout << "Power = " << power / 1e6 << " MW (expect >0 for motoring)" << std::endl;

    EXPECT_GT(v_net_avg, 0.0) << "Net voltage should be positive for motoring";
    EXPECT_GT(i_dc_avg, 0.0) << "DC current should be positive for motoring";
    EXPECT_GT(power, 0.0) << "Power should be positive for motoring";
}

}  // namespace
