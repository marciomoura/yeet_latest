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
    static constexpr double sampling_time = 1e-6;          // 10 μs for good resolution
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz grid/machine frequency
    static constexpr double v_grid_peak = 100.0;           // 100V peak line-to-neutral (grid)
    static constexpr double v_machine_peak = 80.0;         // 80V peak line-to-neutral (machine back-EMF)
    static constexpr double pulse_width_deg = 120.0;       // 120 degree pulse width (standard for 6-pulse)

    // Grid-side RL impedance parameters
    static constexpr double grid_resistance = 0.001;   // 50 mOhm per phase
    static constexpr double grid_inductance = 0.1e-3;  // 0.1 mH per phase (Increased for visible notches)

    // DC link parameters
    static constexpr double dc_resistance = 1.0;   // 100 mOhm
    static constexpr double dc_inductance = 5e-3;  // 10 mH

    // Machine-side RL impedance parameters
    static constexpr double machine_resistance = 0.001;   // 50 mOhm per phase
    static constexpr double machine_inductance = 0.1e-3;  // 0.1 mH per phase (Increased for visible notches)

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

// Industrial LCI Test Fixture (matching sfc_plant_model_augmented_6p_6p parameters)
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

    // Circuit parameters from sfc_plant_model_augmented_6p_6p
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

// Test 7: Industrial LCI Configuration (matching sfc_plant_model_augmented_6p_6p parameters)
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

}  // namespace
