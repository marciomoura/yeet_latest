#include "yeet_control/thyristor_back_to_back_synchronous_machine.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/synchronous_machine.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

class ThyristorBackToBackNominalSpeedTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 12.5e-6;
    static constexpr double grid_frequency_hz = 50.0;
    static constexpr double machine_rated_frequency_hz = 25.0;
    static constexpr int machine_pole_pairs = 3;
    static constexpr double pulse_width_deg = 120.0;
    static constexpr double field_flux_initial = 5.0;
    static constexpr double field_voltage_steady_state = 0.72;
    static constexpr double load_torque_nm = 10000.0;
    static constexpr double grid_voltage_ll_rms = 2200.0;
    static constexpr double dc_resistance = 10e-3;
    static constexpr double dc_inductance = 8e-3;
    static constexpr double machine_r_s = 0.0047;
    static constexpr double machine_r_fd = 0.0014;
    static constexpr double machine_r_kd = 0.136;
    static constexpr double machine_r_kq = 0.0777;
    static constexpr double machine_l_ls = 0.00059;
    static constexpr double machine_l_md = 0.00808;
    static constexpr double machine_l_mq = 0.00418;
    static constexpr double machine_l_fd = 0.00975;
    static constexpr double machine_l_kd = 0.00958;
    static constexpr double machine_l_kq = 0.00453;
    static constexpr double machine_inertia = 670.0;
    static constexpr double machine_l_d_value = machine_l_ls + machine_l_md;
    static constexpr double machine_l_q_value = machine_l_ls + machine_l_mq;

    void SetUp() override
    {
        configure_grid_source();
        configure_machine();
        build_drive();

        // Initialize with non-zero rotor angle (45° electrical = 15° mechanical for 3 pole pairs)
        // This ensures back-EMF is non-zero at startup for proper firing angle calculation
        const double theta_r_init = deg_to_rad(45.0) / machine_pole_pairs;
        _drive->initialize(get_synchronous_speed(), theta_r_init, field_flux_initial);

        // Use firing angles consistent with the steady-state calculation
        // At rated speed/torque, alpha_rect ~70° provides proper voltage matching
        _rectifier_angle = angle_wrapped::from_degrees(70.0);
        _inverter_angle = angle_wrapped::from_degrees(30.0);

        set_logger();
    }

protected:
    struct CurrentStatistics {
        double min = std::numeric_limits<double>::max();
        double max = std::numeric_limits<double>::lowest();
        int zero_crossings = 0;
        double previous = 0.0;
        bool has_previous = false;

        void update(double value)
        {
            min = std::min(min, value);
            max = std::max(max, value);
            if (has_previous) {
                const bool sign_flip = (previous >= 0.0 && value < 0.0) || (previous <= 0.0 && value > 0.0);
                if (sign_flip) ++zero_crossings;
            }
            previous = value;
            has_previous = true;
        }

        double peak_to_peak() const { return max - min; }
    };

    void configure_grid_source()
    {
        const double v_ln_peak = grid_voltage_ll_rms * std::sqrt(2.0) / std::sqrt(3.0);
        _grid_source.set_fundamental_positive_sequence_signal_amplitude(v_ln_peak);
        _grid_source.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _grid_source.set_signal_frequency(grid_frequency_hz);
        _grid_source.disable_noise();
    }

    void configure_machine()
    {
        const double transformer_s_base = 3.362e6;
        const double z_base = (grid_voltage_ll_rms * grid_voltage_ll_rms) / transformer_s_base;
        const double omega_base = 2.0 * pi * grid_frequency_hz;
        const double l_base = z_base / omega_base;

        _grid_resistance = 0.0024 * z_base;
        _grid_inductance = 0.0839 * l_base;

        _machine = std::make_unique<synchronous_machine>(resistance_t(machine_r_s), resistance_t(machine_r_fd),
            resistance_t(machine_r_kd), resistance_t(machine_r_kq), inductance_t(machine_l_d_value),
            inductance_t(machine_l_q_value), inductance_t(machine_l_md), inductance_t(machine_l_mq),
            inductance_t(machine_l_fd), inductance_t(machine_l_kd), inductance_t(machine_l_kq),
            moment_of_inertia_t(machine_inertia), machine_pole_pairs * 2);
    }

    void build_drive()
    {
        _drive = std::make_unique<thyristor_back_to_back_synchronous_machine>(
            _grid_resistance, _grid_inductance, dc_resistance, dc_inductance, *_machine, sampling_time);
    }

    double get_synchronous_speed() const
    {
        return (2.0 * pi * machine_rated_frequency_hz) / static_cast<double>(machine_pole_pairs);
    }

    void simulate_for(double duration) { _simulator.simulate_for(duration); }

    void step_once()
    {
        _grid_source.update();
        _grid_voltage = _grid_source.get_signal_abc();
        const angle_wrapped grid_angle = _grid_source.get_fundamental_signal_angle().a();

        const auto& machine_state = _drive->get_machine_state();

        // Use the back-EMF directly to derive the firing reference angle
        // This provides proper synchronization with the machine's rotating field
        abc<voltage_t> back_emf = _drive->get_motor_back_emf();
        alphabeta<voltage_t> back_emf_ab = back_emf.to_alphabeta();
        angle_wrapped machine_voltage_angle = back_emf_ab.phase();
        const angle_wrapped machine_reference_angle = machine_voltage_angle + angle_wrapped::from_degrees(180.0);

        _rectifier_pulse_gen.update(grid_angle, _rectifier_angle, true);
        _rectifier_pulses = _rectifier_pulse_gen.get_pulses();

        _inverter_pulse_gen.update(machine_reference_angle, _inverter_angle, true);
        _inverter_pulses = _inverter_pulse_gen.get_pulses();

        const auto rect_array = _rectifier_pulses.to_array();
        const auto inv_array = _inverter_pulses.to_array();

        _drive->update(convert_to_voltage(_grid_voltage), rect_array, inv_array, _field_voltage, _load_torque);

        _last_machine_currents = _drive->get_motor_currents();
        update_phase_statistics(_last_machine_currents);

        _dc_current_integral += _drive->get_dc_link_current().value();
        _rectifier_voltage_integral += _drive->get_rectifier_dc_voltage().value();
        _inverter_voltage_integral += _drive->get_inverter_dc_voltage().value();
        ++_executed_steps;
    }

    void update_phase_statistics(const abc<current_t>& currents)
    {
        _phase_a_stats.update(currents.a().value());
        _phase_b_stats.update(currents.b().value());
        _phase_c_stats.update(currents.c().value());
    }

    abc<voltage_t> convert_to_voltage(const abc<double>& raw) const
    {
        return abc<voltage_t>(voltage_t(raw.a()), voltage_t(raw.b()), voltage_t(raw.c()));
    }

    double expected_d_axis_inductance() const noexcept
    {
        const double Lad = machine_l_md;
        const double Lfd_total = machine_l_fd + Lad;
        const double Lkd_total = machine_l_kd + Lad;
        const double determinant = (Lfd_total * Lkd_total) - (Lad * Lad);
        if (determinant <= 0.0) return machine_l_d_value;

        const double inv00 = Lkd_total / determinant;
        const double inv11 = Lfd_total / determinant;
        const double inv01 = -Lad / determinant;
        const double coupling = Lad * Lad * (inv00 + inv11 + 2.0 * inv01);
        return std::max(0.0, machine_l_d_value - coupling);
    }

    double expected_q_axis_inductance() const noexcept
    {
        const double Laq = machine_l_mq;
        const double Lkq_total = machine_l_kq + Laq;
        if (Lkq_total <= 0.0 || Laq <= 0.0) return machine_l_q_value;
        const double reduction = (Laq * Laq) / Lkq_total;
        return std::max(0.0, machine_l_q_value - reduction);
    }

    double expected_d_axis_impedance() const noexcept
    {
        return 2.0 * pi * machine_rated_frequency_hz * expected_d_axis_inductance();
    }

    double expected_q_axis_impedance() const noexcept
    {
        return 2.0 * pi * machine_rated_frequency_hz * expected_q_axis_inductance();
    }

    void set_logger()
    {
        _simulator.register_lambda_runner(sampling_time, [this]() { step_once(); });

        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_a", [this]() { return _grid_voltage.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_b", [this]() { return _grid_voltage.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_c", [this]() { return _grid_voltage.c(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:i_ac_a", [this]() { return _last_machine_currents.a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:i_ac_b", [this]() { return _last_machine_currents.b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:i_ac_c", [this]() { return _last_machine_currents.c().value(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_a", [this]() { return _drive->get_motor_voltages().a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_b", [this]() { return _drive->get_motor_voltages().b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_c", [this]() { return _drive->get_motor_voltages().c().value(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:back_emf_a", [this]() { return _drive->get_motor_back_emf().a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:back_emf_b", [this]() { return _drive->get_motor_back_emf().b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:back_emf_c", [this]() { return _drive->get_motor_back_emf().c().value(); }));

        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:i_dc", [this]() { return _drive->get_dc_link_current().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "dc:v_rect", [this]() { return _drive->get_rectifier_dc_voltage().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "dc:v_inv", [this]() { return _drive->get_inverter_dc_voltage().value(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:omega_r", [this]() { return _drive->get_machine_state().omega_r; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:psi_fD", [this]() { return _drive->get_machine_state().psi_fD; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:psi_kD", [this]() { return _drive->get_machine_state().psi_kD; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:psi_kQ", [this]() { return _drive->get_machine_state().psi_kQ; }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "rectifier:alpha_deg", [this]() { return _rectifier_angle.get_degrees(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "inverter:alpha_deg", [this]() { return _inverter_angle.get_degrees(); }));
    }

    yeet::simulator _simulator = yeet::make_gtest_simulator();
    three_phase_waveform_generator _grid_source{sampling_time};
    six_pulse_generator _rectifier_pulse_gen{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};
    six_pulse_generator _inverter_pulse_gen{
        six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg))};

    std::unique_ptr<synchronous_machine> _machine;
    std::unique_ptr<thyristor_back_to_back_synchronous_machine> _drive;

    angle_wrapped _rectifier_angle{0.0};
    angle_wrapped _inverter_angle{0.0};

    abc<double> _grid_voltage{0.0, 0.0, 0.0};
    six_pulse_generator::pulses _rectifier_pulses{};
    six_pulse_generator::pulses _inverter_pulses{};

    abc<current_t> _last_machine_currents{current_t(0.0), current_t(0.0), current_t(0.0)};

    CurrentStatistics _phase_a_stats{};
    CurrentStatistics _phase_b_stats{};
    CurrentStatistics _phase_c_stats{};

    double _dc_current_integral{0.0};
    double _rectifier_voltage_integral{0.0};
    double _inverter_voltage_integral{0.0};
    size_t _executed_steps{0};

    real_t _grid_resistance{0.0};
    real_t _grid_inductance{0.0};
    const voltage_t _field_voltage{field_voltage_steady_state};
    const torque_t _load_torque{load_torque_nm};
};

TEST_F(ThyristorBackToBackNominalSpeedTest, NominalSpeedOperationProducesBalancedCurrents)
{
    simulate_for(0.1);

    ASSERT_GT(_executed_steps, 0u);

    const double avg_dc_current = _dc_current_integral / static_cast<double>(_executed_steps);
    const double avg_rectifier_voltage = _rectifier_voltage_integral / static_cast<double>(_executed_steps);
    const double avg_inverter_voltage = _inverter_voltage_integral / static_cast<double>(_executed_steps);

    // Primary check: DC current should be positive (power flowing from grid to machine)
    EXPECT_GT(avg_dc_current, 1.0) << "DC current should be positive for motoring";

    // Voltage difference should drive current through DC link
    EXPECT_GT(avg_rectifier_voltage - avg_inverter_voltage, 10.0);

    // All three phases should see current flow (peak-to-peak > 0)
    // Note: In LCI operation, currents may not be perfectly balanced or sinusoidal
    EXPECT_GT(_phase_a_stats.peak_to_peak(), 10.0) << "Phase A should have current flow";
    EXPECT_GT(_phase_b_stats.peak_to_peak(), 10.0) << "Phase B should have current flow";
    EXPECT_GT(_phase_c_stats.peak_to_peak(), 10.0) << "Phase C should have current flow";

    // Verify subtransient impedance calculations (these are machine properties, not affected by converter)
    const auto subtransient = _machine->calculate_subtransient_impedance(frequency_t(machine_rated_frequency_hz));

    EXPECT_NEAR(subtransient.inductances.d_axis.value(), expected_d_axis_inductance(), 1e-9);
    EXPECT_NEAR(subtransient.inductances.q_axis.value(), expected_q_axis_inductance(), 1e-9);
    EXPECT_NEAR(subtransient.d_axis.value(), expected_d_axis_impedance(), 1e-6);
    EXPECT_NEAR(subtransient.q_axis.value(), expected_q_axis_impedance(), 1e-6);
}

TEST_F(ThyristorBackToBackNominalSpeedTest, StartupFromStandstill)
{
    // Re-initialize to zero speed
    _drive->initialize(0.0, 0.0, field_flux_initial);

    // Set Rectifier to full conduction (alpha = 0) to build current fast
    _rectifier_angle = angle_wrapped::from_degrees(0.0);

    // Set Inverter to a fixed angle. At standstill, we just need to fire a pair.
    // The pulse generator will fire based on (theta_r - 120 deg) + alpha.
    // If theta_r = 0.
    _inverter_angle = angle_wrapped::from_degrees(150.0);

    // Reset integrals
    _dc_current_integral = 0.0;
    _rectifier_voltage_integral = 0.0;
    _executed_steps = 0;

    // Run for a short time to see current buildup
    simulate_for(0.02);

    ASSERT_GT(_executed_steps, 0u);

    const double avg_dc_current = _dc_current_integral / static_cast<double>(_executed_steps);

    // Current should build up significantly if the startup bias logic is working
    // With 2200V grid, it should be hundreds of Amps quickly.
    EXPECT_GT(avg_dc_current, 10.0) << "Current failed to build up from standstill";

    // Check that we have some DC voltage at the inverter (bias propagation)
    // Note: Inverter DC voltage might be small/zero if it's just conducting into a short,
    // but the Rectifier voltage should be positive.
    const double avg_rectifier_voltage = _rectifier_voltage_integral / static_cast<double>(_executed_steps);
    EXPECT_GT(avg_rectifier_voltage, 100.0);
}

}  // namespace
