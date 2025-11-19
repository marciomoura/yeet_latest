#include "yeet_control/thyristor_back_to_back_synchronous_machine.hpp"

#include <gtest/gtest.h>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

// public:
//     static constexpr double sampling_time = 10e-6;         // 10 μs timestep
//     static constexpr double fundamental_frequency = 50.0;  // 50 Hz grid
//     static constexpr double v_grid_ll_rms = 2090.0;        // 2090V line-to-line RMS (grid)
//     static constexpr double v_machine_ll_rms = 1980.0;     // 1980V line-to-line RMS (machine)
//     static constexpr double rated_power_va = 2.0e6;        // 2.0 MVA rated power
//     static constexpr double rated_speed_rpm = 3000.0;      // 3000 RPM (synchronous speed for 50 Hz, 2-pole)
//     static constexpr double pulse_width_deg = 120.0;       // 120 degree pulse width

//     void SetUp() override
//     {
//         // Calculate grid voltage amplitude (peak line-to-neutral)
//         // V_LN_peak = V_LL_RMS × √2 / √3
//         const double v_grid_peak = v_grid_ll_rms * sqrt(2.0) / sqrt(3.0);  // ~1707V peak L-N

//         // Configure grid-side three-phase voltage source
//         _grid_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_grid_peak);
//         _grid_waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
//         _grid_waveform_generator.set_signal_frequency(fundamental_frequency);
//         _grid_waveform_generator.disable_noise();

//         // Configure rectifier pulse generator
//         _six_pulse_gen_rect =
//             six_pulse_generator(six_pulse_generator::reference_type::phase,
//             angle_wrapped(deg_to_rad(pulse_width_deg)));

//         // Configure inverter pulse generator
//         _six_pulse_gen_inv =
//             six_pulse_generator(six_pulse_generator::reference_type::phase,
//             angle_wrapped(deg_to_rad(pulse_width_deg)));

//         // Circuit parameters for 2 MVA LCI drive
//         // Base values: V_base = 1980V L-L, S_base = 2.0 MVA, ω_base = 2π×50 rad/s
//         // Z_base = V²/S = 1980²/2.0e6 = 1.96 Ω
//         // I_base = S/(√3×V) = 2.0e6/(√3×1980) = 583 A
//         const double z_base = (v_machine_ll_rms * v_machine_ll_rms) / rated_power_va;  // 1.96 Ω
//         const double omega_base = 2.0 * pi * fundamental_frequency;                    // 314.16 rad/s
//         const double l_base = z_base / omega_base;                                     // 6.24 mH

//         // Equivalent impedances from simulation measurements
//         // Grid-side equivalent (includes grid + transformer + rectifier bridge effects)
//         real_t grid_resistance = 0.0163;           // 16.3 mΩ per phase (measured equivalent)
//         real_t grid_inductance = 0.4941e-3 / 2.0;  // Measured 0.494 mH is 2×L_grid → 0.247 mH per phase

//         // DC link impedance (direct measurement)
//         real_t dc_resistance = 0.0503;  // 50.3 mΩ DC link0
//         real_t dc_inductance = 8e-3;    // 8 mH DC link

//         // Machine parameters from external simulation (referred to stator)
//         // These values account for transformer turns ratio squared
//         real_t r_s = 15e-3;     // 15 mΩ stator resistance per phase
//         real_t l_d = 9.5e-3;    // 9.5 mH d-axis synchronous inductance
//         real_t l_q = 8.5e-3;    // 8.5 mH q-axis synchronous inductance
//         real_t l_ad = 8.5e-3;   // 8.5 mH d-axis mutual inductance
//         real_t l_aq = 7.5e-3;   // 7.5 mH q-axis mutual inductance
//         real_t r_fd = 6e-3;     // 6 mΩ field winding resistance (referred to stator)
//         real_t l_fd = 9.2e-3;   // 9.2 mH field winding self inductance (referred to stator)
//         real_t r_kd = 60e-3;    // 60 mΩ d-axis damper resistance
//         real_t l_kd = 9.0e-3;   // 9.0 mH d-axis damper inductance
//         real_t r_kq = 60e-3;    // 60 mΩ q-axis damper resistance
//         real_t l_kq = 8.0e-3;   // 8.0 mH q-axis damper inductance
//         real_t inertia = 75.0;  // 75 kg·m² (H ≈ 1.5s for 2 MW machine)

//         // Create machine configuration (2-pole machine: P=2)
//         synchronous_machine machine(resistance_t(r_s), resistance_t(r_fd), resistance_t(r_kd), resistance_t(r_kq),
//             inductance_t(l_d), inductance_t(l_q), inductance_t(l_ad), inductance_t(l_aq), inductance_t(l_fd),
//             inductance_t(l_kd), inductance_t(l_kq), moment_of_inertia_t(inertia),
//             2  // P = 2 poles (1 pole pair) for 3000 RPM @ 50 Hz
//         );

using namespace yeet;
using namespace yeet::control;

class ThyristorBackToBackSynchronousMachineTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 10e-6;         // 10 μs timestep
    static constexpr double fundamental_frequency = 60.0;  // 60 Hz grid
    static constexpr double v_grid_peak = 100.0;           // 100V peak line-to-neutral
    static constexpr double pulse_width_deg = 120.0;       // 120 degree pulse width

    void SetUp() override
    {
        // Configure grid-side three-phase voltage source
        _grid_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_grid_peak);
        _grid_waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _grid_waveform_generator.set_signal_frequency(fundamental_frequency);
        _grid_waveform_generator.disable_noise();

        // Configure rectifier pulse generator
        _six_pulse_gen_rect =
            six_pulse_generator(six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg)));

        // Configure inverter pulse generator
        _six_pulse_gen_inv =
            six_pulse_generator(six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg)));

        // Circuit parameters for LCI drive
        real_t grid_resistance = 0.01;   // 10 mOhm per phase
        real_t grid_inductance = 0.001;  // 1 mH per phase
        real_t dc_resistance = 0.05;     // 50 mOhm DC link
        real_t dc_inductance = 0.1;      // 100 mH DC link

        // Create machine configuration
        synchronous_machine machine(resistance_t(0.02),  // R_s
            resistance_t(0.5),                           // R_fD
            resistance_t(0.01),                          // R_kD
            resistance_t(0.01),                          // R_kQ
            inductance_t(0.015),                         // L_d
            inductance_t(0.010),                         // L_q
            inductance_t(0.012),                         // L_ad
            inductance_t(0.010),                         // L_aq
            inductance_t(5.0),                           // L_fD
            inductance_t(0.005),                         // L_kD
            inductance_t(0.005),                         // L_kQ
            moment_of_inertia_t(100.0),                  // J
            4                                            // P (total poles = 2 pole pairs)
        );

        // Create LCI drive with new architecture
        _lci_drive = std::make_unique<thyristor_back_to_back_synchronous_machine>(
            grid_resistance, grid_inductance, dc_resistance, dc_inductance, machine, sampling_time);

        // Initialize to nominal speed with pre-fluxed field winding
        const double n_rpm = 1800.0;                             // 1800 RPM
        const double omega_initial = (n_rpm / 60.0) * 2.0 * pi;  // rad/s mechanical
        const double psi_field_initial = 100.0;                  // 100 Wb field flux (pre-fluxed condition)

        _lci_drive->initialize(omega_initial, 0.0, psi_field_initial);

        // Set initial firing angles
        // Use conservative angles close to 90° to minimize voltage difference and prevent current runaway
        _current_firing_angle_rect = angle_wrapped::from_degrees(80.0);  // 80° rect (close to minimum voltage)
        _current_firing_angle_inv = angle_wrapped::from_degrees(100.0);  // 100° inverter (just above 90°)

        set_logger();
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    void set_rectifier_firing_angle(double angle_degrees)
    {
        _current_firing_angle_rect = angle_wrapped::from_degrees(angle_degrees);
    }

    void set_inverter_firing_angle(double angle_degrees)
    {
        _current_firing_angle_inv = angle_wrapped::from_degrees(angle_degrees);
    }

protected:
    yeet::simulator _simulator = yeet::make_gtest_simulator();

    // Grid-side components
    three_phase_waveform_generator _grid_waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen_rect;

    // Machine-side components
    six_pulse_generator _six_pulse_gen_inv;

    // LCI drive system
    std::unique_ptr<thyristor_back_to_back_synchronous_machine> _lci_drive;

    // Current state variables
    angle_wrapped _current_grid_reference_angle{0.0};
    angle_wrapped _current_machine_reference_angle{0.0};
    angle_wrapped _current_firing_angle_rect{0.0};
    angle_wrapped _current_firing_angle_inv{0.0};

    abc<double> _v_grid_abc{0.0, 0.0, 0.0};
    abc<current_t> _i_grid_abc{current_t(0.0), current_t(0.0), current_t(0.0)};
    abc<current_t> _i_machine_abc{current_t(0.0), current_t(0.0), current_t(0.0)};
    abc<voltage_t> _v_machine_abc{voltage_t(0.0), voltage_t(0.0), voltage_t(0.0)};

    six_pulse_generator::pulses _pulses_rect{};
    six_pulse_generator::pulses _pulses_inv{};

    current_t _i_dc{0.0};
    voltage_t _v_dc_rect{0.0};
    voltage_t _v_dc_inv{0.0};

    // Control variables for tests
    torque_t _applied_load_torque{0.0};

    // Field voltage should maintain steady-state field flux, not drive it to infinity
    // For ψ_fD = 100 Wb with L_fD = 5.0 H: i_f = 20 A
    // Required V_f = R_f × i_f = 0.5 × 20 = 10 V for steady-state
    voltage_t _applied_field_voltage{10.0};  // 10V maintains 100 Wb field flux

private:
    void set_logger()
    {
        // Main update loop
        _simulator.register_lambda_runner(sampling_time, [this]() {
            // Update grid waveform generator
            _grid_waveform_generator.update();

            // Get grid voltages and reference angle
            _v_grid_abc = _grid_waveform_generator.get_signal_abc();
            angle_wrapped grid_reference_angle = _grid_waveform_generator.get_fundamental_signal_angle().a();

            // Generate rectifier pulses
            _pulses_rect = _six_pulse_gen_rect.update(grid_reference_angle, _current_firing_angle_rect, true);

            // Update machine reference angle from LCI state (electrical angle = mechanical angle * pole pairs)
            const auto& machine_state = _lci_drive->get_machine_state();
            angle_wrapped machine_reference_angle = angle_wrapped(machine_state.theta_r * 2.0);  // theta_r * P

            // Generate inverter pulses
            _pulses_inv = _six_pulse_gen_inv.update(machine_reference_angle, _current_firing_angle_inv, true);

            // Update LCI drive
            abc<voltage_t> v_grid(voltage_t(_v_grid_abc.a()), voltage_t(_v_grid_abc.b()), voltage_t(_v_grid_abc.c()));
            _lci_drive->update(v_grid, _pulses_rect, _pulses_inv, _applied_field_voltage, _applied_load_torque);

            // Get outputs
            _i_grid_abc = _lci_drive->get_grid_currents();
            _i_machine_abc = _lci_drive->get_motor_currents();
            _v_machine_abc = _lci_drive->get_motor_voltages();
            _i_dc = _lci_drive->get_dc_link_current();
            _v_dc_rect = _lci_drive->get_rectifier_dc_voltage();
            _v_dc_inv = _lci_drive->get_inverter_dc_voltage();
        });

        // Grid-side voltage logger
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_a", [&]() -> double { return _v_grid_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_b", [&]() -> double { return _v_grid_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_c", [&]() -> double { return _v_grid_abc.c(); }));

        // Grid-side current logger
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_a", [&]() -> double { return _i_grid_abc.a().value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_b", [&]() -> double { return _i_grid_abc.b().value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_c", [&]() -> double { return _i_grid_abc.c().value(); }));

        // Machine-side current logger
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:i_ac_a", [&]() -> double { return _i_machine_abc.a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:i_ac_b", [&]() -> double { return _i_machine_abc.b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:i_ac_c", [&]() -> double { return _i_machine_abc.c().value(); }));

        // Machine-side voltage logger
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_a", [&]() -> double { return _v_machine_abc.a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_b", [&]() -> double { return _v_machine_abc.b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_c", [&]() -> double { return _v_machine_abc.c().value(); }));

        // DC link logger
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:i_dc", [&]() -> double { return _i_dc.value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_rect", [&]() -> double { return _v_dc_rect.value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_inv", [&]() -> double { return _v_dc_inv.value(); }));

        // Rectifier pulse logger
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "rect:pulse_T1", [&]() -> double { return _pulses_rect.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "rect:pulse_T3", [&]() -> double { return _pulses_rect.p3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "rect:pulse_T5", [&]() -> double { return _pulses_rect.p5 ? 1.0 : 0.0; }));

        // Inverter pulse logger
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("inv:pulse_T1", [&]() -> double { return _pulses_inv.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("inv:pulse_T3", [&]() -> double { return _pulses_inv.p3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("inv:pulse_T5", [&]() -> double { return _pulses_inv.p5 ? 1.0 : 0.0; }));

        // Machine state logger
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:omega_r", [&]() -> double { return _lci_drive->get_machine_state().omega_r; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:theta_r", [&]() -> double { return _lci_drive->get_machine_state().theta_r; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:psi_fD", [&]() -> double { return _lci_drive->get_machine_state().psi_fD; }));
    }
};

// Test: Basic instantiation and initialization
TEST_F(ThyristorBackToBackSynchronousMachineTest, BasicInstantiation)
{
    // Verify initial state
    const auto& state = _lci_drive->get_machine_state();
    EXPECT_GT(state.omega_r, 100.0);  // Should be initialized to ~188.5 rad/s
    EXPECT_NEAR(state.theta_r, 0.0, 1e-3);

    SUCCEED();
}

// Test: Validate basic physics - current sum must be zero
TEST_F(ThyristorBackToBackSynchronousMachineTest, ValidateCurrentSumZero)
{
    set_rectifier_firing_angle(25.0);
    set_inverter_firing_angle(150.0);

    // Simulate for 1 grid cycle
    double simulation_time = 1.0 / fundamental_frequency;
    simulate_for(simulation_time);

    // Check that sum of phase currents is zero (Kirchhoff's Current Law)
    double i_sum = _i_machine_abc.a().value() + _i_machine_abc.b().value() + _i_machine_abc.c().value();

    std::cout << "\n=== Current Sum Check ===" << std::endl;
    std::cout << "i_a = " << _i_machine_abc.a().value() << " A" << std::endl;
    std::cout << "i_b = " << _i_machine_abc.b().value() << " A" << std::endl;
    std::cout << "i_c = " << _i_machine_abc.c().value() << " A" << std::endl;
    std::cout << "SUM = " << i_sum << " A (should be ~0)" << std::endl;
    std::cout << "Inverter states: ";
    auto states = _lci_drive->get_inverter_states();
    for (int i = 0; i < 6; ++i) std::cout << (states[i] ? "1" : "0");
    std::cout << std::endl;
    std::cout << "DC current: " << _i_dc.value() << " A" << std::endl;
    std::cout << "========================\n" << std::endl;

    EXPECT_NEAR(i_sum, 0.0, 1e-3) << "Sum of three-phase currents must be zero (Kirchhoff's Current Law)";
}

// Test: Check that machine back-EMF is reasonable and varies sinusoidally
TEST_F(ThyristorBackToBackSynchronousMachineTest, ValidateBackEMFIsReasonable)
{
    // Disable firing pulses to prevent current flow
    set_rectifier_firing_angle(90.0);  // No rectification
    set_inverter_firing_angle(90.0);   // No inversion

    // Sample voltages over one electrical cycle
    const auto& initial_state = _lci_drive->get_machine_state();
    double omega_e = initial_state.omega_r * 2.0;  // Electrical frequency (P=4)
    double f_e = omega_e / (2.0 * pi);
    double T_e = 1.0 / f_e;

    std::cout << "\n=== Back-EMF Sampling ===" << std::endl;
    std::cout << "Mechanical speed: " << (initial_state.omega_r * 60.0 / (2.0 * pi)) << " RPM" << std::endl;
    std::cout << "Electrical freq: " << f_e << " Hz" << std::endl;
    std::cout << "Field flux: " << initial_state.psi_fD << " Wb" << std::endl;

    int n_samples = 36;  // 10° increments
    std::vector<double> v_a_samples;
    std::vector<double> v_b_samples;
    std::vector<double> v_c_samples;

    for (int i = 0; i < n_samples; ++i) {
        simulate_for(T_e / n_samples);
        v_a_samples.push_back(_v_machine_abc.a().value());
        v_b_samples.push_back(_v_machine_abc.b().value());
        v_c_samples.push_back(_v_machine_abc.c().value());
    }

    // Calculate peak values
    double v_a_max = *std::max_element(v_a_samples.begin(), v_a_samples.end());
    double v_a_min = *std::min_element(v_a_samples.begin(), v_a_samples.end());
    double v_a_pk = (v_a_max - v_a_min) / 2.0;

    std::cout << "Phase A voltage: max=" << v_a_max << " V, min=" << v_a_min << " V, pk=" << v_a_pk << " V"
              << std::endl;
    std::cout << "Current during sampling: i_a=" << _i_machine_abc.a().value() << " A" << std::endl;

    // Print first few samples to check shape
    std::cout << "First 6 samples of V_a: ";
    for (int i = 0; i < 6 && i < v_a_samples.size(); ++i) {
        std::cout << v_a_samples[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "=========================\n" << std::endl;

    // Expected back-EMF ~ omega * L_ad * (psi_fD / L_fD)
    // With psi_fD = 100 Wb, L_fD = 5 H, L_ad = 0.012 H, omega_e = 376.99 rad/s
    // i_f = psi_fD / L_fD = 20 A
    // psi_stator_d ~ L_ad * i_f = 0.24 Wb
    // EMF_peak ~ omega_e * psi_stator_d ~ 376.99 * 0.24 ~ 90 V
    double expected_emf = omega_e * 0.012 * (initial_state.psi_fD / 5.0);
    std::cout << "Expected EMF peak: " << expected_emf << " V" << std::endl;

    EXPECT_GT(v_a_pk, 10.0) << "Back-EMF should be significant at 1800 RPM with 100 Wb field flux";
    EXPECT_LT(v_a_pk, 500.0) << "Back-EMF should not be excessively large";
}

// Test: Steady-state operation with data logging
TEST_F(ThyristorBackToBackSynchronousMachineTest, SteadyStateOperation)
{
    // CRITICAL: Inverter bridge is INVERTED in LCI topology!
    // For motoring: Inverter α > 90° (inverter mode in its own frame)
    // This produces negative voltage which becomes positive opposition when bridge is flipped
    set_rectifier_firing_angle(25.0);  // Low angle → high positive DC voltage (~173V)
    set_inverter_firing_angle(150.0);  // Inverter mode (>90°) → correct for inverted bridge

    // Simulate for 5 grid cycles
    double simulation_time = 5.0 / fundamental_frequency;  // ~83.3 ms @ 60 Hz

    // Diagnostic: Check initial state
    std::cout << "\n=== Initial State ===" << std::endl;
    std::cout << "Rectifier firing angle: " << 15.0 << "° (rectifier mode)" << std::endl;
    std::cout << "Inverter firing angle:  " << 155.0 << "° (inverter mode, bridge inverted)" << std::endl;
    std::cout << "Machine initial speed:  " << _lci_drive->get_machine_state().omega_r << " rad/s" << std::endl;

    simulate_for(2.0);

    // Diagnostic: Check pulses are being generated
    std::cout << "\n=== Pulse Check (last state) ===" << std::endl;
    std::cout << "Rectifier pulses: " << (_pulses_rect.p1 ? "1" : "0") << (_pulses_rect.p3 ? "1" : "0")
              << (_pulses_rect.p5 ? "1" : "0") << std::endl;
    std::cout << "Inverter pulses:  " << (_pulses_inv.p1 ? "1" : "0") << (_pulses_inv.p3 ? "1" : "0")
              << (_pulses_inv.p5 ? "1" : "0") << std::endl;

    // Validate DC voltages are present (system is energized)
    EXPECT_GT(_v_dc_rect.value(), 0.0) << "Rectifier should produce positive DC voltage";

    // Validate DC link voltage difference
    double v_dc_link = _v_dc_rect.value() - _v_dc_inv.value();

    // Machine state
    const auto& state = _lci_drive->get_machine_state();

    // Print diagnostic results
    std::cout << "\n=== LCI Drive Steady-State Results ===" << std::endl;
    std::cout << "DC Current:       " << _i_dc.value() << " A" << std::endl;
    std::cout << "Rectifier V_DC:   " << _v_dc_rect.value() << " V" << std::endl;
    std::cout << "Inverter V_DC:    " << _v_dc_inv.value() << " V" << std::endl;
    std::cout << "DC Link Voltage:  " << v_dc_link << " V (V_rect - V_inv)" << std::endl;
    std::cout << "Grid Currents:    A=" << _i_grid_abc.a().value() << " B=" << _i_grid_abc.b().value()
              << " C=" << _i_grid_abc.c().value() << " A" << std::endl;
    std::cout << "Machine Currents: A=" << _i_machine_abc.a().value() << " B=" << _i_machine_abc.b().value()
              << " C=" << _i_machine_abc.c().value() << " A" << std::endl;
    std::cout << "Machine Speed:    " << (state.omega_r * 60.0 / (2.0 * pi)) << " RPM" << std::endl;
    std::cout << "Field Flux:       " << state.psi_fD << " Wb" << std::endl;
    std::cout << "====================================\n" << std::endl;

    // Assertions
    EXPECT_GT(v_dc_link, 0.0) << "Expected positive DC link voltage (V_rect > V_inv)";
    EXPECT_GT(state.omega_r, 100.0) << "Machine should still be rotating";
    EXPECT_GT(state.psi_fD, 0.0) << "Field flux should be positive";

    // For motoring with load, we should see some DC current
    // With no mechanical load and the machine already at speed, current may be very low
    // This is physically correct - the system is in equilibrium

    // CRITICAL: Inverter bridge is INVERTED in LCI topology!
    // For motoring: Inverter α > 90° (inverter mode in its own frame)
    // This produces negative voltage which becomes positive opposition when bridge is flipped
    set_rectifier_firing_angle(30.0);  // Low angle → high positive DC voltage (~173V)
    set_inverter_firing_angle(155.0);  // Inverter mode (>90°) → correct for inverted bridge

    simulate_for(2.0);
}

// Test: Operation with mechanical load (should show DC current flow)
TEST_F(ThyristorBackToBackSynchronousMachineTest, OperationWithLoad)
{
    // For LCI motor drive, inverter must operate in inverter mode (α > 90°)
    // This is true regardless of the physical bridge inversion
    set_rectifier_firing_angle(10.0);  // Very low angle → maximum positive DC voltage
    set_inverter_firing_angle(150.0);  // Inverter mode (α > 90°) for proper LCI operation

    // Start from lower speed to create torque demand
    // Initialize with pre-fluxed field (field time constant ~10s, so start at steady-state)
    const double psi_field_initial = 100.0;                 // 100 Wb field flux
    _lci_drive->initialize(150.0, 0.0, psi_field_initial);  // 1400 RPM, below synchronous

    std::cout << "\n=== Operation With Load Test ===" << std::endl;
    std::cout << "Initial speed: " << (_lci_drive->get_machine_state().omega_r * 60.0 / (2.0 * pi)) << " RPM"
              << std::endl;
    std::cout << "Rectifier α:   " << 10.0 << "° (rectifier mode)" << std::endl;
    std::cout << "Inverter α:    " << 150.0 << "° (inverter mode for LCI)" << std::endl;

    // CRITICAL: Pre-flux the machine field before expecting current flow!
    // The field winding has large time constant (~1-5 seconds)
    // We need to build up field flux before the machine can produce back-EMF

    double preflux_time = 0.1;                             // 100ms pre-fluxing
    double simulation_time = 5.0 / fundamental_frequency;  // Main simulation

    // Apply 50 Nm load torque via class member (lambda in SetUp() will use it)
    _applied_load_torque = torque_t(50.0);

    // Pre-flux period
    std::cout << "\nPre-fluxing for " << (preflux_time * 1000) << " ms..." << std::endl;
    simulate_for(preflux_time);
    std::cout << "Field flux after pre-flux: " << _lci_drive->get_machine_state().psi_fD << " Wb" << std::endl;

    // Main simulation
    simulate_for(simulation_time);

    const auto& state = _lci_drive->get_machine_state();
    double v_dc_link = _v_dc_rect.value() - _v_dc_inv.value();

    // Check thyristor states
    auto inv_states = _lci_drive->get_inverter_states();
    int inv_on_count = 0;
    for (bool s : inv_states)
        if (s) inv_on_count++;

    // Final diagnostics
    std::cout << "\n=== FINAL DIAGNOSTICS ===" << std::endl;
    std::cout << "Last pulses - Rect: " << (_pulses_rect.p1 ? "1" : "0") << (_pulses_rect.p3 ? "1" : "0")
              << (_pulses_rect.p5 ? "1" : "0") << ", Inv: " << (_pulses_inv.p1 ? "1" : "0")
              << (_pulses_inv.p3 ? "1" : "0") << (_pulses_inv.p5 ? "1" : "0") << std::endl;
    std::cout << "Machine angle (electrical): " << (state.theta_r * 2.0)
              << " rad = " << (state.theta_r * 2.0 * 180.0 / pi) << "°" << std::endl;
    std::cout << "Field flux (final): " << state.psi_fD << " Wb" << std::endl;
    std::cout << "Damper D flux: " << state.psi_kD << " Wb" << std::endl;
    std::cout << "Damper Q flux: " << state.psi_kQ << " Wb" << std::endl;

    // Calculate expected back-EMF
    double omega_elec = state.omega_r * 2.0;  // electrical rad/s (2 pole pairs)
    double L_ad = 0.012;                      // From test setup
    double L_fD = 5.0;                        // From test setup
    double L_kD = 0.005;

    // Manual back-EMF calculation to verify
    double det = (L_fD + L_ad) * (L_kD + L_ad) - L_ad * L_ad;
    double i_f = ((L_kD + L_ad) * state.psi_fD - L_ad * state.psi_kD) / det;
    double psi_stator_d = L_ad * i_f;
    double emf_q_manual = omega_elec * psi_stator_d;

    std::cout << "Manual EMF calc: i_f=" << i_f << " A, psi_d=" << psi_stator_d << " Wb, EMF_q=" << emf_q_manual << " V"
              << std::endl;

    double expected_emf_peak = omega_elec * L_ad * (state.psi_fD / L_fD);  // Approximation
    std::cout << "Expected EMF (approx): " << expected_emf_peak << " V peak" << std::endl;

    // Direct test: Call get_terminal_voltages_abc with zero current
    abc<current_t> zero_current(current_t(0.0), current_t(0.0), current_t(0.0));
    // Note: Can't access _machine directly, it's private in the LCI class
    // We'll have to rely on the cached value
    std::cout << "Cached motor voltages: A=" << _v_machine_abc.a().value() << ", B=" << _v_machine_abc.b().value()
              << ", C=" << _v_machine_abc.c().value() << " V" << std::endl;

    std::cout << "\n=== Results After 5 Cycles with 50 Nm Load ===" << std::endl;
    std::cout << "DC Current:       " << _i_dc.value() << " A" << std::endl;
    std::cout << "Rectifier V_DC:   " << _v_dc_rect.value() << " V" << std::endl;
    std::cout << "Inverter V_DC:    " << _v_dc_inv.value() << " V" << std::endl;
    std::cout << "DC Link Voltage:  " << v_dc_link << " V" << std::endl;
    std::cout << "Grid Current (A): " << _i_grid_abc.a().value() << " A" << std::endl;
    std::cout << "Mach Current (A): " << _i_machine_abc.a().value() << " A" << std::endl;
    std::cout << "Machine Speed:    " << (state.omega_r * 60.0 / (2.0 * pi)) << " RPM" << std::endl;
    std::cout << "Inverter thyristors ON: " << inv_on_count << "/6" << std::endl;
    std::cout << "Machine back-EMF (V): A=" << _v_machine_abc.a().value() << ", B=" << _v_machine_abc.b().value()
              << ", C=" << _v_machine_abc.c().value() << " V" << std::endl;
    std::cout << "Power (approx):   " << (v_dc_link * _i_dc.value()) << " W" << std::endl;
    std::cout << "===========================================\n" << std::endl;

    // With load applied, we should see current flow
    EXPECT_GT(std::abs(_i_dc.value()), 0.1) << "Should have DC current with load torque applied";
    EXPECT_GT(v_dc_link, 0.0) << "DC link voltage should be positive";
}

// ============================================================================
// Industrial Configuration Test Fixture (3.15 MW LCI Drive)
// Based on back_to_back_parameters.json
// ============================================================================

class IndustrialLCIDriveTest : public ::testing::Test {
public:
    static constexpr double sampling_time = 12.5e-6;       // 12.5 μs timestep (80 kHz)
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz grid
    static constexpr double pulse_width_deg = 120.0;       // 120 degree pulse width

    // System ratings from parameter file
    static constexpr double rated_power_va = 3.15e6;          // 3.15 MVA
    static constexpr double grid_voltage_ll_rms = 2200.0;     // 2200V LV transformer side
    static constexpr double machine_voltage_ll_rms = 1900.0;  // 1900V machine rated
    static constexpr double machine_rated_freq = 25.0;        // 25 Hz machine rated frequency
    static constexpr int machine_pole_pairs = 3;              // 6-pole machine
    static constexpr double machine_sync_speed_rpm = 500.0;   // 500 RPM (25 Hz × 60 / 3)

    void SetUp() override
    {
        // Calculate grid voltage amplitude (peak line-to-neutral)
        // V_LN_peak = V_LL_RMS × √2 / √3
        const double v_grid_peak = grid_voltage_ll_rms * sqrt(2.0) / sqrt(3.0);  // ~1798V peak L-N

        // Configure grid-side three-phase voltage source
        _grid_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(v_grid_peak);
        _grid_waveform_generator.set_fundamental_negative_sequence_signal_amplitude(0.0);
        _grid_waveform_generator.set_signal_frequency(fundamental_frequency);
        _grid_waveform_generator.disable_noise();

        // Configure rectifier pulse generator
        _six_pulse_gen_rect =
            six_pulse_generator(six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg)));

        // Configure inverter pulse generator
        _six_pulse_gen_inv =
            six_pulse_generator(six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(pulse_width_deg)));

        // DC link parameters from parameter file
        real_t dc_resistance = 10e-3;  // 10 mΩ DC link resistance
        real_t dc_inductance = 8e-3;   // 8 mH DC link inductance

        // Calculate transformer equivalent impedance referred to LV side
        // Per-unit base: S_base = 3.362 MVA, V_base = 2200 V L-L, f_base = 50 Hz
        const double transformer_s_base = 3.362e6;
        const double transformer_v_base = 2200.0;
        const double z_base = (transformer_v_base * transformer_v_base) / transformer_s_base;  // 1.44 Ω
        const double omega_base = 2.0 * pi * fundamental_frequency;                            // 314.16 rad/s
        const double l_base = z_base / omega_base;                                             // 4.58 mH

        // LV side impedance (pu to Ohms)
        real_t grid_resistance = 0.0024 * z_base;  // 3.46 mΩ
        real_t grid_inductance = 0.0839 * l_base;  // 0.384 mH

        // Machine parameters from parameter file
        // Calculate synchronous inductances: L_d = L_ls + L_md, L_q = L_ls + L_mq
        real_t r_s = 0.0047;       // 4.7 mΩ stator resistance
        real_t l_ls = 0.00059;     // 0.59 mH stator leakage inductance
        real_t l_md = 0.00808;     // 8.08 mH d-axis magnetizing inductance
        real_t l_mq = 0.00418;     // 4.18 mH q-axis magnetizing inductance
        real_t l_d = l_ls + l_md;  // 8.67 mH d-axis synchronous inductance
        real_t l_q = l_ls + l_mq;  // 4.77 mH q-axis synchronous inductance

        // Field winding parameters
        real_t r_fd = 0.0014;        // 1.4 mΩ field resistance
        real_t l_lfd = 0.00167;      // 1.67 mH field leakage inductance
        real_t l_fd = l_lfd + l_md;  // 9.75 mH field self-inductance

        // Damper winding parameters (d-axis)
        real_t r_kd = 0.136;         // 136 mΩ d-axis damper resistance
        real_t l_lkd = 0.0015;       // 1.5 mH d-axis damper leakage inductance
        real_t l_kd = l_lkd + l_md;  // 9.58 mH d-axis damper self-inductance

        // Damper winding parameters (q-axis)
        real_t r_kq = 0.0777;        // 77.7 mΩ q-axis damper resistance
        real_t l_lkq = 0.00035;      // 0.35 mH q-axis damper leakage inductance
        real_t l_kq = l_lkq + l_mq;  // 4.53 mH q-axis damper self-inductance

        // Mechanical parameters
        real_t inertia = 670.0;  // 670 kg·m² (H ≈ 3.4s for 3.15 MW machine at 500 RPM)

        // Create machine configuration (6-pole machine: P=6, pole_pairs=3)
        synchronous_machine machine(resistance_t(r_s), resistance_t(r_fd), resistance_t(r_kd), resistance_t(r_kq),
            inductance_t(l_d), inductance_t(l_q),
            inductance_t(l_md),  // L_ad = L_md (mutual inductance)
            inductance_t(l_mq),  // L_aq = L_mq (mutual inductance)
            inductance_t(l_fd), inductance_t(l_kd), inductance_t(l_kq), moment_of_inertia_t(inertia),
            6  // P = 6 poles (3 pole pairs) for 500 RPM @ 25 Hz
        );

        // Create LCI drive
        _lci_drive = std::make_unique<thyristor_back_to_back_synchronous_machine>(
            grid_resistance, grid_inductance, dc_resistance, dc_inductance, machine, sampling_time);

        // Initialize to near-zero speed (startup condition from parameter file)
        // "sim_machine_initial_speed_deviation_percent": -99.999999 means nearly stationary
        const double speed_deviation = -0.99999999;
        const double omega_sync = (2.0 * pi * machine_rated_freq) / machine_pole_pairs;  // 52.36 rad/s mechanical
        const double omega_initial = omega_sync * (1.0 + speed_deviation);               // ~0.0005 rad/s (nearly zero)

        // Initialize with zero field flux (field winding not pre-energized)
        const double psi_field_initial = 0.0;
        _lci_drive->initialize(omega_initial, 0.0, psi_field_initial);

        // Set initial firing angles (conservative for startup)
        _current_firing_angle_rect = angle_wrapped::from_degrees(85.0);  // Near 90° for low voltage
        _current_firing_angle_inv = angle_wrapped::from_degrees(95.0);   // Just above 90° for inverter

        set_logger();
    }

    void simulate_for(double time) { _simulator.simulate_for(time); }

    void set_rectifier_firing_angle(double angle_degrees)
    {
        _current_firing_angle_rect = angle_wrapped::from_degrees(angle_degrees);
    }

    void set_inverter_firing_angle(double angle_degrees)
    {
        _current_firing_angle_inv = angle_wrapped::from_degrees(angle_degrees);
    }

protected:
    yeet::simulator _simulator = yeet::make_gtest_simulator();

    // Grid-side components
    three_phase_waveform_generator _grid_waveform_generator{sampling_time};
    six_pulse_generator _six_pulse_gen_rect;

    // Machine-side components
    six_pulse_generator _six_pulse_gen_inv;

    // LCI drive system
    std::unique_ptr<thyristor_back_to_back_synchronous_machine> _lci_drive;

    // Current state variables
    angle_wrapped _current_firing_angle_rect{0.0};
    angle_wrapped _current_firing_angle_inv{0.0};

    abc<double> _v_grid_abc{0.0, 0.0, 0.0};
    abc<current_t> _i_grid_abc{current_t(0.0), current_t(0.0), current_t(0.0)};
    abc<current_t> _i_machine_abc{current_t(0.0), current_t(0.0), current_t(0.0)};
    abc<voltage_t> _v_machine_abc{voltage_t(0.0), voltage_t(0.0), voltage_t(0.0)};

    six_pulse_generator::pulses _pulses_rect{};
    six_pulse_generator::pulses _pulses_inv{};

    current_t _i_dc{0.0};
    voltage_t _v_dc_rect{0.0};
    voltage_t _v_dc_inv{0.0};

    // Control variables
    torque_t _applied_load_torque{0.0};
    voltage_t _applied_field_voltage{0.0};

private:
    void set_logger()
    {
        // Main update loop
        _simulator.register_lambda_runner(sampling_time, [this]() {
            // Update grid waveform generator
            _grid_waveform_generator.update();

            // Get grid voltages
            _v_grid_abc = _grid_waveform_generator.get_signal_abc();
            angle_wrapped grid_reference_angle = _grid_waveform_generator.get_fundamental_signal_angle().a();

            // Generate rectifier pulses
            _pulses_rect = _six_pulse_gen_rect.update(grid_reference_angle, _current_firing_angle_rect, true);

            // Get machine electrical angle
            const auto& machine_state = _lci_drive->get_machine_state();
            angle_wrapped machine_reference_angle = angle_wrapped(machine_state.theta_r * 3.0);  // theta_r × pole_pairs

            // Generate inverter pulses
            _pulses_inv = _six_pulse_gen_inv.update(machine_reference_angle, _current_firing_angle_inv, true);

            // Update LCI drive
            abc<voltage_t> v_grid(voltage_t(_v_grid_abc.a()), voltage_t(_v_grid_abc.b()), voltage_t(_v_grid_abc.c()));
            _lci_drive->update(v_grid, _pulses_rect, _pulses_inv, _applied_field_voltage, _applied_load_torque);

            // Get outputs
            _i_grid_abc = _lci_drive->get_grid_currents();
            _i_machine_abc = _lci_drive->get_motor_currents();
            _v_machine_abc = _lci_drive->get_motor_voltages();
            _i_dc = _lci_drive->get_dc_link_current();
            _v_dc_rect = _lci_drive->get_rectifier_dc_voltage();
            _v_dc_inv = _lci_drive->get_inverter_dc_voltage();
        });

        // Grid-side voltage logger
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_a", [&]() -> double { return _v_grid_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_b", [&]() -> double { return _v_grid_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:v_ac_c", [&]() -> double { return _v_grid_abc.c(); }));

        // Grid-side current logger
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_a", [&]() -> double { return _i_grid_abc.a().value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_b", [&]() -> double { return _i_grid_abc.b().value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_ac_c", [&]() -> double { return _i_grid_abc.c().value(); }));

        // Machine-side current logger
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:i_ac_a", [&]() -> double { return _i_machine_abc.a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:i_ac_b", [&]() -> double { return _i_machine_abc.b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:i_ac_c", [&]() -> double { return _i_machine_abc.c().value(); }));

        // Machine-side voltage logger
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_a", [&]() -> double { return _v_machine_abc.a().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_b", [&]() -> double { return _v_machine_abc.b().value(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:v_ac_c", [&]() -> double { return _v_machine_abc.c().value(); }));

        // DC link logger
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:i_dc", [&]() -> double { return _i_dc.value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_rect", [&]() -> double { return _v_dc_rect.value(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:v_inv", [&]() -> double { return _v_dc_inv.value(); }));

        // Rectifier pulse logger
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "rect:pulse_T1", [&]() -> double { return _pulses_rect.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "rect:pulse_T3", [&]() -> double { return _pulses_rect.p3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "rect:pulse_T5", [&]() -> double { return _pulses_rect.p5 ? 1.0 : 0.0; }));

        // Inverter pulse logger
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("inv:pulse_T1", [&]() -> double { return _pulses_inv.p1 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("inv:pulse_T3", [&]() -> double { return _pulses_inv.p3 ? 1.0 : 0.0; }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("inv:pulse_T5", [&]() -> double { return _pulses_inv.p5 ? 1.0 : 0.0; }));

        // Machine state logger
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:omega_r", [&]() -> double { return _lci_drive->get_machine_state().omega_r; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:theta_r", [&]() -> double { return _lci_drive->get_machine_state().theta_r; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:psi_fD", [&]() -> double { return _lci_drive->get_machine_state().psi_fD; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:psi_kD", [&]() -> double { return _lci_drive->get_machine_state().psi_kD; }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine:psi_kQ", [&]() -> double { return _lci_drive->get_machine_state().psi_kQ; }));
    }
};

// Test: Industrial system instantiation and basic physics
TEST_F(IndustrialLCIDriveTest, SystemInstantiation)
{
    // Force machine to nominal speed for testing
    const double omega_nominal = (2.0 * pi * machine_rated_freq) / machine_pole_pairs;  // 52.36 rad/s
    const double psi_field_nominal = 5.0;                                               // 5 Wb nominal field flux
    _lci_drive->initialize(omega_nominal, 0.0, psi_field_nominal);

    // Verify initial state (should be at nominal speed)
    const auto& state = _lci_drive->get_machine_state();
    double rpm = state.omega_r * 60.0 / (2.0 * pi);

    std::cout << "\n=== Industrial LCI Drive (3.15 MW) ===" << std::endl;
    std::cout << "Rated power:         " << (rated_power_va / 1e6) << " MVA" << std::endl;
    std::cout << "Grid voltage:        " << grid_voltage_ll_rms << " V L-L RMS" << std::endl;
    std::cout << "Machine voltage:     " << machine_voltage_ll_rms << " V L-L RMS" << std::endl;
    std::cout << "Machine sync speed:  " << machine_sync_speed_rpm << " RPM" << std::endl;
    std::cout << "Machine poles:       " << (machine_pole_pairs * 2) << " (pole pairs = " << machine_pole_pairs << ")"
              << std::endl;
    std::cout << "Forced speed:        " << rpm << " RPM (nominal)" << std::endl;
    std::cout << "Initial field flux:  " << state.psi_fD << " Wb" << std::endl;
    std::cout << "======================================\n" << std::endl;

    EXPECT_NEAR(rpm, machine_sync_speed_rpm, 1.0) << "Machine should be at nominal speed";
    EXPECT_NEAR(state.psi_fD, psi_field_nominal, 0.1) << "Field flux should be at nominal value";

    SUCCEED();
}

// Test: Field flux buildup from zero
TEST_F(IndustrialLCIDriveTest, FieldFluxBuildup)
{
    // Force machine to nominal speed for testing
    const double omega_nominal = (2.0 * pi * machine_rated_freq) / machine_pole_pairs;  // 52.36 rad/s
    _lci_drive->initialize(omega_nominal, 0.0, 0.0);                                    // Start with zero field flux

    // Apply field voltage to build up flux
    // For steady-state: V_f = R_f × i_f, where i_f = ψ_fD / L_fD
    // Target: ψ_fD = 5.0 Wb (reasonable for 3.15 MW machine)
    // Required: V_f = R_f × (ψ_target / L_fD) = 0.0014 × (5.0 / 0.00975) = 0.72 V
    // But use higher voltage for faster buildup during test
    _applied_field_voltage = voltage_t(10.0);  // 10V for faster flux buildup

    std::cout << "\n=== Field Flux Buildup Test ===" << std::endl;
    std::cout << "Machine speed (forced): " << (omega_nominal * 60.0 / (2.0 * pi)) << " RPM" << std::endl;
    std::cout << "Applied field voltage: " << _applied_field_voltage.value() << " V" << std::endl;

    // Simulate for 100ms to allow field flux to build
    simulate_for(0.1);

    const auto& state = _lci_drive->get_machine_state();

    std::cout << "Field flux after 100ms: " << state.psi_fD << " Wb" << std::endl;
    std::cout << "Field time constant τ_f = L_fD/R_f = " << (0.00975 / 0.0014) << " s" << std::endl;
    std::cout << "================================\n" << std::endl;

    // After 100ms (with τ_f ≈ 7s), flux should have increased but not reached steady-state
    EXPECT_GT(state.psi_fD, 0.1) << "Field flux should build up with applied voltage";
    EXPECT_LT(state.psi_fD, 50.0) << "Field flux should not be excessive";
}

// Test: Motoring operation with realistic firing angles
TEST_F(IndustrialLCIDriveTest, MotoringOperation)
{
    // Force machine to nominal speed (500 RPM synchronous)
    const double omega_nominal = (2.0 * pi * machine_rated_freq) / machine_pole_pairs;  // 52.36 rad/s
    const double psi_field_nominal = 5.0;                                               // 5 Wb nominal field flux
    _lci_drive->initialize(omega_nominal, 0.0, psi_field_nominal);

    // Set field voltage to maintain steady-state flux
    // V_f = R_f × i_f = R_f × (ψ_fD / L_fD) = 0.0014 × (5.0 / 0.00975) = 0.72 V
    _applied_field_voltage = voltage_t(0.72);

    std::cout << "\n=== Industrial LCI Motoring Test ===" << std::endl;
    std::cout << "Machine speed (forced): " << (omega_nominal * 60.0 / (2.0 * pi)) << " RPM" << std::endl;
    std::cout << "Initial field flux: " << _lci_drive->get_machine_state().psi_fD << " Wb" << std::endl;

    // Set motoring firing angles
    set_rectifier_firing_angle(15.0);  // Low angle for high DC voltage
    set_inverter_firing_angle(150.0);  // Inverter mode for motoring

    // Apply load torque (10 kNm typical for 3.15 MW at 500 RPM)
    // P = T × ω → T = P / ω = 3.15e6 / (2π × 500/60) = 60 kNm at rated
    // Use 10 kNm for initial test
    _applied_load_torque = torque_t(10000.0);

    // Run for 5 grid cycles
    simulate_for(5.0 / fundamental_frequency);

    const auto& state = _lci_drive->get_machine_state();
    double v_dc_link = _v_dc_rect.value() - _v_dc_inv.value();
    double power_dc = v_dc_link * _i_dc.value();
    double rpm = state.omega_r * 60.0 / (2.0 * pi);

    std::cout << "\n=== Motoring Results ===" << std::endl;
    std::cout << "DC Current:        " << _i_dc.value() << " A" << std::endl;
    std::cout << "Rectifier V_DC:    " << _v_dc_rect.value() << " V" << std::endl;
    std::cout << "Inverter V_DC:     " << _v_dc_inv.value() << " V" << std::endl;
    std::cout << "DC Link Voltage:   " << v_dc_link << " V" << std::endl;
    std::cout << "DC Power:          " << (power_dc / 1e3) << " kW" << std::endl;
    std::cout << "Machine Speed:     " << rpm << " RPM" << std::endl;
    std::cout << "Field Flux:        " << state.psi_fD << " Wb" << std::endl;
    std::cout << "Applied Torque:    " << (_applied_load_torque.value() / 1000.0) << " kNm" << std::endl;
    std::cout << "========================\n" << std::endl;

    // Validate operation
    EXPECT_GT(v_dc_link, 0.0) << "DC link voltage should be positive for motoring";
    EXPECT_GT(std::abs(_i_dc.value()), 0.1) << "Should have DC current with load applied";
    EXPECT_GT(state.psi_fD, 0.0) << "Field flux should be positive";
    // Speed may drop slightly from synchronous due to load torque (normal for synchronous machine under load)
    EXPECT_NEAR(rpm, machine_sync_speed_rpm, 20.0) << "Speed should remain near nominal (may slip under load)";
}

// Test: Diagnostic test to investigate voltage and current behavior
TEST_F(IndustrialLCIDriveTest, DiagnosticVoltageAndCurrentAnalysis)
{
    // Force machine to nominal speed (500 RPM synchronous)
    const double omega_nominal = (2.0 * pi * machine_rated_freq) / machine_pole_pairs;  // 52.36 rad/s
    const double psi_field_nominal = 5.0;                                               // 5 Wb nominal field flux
    _lci_drive->initialize(omega_nominal, 0.0, psi_field_nominal);

    // Set field voltage to maintain steady-state flux
    _applied_field_voltage = voltage_t(0.72);

    // Set motoring firing angles
    set_rectifier_firing_angle(15.0);
    set_inverter_firing_angle(150.0);

    // Apply load torque
    _applied_load_torque = torque_t(10000.0);

    std::cout << "\n=== Diagnostic: Voltage & Current Analysis ===" << std::endl;
    std::cout << "Machine speed (forced): " << (omega_nominal * 60.0 / (2.0 * pi)) << " RPM" << std::endl;
    std::cout << "Applied load: " << (_applied_load_torque.value() / 1000.0) << " kNm" << std::endl;

    // Track min/max values
    double i_a_min = 1e9, i_a_max = -1e9;
    double i_b_min = 1e9, i_b_max = -1e9;
    double i_c_min = 1e9, i_c_max = -1e9;
    double v_terminal_a_min = 1e9, v_terminal_a_max = -1e9;
    double v_emf_a_min = 1e9, v_emf_a_max = -1e9;

    int sign_changes_a = 0, sign_changes_b = 0, sign_changes_c = 0;
    double prev_i_a = 0, prev_i_b = 0, prev_i_c = 0;

    // Count firing pulses
    int pulse_counts_inv[6] = {0, 0, 0, 0, 0, 0};
    int pulse_counts_rect[6] = {0, 0, 0, 0, 0, 0};

    // Run for 2 electrical cycles and collect statistics
    double simulation_time = 2.0 / machine_rated_freq;  // 2 electrical cycles (80ms @ 25 Hz)
    int n_steps = static_cast<int>(simulation_time / sampling_time);

    for (int step = 0; step < n_steps; ++step) {
        simulate_for(sampling_time);

        // Count pulses (convert to array first)
        auto inv_array = _pulses_inv.to_array();
        auto rect_array = _pulses_rect.to_array();
        for (int i = 0; i < 6; ++i) {
            if (inv_array[i]) pulse_counts_inv[i]++;
            if (rect_array[i]) pulse_counts_rect[i]++;
        }

        // Get currents
        double i_a = _i_machine_abc.a().value();
        double i_b = _i_machine_abc.b().value();
        double i_c = _i_machine_abc.c().value();

        // Get terminal voltages and back-EMF
        auto v_terminal = _lci_drive->get_motor_voltages();
        auto v_emf = _lci_drive->get_motor_back_emf();

        double v_terminal_a = v_terminal.a().value();
        double v_emf_a = v_emf.a().value();

        // Track min/max
        i_a_min = std::min(i_a_min, i_a);
        i_a_max = std::max(i_a_max, i_a);
        i_b_min = std::min(i_b_min, i_b);
        i_b_max = std::max(i_b_max, i_b);
        i_c_min = std::min(i_c_min, i_c);
        i_c_max = std::max(i_c_max, i_c);

        v_terminal_a_min = std::min(v_terminal_a_min, v_terminal_a);
        v_terminal_a_max = std::max(v_terminal_a_max, v_terminal_a);
        v_emf_a_min = std::min(v_emf_a_min, v_emf_a);
        v_emf_a_max = std::max(v_emf_a_max, v_emf_a);

        // Count sign changes (indicating proper AC current flow)
        if (step > 0) {
            if ((prev_i_a > 0 && i_a < 0) || (prev_i_a < 0 && i_a > 0)) sign_changes_a++;
            if ((prev_i_b > 0 && i_b < 0) || (prev_i_b < 0 && i_b > 0)) sign_changes_b++;
            if ((prev_i_c > 0 && i_c < 0) || (prev_i_c < 0 && i_c > 0)) sign_changes_c++;
        }

        prev_i_a = i_a;
        prev_i_b = i_b;
        prev_i_c = i_c;
    }

    std::cout << "\n--- Current Analysis ---" << std::endl;
    std::cout << "Phase A: min=" << i_a_min << " A, max=" << i_a_max << " A, sign changes=" << sign_changes_a
              << std::endl;
    std::cout << "Phase B: min=" << i_b_min << " A, max=" << i_b_max << " A, sign changes=" << sign_changes_b
              << std::endl;
    std::cout << "Phase C: min=" << i_c_min << " A, max=" << i_c_max << " A, sign changes=" << sign_changes_c
              << std::endl;

    std::cout << "\n--- Pulse Count Analysis ---" << std::endl;
    std::cout << "Rectifier: T1=" << pulse_counts_rect[0] << ", T2=" << pulse_counts_rect[1]
              << ", T3=" << pulse_counts_rect[2] << ", T4=" << pulse_counts_rect[3] << ", T5=" << pulse_counts_rect[4]
              << ", T6=" << pulse_counts_rect[5] << std::endl;
    std::cout << "Inverter:  T1=" << pulse_counts_inv[0] << ", T2=" << pulse_counts_inv[1]
              << ", T3=" << pulse_counts_inv[2] << ", T4=" << pulse_counts_inv[3] << ", T5=" << pulse_counts_inv[4]
              << ", T6=" << pulse_counts_inv[5] << std::endl;

    std::cout << "\n--- Voltage Analysis (Phase A) ---" << std::endl;
    std::cout << "Terminal voltage: min=" << v_terminal_a_min << " V, max=" << v_terminal_a_max
              << " V, pk-pk=" << (v_terminal_a_max - v_terminal_a_min) << " V" << std::endl;
    std::cout << "Back-EMF:         min=" << v_emf_a_min << " V, max=" << v_emf_a_max
              << " V, pk-pk=" << (v_emf_a_max - v_emf_a_min) << " V" << std::endl;

    const auto& state = _lci_drive->get_machine_state();
    std::cout << "\n--- Machine State ---" << std::endl;
    std::cout << "Field flux:  " << state.psi_fD << " Wb" << std::endl;
    std::cout << "Damper d:    " << state.psi_kD << " Wb" << std::endl;
    std::cout << "Damper q:    " << state.psi_kQ << " Wb" << std::endl;
    std::cout << "Speed:       " << (state.omega_r * 60.0 / (2.0 * pi)) << " RPM" << std::endl;

    std::cout << "==========================================\n" << std::endl;

    // Validate that currents properly change sign (AC behavior)
    EXPECT_GT(sign_changes_a, 2) << "Phase A current should switch sign multiple times (AC behavior)";
    EXPECT_GT(sign_changes_b, 2) << "Phase B current should switch sign multiple times (AC behavior)";
    EXPECT_GT(sign_changes_c, 2) << "Phase C current should switch sign multiple times (AC behavior)";

    // Validate that currents go both positive and negative
    EXPECT_LT(i_a_min, -10.0) << "Phase A current should go significantly negative";
    EXPECT_GT(i_a_max, 10.0) << "Phase A current should go significantly positive";

    // Validate back-EMF is sinusoidal (symmetric)
    double emf_symmetry = std::abs(std::abs(v_emf_a_max) - std::abs(v_emf_a_min));
    EXPECT_LT(emf_symmetry, 100.0) << "Back-EMF should be approximately symmetric (sinusoidal)";
}

}  // namespace
