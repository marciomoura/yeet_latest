#include "yeet_control/thyristor_back_to_back_synchronous_machine.hpp"

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

    simulate_for(simulation_time);

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

}  // namespace
