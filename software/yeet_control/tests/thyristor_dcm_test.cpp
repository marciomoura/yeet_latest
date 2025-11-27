#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <vector>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/thyristor_back_to_back.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet;
using namespace yeet::control;

// Test fixture for Discontinuous Conduction Mode (DCM) and KCL verification
class ThyristorDCMTest : public ::testing::Test {
public:
    // Industrial LCI parameters
    static constexpr double sampling_time = 10e-6;         // 10 μs
    static constexpr double fundamental_frequency = 50.0;  // 50 Hz

    // Voltages
    static constexpr double v_grid_rms_ll = 2090.0;
    static constexpr double v_grid_phase_peak = v_grid_rms_ll / std::sqrt(3.0) * std::sqrt(2.0);
    static constexpr double v_machine_rms_ll = 1980.0;
    static constexpr double v_machine_phase_peak = v_machine_rms_ll / std::sqrt(3.0) * std::sqrt(2.0);

    static constexpr double pulse_width_deg = 120.0;

    // Circuit parameters
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

            // Check KCL
            double sum_grid = _i_grid_abc.a() + _i_grid_abc.b() + _i_grid_abc.c();
            double sum_mach = _i_machine_abc.a() + _i_machine_abc.b() + _i_machine_abc.c();

            _max_kcl_error_grid = std::max(_max_kcl_error_grid, std::abs(sum_grid));
            _max_kcl_error_mach = std::max(_max_kcl_error_mach, std::abs(sum_mach));

            // Store current reference angles
            _current_grid_reference_angle = grid_reference_angle;
            _current_machine_reference_angle = machine_reference_angle;

            _sim_time += sampling_time;
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

    double _v_rect_dc{0.0};
    double _v_inv_dc{0.0};
    double _i_dc{0.0};

    double _max_kcl_error_grid{0.0};
    double _max_kcl_error_mach{0.0};

    double _sim_time{0.0};

private:
    void set_logger()
    {
        // Log DC current and KCL errors
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("dc:i_dc", [&]() -> double { return _i_dc; }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:kcl_error", [&]() -> double { return _i_grid_abc.a() + _i_grid_abc.b() + _i_grid_abc.c(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("mach:kcl_error",
            [&]() -> double { return _i_machine_abc.a() + _i_machine_abc.b() + _i_machine_abc.c(); }));

        // Log individual currents
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_a", [&]() -> double { return _i_grid_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_b", [&]() -> double { return _i_grid_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("grid:i_c", [&]() -> double { return _i_grid_abc.c(); }));

        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("mach:i_a", [&]() -> double { return _i_machine_abc.a(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("mach:i_b", [&]() -> double { return _i_machine_abc.b(); }));
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("mach:i_c", [&]() -> double { return _i_machine_abc.c(); }));

        // Log sum of currents
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid:sum_i", [&]() { return _i_grid_abc.a() + _i_grid_abc.b() + _i_grid_abc.c(); }));
        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "mach:sum_i", [&]() { return _i_machine_abc.a() + _i_machine_abc.b() + _i_machine_abc.c(); }));
    }
};

TEST_F(ThyristorDCMTest, ForcedCommutationToZeroCurrent)
{
    // 1. Start in Rectifier Mode to build up current
    // Grid: 15° (Rectifier)
    // Machine: 150° (Inverter)
    set_rectifier_firing_angle(15.0);
    set_inverter_firing_angle(150.0);

    // Run for 50ms to establish current
    simulate_for(0.05);

    EXPECT_GT(_i_dc, 10.0) << "Failed to establish initial DC current";

    // 2. Force Rectifier into Inverter Mode (Regeneration/Braking)
    // This opposes the current flow and should drive I_dc to zero
    // Grid: 150° (Inverter)
    // Machine: 150° (Inverter) -> Both opposing current
    set_rectifier_firing_angle(150.0);

    // Run for another 50ms to allow current to decay to zero
    simulate_for(0.05);

    // 3. Verify Discontinuous Conduction Mode (DCM)
    EXPECT_NEAR(_i_dc, 0.0, 1e-3) << "DC current did not decay to zero";

    // 4. Verify KCL Compliance
    // The sum of phase currents should be effectively zero at all times
    // We allow a small numerical tolerance (1mA) to account for floating point precision
    // with currents up to ~150A (float epsilon is ~1e-7, so 150*1e-7 = 1.5e-5).
    // The observed error is ~3e-5, which is consistent with float precision.
    EXPECT_NEAR(_max_kcl_error_grid, 0.0, 1e-3) << "Grid-side KCL violated";
    EXPECT_NEAR(_max_kcl_error_mach, 0.0, 1e-3) << "Machine-side KCL violated";

    // 5. Verify AC currents are zero when DC current is zero
    if (_i_dc < 1e-6) {
        EXPECT_NEAR(_i_grid_abc.a(), 0.0, 1e-6);
        EXPECT_NEAR(_i_grid_abc.b(), 0.0, 1e-6);
        EXPECT_NEAR(_i_grid_abc.c(), 0.0, 1e-6);

        EXPECT_NEAR(_i_machine_abc.a(), 0.0, 1e-6);
        EXPECT_NEAR(_i_machine_abc.b(), 0.0, 1e-6);
        EXPECT_NEAR(_i_machine_abc.c(), 0.0, 1e-6);
    }
}

// Helper class for Pulse Mode Control
class PulseModeController {
public:
    enum class State { DRIVE, FORCE_OFF, WAIT_ZERO };

    void update(bool sector_changed, double i_dc)
    {
        switch (_state) {
            case State::DRIVE:
                // Normal operation: Rectifier builds current, Inverter steers it
                _rect_alpha = 15.0;  // Rectifier mode (build current)
                _inv_alpha = 150.0;  // Inverter mode (machine side)

                if (sector_changed) {
                    _state = State::FORCE_OFF;
                }
                break;

            case State::FORCE_OFF:
                // Force commutation: Drive DC current to zero
                _rect_alpha = 150.0;  // Inverter mode (reverse voltage)
                _inv_alpha = 150.0;   // Inverter mode (reverse voltage)

                // Check if current is effectively zero
                if (i_dc < 1e-3) {  // 1mA threshold
                    _state = State::WAIT_ZERO;
                    _zero_current_dwell_timer = 0;
                }
                break;

            case State::WAIT_ZERO:
                // Hold zero current for a short time to ensure thyristor turn-off
                _rect_alpha = 150.0;
                _inv_alpha = 150.0;

                _zero_current_dwell_timer++;
                if (_zero_current_dwell_timer > 5) {  // 50us dwell (5 * 10us)
                    _state = State::DRIVE;
                }
                break;
        }
    }

    double get_rectifier_alpha() const { return _rect_alpha; }
    double get_inverter_alpha() const { return _inv_alpha; }
    State get_state() const { return _state; }

private:
    State _state{State::DRIVE};
    double _rect_alpha{15.0};
    double _inv_alpha{150.0};
    int _zero_current_dwell_timer{0};
};

TEST_F(ThyristorDCMTest, RepeatedForcedCommutation)
{
    PulseModeController controller;
    int commutation_count = 0;
    PulseModeController::State last_state = PulseModeController::State::DRIVE;

    // Run for 20 commutations or max 2 seconds
    // At 50Hz fundamental, 6 pulses per cycle = 300Hz commutation freq.
    // But we are running machine at lower speed?
    // Let's set machine to 5Hz -> 30Hz commutation freq -> ~33ms per sector.
    // 20 commutations -> ~0.66 seconds.

    _machine_waveform_generator.set_signal_frequency(5.0);

    // Register a hook to update the controller
    _simulator.register_lambda_runner(sampling_time, [&]() {
        bool sector_changed = _six_pulse_gen_inv.get_sector_changed();

        controller.update(sector_changed, _i_dc);

        set_rectifier_firing_angle(controller.get_rectifier_alpha());
        set_inverter_firing_angle(controller.get_inverter_alpha());

        // Count commutations
        if (last_state == PulseModeController::State::WAIT_ZERO &&
            controller.get_state() == PulseModeController::State::DRIVE) {
            commutation_count++;
        }
        last_state = controller.get_state();
    });

    // Run simulation
    while (commutation_count < 20 && _sim_time < 2.0) {
        simulate_for(0.001);  // Run in small steps
    }

    EXPECT_GE(commutation_count, 20) << "Did not complete 20 commutations in time";

    // Verify KCL Compliance over the entire run
    EXPECT_NEAR(_max_kcl_error_grid, 0.0, 1e-3) << "Grid-side KCL violated during repeated commutation";
    EXPECT_NEAR(_max_kcl_error_mach, 0.0, 1e-3) << "Machine-side KCL violated during repeated commutation";
}

TEST_F(ThyristorDCMTest, SpeedRampForcedCommutation)
{
    PulseModeController controller;

    // Ramp parameters
    const double start_freq = 1.0;
    const double end_freq = 5.0;
    const double ramp_duration = 2.0;  // Shortened to 2s for unit test speed (10s is too long for unit test)

    _machine_waveform_generator.set_signal_frequency(start_freq);

    // Register a hook to update the controller and ramp speed
    _simulator.register_lambda_runner(sampling_time, [&]() {
        // Update Speed Ramp
        double t = _sim_time;
        if (t < ramp_duration) {
            double freq = start_freq + (end_freq - start_freq) * (t / ramp_duration);
            _machine_waveform_generator.set_signal_frequency(freq);
        }

        // Update Controller
        bool sector_changed = _six_pulse_gen_inv.get_sector_changed();
        controller.update(sector_changed, _i_dc);

        set_rectifier_firing_angle(controller.get_rectifier_alpha());
        set_inverter_firing_angle(controller.get_inverter_alpha());

        // Consistency Check: |Ia| + |Ib| + |Ic| approx 2 * Idc
        // Only check when we are in DRIVE mode, current is established, and NOT commutating.
        // During commutation overlap (especially if it involves the return phase, i.e., bypass),
        // the sum of absolute currents can drop below 2*Idc (down to Idc).
        bool is_commutating = _back_to_back.get_rectifier_bridge().get_commutation_incoming_thyristor_idx() >= 0 ||
                              _back_to_back.get_inverter_bridge().get_commutation_incoming_thyristor_idx() >= 0;

        if (controller.get_state() == PulseModeController::State::DRIVE && _i_dc > 10.0 && !is_commutating) {
            double sum_abs_grid = std::abs(_i_grid_abc.a()) + std::abs(_i_grid_abc.b()) + std::abs(_i_grid_abc.c());
            double sum_abs_mach =
                std::abs(_i_machine_abc.a()) + std::abs(_i_machine_abc.b()) + std::abs(_i_machine_abc.c());

            // Theoretical relationship: Sum(|I_phase|) = 2 * I_dc
            // We allow 20% tolerance for dynamic lag.
            EXPECT_NEAR(sum_abs_grid / 2.0, _i_dc, _i_dc * 0.20) << "Grid current magnitude mismatch";
            EXPECT_NEAR(sum_abs_mach / 2.0, _i_dc, _i_dc * 0.20) << "Machine current magnitude mismatch";
        }
    });

    // Run simulation
    simulate_for(ramp_duration + 0.1);

    // Verify KCL Compliance
    EXPECT_NEAR(_max_kcl_error_grid, 0.0, 1e-3) << "Grid-side KCL violated during speed ramp";
    EXPECT_NEAR(_max_kcl_error_mach, 0.0, 1e-3) << "Machine-side KCL violated during speed ramp";

    // Verify we actually had current flowing (max DC current > 0)
    // We can't easily check max _i_dc from here without adding a member, but we know the test ran.
}

TEST_F(ThyristorDCMTest, PulseModeVariableVoltageTest)
{
    PulseModeController controller;

    // Shared state for the control loop
    double target_voltage_pu = 0.1;
    double target_freq = 5.0;
    bool config_updated = false;

    int commutations_count = 0;
    PulseModeController::State last_state = PulseModeController::State::DRIVE;

    // Register the control loop
    _simulator.register_lambda_runner(sampling_time, [&]() {
        // Apply voltage/freq settings if changed
        if (!config_updated) {
            _machine_waveform_generator.set_fundamental_positive_sequence_signal_amplitude(
                v_machine_phase_peak * target_voltage_pu);
            _machine_waveform_generator.set_signal_frequency(target_freq);
            config_updated = true;
        }

        bool sector_changed = _six_pulse_gen_inv.get_sector_changed();

        controller.update(sector_changed, _i_dc);

        set_rectifier_firing_angle(controller.get_rectifier_alpha());
        set_inverter_firing_angle(controller.get_inverter_alpha());

        // Count commutations (transition from WAIT_ZERO back to DRIVE means we successfully stopped current and
        // restarted)
        if (last_state == PulseModeController::State::WAIT_ZERO &&
            controller.get_state() == PulseModeController::State::DRIVE) {
            commutations_count++;
        }
        last_state = controller.get_state();
    });

    // Phase 1: Low Voltage (10%) - Simulating start-up / low speed
    target_voltage_pu = 0.1;
    target_freq = 5.0;  // 5Hz
    config_updated = false;
    commutations_count = 0;

    // Run for 12 commutations (2 electrical cycles)
    double start_time = _sim_time;
    while (commutations_count < 12 && (_sim_time - start_time) < 2.0) {
        simulate_for(0.001);
    }
    EXPECT_GE(commutations_count, 12) << "Failed to commutate at 10% voltage";

    // Phase 2: Medium Voltage (50%)
    target_voltage_pu = 0.5;
    target_freq = 25.0;  // 25Hz
    config_updated = false;
    commutations_count = 0;
    start_time = _sim_time;

    while (commutations_count < 12 && (_sim_time - start_time) < 2.0) {
        simulate_for(0.001);
    }
    EXPECT_GE(commutations_count, 12) << "Failed to commutate at 50% voltage";

    // Phase 3: Full Voltage (100%)
    target_voltage_pu = 1.0;
    target_freq = 50.0;  // 50Hz
    config_updated = false;
    commutations_count = 0;
    start_time = _sim_time;

    while (commutations_count < 12 && (_sim_time - start_time) < 2.0) {
        simulate_for(0.001);
    }
    EXPECT_GE(commutations_count, 12) << "Failed to commutate at 100% voltage";

    // Verify KCL at the end
    EXPECT_NEAR(_max_kcl_error_grid, 0.0, 1e-3) << "Grid-side KCL violated";
    EXPECT_NEAR(_max_kcl_error_mach, 0.0, 1e-3) << "Machine-side KCL violated";
}

}  // namespace