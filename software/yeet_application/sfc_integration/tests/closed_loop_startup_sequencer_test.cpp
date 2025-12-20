/**
 * @file closed_loop_startup_sequencer_test.cpp
 * @brief Integration tests for startup sequencer in closed-loop SFC system
 *
 * Tests for startup sequencer state transitions, observer preset during virtual startup,
 * and auto-start coordination with state machine.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet/signal_wiring.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class ClosedLoopStartupSequencerTest : public ::testing::Test {
public:
    struct mock_storage {
        bool is_current_extinguished{false};
    } _storage;

    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(ms(1.0f));

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Wire mock inputs to storage
        yeet::wire(system.grid_side_control.state_machine)
            .mocked(_storage.is_current_extinguished)
            .to(&sfc_state_machine_datamodel::inputs::is_current_extinguished);

        // Configure topology to use internal model
        system.plant.get_parameters().topology.write(sfc_plant_model_datamodel::topology_type::simulink);

        // Configure plant for gas turbine soft-starter application
        auto gas_turbine_config = yeet::application::sfc::integration::test::create_gas_turbine_config();
        yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, gas_turbine_config);

        // Initialize the system
        initialize();

        // Register loggers for debugging
        register_minimal_logger(simulator, system.plant);
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        register_complete_logger(simulator, system.machine_side_control.startup_sequencer);
        register_complete_logger(simulator, system.machine_side_control.machine_side_observer);
    }

    void initialize()
    {
        set_turn_on_command(false);
        set_start_command(false);
        set_stop_command(false);
        set_current_extinguished(false);

        system.initialize();
    }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // State machine control helpers
    // turn_on_command: Energizes system (IDLE → AUXILIARIES_ON → BREAKERS_ON via IO mock)
    // start_command: Starts operation (BREAKERS_ON → OPERATION_ON)
    void set_turn_on_command(bool on)
    {
        system.grid_side_control.state_machine.get_parameters().turn_on_command.write(on);
    }

    void set_start_command(bool start) { system.grid_io_mock.get_parameters().start_command.write(start); }

    void set_stop_command(bool stop) { system.grid_io_mock.get_parameters().stop_command.write(stop); }

    // Direct IO control helpers for step-by-step testing of state transitions
    // These use overrides for granular control in startup sequencer tests
    void set_auxiliaries_on(bool on)
    {
        system.grid_io_mock.get_parameters().override_auxiliaries_are_on.write(true);
        system.grid_io_mock.get_parameters().override_auxiliaries_are_on_value.write(on);
    }

    void set_breakers_on(bool on)
    {
        system.grid_io_mock.get_parameters().override_mv_breaker_is_closed.write(true);
        system.grid_io_mock.get_parameters().override_mv_breaker_is_closed_value.write(on);
    }

    void set_current_extinguished(bool extinguished) { _storage.is_current_extinguished = extinguished; }

    // Complete startup sequence following state machine logic
    // Uses turn_on_command to trigger IO mock → auxiliaries → breakers flow
    void execute_startup_sequence()
    {
        set_turn_on_command(true);
        simulate_for(0.3);  // Allow IO mock to process auxiliaries and breakers with delays
        set_start_command(true);
        simulate_for(0.1);
    }

    // System under test
    yeet::application::sfc::integration::plant_complete_sfc_closed_loop_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

/**
 * @test StartupSequencerIntegrationTest
 * @brief Validates startup sequencer coordinated startup to 1.0 pu speed
 *
 * This test validates the integration of the startup sequencer with the complete
 * control system. It tests the complete startup procedure including:
 * - Flux buildup phase (field excitation establishment)
 * - Virtual reference frame startup procedure (V/Hz control)
 * - Transition to closed-loop observer-based control
 * - Speed reference tracking to 1.0 pu
 */
TEST_F(ClosedLoopStartupSequencerTest, StartupSequencerIntegrationTest)
{
    // Configure Startup Sequencer
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{10.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.3};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Configure rate limiter: 20 seconds per per-unit = 0.05 pu/s
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{20.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Set speed reference BEFORE startup to prevent dip when transitioning to closed-loop
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{1.0});

    // Speed tracking checker for steady-state validation
    const speed_pu_t speed_error_tolerance{0.10f};
    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed tracking error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_error_tolerance, speed_error_tolerance, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();

    // Monotonicity checker: speed reference should never decrease after closed-loop transition
    speed_pu_t previous_speed_reference{0.0};
    bool monotonicity_check_active = false;
    auto speed_ref_monotonicity_checker = simulator.register_checker(yeet::lambda_checker(
        "speed reference monotonicity",
        [&]() -> bool {
            if (!monotonicity_check_active) return true;

            auto current_speed_ref = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
            bool is_monotonic = current_speed_ref.value() >= previous_speed_reference.value() - 0.001f;
            previous_speed_reference = current_speed_ref;
            return is_monotonic;
        },
        [&]() -> std::string {
            return "Speed reference decreased from " + std::to_string(previous_speed_reference.value()) + " to " +
                   std::to_string(
                       system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read().value());
        }));

    // Execute State Machine Startup
    execute_startup_sequence();

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "State machine should enable pulse releases after startup sequence";

    bool success = true;

    // Trigger Startup Sequencer
    success &= simulator.simulate_for(1.0);  // Pre-flux duration

    // Configure moderate load torque
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.15f));

    // Wait for Flux Buildup and Virtual Startup
    success &= simulator.simulate_for(15.0);

    // Verify startup sequencer completed its sequence
    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup sequencer should have transitioned to CLOSED_LOOP_OPERATION";

    // Verify reference frame switched to estimated (observer-based)
    auto ref_frame = system.machine_side_control.startup_sequencer.get_outputs().reference_frame_source.read();
    EXPECT_EQ(ref_frame, reference_frame_source_t::ESTIMATED)
        << "Reference frame should be ESTIMATED during closed-loop operation";

    // Verify torque control mode is back to speed control
    auto torque_mode = system.machine_side_control.startup_sequencer.get_outputs().torque_control_mode.read();
    EXPECT_EQ(torque_mode, sfc_torque_controller::control_mode::SPEED_CONTROL)
        << "Torque controller should be in SPEED_CONTROL mode";

    // Activate Monotonicity Checker
    previous_speed_reference = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    monotonicity_check_active = true;

    // Get the current speed reference as baseline
    auto baseline_speed_ref = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    const real_t virtual_frequency_final = 0.1f;
    EXPECT_GE(baseline_speed_ref.value(), virtual_frequency_final)
        << "Speed reference should be at least virtual_frequency_final after transition";
    EXPECT_LT(baseline_speed_ref.value(), 1.0f) << "Speed reference should not have reached target yet";

    // Calculate expected speed reference values based on rate limiting from baseline
    const real_t rate_pu_per_s = 0.05f;  // 20 s/pu = 0.05 pu/s
    const real_t expected_ref_5s = baseline_speed_ref.value() + rate_pu_per_s * 5.0f;
    const real_t expected_ref_10s = baseline_speed_ref.value() + rate_pu_per_s * 10.0f;

    // Simulate for 5 seconds
    success &= simulator.simulate_for(5.0);
    auto speed_ref_5s = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    EXPECT_NEAR(speed_ref_5s.value(), expected_ref_5s, 0.05f)
        << "Speed reference should ramp at 0.05 pu/s from baseline after closed-loop transition";

    // Simulate for 5 more seconds (10s total after closed-loop)
    success &= simulator.simulate_for(5.0);
    auto speed_ref_10s = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    EXPECT_NEAR(speed_ref_10s.value(), expected_ref_10s, 0.05f)
        << "Speed reference should continue ramping at 0.05 pu/s";

    // Simulate until reference reaches 1.0 pu
    success &= simulator.simulate_for(10.0);
    auto speed_ref_final = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    EXPECT_NEAR(speed_ref_final.value(), 1.0f, 0.03f) << "Speed reference should reach 1.0 pu with 20 s/pu rate limit";

    // Verify Speed Tracking
    auto final_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(final_speed.value(), 1.0f, 0.15f)
        << "Machine should track the rate-limited speed reference to approximately 1.0 pu";

    // Steady-State Validation
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_tracking_checker->deactivate();

    // Final Validation
    EXPECT_TRUE(success) << "Startup sequencer integration test completed with simulation errors";

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "System should maintain pulse releases during steady-state operation";

    auto startup_complete = system.machine_side_control.startup_sequencer.get_outputs().is_startup_complete.read();
    EXPECT_TRUE(startup_complete) << "Startup sequencer should indicate startup complete";

    success &= simulator.simulate_for(10.0);
}

/**
 * @test ObserverPresetDuringVirtualStartupTest
 * @brief Validates bumpless handover using supervisor-driven speed PI initialization
 *
 * New strategy:
 * - Observer runs in the background during virtual startup, but is not preset/held for switchover.
 * - Integrator supervisor issues a one-shot preset_request when the speed controller is enabled.
 * - Speed controller presets its PI integrator using the last applied torque, minimizing torque steps.
 */
TEST_F(ClosedLoopStartupSequencerTest, ObserverPresetDuringVirtualStartupTest)
{
    // Configure Startup Sequencer
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.05};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{10.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.2};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Set speed reference before startup.
    // Keep it equal to the final virtual frequency to avoid an operating-point step at handover.
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.05});

    bool preset_observer_seen = false;

    bool supervisor_preset_seen = false;
    startup_sequencer_state_t state_at_supervisor_preset = startup_sequencer_state_t::IDLE;

    bool handover_seen = false;
    double handover_time_s = 0.0;
    reference_frame_source_t previous_ref_source = reference_frame_source_t::VIRTUAL;

    // Torque continuity check around handover
    std::vector<float> torque_samples;
    std::vector<double> time_samples;

    // Execute State Machine Startup
    execute_startup_sequence();

    bool success = true;

    success &= simulator.simulate_for(1.0);  // Pre-flux duration

    // Configure moderate load torque
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    // Manual Simulation Loop to monitor handover and preset pulses
    const double step_size = 0.001;  // 1ms
    const double total_time = 20.0;
    const int total_steps = static_cast<int>(total_time / step_size);

    for (int i = 0; i < total_steps; ++i) {
        const double t = i * step_size;
        simulator.simulate_for(step_size);

        const auto ref_source =
            system.machine_side_control.startup_sequencer.get_outputs().reference_frame_source.read();
        if (!handover_seen && previous_ref_source == reference_frame_source_t::VIRTUAL &&
            ref_source == reference_frame_source_t::ESTIMATED) {
            handover_seen = true;
            handover_time_s = t;
        }
        previous_ref_source = ref_source;

        // Ensure observer preset is not used in the new strategy
        preset_observer_seen |= system.machine_side_control.startup_sequencer.get_outputs().preset_observer.read();

        const bool supervisor_preset =
            system.machine_side_control.machine_integrator_supervisor.get_outputs().preset_request.read();
        if (supervisor_preset && !supervisor_preset_seen) {
            supervisor_preset_seen = true;
            state_at_supervisor_preset =
                system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
        }

        // Sample applied torque reference from torque controller (post limiting)
        torque_samples.push_back(
            system.machine_side_control.torque_controller.get_outputs().torque_reference.read().value());
        time_samples.push_back(t);
    }

    EXPECT_FALSE(preset_observer_seen) << "preset_observer must remain false (new handover strategy)";

    EXPECT_TRUE(handover_seen) << "Reference frame source should switch VIRTUAL -> ESTIMATED";

    EXPECT_TRUE(supervisor_preset_seen) << "Integrator supervisor should assert preset_request at handover";
    EXPECT_EQ(state_at_supervisor_preset, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Supervisor preset should occur when entering CLOSED_LOOP_OPERATION";

    // Validate we reached closed-loop
    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Should be in CLOSED_LOOP_OPERATION after virtual startup";

    auto observer_angle =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_reference_angle.read();
    auto virtual_angle = system.machine_side_control.startup_sequencer.get_outputs().virtual_rotor_angle.read();

    // Torque continuity around handover
    const double window_s = 0.2;  // 200ms

    auto mean_in_window = [&](double t0, double t1) {
        double sum = 0.0;
        int count = 0;
        for (size_t k = 0; k < time_samples.size(); ++k) {
            if (time_samples[k] >= t0 && time_samples[k] < t1) {
                sum += torque_samples[k];
                ++count;
            }
        }
        return (count > 0) ? static_cast<float>(sum / count) : 0.0f;
    };

    const float pre_mean = mean_in_window(handover_time_s - window_s, handover_time_s);
    const float post_mean = mean_in_window(handover_time_s, handover_time_s + window_s);

    float max_abs_dev = 0.0f;
    for (size_t k = 0; k < time_samples.size(); ++k) {
        if (time_samples[k] >= handover_time_s && time_samples[k] < handover_time_s + window_s) {
            max_abs_dev = std::max(max_abs_dev, std::abs(torque_samples[k] - pre_mean));
        }
    }

    EXPECT_NEAR(post_mean, pre_mean, 0.08f) << "Applied torque should be continuous across handover";
    EXPECT_LT(max_abs_dev, 0.20f) << "No large applied torque spike allowed at handover";

    EXPECT_TRUE(success) << "Observer preset test completed with simulation errors";
}

/**
 * @test AutoStartOnOperationOnTest
 * @brief Validates automatic startup when state machine enters OPERATION_ON
 *
 * This test verifies the auto-start coordination feature where the startup
 * sequencer automatically begins the startup sequence when the grid-side
 * state machine reaches OPERATION_ON state.
 */
TEST_F(ClosedLoopStartupSequencerTest, AutoStartOnOperationOnTest)
{
    // Configure Startup Sequencer
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.08};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{4.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.15};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.5};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{8.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Enable auto-start/stop coordination
    system.machine_side_control.startup_sequencer.get_parameters().auto_start_stop_enabled = true;

    // Set desired speed reference early
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.2});

    // Progress State Machine to OPERATION_ON
    bool success = true;

    // Enable turn_on_command to allow state machine transitions
    set_turn_on_command(true);

    set_auxiliaries_on(true);
    success &= simulator.simulate_for(0.1);

    set_breakers_on(true);
    success &= simulator.simulate_for(0.2);

    // Verify startup sequencer still in IDLE
    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE)
        << "Sequencer should remain in IDLE until OPERATION_ON is reached";

    // Send START command to reach OPERATION_ON
    set_start_command(true);
    success &= simulator.simulate_for(0.2);

    // Wait for observer settling time (2.0s) + margin
    success &= simulator.simulate_for(2.5);

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_NE(startup_state, startup_sequencer_state_t::IDLE)
        << "Auto-start should automatically begin startup sequence after observer settling";

    EXPECT_TRUE(startup_state == startup_sequencer_state_t::FLUX_BUILDUP ||
                startup_state == startup_sequencer_state_t::STARTING_PROCEDURE ||
                startup_state == startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Auto-start should progress through normal startup states";

    // Complete Startup Sequence
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.06f));

    success &= simulator.simulate_for(8.0);

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Auto-started sequence should reach CLOSED_LOOP_OPERATION";

    // Verify Normal Operation
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.2});
    success &= simulator.simulate_for(5.0);

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 0.2f, 0.08f) << "Auto-started system should achieve speed setpoint normally";

    EXPECT_TRUE(success) << "Auto-start on OPERATION_ON test completed";
}

/**
 * @test FlyingRestartSkipsVirtualStartupTest
 * @brief Validates flying-restart path skips virtual startup after a normal stop-restart sequence
 *
 * Test scenario:
 * 1. Start machine normally to nominal speed (1.0 pu)
 * 2. Send stop command (machine coasts down)
 * 3. Coast briefly (1 second) to verify sequencer returns to IDLE
 * 4. Send start command
 * 5. Verify flying start: flux builds first, then direct transition to CLOSED_LOOP_OPERATION (skips STARTING_PROCEDURE)
 * 6. Verify supervisor preset_request is asserted for bumpless integrator initialization
 *
 * Note: Start command must be cleared during stop to prevent auto-start from re-engaging.
 * This ensures proper IDLE state and allows the supervisor edge detector to see the
 * rising edge of release_speed_controller on restart.
 */
TEST_F(ClosedLoopStartupSequencerTest, FlyingRestartSkipsVirtualStartupTest)
{
    // Configure Startup Sequencer for normal startup
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{10.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.3};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Configure speed reference ramping
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{20.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Set speed reference to nominal
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{1.0});

    // Configure flying restart parameters
    system.machine_side_control.startup_sequencer.get_parameters().flying_restart_observer_settling_time =
        duration_t{1.0};
    system.machine_side_control.startup_sequencer.get_parameters().flying_restart_speed_threshold = speed_pu_t{0.15};
    system.machine_side_control.startup_sequencer.get_parameters().flying_restart_voltage_threshold = voltage_pu_t{0.1};

    // Enable auto-start/stop coordination
    system.machine_side_control.startup_sequencer.get_parameters().auto_start_stop_enabled = true;

    bool success = true;

    // === PHASE 1: Normal startup to nominal speed ===
    execute_startup_sequence();

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "State machine should enable pulse releases after startup sequence";

    success &= simulator.simulate_for(1.0);  // Pre-flux duration

    // Configure moderate load torque
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.15f));

    // Complete virtual startup and transition to closed-loop
    success &= simulator.simulate_for(15.0);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Should reach CLOSED_LOOP_OPERATION after normal startup";

    // Ramp up to nominal speed
    success &= simulator.simulate_for(25.0);

    auto speed_before_stop = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(speed_before_stop.value(), 0.8f) << "Machine should reach near-nominal speed before stop";

    // === PHASE 2: Stop command ===
    set_start_command(false);  // Remove start command first
    set_stop_command(true);
    success &= simulator.simulate_for(0.5);

    // Signal current extinguished so state machine can complete stop sequence
    set_current_extinguished(true);
    success &= simulator.simulate_for(1.0);

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE) << "Should return to IDLE after stop command";

    set_current_extinguished(false);
    set_stop_command(false);

    // === PHASE 3: Coast for 1 second (machine coasting down but still spinning) ===
    success &= simulator.simulate_for(1.0);
    auto speed_after_coast = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();

    // === PHASE 4: Restart command - flying start should engage after flux is established ===

    // Reset monitoring flags BEFORE issuing start command
    bool entered_flux_buildup = false;
    bool entered_starting_procedure = false;
    bool reached_closed_loop = false;
    bool supervisor_preset_seen = false;
    bool preset_observer_seen = false;
    bool latched_closed_loop_entry = false;
    speed_pu_t closed_loop_entry_speed_reference{0.0f};
    speed_pu_t closed_loop_entry_measured_speed{0.0f};

    set_start_command(true);

    // Monitor state transitions during restart (need time for observer settling + transition)
    const double step_size = 0.001;  // 1ms
    const double total_time = 5.0;   // Observer settling (1.0s) + transition time
    const int total_steps = static_cast<int>(total_time / step_size);

    for (int i = 0; i < total_steps; ++i) {
        simulator.simulate_for(step_size);

        const auto state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
        entered_flux_buildup |= (state == startup_sequencer_state_t::FLUX_BUILDUP);
        entered_starting_procedure |= (state == startup_sequencer_state_t::STARTING_PROCEDURE);
        reached_closed_loop |= (state == startup_sequencer_state_t::CLOSED_LOOP_OPERATION);

        if (!latched_closed_loop_entry && state == startup_sequencer_state_t::CLOSED_LOOP_OPERATION) {
            latched_closed_loop_entry = true;
            closed_loop_entry_speed_reference =
                system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
            closed_loop_entry_measured_speed =
                system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
        }

        preset_observer_seen |= system.machine_side_control.startup_sequencer.get_outputs().preset_observer.read();
        supervisor_preset_seen |=
            system.machine_side_control.machine_integrator_supervisor.get_outputs().preset_request.read();
    }

    // === PHASE 5: Validate flying restart behavior ===

    EXPECT_FALSE(preset_observer_seen) << "preset_observer must remain false during flying start";
    EXPECT_TRUE(reached_closed_loop) << "Flying start should reach CLOSED_LOOP_OPERATION";
    EXPECT_TRUE(entered_flux_buildup) << "Flying start should enter FLUX_BUILDUP to establish flux";
    EXPECT_FALSE(entered_starting_procedure) << "Flying start must skip STARTING_PROCEDURE";
    EXPECT_TRUE(supervisor_preset_seen) << "Supervisor must assert preset_request at handover in flying start";
    EXPECT_TRUE(latched_closed_loop_entry) << "Test must observe first entry into CLOSED_LOOP_OPERATION";

    // Speed controller initialization support: on flying start, speed reference is seeded to measured speed
    EXPECT_NEAR(closed_loop_entry_speed_reference.value(), closed_loop_entry_measured_speed.value(), 0.05f)
        << "Speed reference should be initialized near measured speed at closed-loop entry";

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Should be in CLOSED_LOOP_OPERATION after flying restart";

    const auto ref_frame = system.machine_side_control.startup_sequencer.get_outputs().reference_frame_source.read();
    EXPECT_EQ(ref_frame, reference_frame_source_t::ESTIMATED);

    const auto torque_mode = system.machine_side_control.startup_sequencer.get_outputs().torque_control_mode.read();
    EXPECT_EQ(torque_mode, sfc_torque_controller::control_mode::SPEED_CONTROL);

    // Verify system can operate normally after flying restart
    success &= simulator.simulate_for(5.0);

    auto final_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(final_speed.value(), 0.5f) << "Machine should continue operating after flying restart";

    EXPECT_TRUE(success) << "Flying restart integration test completed";
}
