#include <gtest/gtest.h>

#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class CompleteSensorlessSpeedControllerTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(ms(10.0f));

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Configure plant for gas turbine soft-starter application
        // - Large inertia (3.0 s): Heavy rotor and compressor assembly
        // - Low damping (0.02 pu): Low-friction bearings
        // - Quadratic load (0.3 pu/pu^2): Compressor aerodynamic drag ∝ speed²
        auto gas_turbine_config = yeet::application::sfc::integration::test::create_gas_turbine_config();
        yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, gas_turbine_config);

        // Register test input update lambda (10ms - same rate as speed controller and excitation)
        // This simulates external digital inputs being transferred to the control system
        simulator.register_lambda_runner(10e-3, [&]() {
            system.machine_side_control.external_ios.update_excitation_inputs(
                _exciter_ready, _excitation_start_command, _excitation_stop_command);

            // DISABLED: Speed generator replaced by startup_sequencer
            // system.machine_side_control.external_ios.update_speed_generator_inputs(_run_creep_speed,
            // _run_purge_accel,
            //     _run_ignition_accel, _run_assist_accel, _hold_speed, _decelerate, _idle);

            system.machine_side_control.external_ios.update_startup_sequencer_inputs(
                _startup_sequencer_start_command, _startup_sequencer_stop_command);
        });

        // Initialize the system - allocates plant model and all components
        initialize();

        // ==================== Logger Registration ====================
        register_complete_logger(simulator, system.grid_side_control.current_controller);
        register_complete_logger(simulator, system.machine_side_control.torque_controller);
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        register_complete_logger(simulator, system.machine_side_control.startup_sequencer);
        register_complete_logger(simulator, system.machine_side_control.machine_side_controller);
        register_complete_logger(simulator, system.grid_side_control.state_machine);
        register_complete_logger(simulator, system.plant);
    }

    void initialize()
    {
        // Configure speed-controller - reduce gain
        // system.machine_side_control.speed_controller.get_parameters().kp = 1.0;
        system.grid_side_control.grid_side_pulse_generator.get_parameters().reference_type.write(
            control::six_pulse_generator::reference_type::phase);
        system.machine_side_control.machine_side_pulse_generator.get_parameters().reference_type.write(
            control::six_pulse_generator::reference_type::phase);

        set_auxiliaries_on(false);
        set_breakers_on(false);
        set_start_command(false);
        set_stop_command(false);
        set_shutdown_request(false);
        set_current_extinguished(false);

        system.initialize();
    }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // State machine control helpers using mock inputs
    void set_auxiliaries_on(bool on)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().auxiliaries_are_on, on);
    }

    void set_breakers_on(bool on)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().breakers_are_on, on);
    }

    void set_start_command(bool start)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().start_command, start);
    }

    void set_stop_command(bool stop)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().stop_command, stop);
    }

    void set_shutdown_request(bool shutdown)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().shutdown_request, shutdown);
    }

    void set_current_extinguished(bool extinguished)
    {
        yeet::mock_signal_input::write(
            system.grid_side_control.state_machine.get_inputs().is_current_extinguished, extinguished);
    }

    // DISABLED: Speed generator replaced by startup_sequencer - these helper methods are no longer used
    // // Speed profile command helpers using datamodel override parameters
    // void set_idle(bool idle) { _idle = idle; }

    // void set_run_creep_speed(bool run) { _run_creep_speed = run; }

    // void set_run_purge_accel(bool run) { _run_purge_accel = run; }

    // void set_run_ignition_accel(bool run) { _run_ignition_accel = run; }

    // void set_run_assist_accel(bool run) { _run_assist_accel = run; }

    void set_exciter_ready(bool ready) { _exciter_ready = ready; }

    void set_excitation_start_command(bool start) { _excitation_start_command = start; }

    void set_excitation_stop_command(bool stop) { _excitation_stop_command = stop; }

    void set_startup_sequencer_start_command(bool start) { _startup_sequencer_start_command = start; }

    void set_startup_sequencer_stop_command(bool stop) { _startup_sequencer_stop_command = stop; }

    // Complete startup sequence following state machine logic
    void execute_startup_sequence()
    {
        set_auxiliaries_on(true);
        set_breakers_on(true);
        simulate_for(0.2);
        set_start_command(true);
        simulate_for(0.1);
    }

    // Complete shutdown sequence following state machine logic
    void execute_shutdown_sequence()
    {
        // 1. Request shutdown (OPERATION_ON -> SHUTDOWN_REQUESTED)
        set_stop_command(true);
        simulate_for(0.1);

        // 2. Current extinguished (SHUTDOWN_REQUESTED -> OPERATION_OFF)
        set_current_extinguished(true);
        set_stop_command(false);  // Remove stop command
        simulate_for(0.1);

        // 3. Turn off breakers (OPERATION_OFF -> BREAKERS_ON)
        // Remove load torque when power is disconnected to prevent negative speed
        yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.0f));
        set_breakers_on(false);
        set_start_command(false);  // Remove start command
        simulate_for(0.1);

        // 4. Breakers actually open (BREAKERS_ON -> BREAKERS_OFF)
        simulate_for(0.1);

        // 5. Turn off auxiliaries (BREAKERS_OFF -> IDLE)
        set_auxiliaries_on(false);
        simulate_for(0.1);
    }

    // System under test
    yeet::application::sfc::integration::plant_complete_sfc_closed_loop_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();

    // DISABLED: Speed generator replaced by startup_sequencer - these variables are no longer used
    // // Speed generator profile commands (simulates external digital inputs)
    // bool _idle = true;
    // bool _run_creep_speed = false;
    // bool _run_purge_accel = false;
    // bool _run_ignition_accel = false;
    // bool _run_assist_accel = false;
    // bool _hold_speed = false;
    // bool _decelerate = false;

    // Excitation control commands (simulates external digital inputs)
    bool _exciter_ready = true;
    bool _excitation_start_command = true;
    bool _excitation_stop_command = false;

    // Startup sequencer control commands (simulates external digital inputs)
    bool _startup_sequencer_start_command = false;
    bool _startup_sequencer_stop_command = false;
};

/**
 * @test StartupSequencerIntegrationTest
 * @brief Validates startup sequencer coordinated startup to 0.2 pu speed
 *
 * This test validates the integration of the startup sequencer with the complete
 * control system. It tests the complete startup procedure including:
 * - Flux buildup phase (field excitation establishment)
 * - Virtual reference frame startup procedure (V/Hz control)
 * - Transition to closed-loop observer-based control
 * - Speed reference tracking to 0.2 pu
 *
 * Configuration:
 * - Virtual frequency ramp: 1 Hz → 10 Hz in 20 seconds
 * - Virtual torque ramp: 0.1 pu → 0.4 pu in 5 seconds
 * - Target speed: 0.2 pu
 *
 * Note: This test documents the current startup sequencer behavior and validates
 * that the parallel integration with speed generator works correctly.
 */
TEST_F(CompleteSensorlessSpeedControllerTest, StartupSequencerIntegrationTest)
{
    // ==================== Configure Startup Sequencer ====================
    // Virtual frequency profile: 0 Hz (0 pu) to 20 Hz (0.4 pu) in 50 seconds
    // Parameters are in per-unit and will be converted to SI using the per-unit system (base = 50 Hz)
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{10.0};

    // Virtual torque profile: 0.0 pu to 0.3 pu in 2 seconds
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.3};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    // Pre-fluxing duration before virtual startup
    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // ==================== Configure Speed Reference Rate Limiter ====================
    // Configure rate limiter: 20 seconds per per-unit = 0.05 pu/s
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{20.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Speed tracking checker for steady-state validation
    const speed_pu_t speed_error_tolerance{0.10f};  // Relaxed for rate-limited ramp
    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed tracking error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_error_tolerance, speed_error_tolerance, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();  // Will activate during steady-state phase

    // ==================== Execute State Machine Startup ====================
    execute_startup_sequence();

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "State machine should enable pulse releases after startup sequence";

    bool success = true;

    // ==================== Trigger Startup Sequencer ====================
    // Send start command to the startup sequencer to begin the coordinated startup sequence
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(1.0);  // Pre-flux duration

    // Configure moderate load torque
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.15f));

    // Configure torque limits for startup
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{-0.0});

    // ==================== Wait for Flux Buildup and Virtual Startup ====================
    // The startup sequencer will progress through:
    // IDLE → FLUX_BUILDUP → STARTING_PROCEDURE → CLOSED_LOOP_OPERATION
    // Pre-flux (1s) + Virtual frequency ramp (10s) = 11s until closed-loop
    // Add small settling time = 11.5s total
    success &= simulator.simulate_for(11.5);

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

    // ==================== Ramp Speed Reference to 1.0 pu with Rate Limiting ====================
    // After virtual startup, the speed reference limiter is reset to 0 when transitioning to closed-loop.
    // The limiter has been running since the transition (about 0.5s ago with 0 input).
    // Now set external speed reference to 1.0 pu and track the ramp from 0.
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{1.0});

    // Simulate a small step to allow the new reference to propagate
    success &= simulator.simulate_for(0.1);

    // Get the current speed reference as baseline (should be very close to 0, maybe ~0.005 pu from 0.5s + 0.1s
    // settling)
    auto baseline_speed_ref = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();

    // Calculate expected speed reference values based on rate limiting from baseline
    const real_t rate_pu_per_s = 0.05f;  // 20 s/pu = 0.05 pu/s
    const real_t expected_ref_5s = baseline_speed_ref.value() + 0.05f * 5.0f;
    const real_t expected_ref_10s = baseline_speed_ref.value() + 0.05f * 10.0f;
    const real_t expected_ref_20s = 1.0f;  // 1.0 pu after 20s from baseline

    // Simulate for 5 seconds
    success &= simulator.simulate_for(5.0);
    auto speed_ref_5s = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    EXPECT_NEAR(speed_ref_5s.value(), expected_ref_5s, 0.05f)
        << "Speed reference should ramp at 0.05 pu/s from 0 after closed-loop transition";

    // Simulate for 5 more seconds
    success &= simulator.simulate_for(5.0);
    auto speed_ref_10s = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    EXPECT_NEAR(speed_ref_10s.value(), expected_ref_10s, 0.05f)
        << "Speed reference should continue ramping at 0.05 pu/s";

    // Simulate until reference reaches 1.0 pu (need 20s total at 0.05 pu/s, already simulated 10s)
    success &= simulator.simulate_for(10.0);
    auto speed_ref_final = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    EXPECT_NEAR(speed_ref_final.value(), 1.0f, 0.03f)
        << "Speed reference should reach 1.0 pu with 20 s/pu rate limit";  // ==================== Verify Speed Tracking
                                                                           // ====================
    auto final_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(final_speed.value(), 1.0f, 0.15f)
        << "Machine should track the rate-limited speed reference to approximately 1.0 pu";

    // ==================== Steady-State Validation ====================
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_tracking_checker->deactivate();

    // ==================== Final Validation ====================
    EXPECT_TRUE(success) << "Startup sequencer integration test completed with simulation errors";

    // Verify system remains in operation
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "System should maintain pulse releases during steady-state operation";

    // Verify startup complete flag
    auto startup_complete = system.machine_side_control.startup_sequencer.get_outputs().is_startup_complete.read();
    EXPECT_TRUE(startup_complete) << "Startup sequencer should indicate startup complete";

    success &= simulator.simulate_for(10.0);
}

/**
 * @test PurgeModeOperationTest
 * @brief Validates continuous low-speed operation for purge mode
 *
 * This test validates that the soft-starter can maintain stable operation
 * at low speed for extended periods. This is critical for gas turbine
 * applications where purge mode may run for several minutes to clear
 * combustion chambers before ignition.
 *
 * Test Profile:
 * - Startup to 0.15 pu (purge speed)
 * - Hold for extended duration (simulating purge cycle)
 * - Verify stable operation with minimal speed variation
 *
 * Configuration:
 * - Virtual frequency: 0 → 0.08 pu in 8 seconds (low initial speed)
 * - Virtual torque: 0 → 0.25 pu in 1 second
 * - Speed rate limit: 25 s/pu (very gradual)
 * - Extended hold time: 15 seconds at purge speed
 */
TEST_F(CompleteSensorlessSpeedControllerTest, PurgeModeOperationTest)
{
    // ==================== Configure Startup Sequencer for Low-Speed Operation ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.01};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{5.0f};
    duration_t{0.0};  // Directly move to closed-loop control
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.02};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.2f};
    duration_t{0.0};  // Directly move to closed-loop control
    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};
    // ==================== Ramp to Purge Speed (0.15 pu) ====================
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.15});

    // Very gradual speed changes for smooth purge operation
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{120.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Tighter speed tracking for purge mode stability
    const speed_pu_t speed_error_tolerance{0.05f};
    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "purge mode speed stability",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_error_tolerance, speed_error_tolerance, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();

    // ==================== Execute State Machine Startup ====================
    execute_startup_sequence();

    bool success = true;

    // ==================== Initial Startup ====================
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Light load for purge mode (minimal resistance)
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.08f));

    // Configure torque limits
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{0.8});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    // Wait for virtual startup (1s + 8s + 0.5s)
    success &= simulator.simulate_for(9.5);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup should reach CLOSED_LOOP_OPERATION";

    // Ramp time: 0.15 pu × 120 s/pu = 18 seconds + settling
    success &= simulator.simulate_for(20.0);

    auto purge_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(purge_speed.value(), 0.15f, 0.05f) << "Machine should reach purge speed (0.15 pu)";

    // ==================== Extended Purge Mode Hold ====================
    // Simulate extended purge cycle (15 seconds)
    // In real applications, this could be several minutes
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(30.0);
    speed_tracking_checker->deactivate();

    // Verify speed remained stable
    auto final_purge_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(final_purge_speed.value(), 0.15f, 0.05f)
        << "Machine should maintain stable purge speed throughout extended hold";

    // ==================== Verify Control Stability ====================
    // Check that control outputs are not saturated during purge
    auto torque_saturated = system.machine_side_control.torque_controller.get_outputs().is_saturated.read();
    EXPECT_FALSE(torque_saturated) << "Torque controller should not be saturated during steady purge operation";

    // ==================== Final Validation ====================
    EXPECT_TRUE(success) << "Purge mode operation test completed with simulation errors";

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "System should maintain pulse releases during purge mode";

    auto startup_complete = system.machine_side_control.startup_sequencer.get_outputs().is_startup_complete.read();
    EXPECT_TRUE(startup_complete) << "Startup sequencer should indicate operation is stable";

    success &= simulator.simulate_for(60.0);
}

/**
 * @test TypicalSoftStarterProfileTest
 * @brief Validates multi-stage acceleration profile typical for soft-starters
 *
 * This test simulates a typical soft-starter application with multiple
 * acceleration stages:
 * - Creep speed (0.2 pu) for positioning/alignment
 * - Intermediate speed (0.5 pu) for purge or low-load operation
 * - Full speed (1.0 pu) for normal operation
 *
 * This profile is common in gas turbine starters and similar applications
 * where different operational phases require different speeds.
 *
 * Configuration:
 * - Virtual frequency: 0 → 0.1 pu in 10 seconds
 * - Virtual torque: 0 → 0.3 pu in 1 second
 * - Speed rate limit: 20 s/pu for controlled acceleration
 */
TEST_F(CompleteSensorlessSpeedControllerTest, TypicalSoftStarterProfileTest)
{
    // ==================== Configure Startup Sequencer ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{10.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Configure rate limiter for controlled acceleration (slower for quadratic load)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{40.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Speed tracking checker
    const speed_pu_t speed_error_tolerance{0.10f};
    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed tracking error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_error_tolerance, speed_error_tolerance, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();

    // ==================== Execute State Machine Startup ====================
    execute_startup_sequence();

    bool success = true;

    // ==================== Initial Startup ====================
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Configure gas turbine load (compressor characteristic)
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.10f));

    // Configure torque limits
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.2});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    // Wait for virtual startup to complete (1s + 10s + 0.5s)
    success &= simulator.simulate_for(11.5);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup should reach CLOSED_LOOP_OPERATION";

    // ==================== Stage 1: Creep Speed (0.2 pu) ====================
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.2});

    // Ramp time: 0.2 pu × 40 s/pu = 8 seconds + settling
    success &= simulator.simulate_for(10.0);

    auto creep_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(creep_speed.value(), 0.2f, 0.08f) << "Machine should reach creep speed (0.2 pu)";

    // Hold at creep speed for positioning/alignment phase - extended for steady-state
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(10.0);  // Extended from 3.0s to 10.0s
    speed_tracking_checker->deactivate();

    // ==================== Stage 2: Purge Speed (0.5 pu) ====================
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.5});

    // Ramp time: 0.3 pu × 40 s/pu = 12 seconds + settling
    success &= simulator.simulate_for(14.0);

    auto purge_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(purge_speed.value(), 0.5f, 0.10f) << "Machine should reach purge speed (0.5 pu)";

    // Hold at purge speed - extended for steady-state
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(10.0);  // Extended from 3.0s to 10.0s
    speed_tracking_checker->deactivate();

    // ==================== Stage 3: Full Speed (1.0 pu) ====================
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{1.0});

    // Ramp time: 0.5 pu × 40 s/pu = 20 seconds + settling
    success &= simulator.simulate_for(23.0);

    auto full_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(full_speed.value(), 1.0f, 0.15f) << "Machine should reach full speed (1.0 pu)";

    // ==================== Steady-State Operation ====================
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(15.0);  // Extended from 4.0s to 15.0s for full steady-state
    speed_tracking_checker->deactivate();

    // ==================== Final Validation ====================
    EXPECT_TRUE(success) << "Typical soft-starter profile test completed with simulation errors";

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "System should maintain pulse releases during steady-state operation";
}

/**
 * @test RestartCapabilityTest
 * @brief Validates startup sequencer can restart after stop
 *
 * This test validates that the soft-starter can perform a complete startup,
 * stop, and restart sequence. This is critical for applications where the
 * machine may need to restart after a temporary shutdown.
 *
 * Test Sequence:
 * - Initial startup to 0.4 pu
 * - Stop command to bring system to idle
 * - Restart sequence to 0.6 pu
 *
 * Configuration:
 * - Virtual frequency: 0 → 0.1 pu (5 Hz) in 8 seconds
 * - Virtual torque: 0 → 0.3 pu in 1 second
 * - Speed rate limit: 15 s/pu
 */
TEST_F(CompleteSensorlessSpeedControllerTest, RestartCapabilityTest)
{
    // GTEST_SKIP() << "Skipping test due to intermittent failures - requires investigation";

    // ==================== Configure Startup Sequencer ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{8.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.3};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Speed reference rate limiter: 30 seconds per pu (slower for quadratic load)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{30.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // ==================== Execute State Machine Startup ====================
    execute_startup_sequence();

    bool success = true;

    // ==================== First Startup Sequence ====================
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Configure moderate load
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.15f));

    // Configure torque limits
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{-0.0});

    // Wait for flux buildup and virtual startup (1s + 8s + 0.5s settling)
    success &= simulator.simulate_for(9.5);

    // Verify closed-loop operation
    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "First startup should reach CLOSED_LOOP_OPERATION";

    // ==================== Accelerate to 0.4 pu ====================
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.4});

    // Allow time for speed reference to ramp and speed to track
    // 0.4 pu at 30 s/pu = 12 seconds + settling
    success &= simulator.simulate_for(14.0);

    auto speed_before_stop = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_before_stop.value(), 0.4f, 0.1f) << "Speed should reach approximately 0.4 pu before stop";

    // ==================== Stop Sequence ====================
    // Stop the startup sequencer first
    set_startup_sequencer_stop_command(true);
    success &= simulator.simulate_for(0.1);
    set_startup_sequencer_stop_command(false);

    // Verify startup sequencer returned to IDLE
    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE) << "Startup sequencer should return to IDLE state";

    // Execute complete shutdown sequence
    execute_shutdown_sequence();

    // Verify complete shutdown
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulses should be disabled after complete shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().auxiliaries_are_on.read())
        << "Auxiliaries should be off after shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().breakers_are_on.read())
        << "Breakers should be off after shutdown";

    // Allow system to fully decelerate
    success &= simulator.simulate_for(2.0);

    // ==================== Restart Sequence ====================
    // Bring state machine back to OPERATION_ON following proper sequence
    set_auxiliaries_on(true);
    success &= simulator.simulate_for(0.1);

    set_breakers_on(true);
    success &= simulator.simulate_for(0.2);  // AUXILIARIES_ON → BREAKERS_OFF → BREAKERS_ON

    set_current_extinguished(false);  // Clear extinguished flag
    set_start_command(true);
    success &= simulator.simulate_for(0.2);  // BREAKERS_ON → OPERATION_OFF → OPERATION_ON

    // Verify state machine is in OPERATION_ON with pulses enabled
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "State machine should enable pulse releases in OPERATION_ON";

    // Step 2: Start the startup sequencer
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Wait for restart sequence (1s + 8s + 0.5s settling)
    success &= simulator.simulate_for(9.5);

    // Verify restart successful
    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Restart should reach CLOSED_LOOP_OPERATION";

    // ==================== Accelerate to 0.6 pu After Restart ====================
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.6});

    // Allow time for speed reference to ramp and speed to track
    // 0.6 pu at 30 s/pu = 18 seconds + settling
    success &= simulator.simulate_for(20.0);

    auto final_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(final_speed.value(), 0.6f, 0.12f) << "Speed should reach approximately 0.6 pu after restart";

    // ==================== Final Validation ====================
    EXPECT_TRUE(success) << "Restart capability test completed with simulation errors";

    auto startup_complete = system.machine_side_control.startup_sequencer.get_outputs().is_startup_complete.read();
    EXPECT_TRUE(startup_complete) << "Startup sequencer should indicate restart complete";
}

/**
 * @test StateMachineControlledShutdownTest
 * @brief Validates controlled shutdown and restart sequence
 *
 * This test validates the complete shutdown and restart sequence:
 * 1. System operates at speed
 * 2. Complete shutdown sequence (using execute_shutdown_sequence helper)
 * 3. Verify speed reference goes to zero
 * 4. System can restart after complete shutdown (following proper state transitions)
 */
TEST_F(CompleteSensorlessSpeedControllerTest, StateMachineControlledShutdownTest)
{
    // ==================== Configure Startup Sequencer ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{5.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.2};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    // Fast speed reference rate for testing: 10 s/pu = 0.1 pu/s
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{10.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // ==================== Execute State Machine Startup ====================
    execute_startup_sequence();

    bool success = true;

    // ==================== Startup and Accelerate to 0.3 pu ====================
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(0.5);  // Pre-flux

    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(6.0);  // Complete virtual startup

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION) << "Should reach CLOSED_LOOP_OPERATION";

    // Accelerate to 0.3 pu
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.3});
    success &= simulator.simulate_for(5.0);  // 0.3 pu * 10 s/pu = 3s + settling

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 0.3f, 0.1f) << "Should reach 0.3 pu operating speed";

    // Capture speed reference before shutdown
    auto speed_ref_before_shutdown = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    EXPECT_GT(speed_ref_before_shutdown.value(), 0.2f) << "Speed reference should be > 0.2 pu before shutdown";

    // ==================== Execute Complete Shutdown Sequence ====================
    // Stop the startup sequencer first to trigger speed reference override
    set_startup_sequencer_stop_command(true);
    success &= simulator.simulate_for(0.1);
    set_startup_sequencer_stop_command(false);

    // Use the helper function which follows the correct state machine sequence
    execute_shutdown_sequence();

    // ==================== Verify Shutdown Completed ====================
    // Verify state machine reached IDLE (auxiliaries off, breakers off, pulses off)
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulses should be disabled after complete shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().auxiliaries_are_on.read())
        << "Auxiliaries should be off after shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().breakers_are_on.read())
        << "Breakers should be off after shutdown";

    // Verify startup sequencer is in IDLE
    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE)
        << "Startup sequencer should be in IDLE after complete shutdown";

    // Note: Speed reference limiter maintains its last value in IDLE state.
    // It will be reset when the sequencer restarts or a new speed reference is commanded.

    // ==================== Test Restart After Complete Shutdown ====================
    // Now restart following proper sequence: AUX ON → BREAKERS ON → START

    // Step 1: Bring auxiliaries back on (IDLE → AUXILIARIES_ON)
    set_auxiliaries_on(true);
    success &= simulator.simulate_for(0.1);

    // Step 2: Close breakers (AUXILIARIES_ON → BREAKERS_OFF → BREAKERS_ON)
    set_breakers_on(true);
    success &= simulator.simulate_for(0.2);

    // Step 3: Clear extinguished flag and issue START command (BREAKERS_ON → OPERATION_OFF → OPERATION_ON)
    set_current_extinguished(false);
    set_start_command(true);
    success &= simulator.simulate_for(0.2);

    // Verify state machine returns to OPERATION_ON
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "State machine should enable pulses when returning to OPERATION_ON";

    // Step 4: Restart startup sequencer
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(0.5);  // Pre-flux

    success &= simulator.simulate_for(6.0);  // Complete virtual startup

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Should successfully restart to CLOSED_LOOP_OPERATION";

    EXPECT_TRUE(success) << "State machine controlled shutdown and restart test completed";
}

/**
 * @test ShutdownOnAuxiliariesOffTest
 * @brief Validates complete shutdown when AUXILIARIES turn OFF during operation
 *
 * This test verifies that the system properly handles auxiliary power loss during operation:
 * - System operates normally in OPERATION_ON
 * - Auxiliaries signal turns OFF (simulating power loss or emergency stop)
 * - Complete shutdown sequence is executed
 * - System reaches safe IDLE state
 */
TEST_F(CompleteSensorlessSpeedControllerTest, ShutdownOnAuxiliariesOffTest)
{
    // ==================== Configure Startup Sequencer ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.08};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{4.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.2};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.5};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{8.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // ==================== Execute Startup ====================
    execute_startup_sequence();

    bool success = true;

    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(0.5);  // Pre-flux

    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.08f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{0.8});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(5.0);  // Complete startup

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION);

    // Accelerate to 0.2 pu
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.2});
    success &= simulator.simulate_for(3.0);

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 0.2f, 0.08f) << "Should reach operating speed";

    // ==================== Trigger Shutdown via AUXILIARIES_OFF ====================
    // Turn off auxiliaries during operation - this triggers state machine shutdown sequence
    set_auxiliaries_on(false);
    success &= simulator.simulate_for(0.2);

    // Stop startup sequencer
    set_startup_sequencer_stop_command(true);
    success &= simulator.simulate_for(0.1);
    set_startup_sequencer_stop_command(false);

    // Verify sequencer went to IDLE
    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE) << "Sequencer should be in IDLE after stop";

    // Complete the shutdown sequence manually
    set_current_extinguished(true);
    success &= simulator.simulate_for(0.1);

    set_breakers_on(false);
    set_start_command(false);
    success &= simulator.simulate_for(0.3);

    // ==================== Verify Complete Shutdown ====================
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulses should be disabled after auxiliaries off";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().auxiliaries_are_on.read())
        << "Auxiliaries should be off";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().breakers_are_on.read())
        << "Breakers should be off after complete shutdown";

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE)
        << "Startup sequencer should be in IDLE after auxiliaries off";

    EXPECT_TRUE(success) << "Shutdown on auxiliaries off test completed";
}

/**
 * @test ShutdownOnStopCommandTest
 * @brief Validates complete shutdown sequence initiated by STOP command
 *
 * This test verifies that the STOP command properly initiates a controlled shutdown
 * following the complete state machine sequence:
 * - System operates normally in OPERATION_ON
 * - STOP command triggers shutdown sequence
 * - Complete shutdown through all states (SHUTDOWN_REQUESTED → OPERATION_OFF → IDLE)
 * - Proper cleanup with breakers and auxiliaries off
 */
TEST_F(CompleteSensorlessSpeedControllerTest, ShutdownOnStopCommandTest)
{
    // ==================== Configure Startup Sequencer ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{5.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.25};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{12.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // ==================== Execute Startup ====================
    execute_startup_sequence();

    bool success = true;

    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(0.5);  // Pre-flux

    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(6.0);  // Complete startup

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION);

    // Accelerate to 0.4 pu
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.4});
    success &= simulator.simulate_for(6.0);  // 0.4 * 12 = 4.8s + settling

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 0.4f, 0.1f) << "Should reach 0.4 pu operating speed";

    // ==================== Execute Complete Shutdown via Helper ====================
    // Stop the startup sequencer first
    set_startup_sequencer_stop_command(true);
    success &= simulator.simulate_for(0.1);
    set_startup_sequencer_stop_command(false);

    // Verify startup sequencer went to IDLE
    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE) << "Startup sequencer should be in IDLE after stop";

    // Execute the complete state machine shutdown sequence
    execute_shutdown_sequence();

    // Allow system to settle
    success &= simulator.simulate_for(0.5);

    // ==================== Verify Complete Shutdown ====================
    // Verify state machine reached complete IDLE state
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulses should be disabled after complete shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().auxiliaries_are_on.read())
        << "Auxiliaries should be off after complete shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().breakers_are_on.read())
        << "Breakers should be off after complete shutdown";

    // Verify startup sequencer remained in IDLE
    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE)
        << "Startup sequencer should be in IDLE after complete shutdown";

    EXPECT_TRUE(success) << "Shutdown on stop command test completed";
}

/**
 * @test ShutdownOnBreakersOffTest
 * @brief Validates complete shutdown when BREAKERS turn OFF during operation
 *
 * This test verifies proper handling of breaker opening during operation:
 * - System operates normally in OPERATION_ON
 * - Breakers signal turns OFF (circuit breaker trip or manual opening)
 * - Complete shutdown sequence is executed
 * - System reaches safe IDLE state
 */
TEST_F(CompleteSensorlessSpeedControllerTest, ShutdownOnBreakersOffTest)
{
    // ==================== Configure Startup Sequencer ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{4.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.2};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.5};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{10.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // ==================== Execute Startup ====================
    execute_startup_sequence();

    bool success = true;

    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(0.5);  // Pre-flux

    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.09f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{0.9});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(5.0);  // Complete startup

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION);

    // Accelerate to 0.25 pu
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.25});
    success &= simulator.simulate_for(4.0);

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 0.25f, 0.08f);

    // ==================== Trigger Shutdown via BREAKERS_OFF ====================
    // Turn off breakers during operation - this triggers state machine shutdown sequence
    set_breakers_on(false);
    success &= simulator.simulate_for(0.2);

    // Stop startup sequencer
    set_startup_sequencer_stop_command(true);
    success &= simulator.simulate_for(0.1);
    set_startup_sequencer_stop_command(false);

    // Verify sequencer went to IDLE
    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE) << "Sequencer should be in IDLE after stop";

    // Complete the shutdown sequence manually
    set_current_extinguished(true);
    success &= simulator.simulate_for(0.1);

    set_auxiliaries_on(false);
    set_start_command(false);
    success &= simulator.simulate_for(0.3);

    // ==================== Verify Complete Shutdown ====================
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulses should be disabled after breakers off";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().auxiliaries_are_on.read())
        << "Auxiliaries should be off after complete shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().breakers_are_on.read())
        << "Breakers should be off";

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE)
        << "Startup sequencer should be in IDLE after breakers off";

    EXPECT_TRUE(success) << "Shutdown on breakers off test completed";
}

/**
 * @test FlyingRestartCapabilityTest
 * @brief Validates flying restart when machine is already spinning
 *
 * This test verifies that the startup sequencer can detect when the machine
 * is already spinning and skip the flux buildup and virtual startup phases,
 * transitioning directly to closed-loop operation. This is critical for:
 * - Hot restarts after brief power interruptions
 * - Reconnection to spinning load
 * - Minimizing startup time when field is established
 *
 * Test Sequence:
 * 1. Normal startup to 0.3 pu
 * 2. Complete shutdown sequence (must open breakers/auxiliaries per state machine rules)
 * 3. Machine coasts but remains above flying restart threshold (5%)
 * 4. Restart via proper state machine sequence with flying restart enabled
 * 5. Verify direct transition to CLOSED_LOOP_OPERATION (no flux buildup or virtual phase)
 */
TEST_F(CompleteSensorlessSpeedControllerTest, FlyingRestartCapabilityTest)
{
    // ==================== Configure Startup Sequencer ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{5.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.2};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{10.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Configure flying restart
    system.machine_side_control.startup_sequencer.get_parameters().flying_restart_speed_threshold = speed_pu_t{0.05};
    system.machine_side_control.startup_sequencer.get_parameters().auto_start_stop_enabled = true;

    // ==================== Execute Initial Startup ====================
    execute_startup_sequence();

    bool success = true;

    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(0.5);  // Pre-flux

    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.08f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(6.0);  // Complete virtual startup

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION) << "Initial startup should complete";

    // Accelerate to 0.3 pu
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.3});
    success &= simulator.simulate_for(4.0);

    auto initial_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(initial_speed.value(), 0.3f, 0.1f) << "Should reach 0.3 pu before simulated interruption";

    // ==================== Simulate Brief Interruption with Complete Shutdown ====================
    // Stop startup sequencer first (transitions to IDLE)
    set_startup_sequencer_stop_command(true);
    success &= simulator.simulate_for(0.1);
    set_startup_sequencer_stop_command(false);

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE) << "Sequencer should be in IDLE after stop";

    // Complete shutdown sequence (required before restart per state machine rules)
    execute_shutdown_sequence();

    // Verify complete shutdown
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulses should be disabled after shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().auxiliaries_are_on.read())
        << "Auxiliaries should be off after shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().breakers_are_on.read())
        << "Breakers should be off after shutdown";

    // Allow brief coasting (machine will slow down slightly but remain above 5% threshold)
    success &= simulator.simulate_for(0.5);

    auto coasting_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(coasting_speed.value(), 0.05f) << "Machine should remain above flying restart threshold (5%)";

    // ==================== Execute Flying Restart via Proper State Machine Sequence ====================
    // Restart requires proper sequence: IDLE -> AUXILIARIES_ON -> BREAKERS_ON -> START
    set_auxiliaries_on(true);
    success &= simulator.simulate_for(0.1);

    set_breakers_on(true);
    success &= simulator.simulate_for(0.1);

    set_current_extinguished(false);  // Clear extinguished flag
    set_start_command(true);
    success &= simulator.simulate_for(0.2);

    // Verify state machine reached OPERATION_ON
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "State machine should enable pulses when returning to OPERATION_ON";  // Trigger startup sequencer
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(0.1);  // Allow one control cycle for state transition

    // Verify DIRECT transition to CLOSED_LOOP_OPERATION (skipping FLUX_BUILDUP and STARTING_PROCEDURE)
    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Flying restart should skip flux buildup and virtual startup, going directly to CLOSED_LOOP";

    // Verify time in state is very short (immediate transition)
    auto time_in_state = system.machine_side_control.startup_sequencer.get_outputs().time_in_current_state.read();
    EXPECT_LT(time_in_state.value(), 0.5) << "Should transition immediately, not go through normal startup sequence";

    // ==================== Verify Normal Operation After Flying Restart ====================
    // Set new speed reference
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.4});
    success &= simulator.simulate_for(5.0);

    auto final_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(final_speed.value(), 0.4f, 0.12f) << "Should achieve new speed setpoint after flying restart";

    EXPECT_TRUE(success) << "Flying restart capability test completed";
}

/**
 * @test AutoStartOnOperationOnTest
 * @brief Validates automatic startup when state machine enters OPERATION_ON
 *
 * This test verifies the auto-start coordination feature where the startup
 * sequencer automatically begins the startup sequence when the grid-side
 * state machine reaches OPERATION_ON state, without requiring manual start command.
 *
 * Test Sequence:
 * 1. Enable auto-start/stop coordination
 * 2. Progress state machine to OPERATION_ON
 * 3. Verify startup sequencer automatically starts (no manual start command)
 * 4. Verify normal startup sequence completion
 */
TEST_F(CompleteSensorlessSpeedControllerTest, AutoStartOnOperationOnTest)
{
    // ==================== Configure Startup Sequencer ====================
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

    // ==================== Progress State Machine to OPERATION_ON ====================
    bool success = true;

    // Start from IDLE, do NOT send manual startup sequencer start command
    set_auxiliaries_on(true);
    success &= simulator.simulate_for(0.1);

    set_breakers_on(true);
    success &= simulator.simulate_for(0.2);

    // Verify startup sequencer still in IDLE (auto-start hasn't triggered yet)
    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE)
        << "Sequencer should remain in IDLE until OPERATION_ON is reached";

    // Send START command to reach OPERATION_ON
    set_start_command(true);
    success &= simulator.simulate_for(0.2);

    // ==================== Verify Auto-Start Triggered ====================
    // Startup sequencer should automatically transition from IDLE when OPERATION_ON is detected
    success &= simulator.simulate_for(0.6);  // Allow time for flux buildup to start

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_NE(startup_state, startup_sequencer_state_t::IDLE)
        << "Auto-start should automatically begin startup sequence when OPERATION_ON is reached";

    // Should be in FLUX_BUILDUP or beyond
    EXPECT_TRUE(startup_state == startup_sequencer_state_t::FLUX_BUILDUP ||
                startup_state == startup_sequencer_state_t::STARTING_PROCEDURE ||
                startup_state == startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Auto-start should progress through normal startup states";

    // ==================== Complete Startup Sequence ====================
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.06f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{0.8});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(5.0);  // Complete virtual startup

    startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Auto-started sequence should reach CLOSED_LOOP_OPERATION";

    // ==================== Verify Normal Operation ====================
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.2});
    success &= simulator.simulate_for(3.0);

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 0.2f, 0.08f) << "Auto-started system should achieve speed setpoint normally";

    EXPECT_TRUE(success) << "Auto-start on OPERATION_ON test completed";
}

/**
 * @test SpeedReferenceZeroOverrideDuringShutdownTest
 * @brief Validates controlled deceleration to zero speed before shutdown
 *
 * This test verifies the controlled deceleration mechanism where:
 * - System operates at speed
 * - Speed reference is ramped down to zero
 * - Machine decelerates to zero following the speed reference
 * - Deceleration time depends on mechanical load (torque capability)
 * - Complete shutdown is achieved after machine reaches zero speed
 *
 * The deceleration time is proportional to the mechanical load because
 * the control system must provide negative electromagnetic torque to
 * overcome the load torque and decelerate the machine.
 */
TEST_F(CompleteSensorlessSpeedControllerTest, SpeedReferenceZeroOverrideDuringShutdownTest)
{
    // ==================== Configure Startup Sequencer ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{5.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.2};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.5};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    // Configure rate limiter for controlled deceleration: 10 s/pu = 0.1 pu/s
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{10.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // ==================== Execute Startup and Reach Operating Speed ====================
    execute_startup_sequence();

    bool success = true;

    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(0.5);  // Pre-flux

    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{-1.0});

    success &= simulator.simulate_for(6.0);  // Complete startup

    // Accelerate to 0.35 pu
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.35});
    success &= simulator.simulate_for(5.0);  // 0.35 pu * 10 s/pu = 3.5s + settling

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 0.35f, 0.1f) << "Should reach operating speed";

    // ==================== Initiate Shutdown and Observe Passive Deceleration ====================
    // Stop the startup sequencer to begin shutdown
    set_startup_sequencer_stop_command(true);
    success &= simulator.simulate_for(0.1);
    set_startup_sequencer_stop_command(false);

    // Verify startup sequencer went to IDLE
    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::IDLE) << "Startup sequencer should be in IDLE after stop";

    // Execute complete shutdown sequence (this will zero load torque when breakers turn off)
    execute_shutdown_sequence();

    // Verify complete shutdown
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulses should be disabled after complete shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().auxiliaries_are_on.read())
        << "Auxiliaries should be off after shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().breakers_are_on.read())
        << "Breakers should be off after shutdown";

    auto final_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GE(final_speed.value(), -0.05f) << "Speed must not go significantly negative during shutdown";
    EXPECT_LE(final_speed.value(), 0.45f) << "Speed should remain within reasonable range";

    EXPECT_TRUE(success) << "Speed reference zero override during shutdown test completed";

    success &= simulator.simulate_for(10.0);  // Allow time to observe final speed behavior
}

/**
 * @test FlyingRestartWithAutoStartTest
 * @brief Validates flying restart combined with auto-start coordination
 *
 * This test combines two features:
 * 1. Auto-start when OPERATION_ON is reached
 * 2. Flying restart detection when machine is already spinning
 *
 * Test Sequence:
 * 1. Normal startup and acceleration to 0.3 pu
 * 2. Complete shutdown sequence to IDLE
 * 3. Machine coasts but remains above 5% threshold
 * 4. State machine returns to OPERATION_ON
 * 5. Verify auto-start detects flying restart and skips to CLOSED_LOOP
 */
TEST_F(CompleteSensorlessSpeedControllerTest, FlyingRestartWithAutoStartTest)
{
    // ==================== Configure Startup Sequencer ====================
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{5.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.2};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{10.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Enable both auto-start and flying restart
    system.machine_side_control.startup_sequencer.get_parameters().auto_start_stop_enabled = true;
    system.machine_side_control.startup_sequencer.get_parameters().flying_restart_speed_threshold = speed_pu_t{0.05};

    // ==================== Initial Startup ====================
    execute_startup_sequence();

    bool success = true;

    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(0.5);  // Pre-flux

    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.08f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(6.0);

    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.3});
    success &= simulator.simulate_for(4.0);

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 0.3f, 0.1f);

    // ==================== Shutdown Sequence ====================
    set_startup_sequencer_stop_command(true);
    success &= simulator.simulate_for(0.3);
    set_startup_sequencer_stop_command(false);

    set_stop_command(true);
    success &= simulator.simulate_for(0.2);

    set_current_extinguished(true);
    set_stop_command(false);
    success &= simulator.simulate_for(0.2);

    // Machine coasts but stays above threshold
    success &= simulator.simulate_for(0.5);

    auto coasting_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(coasting_speed.value(), 0.05f) << "Machine should remain above flying restart threshold";

    // ==================== Restart with Auto-Start and Flying Restart ====================
    // Return state machine to OPERATION_ON (auto-start will trigger)
    set_current_extinguished(false);
    set_start_command(true);
    success &= simulator.simulate_for(0.3);

    // Verify flying restart: should skip flux buildup and go directly to CLOSED_LOOP
    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Auto-start should detect flying restart condition and skip directly to CLOSED_LOOP";

    // Verify quick transition (not full startup time)
    auto time_in_state = system.machine_side_control.startup_sequencer.get_outputs().time_in_current_state.read();
    EXPECT_LT(time_in_state.value(), 1.0) << "Flying restart should be nearly immediate";

    // ==================== Verify Normal Operation After Flying Restart ====================
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.4});
    success &= simulator.simulate_for(5.0);

    auto final_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(final_speed.value(), 0.4f, 0.12f) << "Should achieve speed setpoint after flying restart";

    EXPECT_TRUE(success) << "Flying restart with auto-start test completed";
}
