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

            system.machine_side_control.external_ios.update_speed_generator_inputs(_run_creep_speed, _run_purge_accel,
                _run_ignition_accel, _run_assist_accel, _hold_speed, _decelerate, _idle);

            system.machine_side_control.external_ios.update_startup_sequencer_inputs(
                _startup_sequencer_start_command, _startup_sequencer_stop_command);
        });

        // Initialize the system - allocates plant model and all components
        initialize();

        // ==================== Logger Registration ====================
        // register_complete_logger(simulator, system.plant);
        // register_complete_logger(simulator, system.grid_side_control.state_machine);
        // register_complete_logger(simulator, system.machine_side_control.machine_side_controller);
        // register_complete_logger(simulator, system.machine_side_control.machine_side_pulse_generator);
        // register_complete_logger(simulator, system.grid_side_control.grid_side_pulse_generator);

        // register_complete_logger(simulator, system.machine_side_control.speed_generator);  // TODO: Create logger
        register_complete_logger(simulator, system.grid_side_control.grid_side_pulse_generator);
        register_complete_logger(simulator, system.grid_side_control.grid_side_modulator);
        register_complete_logger(simulator, system.grid_side_control.current_controller);
        register_complete_logger(simulator, system.machine_side_control.torque_controller);
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        // register_complete_logger(simulator, system.machine_side_control.speed_generator);
        register_complete_logger(simulator, system.machine_side_control.startup_sequencer);
        register_complete_logger(simulator, system.grid_side_control.state_machine);
        register_complete_logger(simulator, system.machine_side_control.excitation_controller);
        register_complete_logger(simulator, system.machine_side_control.machine_side_controller);
        register_complete_logger(simulator, system.plant);
        // register_complete_logger(simulator, system.machine_side_control.machine_side_modulator);
        // register_complete_logger(simulator, system.grid_side_control.grid_side_observer);
        // register_complete_logger(simulator, system.grid_side_control.grid_side_modulator);
        // register_complete_logger(simulator, system.grid_side_control.dc_link_measurements);
        // register_complete_logger(simulator, system.grid_side_control.grid_measurements);
        // register_complete_logger(simulator, system.machine_side_control.machine_measurements);
        // register_minimal_operation_mode_monitor_logger(simulator, system.grid_side_control.operation_mode_monitor);
    }

    void initialize()
    {
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

    // Speed profile command helpers using datamodel override parameters
    void set_idle(bool idle) { _idle = idle; }

    void set_run_creep_speed(bool run) { _run_creep_speed = run; }

    void set_run_purge_accel(bool run) { _run_purge_accel = run; }

    void set_run_ignition_accel(bool run) { _run_ignition_accel = run; }

    void set_run_assist_accel(bool run) { _run_assist_accel = run; }

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

    // Speed generator profile commands (simulates external digital inputs)
    bool _idle = true;
    bool _run_creep_speed = false;
    bool _run_purge_accel = false;
    bool _run_ignition_accel = false;
    bool _run_assist_accel = false;
    bool _hold_speed = false;
    bool _decelerate = false;

    // Excitation control commands (simulates external digital inputs)
    bool _exciter_ready = true;
    bool _excitation_start_command = true;
    bool _excitation_stop_command = false;

    // Startup sequencer control commands (simulates external digital inputs)
    bool _startup_sequencer_start_command = false;
    bool _startup_sequencer_stop_command = false;
};

// Test coordinated startup and multi-phase acceleration using startup sequencer
// This test validates the startup sequencer followed by rate-limited speed reference ramping
// Phase 1: Startup sequencer (0 -> 0.05 pu) - Creep speed equivalent
// Phase 2: Rate-limited ramp (0.05 -> 0.20 pu) - Purge acceleration equivalent
// Phase 3: Rate-limited ramp (0.20 -> 0.55 pu) - Ignition acceleration equivalent
// Phase 4: Rate-limited ramp (0.55 -> 0.95 pu) - Assist acceleration equivalent
TEST_F(CompleteSensorlessSpeedControllerTest, GasTurbineSoftStartSequence)
{
    // ==================== Configure Startup Sequencer ====================
    // Virtual frequency ramp: 0 Hz (0 pu) to 2.5 Hz (0.05 pu at 50 Hz base) in 30 seconds
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.05};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{30.0};

    // Virtual torque ramp: 0.0 pu to 0.3 pu in 2 seconds
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.3};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{2.0};

    // Pre-fluxing duration
    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Configure speed reference rate limiter for subsequent phases
    // Use a moderate rate limit that allows reasonable test execution time
    // 30 s/pu = 0.0333 pu/s allows smooth acceleration
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{30.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Speed tracking tolerance (wider for low-speed operation)
    const speed_pu_t speed_error_max_value{0.10f};
    const speed_pu_t speed_error_min_value{-0.10f};

    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed tracking error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        speed_error_min_value, speed_error_max_value, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();  // Will activate during steady-state phases

    // ==================== State Machine Startup ====================
    execute_startup_sequence();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();

    bool success = true;

    // Configure torque limits for startup and acceleration
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{-0.5});

    // ==================== Phase 1: Startup Sequencer to Creep Speed (~5% rated) ====================
    // Purpose: Initial rotation, oil circulation, pre-warming via coordinated startup
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(1.0);  // Pre-flux duration

    // Configure realistic load torque profile (increases with speed)
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.15f));

    // Allow virtual frequency ramp to complete (30s) + small settling
    success &= simulator.simulate_for(30.5);

    // Verify startup sequencer completed and is in closed-loop
    EXPECT_EQ(system.machine_side_control.startup_sequencer.get_outputs().current_state.read(),
        startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup sequencer should have transitioned to CLOSED_LOOP_OPERATION";

    // Verify target speed reached
    const auto speed_after_creep =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_creep.value(), 0.05f, 0.02f) << "Phase 1 (Creep) did not reach target speed";

    // Hold at creep speed for inspection/settling
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_tracking_checker->deactivate();

    // ==================== Phase 2: Purge Acceleration (~20% rated) ====================
    // Purpose: Combustion chamber ventilation to clear residual fuel vapors
    // Set external speed reference to 0.20 pu and let rate limiter ramp smoothly
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.20});
    success &= simulator.simulate_for(0.1);  // Allow reference to propagate

    // Calculate time needed: delta = 0.20 - 0.05 = 0.15 pu at 30 s/pu = 4.5s
    // Add margin for tracking: ~5s total
    success &= simulator.simulate_for(5.0);

    // Verify target speed reached
    const auto speed_after_purge =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_purge.value(), 0.20f, 0.02f) << "Phase 2 (Purge) did not reach target speed";

    // Hold at purge speed (typically 2-5 minutes in real operation, shortened for test)
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_tracking_checker->deactivate();

    // ==================== Phase 3: Ignition Acceleration (~55% rated) ====================
    // Purpose: Reach speed for fuel ignition and flame stabilization
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.55});
    success &= simulator.simulate_for(0.1);

    // Increase load as turbine begins to generate power
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.25f));

    // Calculate time needed: delta = 0.55 - 0.20 = 0.35 pu at 30 s/pu = 10.5s
    success &= simulator.simulate_for(11.0);

    // Verify target speed reached
    const auto speed_after_ignition =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_ignition.value(), 0.55f, 0.03f) << "Phase 3 (Ignition) did not reach target speed";

    // Hold at ignition speed for flame stabilization
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_tracking_checker->deactivate();

    // ==================== Phase 4: Assist to Self-Sustaining (~95% rated) ====================
    // Purpose: Accelerate to self-sustaining speed where turbine can maintain rotation
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.95});
    success &= simulator.simulate_for(0.1);

    // Further increase load as turbine approaches full power
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.30f));

    // Calculate time needed: delta = 0.95 - 0.55 = 0.40 pu at 30 s/pu = 12s
    success &= simulator.simulate_for(13.0);

    // Verify final target speed reached
    const auto speed_after_assist =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_assist.value(), 0.95f, 0.03f) << "Phase 4 (Assist) did not reach target speed";

    // Hold at self-sustaining speed (ready for synchronization to grid)
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_tracking_checker->deactivate();

    // ==================== Validation ====================
    EXPECT_TRUE(success) << "Gas turbine soft-start sequence did not complete successfully";

    // Verify state machine remained in OPERATION_ON throughout
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulse releases should be active during entire starting sequence";

    // Verify final state: Ready for synchronous handoff
    EXPECT_GT(speed_after_assist.value(), 0.90f) << "Final speed insufficient for turbine self-sustaining operation";
}

/**
 * @test TwoPhaseAccelerationTest
 * @brief Validates two-phase acceleration using startup sequencer and rate-limited speed reference
 *
 * This test exercises the startup sequencer's coordinated startup followed by a rate-limited
 * speed reference ramp to a higher speed. It validates the transition from virtual reference
 * frame control to observer-based closed-loop control with smooth speed tracking.
 *
 * @par Test Sequence:
 * -# Configure startup sequencer with reduced times for faster testing
 *    - Virtual frequency: 0 → 0.05 pu in 20s
 *    - Virtual torque: 0 → 0.3 pu in 2s
 * -# Execute state machine startup sequence
 * -# Trigger startup sequencer (flux buildup + virtual ramp)
 * -# Wait for closed-loop transition (~21s)
 * -# Set external speed reference to 0.55 pu
 * -# Wait for rate-limited ramp to complete
 * -# Hold at final speed with tracking verification
 *
 * @par Validations:
 * - Speed reaches 0.05 pu ± 0.02 after startup sequencer
 * - Startup sequencer transitions to CLOSED_LOOP_OPERATION
 * - Speed ramps smoothly to 0.55 pu ± 0.03 with rate limiting
 * - Speed tracking error < 0.10 pu during steady-state phases
 * - State machine remains in OPERATION_ON throughout
 *
 * @par Key Features Tested:
 * - Coordinated startup (flux buildup + virtual ramp)
 * - Transition from virtual to estimated reference frame
 * - Rate-limited speed reference tracking
 * - Controller performance across speed ranges (0-55% rated)
 */
TEST_F(CompleteSensorlessSpeedControllerTest, SpeedGeneratorMultiProfileSequenceTest)
{
    // ==================== Configure Startup Sequencer ====================
    // Virtual frequency: 0 Hz (0 pu) to 2.5 Hz (0.05 pu at 50 Hz base) in 20 seconds
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.05};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{20.0};

    // Virtual torque: 0.0 pu to 0.3 pu in 2 seconds
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.3};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{2.0};

    // Pre-fluxing duration
    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Configure speed reference rate limiter
    // For second phase: 0.05 → 0.55 pu = 0.5 pu delta in 40s → 80 s/pu rate limit
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{80.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Speed tracking tolerance
    const speed_pu_t speed_error_max_value{0.10f};
    const speed_pu_t speed_error_min_value{-0.10f};

    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed tracking error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        speed_error_min_value, speed_error_max_value, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();

    bool success = true;

    // Configure torque limits
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{-0.5});

    // ==================== Phase 1: Startup Sequencer (0 → 0.05 pu) ====================
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Configure realistic load torque
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.15f));

    // Wait for virtual frequency ramp completion (20s + small settling)
    success &= simulator.simulate_for(20.5);

    // Verify startup sequencer completed
    EXPECT_EQ(system.machine_side_control.startup_sequencer.get_outputs().current_state.read(),
        startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup sequencer should have transitioned to CLOSED_LOOP_OPERATION";

    // Verify Phase 1 target reached
    const auto speed_after_startup =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_startup.value(), 0.05f, 0.02f) << "Phase 1 (Startup) did not reach target speed";

    // Activate tracking checker for steady-state validation
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(3.0);
    speed_tracking_checker->deactivate();

    // ==================== Phase 2: Rate-Limited Ramp (0.05 → 0.55 pu) ====================
    system.machine_side_control.external_ios.get_fixed_speed_reference().write(speed_pu_t{0.55});
    success &= simulator.simulate_for(0.1);  // Allow reference to propagate

    // Increase load as speed increases (simulates turbine power generation)
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.25f));

    // Calculate time needed: delta = 0.55 - 0.05 = 0.5 pu at 80 s/pu = 40s
    success &= simulator.simulate_for(41.0);

    // Verify Phase 2 target reached
    const auto speed_after_accel =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_accel.value(), 0.55f, 0.03f) << "Phase 2 (Acceleration) did not reach target speed";

    // Final steady-state validation
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(3.0);
    speed_tracking_checker->deactivate();

    // ==================== Validation ====================
    EXPECT_TRUE(success) << "Two-phase acceleration test did not complete successfully";

    // Verify state machine remained operational throughout
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulse releases should remain active during acceleration sequence";
}

/**
 * @test VirtualStartupLoadRejectionTest
 * @brief Tests virtual frequency ramp with load disturbance rejection
 *
 * This test validates the startup sequencer's virtual reference frame control operating
 * in the low-speed region (0-20% rated) with emphasis on load disturbance rejection during
 * the virtual frequency ramp. This is more realistic than the previous profile-based test
 * as it validates the actual startup procedure under varying load conditions.
 *
 * @par Test Sequence:
 * -# Configure startup sequencer with 60s virtual frequency ramp to 0.20 pu
 * -# Start from standstill with initial load of 0.10 pu
 * -# Execute state machine startup
 * -# Trigger startup sequencer (flux buildup + virtual ramp)
 * -# During virtual ramp (~30s in, mid-ramp at ~0.10 pu):
 *    - Apply load step UP: 0.10 → 0.20 pu
 *    - Wait 5s for regulation
 *    - Apply load step DOWN: 0.20 → 0.10 pu
 * -# Continue to virtual ramp completion (0.20 pu)
 * -# Hold in closed-loop with additional load step testing
 *
 * @par Validations:
 * - Speed continues ramping smoothly during load transients
 * - Speed tracking error < 0.10 pu during steady-state after steps
 * - Final speed reaches 0.20 pu ± 0.02
 * - Controller maintains regulation without saturation
 * - Transition to closed-loop operation is smooth
 *
 * @par Key Features Tested:
 * - Virtual reference frame startup execution
 * - Load disturbance rejection during virtual frequency ramp
 * - Speed regulation stability in low-speed region
 * - Transition from virtual to closed-loop control under load
 */
TEST_F(CompleteSensorlessSpeedControllerTest, PurgeModeWithLoadStepTest)
{
    // ==================== Configure Startup Sequencer ====================
    // Virtual frequency: 0 Hz (0 pu) to 10 Hz (0.20 pu at 50 Hz base) in 60 seconds
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.20};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{60.0};

    // Virtual torque: 0.0 pu to 0.25 pu in 5 seconds (higher for load steps)
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.25};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{5.0};

    // Pre-fluxing duration
    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Configure speed reference rate limiter for closed-loop phase
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{60.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Wider error tolerance for low-speed operation
    const speed_pu_t speed_err_margin{0.10f};

    auto speed_error_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "virtual startup speed error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_err_margin, speed_err_margin, yeet::checker_activation_always{}));

    speed_error_checker->deactivate();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();

    // Configure torque limits
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{-0.5});

    // Initial load
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.10f));

    bool success = true;

    // ==================== Trigger Startup Sequencer ====================
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Simulate to mid-ramp (~30s into 60s virtual frequency ramp)
    // At this point, virtual frequency should be around 0.10 pu
    success &= simulator.simulate_for(30.0);

    const auto speed_mid_ramp = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(speed_mid_ramp.value(), 0.08f) << "Virtual ramp should be progressing";
    EXPECT_LT(speed_mid_ramp.value(), 0.14f) << "Should not yet reach target during mid-ramp";

    // ==================== Load Step UP During Virtual Ramp ====================
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.20f));
    success &= simulator.simulate_for(2.0);  // Transient
    speed_error_checker->activate();
    success &= simulator.simulate_for(3.0);  // Steady-state validation
    speed_error_checker->deactivate();

    // ==================== Load Step DOWN During Virtual Ramp ====================
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.10f));
    success &= simulator.simulate_for(2.0);  // Transient
    speed_error_checker->activate();
    success &= simulator.simulate_for(3.0);  // Steady-state validation
    speed_error_checker->deactivate();

    // ==================== Complete Virtual Frequency Ramp ====================
    // Remaining time: ~60s total - 30s elapsed - 10s for load steps = ~20s remaining
    success &= simulator.simulate_for(21.0);

    // Verify startup sequencer completed and transitioned to closed-loop
    EXPECT_EQ(system.machine_side_control.startup_sequencer.get_outputs().current_state.read(),
        startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup sequencer should have transitioned to CLOSED_LOOP_OPERATION";

    const auto speed_after_virtual_ramp =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_virtual_ramp.value(), 0.20f, 0.02f) << "Virtual frequency ramp did not reach target speed";

    // ==================== Hold in Closed-Loop with Additional Load Step ====================
    speed_error_checker->activate();
    success &= simulator.simulate_for(5.0);  // Hold steady-state
    speed_error_checker->deactivate();

    // Final load step at target speed
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.18f));
    success &= simulator.simulate_for(2.0);
    speed_error_checker->activate();
    success &= simulator.simulate_for(3.0);
    speed_error_checker->deactivate();

    // ==================== Validation ====================
    EXPECT_TRUE(success) << "Virtual startup load rejection test did not complete successfully";

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulse releases should remain active during virtual startup and closed-loop operation";
}

/**
 * @test VirtualTorqueRampTest
 * @brief Tests virtual torque ramping during low-speed startup with high load
 *
 * This test validates the startup sequencer's virtual torque ramp functionality during
 * low-speed startup. The virtual torque ramp provides a similar effect to breakaway boost
 * by starting with higher torque to overcome static friction, then ramping down to normal
 * operating torque. This test validates startup from near-standstill with high friction.
 *
 * @par Test Sequence:
 * -# Configure startup sequencer with 15s virtual frequency ramp to 0.05 pu
 * -# Configure virtual torque ramp: 0.3 pu → 0.2 pu over 2 seconds (high initial torque)
 * -# Start from standstill simulating high static friction
 * -# Apply high load torque (0.20 pu) representing bearing friction
 * -# Execute state machine startup
 * -# Trigger startup sequencer
 * -# Monitor torque reference during first 2 seconds (should start high, ramp down)
 * -# Verify smooth acceleration despite high load
 * -# Continue to completion at 0.05 pu
 *
 * @par Validations:
 * - Torque reference starts elevated during first 2s (>0.25 pu)
 * - Torque reference ramps down to normal level (~0.20 pu) after 2s
 * - Speed reaches 0.05 pu ± 0.02 despite high load
 * - Speed ramp is smooth without stalling or oscillation
 * - Machine overcomes static friction and accelerates smoothly
 * - Transition to closed-loop operation is successful
 *
 * @par Key Features Tested:
 * - Virtual torque ramping for breakaway assistance
 * - Low-speed startup from near-standstill
 * - Load rejection with high static friction
 * - Virtual reference frame control at very low speeds
 */
TEST_F(CompleteSensorlessSpeedControllerTest, LowSpeedStartupWithBreakawayBoostTest)
{
    // ==================== Configure Startup Sequencer ====================
    // Virtual frequency: 0 Hz (0 pu) to 2.5 Hz (0.05 pu at 50 Hz base) in 15 seconds
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.05};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{15.0};

    // Virtual torque ramp: Start HIGH (0.3 pu) and ramp DOWN to normal (0.2 pu) over 2 seconds
    // This simulates breakaway boost effect - extra torque at startup, then reduced
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.3};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.2};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{2.0};

    // Pre-fluxing duration
    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Configure speed reference rate limiter
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{30.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Wider tolerance for low-speed ripple
    const speed_pu_t speed_err_margin{0.10f};

    auto speed_error_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "creep mode speed error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_err_margin, speed_err_margin, yeet::checker_activation_always{}));

    speed_error_checker->deactivate();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read());

    bool success = true;

    // Configure torque limits
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{-0.5});

    // High load simulating static friction
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.20f));

    // ==================== Trigger Startup Sequencer with Torque Monitoring ====================
    set_startup_sequencer_start_command(true);
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // ==================== Phase 1: High Torque (First ~2 seconds) ====================
    success &= simulator.simulate_for(1.0);

    // Verify elevated torque reference during initial phase
    auto torque_ref_high = system.machine_side_control.startup_sequencer.get_outputs().virtual_torque_reference.read();
    EXPECT_GT(torque_ref_high.value(), 0.25f)
        << "Torque reference should be elevated during initial startup (virtual torque ramp high value)";

    // Continue through torque ramp period
    success &= simulator.simulate_for(1.5);  // Total: 2.5s elapsed

    // ==================== Phase 2: Torque Ramped Down (After ~2 seconds) ====================
    auto torque_ref_after_ramp =
        system.machine_side_control.startup_sequencer.get_outputs().virtual_torque_reference.read();
    EXPECT_NEAR(torque_ref_after_ramp.value(), 0.20f, 0.05f)
        << "Torque reference should ramp down to normal operating value after virtual torque duration";

    // ==================== Phase 3: Complete Virtual Frequency Ramp ====================
    // Remaining time to reach 0.05 pu: ~15s total - 2.5s elapsed = ~12.5s + settling
    success &= simulator.simulate_for(13.5);

    // Verify startup sequencer completed
    EXPECT_EQ(system.machine_side_control.startup_sequencer.get_outputs().current_state.read(),
        startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup sequencer should have transitioned to CLOSED_LOOP_OPERATION";

    const auto speed_after_startup =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_startup.value(), 0.05f, 0.02f)
        << "Virtual frequency ramp should reach target despite high load with torque ramp assistance";

    // Verify smooth speed ramp (no stalling)
    EXPECT_GT(speed_after_startup.value(), 0.03f) << "Speed should have progressed significantly from standstill";

    // ==================== Steady-State Hold ====================
    speed_error_checker->activate();
    success &= simulator.simulate_for(3.0);
    speed_error_checker->deactivate();

    // ==================== Validation ====================
    EXPECT_TRUE(success) << "Low-speed startup with virtual torque ramp test did not complete successfully";

    // Verify machine started successfully from near-standstill
    EXPECT_GT(speed_after_startup.value(), 0.04f)
        << "Machine should overcome static friction and reach creep speed with torque ramp";

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulse releases should remain active during low-speed creep operation";
}

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
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{-0.5});

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
}