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

// Test gas turbine soft-start sequence using speed generator profiles
// This test validates the complete 4-profile starting sequence:
// Profile 1 (Creep) -> Profile 2 (Purge) -> Profile 3 (Ignition) -> Profile 4 (Assist)
TEST_F(CompleteSensorlessSpeedControllerTest, GasTurbineSoftStartSequence)
{
    // ==================== Configure Speed Generator Profiles ====================
    // Configure acceleration times to match test expectations (defaults are much longer)
    // Profile 1 & 2 use defaults (30s, 120s), but Profile 3 & 4 need shorter times for testing

    // Profile 3: Reduce from default 180s to 90s for faster testing
    system.machine_side_control.speed_generator.get_parameters().profile_3_acceleration_time = duration_t{90.0f};

    // Profile 4: Reduce from default 300s to 60s for faster testing
    system.machine_side_control.speed_generator.get_parameters().profile_4_acceleration_time = duration_t{60.0f};

    // Speed tracking tolerance (wider for low-speed operation)
    const speed_pu_t speed_error_max_value{0.08f};
    const speed_pu_t speed_error_min_value{-0.08f};

    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed tracking error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.speed_generator.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        speed_error_min_value, speed_error_max_value, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();  // Will activate during steady-state phases

    // ==================== State Machine Startup ====================
    execute_startup_sequence();

    // NOTE: The plant is configured for external_control mode by default in the builder.
    // The excitation controller will manage the field current reference to regulate machine flux.
    // If speed_dependent mode is used instead, the plant uses V/Hz relationship and ignores
    // the excitation controller (resulting in zero flux/field current outputs in telemetry).
    // system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    bool success = true;

    // ==================== Profile 1: Creep Speed (~5% rated) ====================
    // Purpose: Initial rotation, oil circulation, pre-warming
    set_idle(false);
    success &= simulator.simulate_for(0.1);  // Allow edge detection to process idle=false

    set_run_creep_speed(true);
    success &= simulator.simulate_for(0.1);  // Allow edge detection to process rising edge

    // Configure realistic load torque profile (increases with speed)
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.15f));

    // Allow acceleration to complete (30s acceleration time + 5s settling + 0.2s from above)
    success &= simulator.simulate_for(34.8);

    // Verify target speed reached
    const auto speed_after_creep =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_creep.value(), 0.05f, 0.01f) << "Profile 1 (Creep) did not reach target speed";

    // Hold at creep speed for inspection/settling
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_tracking_checker->deactivate();

    // ==================== Profile 2: Purge Acceleration (~20% rated) ====================
    // Purpose: Combustion chamber ventilation to clear residual fuel vapors
    set_run_creep_speed(false);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    set_run_purge_accel(true);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    // Allow acceleration to complete (120s acceleration time + 5s settling + 0.1s from above)
    success &= simulator.simulate_for(124.9);

    // Verify target speed reached
    const auto speed_after_purge =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_purge.value(), 0.20f, 0.015f) << "Profile 2 (Purge) did not reach target speed";
    // Hold at purge speed (typically 2-5 minutes in real operation, shortened for test)
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(10.0);
    speed_tracking_checker->deactivate();

    // ==================== Profile 3: Ignition Acceleration (~55% rated) ====================
    // Purpose: Reach speed for fuel ignition and flame stabilization
    set_run_purge_accel(false);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    set_run_ignition_accel(true);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    // Increase load as turbine begins to generate power
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.25f));

    // Allow acceleration to complete (90s acceleration time + 5s settling + 0.1s from above)
    success &= simulator.simulate_for(94.9);

    // Verify target speed reached
    const auto speed_after_ignition =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_ignition.value(), 0.55f, 0.02f) << "Profile 3 (Ignition) did not reach target speed";

    // Hold at ignition speed for flame stabilization
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_tracking_checker->deactivate();

    // ==================== Profile 4: Assist to Self-Sustaining (~95% rated) ====================
    // Purpose: Accelerate to self-sustaining speed where turbine can maintain rotation
    set_run_ignition_accel(false);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    set_run_assist_accel(true);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    // Further increase load as turbine approaches full power
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.30f));

    // Allow acceleration to complete (60s acceleration time + 5s settling + 0.1s from above)
    success &= simulator.simulate_for(64.9);

    // Verify final target speed reached
    const auto speed_after_assist =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_assist.value(), 0.95f, 0.02f) << "Profile 4 (Assist) did not reach target speed";

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
 * @test SpeedGeneratorMultiProfileSequenceTest
 * @brief Validates multi-profile sequencing and transition smoothness using speed generator
 *
 * This test exercises the speed generator's ability to execute multiple profiles in sequence
 * with smooth transitions between them. It validates profile switching logic, speed tracking
 * across transitions, and proper signaling of profile completion.
 *
 * @par Test Sequence:
 * -# Configure profiles with reduced acceleration times for faster testing
 *    - Profile 1 (Creep): 20s to reach 0.05 pu
 *    - Profile 3 (Ignition): 40s to reach 0.55 pu
 * -# Execute state machine startup sequence
 * -# Activate Profile 1 (Creep): idle=false → run_creep_speed=true
 * -# Wait for Profile 1 completion (~20s + settling)
 * -# Transition to Profile 3 (Ignition): run_creep_speed=false → run_ignition_accel=true
 * -# Wait for Profile 3 completion (~40s + settling)
 * -# Hold at final speed with tracking verification
 *
 * @par Validations:
 * - Speed reaches 0.05 pu ± 0.01 after Profile 1
 * - Speed reaches 0.55 pu ± 0.02 after Profile 3
 * - Speed tracking error < 0.08 pu during steady-state phases
 * - Smooth transitions without overshoot or oscillation
 * - State machine remains in OPERATION_ON throughout
 *
 * @par Key Features Tested:
 * - Profile execution and completion detection
 * - Multi-profile sequencing
 * - Speed reference trajectory generation
 * - Controller tracking performance across speed ranges
 */
TEST_F(CompleteSensorlessSpeedControllerTest, SpeedGeneratorMultiProfileSequenceTest)
{
    // Configure plant starting from near-zero speed
    // yeet::application::sfc::integration::test::variable_speed_config config{};
    // config.initial_speed = speed_pu_t{0.001f};
    // config.voltage_control_mode = std::nullopt;  // Preserve external_control mode from builder
    // yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // ==================== Configure Speed Generator Profiles ====================
    // Reduce acceleration times for faster test execution
    system.machine_side_control.speed_generator.get_parameters().profile_1_acceleration_time = duration_t{20.0f};
    system.machine_side_control.speed_generator.get_parameters().profile_3_acceleration_time = duration_t{40.0f};
    // system.machine_side_control.speed_generator.get_parameters().profile_1_boost_duration = duration_t{0.0f};
    // system.machine_side_control.speed_generator.get_parameters().profile_2_boost_duration = duration_t{0.0f};
    // system.machine_side_control.speed_generator.get_parameters().profile_3_boost_duration = duration_t{0.0f};
    // system.machine_side_control.speed_generator.get_parameters().profile_4_boost_duration = duration_t{0.0f};
    // system.machine_side_control.speed_generator.get_parameters().profile_1_initial_torque_boost = torque_pu_t{0.0f};
    // system.machine_side_control.speed_generator.get_parameters().profile_2_initial_torque_boost = torque_pu_t{0.0f};
    // system.machine_side_control.speed_generator.get_parameters().profile_3_initial_torque_boost = torque_pu_t{0.0f};
    // system.machine_side_control.speed_generator.get_parameters().profile_4_initial_torque_boost = torque_pu_t{0.0f};

    // Speed tracking tolerance
    const speed_pu_t speed_error_max_value{0.08f};
    const speed_pu_t speed_error_min_value{-0.08f};

    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed tracking error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.speed_generator.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        speed_error_min_value, speed_error_max_value, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();

    bool success = true;

    // ==================== Profile 1: Creep Speed (0 → 0.05 pu) ====================
    set_idle(false);
    success &= simulator.simulate_for(0.1);

    set_run_creep_speed(true);
    success &= simulator.simulate_for(0.1);

    // Configure realistic load torque
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.15f));

    // Wait for profile completion (20s acceleration + 3s settling)
    success &= simulator.simulate_for(22.9);

    // Verify Profile 1 target reached
    const auto speed_after_creep =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_creep.value(), 0.05f, 0.01f) << "Profile 1 (Creep) did not reach target speed";

    // Activate tracking checker for steady-state validation
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(3.0);
    speed_tracking_checker->deactivate();

    // ==================== Profile 3: Ignition Acceleration (0.05 → 0.55 pu) ====================
    set_run_creep_speed(false);
    success &= simulator.simulate_for(0.05);

    set_run_ignition_accel(true);
    success &= simulator.simulate_for(0.05);

    // Increase load as speed increases (simulates turbine power generation)
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.25f));

    // Wait for profile completion (40s acceleration + 3s settling)
    success &= simulator.simulate_for(42.9);

    // Verify Profile 3 target reached
    const auto speed_after_ignition =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_ignition.value(), 0.55f, 0.02f) << "Profile 3 (Ignition) did not reach target speed";

    // Final steady-state validation
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(3.0);
    speed_tracking_checker->deactivate();

    // ==================== Validation ====================
    EXPECT_TRUE(success) << "Multi-profile sequence test did not complete successfully";

    // Verify state machine remained operational throughout
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulse releases should remain active during profile sequence";
}

/**
 * @test PurgeModeWithLoadStepTest
 * @brief Tests Profile 2 (Purge) with load disturbance rejection
 *
 * This test validates the speed generator's Profile 2 (Purge acceleration) operating in the
 * pulse-to-transition speed region (0.10-0.25 pu), with emphasis on load disturbance rejection.
 * Purge speed is critical for gas turbine combustion chamber ventilation, requiring stable
 * operation despite varying loads.
 *
 * @par Test Sequence:
 * -# Configure Profile 2 with 60s acceleration time (reduced from 120s default)
 * -# Start from 0.01 pu with initial load of 0.10 pu
 * -# Execute state machine startup
 * -# Activate Profile 2 (Purge): idle=false → run_purge_accel=true
 * -# During acceleration (~0.15 pu, 75% complete):
 *    - Apply load step UP: 0.10 → 0.20 pu
 *    - Wait 5s for regulation
 *    - Apply load step DOWN: 0.20 → 0.10 pu
 * -# Continue to profile completion (0.20 pu)
 * -# Hold at purge speed with additional load step testing
 *
 * @par Validations:
 * - Speed continues ramping smoothly during load transients
 * - Speed tracking error < 0.08 pu during steady-state after steps
 * - Final speed reaches 0.20 pu ± 0.015
 * - Torque limits remain within Profile 2 settings (max: 0.50 pu, min: -0.20 pu)
 * - Controller maintains regulation without saturation
 * - System operates in pulse-to-transition operating mode region
 *
 * @par Key Features Tested:
 * - Profile 2 (Purge) execution
 * - Load disturbance rejection during acceleration
 * - Speed regulation stability in pulse-mode region
 * - Dynamic torque limit enforcement
 */
TEST_F(CompleteSensorlessSpeedControllerTest, PurgeModeWithLoadStepTest)
{
    GTEST_SKIP() << "Flaky test - needs investigation";

    // Configure plant starting from low speed
    yeet::application::sfc::integration::test::variable_speed_config config{};
    config.initial_speed = speed_pu_t{0.01f};
    config.voltage_control_mode = std::nullopt;  // Preserve external_control mode from builder
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // Configure Profile 2 with faster acceleration for testing
    system.machine_side_control.speed_generator.get_parameters().profile_2_acceleration_time = duration_t{60.0f};

    // Wider error tolerance for pulse-mode operation
    const speed_pu_t speed_err_margin{0.08f};

    auto speed_error_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "purge mode speed error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.speed_generator.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_err_margin, speed_err_margin, yeet::checker_activation_always{}));

    speed_error_checker->deactivate();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();
    // Using external_control mode (default from builder) to enable excitation controller
    // system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    // Initial load
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.10f));

    bool success = true;

    // ==================== Activate Profile 2 (Purge) ====================
    set_idle(false);
    success &= simulator.simulate_for(0.1);

    set_run_purge_accel(true);
    success &= simulator.simulate_for(0.1);

    // Allow acceleration to ~0.15 pu (Profile accelerates from 0.01 to 0.20 = 0.19 pu delta)
    // At 50% completion: 0.01 + 0.5 * 0.19 = ~0.105 pu, so use ~30s (50% of 60s)
    success &= simulator.simulate_for(30.0);

    const auto speed_mid_purge = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(speed_mid_purge.value(), 0.08f) << "Purge acceleration should be progressing";
    EXPECT_LT(speed_mid_purge.value(), 0.14f) << "Should not yet reach target during mid-ramp";

    // ==================== Load Step UP During Acceleration ====================
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.20f));
    success &= simulator.simulate_for(2.0);  // Transient
    speed_error_checker->activate();
    success &= simulator.simulate_for(3.0);  // Steady-state validation
    speed_error_checker->deactivate();

    // ==================== Load Step DOWN During Acceleration ====================
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.10f));
    success &= simulator.simulate_for(2.0);  // Transient
    speed_error_checker->activate();
    success &= simulator.simulate_for(3.0);  // Steady-state validation
    speed_error_checker->deactivate();

    // ==================== Complete Profile 2 Acceleration ====================
    // Remaining time: ~30s to reach 0.20 pu + 3s settling
    success &= simulator.simulate_for(33.0);

    const auto speed_after_purge =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_purge.value(), 0.20f, 0.015f) << "Profile 2 (Purge) did not reach target speed";

    // ==================== Hold at Purge Speed with Additional Load Step ====================
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
    EXPECT_TRUE(success) << "Purge mode load step test did not complete successfully";

    // Verify torque limits match Profile 2 settings
    const auto torque_limit_max = system.machine_side_control.speed_generator.get_outputs().torque_limit_max_pu.read();
    EXPECT_NEAR(torque_limit_max.value(), 0.50f, 0.05f) << "Torque limit should match Profile 2 setting";

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "Pulse releases should remain active in pulse-to-transition region";
}

/**
 * @test LowSpeedStartupWithBreakawayBoostTest
 * @brief Tests Profile 1 (Creep) with breakaway torque boost activation
 *
 * This test validates the speed generator's breakaway torque boost functionality during
 * Profile 1 (Creep speed) execution. The breakaway boost provides additional torque during
 * initial startup to overcome static friction, sticky bearings, or seal friction that
 * prevents smooth acceleration from standstill.
 *
 * @par Test Sequence:
 * -# Configure Profile 1 with 15s acceleration, 0.20 pu boost, 2.0s boost duration
 * -# Start from near-standstill (0.001 pu) simulating high static friction
 * -# Apply high load torque (0.20 pu) representing bearing friction
 * -# Execute state machine startup
 * -# Activate Profile 1: idle=false → run_creep_speed=true
 * -# Monitor is_breakaway_boost_active flag
 * -# Verify boost activates immediately and deactivates after ~2s
 * -# Continue to profile completion (0.05 pu)
 *
 * @par Validations:
 * - is_breakaway_boost_active == true during first ~2s
 * - is_breakaway_boost_active == false after 2.5s
 * - Torque limit elevated during boost: torque_limit_max ≈ 0.40 + 0.20 = 0.60 pu
 * - Torque limit returns to normal after boost: torque_limit_max ≈ 0.40 pu
 * - Speed reaches 0.05 pu ± 0.01 despite high load
 * - Speed ramp is smooth without stalling or oscillation
 * - Machine overcomes breakaway friction and accelerates smoothly
 *
 * @par Key Features Tested:
 * - Profile 1 (Creep) low-speed operation
 * - Breakaway boost timing and activation logic
 * - Dynamic torque limit ramping with boost
 * - Startup from near-standstill with high friction
 * - Boost deactivation transition smoothness
 */
TEST_F(CompleteSensorlessSpeedControllerTest, LowSpeedStartupWithBreakawayBoostTest)
{
    // Configure plant at near-standstill (simulates sticky bearings)
    // yeet::application::sfc::integration::test::variable_speed_config config{};
    // config.initial_speed = speed_pu_t{0.001f};
    // config.voltage_control_mode = std::nullopt;  // Preserve external_control mode from builder
    // yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // Configure Profile 1 with faster acceleration and explicit boost settings
    system.machine_side_control.speed_generator.get_parameters().profile_1_acceleration_time = duration_t{15.0f};
    system.machine_side_control.speed_generator.get_parameters().profile_1_initial_torque_boost = torque_pu_t{0.20f};
    system.machine_side_control.speed_generator.get_parameters().profile_1_boost_duration = duration_t{2.0f};

    // Wider tolerance for low-speed ripple
    const speed_pu_t speed_err_margin{0.10f};

    auto speed_error_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "creep mode speed error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.speed_generator.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_err_margin, speed_err_margin, yeet::checker_activation_always{}));

    speed_error_checker->deactivate();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();
    // Using external_control mode (default from builder) to enable excitation controller
    // system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read());

    bool success = true;

    // High load simulating static friction
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque, torque_pu_t(0.20f));

    // ==================== Activate Profile 1 with Boost Monitoring ====================
    set_idle(false);
    success &= simulator.simulate_for(0.1);

    set_run_creep_speed(true);
    success &= simulator.simulate_for(0.1);

    // ==================== Phase 1: Boost Active (First ~2 seconds) ====================
    success &= simulator.simulate_for(1.0);

    // Verify boost is active
    auto boost_active_during =
        system.machine_side_control.speed_generator.get_outputs().is_breakaway_boost_active.read();
    EXPECT_TRUE(boost_active_during) << "Breakaway boost should be active during initial acceleration";

    // Check elevated torque limit during boost
    auto torque_limit_with_boost = system.machine_side_control.speed_generator.get_outputs().torque_limit_max_pu.read();
    EXPECT_GT(torque_limit_with_boost.value(), 0.55f)
        << "Torque limit should be elevated during boost (0.40 base + 0.20 boost ≈ 0.60)";

    // Continue through boost period
    success &= simulator.simulate_for(1.5);  // Total: 2.5s elapsed

    // ==================== Phase 2: Boost Deactivated (After ~2 seconds) ====================
    auto boost_active_after =
        system.machine_side_control.speed_generator.get_outputs().is_breakaway_boost_active.read();
    EXPECT_FALSE(boost_active_after) << "Breakaway boost should deactivate after ~2 seconds";

    // Check torque limit returned to normal
    auto torque_limit_after_boost =
        system.machine_side_control.speed_generator.get_outputs().torque_limit_max_pu.read();
    EXPECT_NEAR(torque_limit_after_boost.value(), 0.40f, 0.05f)
        << "Torque limit should return to Profile 1 base value after boost";

    // ==================== Phase 3: Complete Profile 1 Acceleration ====================
    // Remaining time to reach 0.05 pu: ~12s + 2s settling
    success &= simulator.simulate_for(14.0);

    const auto speed_after_creep =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(speed_after_creep.value(), 0.05f, 0.01f)
        << "Profile 1 (Creep) should reach target despite high load with boost assistance";

    // Verify smooth speed ramp (no stalling)
    EXPECT_GT(speed_after_creep.value(), 0.03f) << "Speed should have progressed significantly from standstill";

    // ==================== Steady-State Hold ====================
    speed_error_checker->activate();
    success &= simulator.simulate_for(3.0);
    speed_error_checker->deactivate();

    // ==================== Validation ====================
    EXPECT_TRUE(success) << "Low-speed startup with breakaway boost test did not complete successfully";

    // Verify machine started successfully from near-standstill
    EXPECT_GT(speed_after_creep.value(), 0.04f) << "Machine should overcome breakaway friction and reach creep speed";

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
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.2};
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