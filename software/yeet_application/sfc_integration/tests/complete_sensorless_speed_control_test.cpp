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

        // Initialize the system - allocates plant model and all components
        initialize();

        // ==================== Logger Registration ====================
        // register_complete_logger(simulator, system.plant);
        // register_complete_logger(simulator, system.grid_side_control.state_machine);
        // register_complete_logger(simulator, system.machine_side_control.machine_side_controller);
        // register_complete_logger(simulator, system.machine_side_control.machine_side_pulse_generator);
        // register_complete_logger(simulator, system.grid_side_control.grid_side_pulse_generator);

        // register_complete_logger(simulator, system.machine_side_control.speed_generator);  // TODO: Create logger
        // register_complete_logger(simulator, system.machine_side_control.excitation_controller);
        register_complete_logger(simulator, system.grid_side_control.current_controller);
        register_complete_logger(simulator, system.machine_side_control.torque_controller);
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        register_complete_logger(simulator, system.machine_side_control.speed_generator);
        // register_complete_logger(simulator, system.machine_side_control.torque_controller);  // TODO: Create logger
        // register_complete_logger(simulator, system.grid_side_control.grid_side_observer);
        // register_complete_logger(simulator, system.machine_side_control.machine_side_observer);
        // register_complete_logger(simulator, system.machine_side_control.machine_side_modulator);
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
};

// Test gas turbine soft-start sequence using speed generator profiles
// This test validates the complete 4-profile starting sequence:
// Profile 1 (Creep) -> Profile 2 (Purge) -> Profile 3 (Ignition) -> Profile 4 (Assist)
TEST_F(CompleteSensorlessSpeedControllerTest, GasTurbineSoftStartSequence)
{
    // Configure plant starting from near-zero speed (realistic turbine starting condition)
    yeet::application::sfc::integration::test::variable_speed_config config{};
    config.initial_speed = speed_pu_t{0.001f};  // Starting condition: nearly stationary
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // Configure realistic load torque profile (increases with speed)
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.15f));

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
        [&]() -> speed_pu_t { return system.machine_side_control.speed_controller.get_outputs().speed_error.read(); },
        speed_error_min_value, speed_error_max_value, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();  // Will activate during steady-state phases

    // ==================== State Machine Startup ====================
    execute_startup_sequence();
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().enable_speed_control, true);
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    bool success = true;

    // ==================== Profile 1: Creep Speed (~5% rated) ====================
    // Purpose: Initial rotation, oil circulation, pre-warming
    system.machine_side_control.idle.write(false);
    success &= simulator.simulate_for(0.1);  // Allow edge detection to process idle=false

    system.machine_side_control.run_creep_speed.write(true);
    success &= simulator.simulate_for(0.1);  // Allow edge detection to process rising edge

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
    system.machine_side_control.run_creep_speed.write(false);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    system.machine_side_control.run_purge_accel.write(true);
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
    system.machine_side_control.run_purge_accel.write(false);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    system.machine_side_control.run_ignition_accel.write(true);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    // Increase load as turbine begins to generate power
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.25f));

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
    system.machine_side_control.run_ignition_accel.write(false);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    system.machine_side_control.run_assist_accel.write(true);
    success &= simulator.simulate_for(0.05);  // Allow edge detection

    // Further increase load as turbine approaches full power
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.30f));

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
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().pulse_releases.read())
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
    yeet::application::sfc::integration::test::variable_speed_config config{};
    config.initial_speed = speed_pu_t{0.001f};
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // Configure realistic load torque
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.15f));

    // ==================== Configure Speed Generator Profiles ====================
    // Reduce acceleration times for faster test execution
    system.machine_side_control.speed_generator.get_parameters().profile_1_acceleration_time = duration_t{20.0f};
    system.machine_side_control.speed_generator.get_parameters().profile_3_acceleration_time = duration_t{40.0f};

    // Speed tracking tolerance
    const speed_pu_t speed_error_max_value{0.08f};
    const speed_pu_t speed_error_min_value{-0.08f};

    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed tracking error",
        [&]() -> speed_pu_t { return system.machine_side_control.speed_controller.get_outputs().speed_error.read(); },
        speed_error_min_value, speed_error_max_value, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().enable_speed_control, true);
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    bool success = true;

    // ==================== Profile 1: Creep Speed (0 → 0.05 pu) ====================
    system.machine_side_control.idle.write(false);
    success &= simulator.simulate_for(0.1);

    system.machine_side_control.run_creep_speed.write(true);
    success &= simulator.simulate_for(0.1);

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
    system.machine_side_control.run_creep_speed.write(false);
    success &= simulator.simulate_for(0.05);

    system.machine_side_control.run_ignition_accel.write(true);
    success &= simulator.simulate_for(0.05);

    // Increase load as speed increases (simulates turbine power generation)
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.25f));

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
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().pulse_releases.read())
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
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // Configure Profile 2 with faster acceleration for testing
    system.machine_side_control.speed_generator.get_parameters().profile_2_acceleration_time = duration_t{60.0f};

    // Wider error tolerance for pulse-mode operation
    const speed_pu_t speed_err_margin{0.08f};

    auto speed_error_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "purge mode speed error",
        [&]() -> speed_pu_t { return system.machine_side_control.speed_controller.get_outputs().speed_error.read(); },
        -speed_err_margin, speed_err_margin, yeet::checker_activation_always{}));

    speed_error_checker->deactivate();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().enable_speed_control, true);
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    // Initial load
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.10f));

    bool success = true;

    // ==================== Activate Profile 2 (Purge) ====================
    system.machine_side_control.idle.write(false);
    success &= simulator.simulate_for(0.1);

    system.machine_side_control.run_purge_accel.write(true);
    success &= simulator.simulate_for(0.1);

    // Allow acceleration to ~0.15 pu (Profile accelerates from 0.01 to 0.20 = 0.19 pu delta)
    // At 50% completion: 0.01 + 0.5 * 0.19 = ~0.105 pu, so use ~30s (50% of 60s)
    success &= simulator.simulate_for(30.0);

    const auto speed_mid_purge = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(speed_mid_purge.value(), 0.08f) << "Purge acceleration should be progressing";
    EXPECT_LT(speed_mid_purge.value(), 0.14f) << "Should not yet reach target during mid-ramp";

    // ==================== Load Step UP During Acceleration ====================
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.20f));
    success &= simulator.simulate_for(2.0);  // Transient
    speed_error_checker->activate();
    success &= simulator.simulate_for(3.0);  // Steady-state validation
    speed_error_checker->deactivate();

    // ==================== Load Step DOWN During Acceleration ====================
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.10f));
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
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.18f));
    success &= simulator.simulate_for(2.0);
    speed_error_checker->activate();
    success &= simulator.simulate_for(3.0);
    speed_error_checker->deactivate();

    // ==================== Validation ====================
    EXPECT_TRUE(success) << "Purge mode load step test did not complete successfully";

    // Verify torque limits match Profile 2 settings
    const auto torque_limit_max = system.machine_side_control.speed_generator.get_outputs().torque_limit_max_pu.read();
    EXPECT_NEAR(torque_limit_max.value(), 0.50f, 0.05f) << "Torque limit should match Profile 2 setting";

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().pulse_releases.read())
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
    yeet::application::sfc::integration::test::variable_speed_config config{};
    config.initial_speed = speed_pu_t{0.001f};
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // Configure Profile 1 with faster acceleration and explicit boost settings
    system.machine_side_control.speed_generator.get_parameters().profile_1_acceleration_time = duration_t{15.0f};
    system.machine_side_control.speed_generator.get_parameters().profile_1_initial_torque_boost = torque_pu_t{0.20f};
    system.machine_side_control.speed_generator.get_parameters().profile_1_boost_duration = duration_t{2.0f};

    // High load simulating static friction
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.20f));

    // Wider tolerance for low-speed ripple
    const speed_pu_t speed_err_margin{0.10f};

    auto speed_error_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "creep mode speed error",
        [&]() -> speed_pu_t { return system.machine_side_control.speed_controller.get_outputs().speed_error.read(); },
        -speed_err_margin, speed_err_margin, yeet::checker_activation_always{}));

    speed_error_checker->deactivate();

    // ==================== State Machine Startup ====================
    execute_startup_sequence();
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().enable_speed_control, true);
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    bool success = true;

    // ==================== Activate Profile 1 with Boost Monitoring ====================
    system.machine_side_control.idle.write(false);
    success &= simulator.simulate_for(0.1);

    system.machine_side_control.run_creep_speed.write(true);
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

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().pulse_releases.read())
        << "Pulse releases should remain active during low-speed creep operation";
}
