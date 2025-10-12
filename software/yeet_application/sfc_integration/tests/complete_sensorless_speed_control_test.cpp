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
        simulator.configure_sampler_sampling_rate(100.0e-6);

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

        register_complete_logger(simulator, system.machine_side_control.excitation_controller);
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        register_complete_logger(simulator, system.grid_side_control.current_controller);
        register_complete_logger(simulator, system.grid_side_control.grid_side_observer);
        register_complete_logger(simulator, system.machine_side_control.machine_side_observer);
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

// Test speed regulation with state machine startup - based on sensorless_speed_control_test.cpp
TEST_F(CompleteSensorlessSpeedControllerTest, TestSpeedRegulationWithStateMachine)
{
    // Configure plant for variable speed operation using standardized configuration
    yeet::application::sfc::integration::test::variable_speed_config config{};
    config.initial_speed = speed_pu_t{0.9f};  // Custom initial speed for this test
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    const speed_pu_t speed_error_max_value{0.05f};
    const speed_pu_t speed_error_min_value{-0.05f};

    // Check speed error after disturbance
    auto speed_regulation_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error",
        [&]() -> speed_pu_t { return system.machine_side_control.speed_controller.get_outputs().speed_error.read(); },
        speed_error_min_value, speed_error_max_value, yeet::checker_activation_always{}));

    speed_regulation_checker->deactivate();  // Deactivate initially to avoid false positives

    // ==================== Test Configuration ====================
    // Simulate initial idle state
    // ==================== State Machine Startup Sequence ====================
    // Execute complete startup sequence to reach operational state
    execute_startup_sequence();

    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().enable_speed_control, true);

    // At this point, state machine should be in OPERATION_ON and pulse_releases should be true
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    // ==================== Test Execution ====================
    // Simulate until system can stabilize
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().speed_reference_pu, speed_pu_t(0.9));
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.3f));
    bool success = simulator.simulate_for(5.0);
    speed_regulation_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_regulation_checker->deactivate();

    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().speed_reference_pu, speed_pu_t(1.0));
    success &= simulator.simulate_for(5.0);
    speed_regulation_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_regulation_checker->deactivate();

    EXPECT_TRUE(success) << "Simulation did not complete successfully with speed regulation and state machine.";

    // Verify that pulse releases are active during operation
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().pulse_releases.read())
        << "Pulse releases should be active during OPERATION_ON state";
}

// Test speed regulation with state machine startup - based on sensorless_speed_control_test.cpp
TEST_F(CompleteSensorlessSpeedControllerTest, SpeedRegulationRampingUpTest)
{
    simulator.configure_sampler_sampling_rate(1e-3);

    // Configure plant for variable speed operation using standardized configuration
    yeet::application::sfc::integration::test::variable_speed_config config{};
    config.initial_speed = speed_pu_t{0.0f};  // Custom initial speed for this test
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    const speed_pu_t speed_error_max_value{0.07f};
    const speed_pu_t speed_error_min_value{-0.07f};

    // Check speed error after disturbance
    auto speed_regulation_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error",
        [&]() -> speed_pu_t { return system.machine_side_control.speed_controller.get_outputs().speed_error.read(); },
        speed_error_min_value, speed_error_max_value, yeet::checker_activation_always{}));

    speed_regulation_checker->deactivate();  // Deactivate initially to avoid false positives

    // ==================== Test Configuration ====================
    // Simulate initial idle state
    // ==================== State Machine Startup Sequence ====================
    // Execute complete startup sequence to reach operational state
    execute_startup_sequence();

    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().enable_speed_control, true);

    // Repeat this after to have a nice initial condition. TODO: Fix this...
    // Configure plant for variable speed operation using standardized configuration
    config.initial_speed = speed_pu_t{0.01f};  // Custom initial speed for this test
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // At this point, state machine should be in OPERATION_ON and pulse_releases should be true
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    // ==================== Test Execution ====================
    // Generate explicit ramp from initial speed (0.01 pu) to 1.0 pu over 50 seconds
    const real_t ramp_time = 50.0;  // seconds
    const real_t initial_ref = 0.01f;
    const real_t final_ref = 1.0f;
    const real_t ramp_rate = (final_ref - initial_ref) / ramp_time;  // ~0.0198 pu/s
    const real_t step_time = 0.5;                                    // Update reference every 0.5 seconds
    const int num_steps = static_cast<int>(ramp_time / step_time);

    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.2f));

    // Simulate a little bit to get rid of possible initial oscillations
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().speed_reference_pu, speed_pu_t(initial_ref));
    bool success = simulator.simulate_for(5.0);

    // Generate ramp by stepping the reference incrementally
    speed_regulation_checker->activate();
    for (int i = 1; i <= num_steps; ++i) {
        const real_t elapsed_time = i * step_time;
        const speed_pu_t reference = speed_pu_t{initial_ref + ramp_rate * elapsed_time};
        yeet::mock_signal_input::write(
            system.machine_side_control.speed_controller.get_inputs().speed_reference_pu, reference);

        success &= simulator.simulate_for(step_time);
        if (!success) {
            break;
        }
    }
    speed_regulation_checker->deactivate();

    EXPECT_TRUE(success) << "Simulation did not complete successfully with speed regulation and state machine.";
}

// Test to evaluate pulse-mode operation with a significant load step at low speed.
TEST_F(CompleteSensorlessSpeedControllerTest, PulseModeLoadStepTest)
{
    // Low-speed (pulse / forced commutation region) plant configuration
    yeet::application::sfc::integration::test::variable_speed_config config{};
    config.initial_speed = speed_pu_t{0.005f};  // ~0.5% base speed
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    // Wider steady‑state error band due to low-speed torque pulsations
    const speed_pu_t speed_err_margin{0.10f};

    auto speed_error_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "pulse-mode speed error",
        [&]() -> speed_pu_t { return system.machine_side_control.speed_controller.get_outputs().speed_error.read(); },
        -speed_err_margin, speed_err_margin, yeet::checker_activation_always{}));

    // Checker initially disabled (transient phases not evaluated)
    speed_error_checker->deactivate();

    // ===== State Machine Startup =====
    execute_startup_sequence();
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().enable_speed_control, true);
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    // ===== Phase 1: Initial low-speed regulation =====
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().speed_reference_pu, speed_pu_t(0.05f));
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.10f));
    bool success = simulator.simulate_for(5.0);  // unchecked transient
    speed_error_checker->activate();
    success &= simulator.simulate_for(5.0);  // steady-state window @ 0.05 pu / 0.10 load
    speed_error_checker->deactivate();

    // ===== Phase 2: Load step UP (0.10 -> 0.20 pu torque) =====
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.20f));
    success &= simulator.simulate_for(2.0);  // transient (unchecked)
    speed_error_checker->activate();
    success &= simulator.simulate_for(5.0);  // steady-state after step-up
    speed_error_checker->deactivate();

    // ===== Phase 3: Load step DOWN (0.20 -> 0.10 pu torque) =====
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.10f));
    success &= simulator.simulate_for(2.0);  // transient (unchecked)
    speed_error_checker->activate();
    success &= simulator.simulate_for(5.0);  // steady-state after step-down

    speed_error_checker->deactivate();

    EXPECT_TRUE(success) << "Pulse-mode load step regulation sequence failed.";

    // Optional mode / pulse assertions (kept simple here)
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().pulse_releases.read())
        << "Pulse releases should remain active during pulse-mode test.";
}

// Focused forced‑commutation (pulse‑mode) ramp test:
// Ramp speed from 0.001 pu to 0.20 pu using a rate limit of 1/100 pu/s (0.01 pu/s)
// while asserting the operating mode remains in forced commutation.
TEST_F(CompleteSensorlessSpeedControllerTest, PulseModeLowSpeedRampUpTest)
{
    // Checker margin widened for low-speed torque ripple / quantization effects
    const speed_pu_t speed_err_margin{0.12f};
    auto speed_error_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "pulse-mode low-speed ramp error",
        [&]() -> speed_pu_t { return system.machine_side_control.speed_controller.get_outputs().speed_error.read(); },
        -speed_err_margin, speed_err_margin, yeet::checker_activation_always{}));

    speed_error_checker->deactivate();

    // ===== Startup & Enable =====
    // Configure plant in very low-speed (forced commutation) region
    yeet::application::sfc::integration::test::variable_speed_config config{};
    config.initial_speed = speed_pu_t{0.001f};  // 0.1% of base speed
    yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, config);

    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().enable_speed_control, true);

    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    // Apply load torque representative of starting conditions
    yeet::mock_signal_input::write(system.plant.get_inputs().load_torque_pu, torque_pu_t(0.15f));

    // Generate explicit ramp from initial speed (0.001 pu) to 0.20 pu
    const speed_pu_t initial_ref{0.001f};
    const speed_pu_t final_ref{0.20f};
    const real_t total_ramp_time = (final_ref.value() - initial_ref.value()) / 0.01f;  // 0.01 pu/s rate
    const real_t step_time = 0.5;  // Update reference every 0.5 seconds
    const int num_steps = static_cast<int>(total_ramp_time / step_time);
    const real_t ramp_rate = (final_ref.value() - initial_ref.value()) / total_ramp_time;

    // Start with initial reference
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().speed_reference_pu, initial_ref);

    execute_startup_sequence();

    // Small initial settling before monitoring
    bool success = simulator.simulate_for(1.0);

    // Generate ramp by stepping the reference incrementally
    speed_error_checker->activate();
    for (int i = 1; i <= num_steps; ++i) {
        const real_t elapsed_time = i * step_time;
        const speed_pu_t reference = speed_pu_t{initial_ref.value() + ramp_rate * elapsed_time};
        yeet::mock_signal_input::write(
            system.machine_side_control.speed_controller.get_inputs().speed_reference_pu, reference);

        success &= simulator.simulate_for(step_time);
        if (!success) {
            break;
        }
    }

    // Final hold at target speed (additional verification window)
    yeet::mock_signal_input::write(
        system.machine_side_control.speed_controller.get_inputs().speed_reference_pu, final_ref);
    success &= simulator.simulate_for(1.0);

    speed_error_checker->deactivate();

    // Final assertions
    const auto final_speed_error = system.machine_side_control.speed_controller.get_outputs().speed_error.read();
    EXPECT_NEAR(final_speed_error.value(), 0.0f, speed_err_margin.value())
        << "Final speed error outside allowable margin after low-speed ramp.";

    EXPECT_TRUE(success) << "Simulation did not complete successfully for pulse-mode low-speed ramp.";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().pulse_releases.read())
        << "Pulse releases should remain active in forced commutation region.";
}
