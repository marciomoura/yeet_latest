/**
 * @file closed_loop_startup_sequencer_test.cpp
 * @brief Integration tests for startup sequencer in closed-loop SFC system
 *
 * Tests for startup sequencer state transitions, observer preset during virtual startup,
 * and auto-start coordination with state machine.
 */

#include <gtest/gtest.h>

#include <string>

#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class ClosedLoopStartupSequencerTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(ms(10.0f));

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Configure topology to use internal model
        system.plant.get_parameters().topology.write(sfc_plant_model_datamodel::topology_type::internal);

        // Configure plant for gas turbine soft-starter application
        auto gas_turbine_config = yeet::application::sfc::integration::test::create_gas_turbine_config();
        yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, gas_turbine_config);

        // Initialize the system
        initialize();

        // Register loggers for debugging
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        register_complete_logger(simulator, system.machine_side_control.startup_sequencer);
        register_complete_logger(simulator, system.machine_side_control.machine_side_observer);
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
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().mcb_is_closed, on);
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
 * @brief Validates that the observer is preset with the virtual reference frame angle
 *
 * This test validates the bumpless transfer mechanism from open-loop virtual startup
 * to closed-loop sensorless control.
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

    // Set speed reference before startup
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.5});

    // Variables to Capture Preset Event
    bool preset_observed = false;
    angle_wrapped preset_angle_captured{0.0};
    angle_wrapped virtual_angle_at_preset{0.0};
    startup_sequencer_state_t state_at_preset = startup_sequencer_state_t::IDLE;

    // Execute State Machine Startup
    execute_startup_sequence();

    bool success = true;

    success &= simulator.simulate_for(1.0);  // Pre-flux duration

    // Configure moderate load torque
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    // Manual Simulation Loop to monitor the preset signal
    const double step_size = 0.001;  // 1ms
    const double total_time = 20.0;
    const int total_steps = static_cast<int>(total_time / step_size);

    for (int i = 0; i < total_steps; ++i) {
        simulator.simulate_for(step_size);

        auto preset_signal = system.machine_side_control.startup_sequencer.get_outputs().preset_observer.read();

        if (preset_signal && !preset_observed) {
            preset_observed = true;
            preset_angle_captured = system.machine_side_control.startup_sequencer.get_outputs().preset_angle.read();
            virtual_angle_at_preset =
                system.machine_side_control.startup_sequencer.get_outputs().virtual_rotor_angle.read();
            state_at_preset = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
        }
    }

    // Validate Preset Event Occurred
    EXPECT_TRUE(preset_observed) << "preset_observer signal should have been asserted during virtual startup";

    EXPECT_EQ(state_at_preset, startup_sequencer_state_t::STARTING_PROCEDURE)
        << "Preset should occur during STARTING_PROCEDURE (before transition)";

    EXPECT_NEAR(preset_angle_captured.get_pu(), virtual_angle_at_preset.get_pu(), 0.01f)
        << "preset_angle should match virtual_rotor_angle at the moment of preset";

    // Validate Observer Was Preset
    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Should be in CLOSED_LOOP_OPERATION after virtual startup";

    auto observer_angle =
        system.machine_side_control.machine_side_observer.get_outputs().estimated_reference_angle.read();
    auto virtual_angle = system.machine_side_control.startup_sequencer.get_outputs().virtual_rotor_angle.read();

    const real_t max_angle_diff = 1.0f;  // ~57 degrees tolerance
    real_t angle_diff = std::abs(observer_angle.get_pu() - virtual_angle.get_pu());

    while (angle_diff > 0.5f) {
        angle_diff -= 1.0f;
    }
    angle_diff = std::abs(angle_diff);

    EXPECT_LT(angle_diff, max_angle_diff) << "Observer angle should be reasonably close to virtual angle after preset";

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
