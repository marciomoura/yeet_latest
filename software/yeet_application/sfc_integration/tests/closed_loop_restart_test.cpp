/**
 * @file closed_loop_restart_test.cpp
 * @brief Integration tests for restart procedures in closed-loop SFC system
 *
 * Tests for restart capabilities including full restart after shutdown,
 * flying restart while machine is still spinning, and auto-start functionality.
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

class ClosedLoopRestartTest : public ::testing::Test {
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
        set_turn_on_command(false);
        set_start_command(false);
        set_stop_command(false);
        set_current_extinguished(false);

        system.initialize();
    }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // State machine control helpers
    void set_turn_on_command(bool on)
    {
        system.grid_side_control.state_machine.get_parameters().turn_on_command.write(on);
    }

    void set_start_command(bool start) { system.grid_io_mock.get_parameters().start_command.write(start); }

    void set_stop_command(bool stop) { system.grid_io_mock.get_parameters().stop_command.write(stop); }

    void set_current_extinguished(bool extinguished)
    {
        yeet::utils::mock_signal_input::write(
            system.grid_side_control.state_machine.get_inputs().is_current_extinguished, extinguished);
    }

    // Fault injection: Simulate breaker trip (for flying restart tests)
    // Uses override to force breaker status without going through state machine
    void force_breaker_trip()
    {
        system.grid_io_mock.get_parameters().override_mv_breaker_is_closed.write(true);
        system.grid_io_mock.get_parameters().override_mv_breaker_is_closed_value.write(false);
    }

    // Clear fault injection and restore normal IO mock operation
    void clear_breaker_override() { system.grid_io_mock.get_parameters().override_mv_breaker_is_closed.write(false); }

    // Complete startup sequence following state machine logic
    // Uses turn_on_command to trigger IO mock → auxiliaries → breakers flow
    void execute_startup_sequence()
    {
        set_turn_on_command(true);
        simulate_for(0.3);  // Allow IO mock to process auxiliaries and breakers with delays
        set_start_command(true);
        simulate_for(0.1);
    }

    // Configure and bring system to operational state
    void bring_to_operation()
    {
        // Configure Startup Sequencer
        system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
        system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.05};
        system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{3.0};

        system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
        system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.05};
        system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.5};

        system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

        // Configure speed reference
        system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.5});
        system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
            real_t{30.0f};
        system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

        // Light load
        system.plant.get_parameters().constant_torque.write(torque_pu_t(0.08f));

        // Execute startup
        execute_startup_sequence();

        // Wait for startup sequence to complete and reach operating speed
        simulator.simulate_for(25.0);  // Pre-flux + virtual startup + speed ramp
    }

    // System under test
    yeet::application::sfc::integration::plant_complete_sfc_closed_loop_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

/**
 * @test RestartCapabilityTest
 * @brief Validates system can perform a full restart after complete shutdown
 *
 * This test verifies that after a complete shutdown sequence, the system
 * can successfully restart and return to normal operation.
 */
TEST_F(ClosedLoopRestartTest, RestartCapabilityTest)
{
    // Configure Startup Sequencer
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.05};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{3.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.05};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.5};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Configure speed reference
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.4});
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{30.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Light load
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.08f));

    // First Startup Cycle
    execute_startup_sequence();

    bool success = true;
    success &= simulator.simulate_for(20.0);  // Complete first startup

    auto first_startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(first_startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "First startup should reach CLOSED_LOOP_OPERATION";

    auto first_cycle_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(first_cycle_speed.value(), 0.4f, 0.15f) << "First cycle should reach target speed";

    // Initiate Shutdown
    set_stop_command(true);
    success &= simulator.simulate_for(1.0);

    // Complete shutdown cycle
    set_current_extinguished(true);
    success &= simulator.simulate_for(0.5);

    auto shutdown_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(shutdown_state, startup_sequencer_state_t::IDLE) << "Startup sequencer should be IDLE after shutdown";

    // Prepare for restart - clear shutdown conditions
    set_stop_command(false);
    set_current_extinguished(false);
    set_start_command(false);
    success &= simulator.simulate_for(0.5);

    // Restart Sequence
    set_start_command(true);
    success &= simulator.simulate_for(20.0);  // Complete restart

    auto restart_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(restart_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Restart should reach CLOSED_LOOP_OPERATION";

    auto restart_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(restart_speed.value(), 0.4f, 0.15f) << "Restart should reach target speed";

    EXPECT_TRUE(success) << "Restart capability test should complete without errors";
}

/**
 * @test FlyingRestartCapabilityTest
 * @brief Validates restart capability with minimal coast time
 *
 * This test verifies that after a brief trip condition, the system can
 * successfully restart without needing a full cool-down period. While the
 * internal model doesn't simulate actual coast-down, this validates the
 * control system's restart capability.
 */
TEST_F(ClosedLoopRestartTest, FlyingRestartCapabilityTest)
{
    // Configure for faster response
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{2.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.5};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    // flying restart enabled
    system.machine_side_control.startup_sequencer.get_parameters().flying_restart_enabled = true;

    // Target speed for operation
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.6});
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{20.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Light load - allows long coast down
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.05f));

    // First Startup to Operating Speed
    execute_startup_sequence();

    bool success = true;
    success &= simulator.simulate_for(20.0);

    auto first_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(first_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "First startup should reach CLOSED_LOOP_OPERATION";

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(operating_speed.value(), 0.4f) << "Machine should be at significant speed";

    // Simulate a trip - breakers open unexpectedly (fault injection)
    force_breaker_trip();
    success &= simulator.simulate_for(0.2);

    // Verify pulses blocked
    auto pulses_blocked = !system.grid_side_control.state_machine.get_outputs().release_pulses.read();
    EXPECT_TRUE(pulses_blocked) << "Pulses should be blocked after breaker trip";

    // Brief pause before restart (no coast simulation in internal model)
    success &= simulator.simulate_for(0.5);

    // Flying Restart - clear fault and let IO mock restore breaker status
    clear_breaker_override();
    set_start_command(true);
    success &= simulator.simulate_for(0.3);  // Allow IO mock to process breaker close with delay

    // Verify pulses released for restart
    auto pulses_released = system.grid_side_control.state_machine.get_outputs().release_pulses.read();
    EXPECT_TRUE(pulses_released) << "Pulses should be released for flying restart";

    // Continue to re-accelerate
    success &= simulator.simulate_for(18.0);

    // Verify system returned to operation
    auto restart_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(restart_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Flying restart should reach CLOSED_LOOP_OPERATION";

    auto restart_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(restart_speed.value(), 0.4f) << "Flying restart should return to operating speed";

    EXPECT_TRUE(success) << "Flying restart capability test should complete without errors";
}

/**
 * @test FlyingRestartWithAutoStartTest
 * @brief Validates flying restart with auto-start feature enabled
 *
 * When auto-start is enabled and the breakers are closed, the system should
 * automatically initiate a restart sequence without requiring a new start command.
 */
TEST_F(ClosedLoopRestartTest, FlyingRestartWithAutoStartTest)
{
    // Configure for faster response
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{2.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.5};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{0.5f};

    // flying restart enabled
    system.machine_side_control.startup_sequencer.get_parameters().flying_restart_enabled = true;

    // Target speed for operation
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.5});
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{25.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Light load
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.05f));

    // First Startup
    execute_startup_sequence();

    bool success = true;
    success &= simulator.simulate_for(18.0);

    auto first_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(first_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "First startup should reach CLOSED_LOOP_OPERATION";

    // Simulate a temporary trip - breakers open (fault injection)
    force_breaker_trip();
    success &= simulator.simulate_for(0.5);

    // Verify shutdown
    auto pulses_blocked = !system.grid_side_control.state_machine.get_outputs().release_pulses.read();
    EXPECT_TRUE(pulses_blocked) << "Pulses should be blocked after breaker trip";

    // Auto-restart scenario: start_command remains true, clear fault to restore breaker
    // (simulating auto-reclose or operator re-engagement)
    clear_breaker_override();
    // Note: start_command is already true from initial startup
    success &= simulator.simulate_for(0.3);  // Allow IO mock to process breaker close with delay

    // Verify auto-restart initiated
    auto pulses_released = system.grid_side_control.state_machine.get_outputs().release_pulses.read();
    EXPECT_TRUE(pulses_released) << "Pulses should be released for auto-restart";

    // Continue to full operation
    success &= simulator.simulate_for(15.0);

    auto restart_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(restart_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Auto-restart should reach CLOSED_LOOP_OPERATION";

    auto restart_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(restart_speed.value(), 0.35f) << "Auto-restart should return to operating speed";

    EXPECT_TRUE(success) << "Flying restart with auto-start test should complete without errors";
}
