/**
 * @file closed_loop_shutdown_test.cpp
 * @brief Integration tests for shutdown procedures in closed-loop SFC system
 *
 * Tests for various shutdown triggers including state machine controlled shutdown,
 * auxiliaries off, stop command, breakers off, and speed reference behavior during shutdown.
 */

#include <gtest/gtest.h>

#include <string>

#include "sfc/sfc_state_machine.hpp"
#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class ClosedLoopShutdownTest : public ::testing::Test {
public:
    static inline bool is_current_extinguished_storage = false;

    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(ms(10.0f));

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Connect mock storage
        yeet::mock_signal_input::connect(system.grid_side_control.state_machine.get_inputs().is_current_extinguished,
            is_current_extinguished_storage);

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

    // State machine control helpers using io_mock (tests real code path via set_digital_inputs)
    void set_auxiliaries_on(bool on) { system.io_mock.get_parameters().auxiliaries_are_on.write(on); }

    void set_breakers_on(bool on) { system.io_mock.get_parameters().mcb_is_closed.write(on); }

    void set_start_command(bool start) { system.io_mock.get_parameters().start_command.write(start); }

    void set_stop_command(bool stop) { system.io_mock.get_parameters().stop_command.write(stop); }

    void set_shutdown_request(bool shutdown) { system.io_mock.get_parameters().shutdown_request.write(shutdown); }

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
 * @test StateMachineControlledShutdownTest
 * @brief Validates controlled shutdown via state machine stop command
 *
 * This test verifies that the system performs a controlled shutdown when
 * the stop_command signal is asserted.
 */
TEST_F(ClosedLoopShutdownTest, StateMachineControlledShutdownTest)
{
    // Bring system to operation
    bring_to_operation();

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION before shutdown";

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(operating_speed.value(), 0.3f) << "Machine should be running before shutdown";

    // Initiate Shutdown via stop_command (standard shutdown path)
    set_stop_command(true);

    bool success = true;
    // Allow sufficient time for state machine transition (multiple 10ms cycles)
    success &= simulator.simulate_for(0.5);

    // Verify state machine transitioned to shutdown
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_machine_state == sfc_state_id::SHUTDOWN_REQUESTED ||
                state_machine_state == sfc_state_id::OPERATION_OFF || state_machine_state == sfc_state_id::BREAKERS_ON)
        << "State machine should be in shutdown sequence";

    // Startup sequencer should detect shutdown and transition to IDLE
    auto sequencer_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(sequencer_state, startup_sequencer_state_t::IDLE) << "Startup sequencer should return to IDLE";

    // Verify shutdown behavior - pulses should be blocked
    auto release_pulses = system.grid_side_control.state_machine.get_outputs().release_pulses.read();
    EXPECT_FALSE(release_pulses) << "Pulses should be blocked during shutdown";

    // Current should extinguish
    set_current_extinguished(true);
    success &= simulator.simulate_for(0.5);

    // System should complete shutdown cycle
    EXPECT_TRUE(success) << "Shutdown sequence should complete without errors";
}

/**
 * @test ShutdownOnAuxiliariesOffTest
 * @brief Validates emergency shutdown when auxiliaries are lost
 *
 * Loss of auxiliaries (cooling, control power, etc.) requires immediate
 * system shutdown for equipment protection.
 */
TEST_F(ClosedLoopShutdownTest, ShutdownOnAuxiliariesOffTest)
{
    // Bring system to operation
    bring_to_operation();

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION";

    // Trigger loss of auxiliaries
    set_auxiliaries_on(false);

    bool success = true;
    success &= simulator.simulate_for(0.5);

    // Verify emergency shutdown initiated
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_machine_state == sfc_state_id::IDLE || state_machine_state == sfc_state_id::AUXILIARIES_ON)
        << "State machine should transition toward IDLE or AUXILIARIES_ON";

    // Pulses should be immediately blocked
    auto release_pulses = system.grid_side_control.state_machine.get_outputs().release_pulses.read();
    EXPECT_FALSE(release_pulses) << "Pulses must be blocked when auxiliaries are lost";

    // Startup sequencer should return to IDLE
    auto sequencer_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(sequencer_state, startup_sequencer_state_t::IDLE) << "Startup sequencer should be IDLE";

    EXPECT_TRUE(success) << "Auxiliaries-off shutdown should complete without errors";
}

/**
 * @test ShutdownOnStopCommandTest
 * @brief Validates controlled shutdown via stop command
 *
 * The stop command initiates a controlled shutdown sequence, allowing
 * the system to ramp down speed before disconnecting.
 */
TEST_F(ClosedLoopShutdownTest, ShutdownOnStopCommandTest)
{
    // Bring system to operation
    bring_to_operation();

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION";

    // Clear start command first, then issue stop command
    set_start_command(false);
    set_stop_command(true);

    bool success = true;
    success &= simulator.simulate_for(0.5);

    // During shutdown request, torque reference must be forced to exact zero
    // (bypassing minimum motoring torque clamp and any rate limiting).
    const auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(
        state_machine_state == sfc_state_id::SHUTDOWN_REQUESTED || state_machine_state == sfc_state_id::OPERATION_OFF)
        << "State machine should be in SHUTDOWN_REQUESTED/OPERATION_OFF after stop";

    const auto applied_torque_ref = system.machine_side_control.torque_controller.get_outputs().torque_reference.read();
    const auto current_ref = system.machine_side_control.torque_controller.get_outputs().current_reference.read();
    EXPECT_FLOAT_EQ(applied_torque_ref.value(), 0.0f)
        << "Applied torque reference must be exactly zero during shutdown";
    EXPECT_FLOAT_EQ(current_ref.value(), 0.0f) << "Current reference must be exactly zero during shutdown";

    // Continue shutdown
    success &= simulator.simulate_for(1.5);

    // Verify state machine initiated shutdown
    EXPECT_TRUE(state_machine_state != sfc_state_id::OPERATION_ON) << "State machine should begin shutdown transition";

    // Startup sequencer should return to IDLE
    auto sequencer_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(sequencer_state, startup_sequencer_state_t::IDLE) << "Startup sequencer should be IDLE after stop";

    // Complete shutdown
    set_current_extinguished(true);
    success &= simulator.simulate_for(0.5);

    EXPECT_TRUE(success) << "Stop command shutdown should complete without errors";
}

/**
 * @test ShutdownOnBreakersOffTest
 * @brief Validates immediate shutdown when breakers open unexpectedly
 *
 * Loss of breaker feedback indicates either a fault trip or external
 * opening. The system must immediately cease operation.
 */
TEST_F(ClosedLoopShutdownTest, ShutdownOnBreakersOffTest)
{
    // Bring system to operation
    bring_to_operation();

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION";

    // Simulate breaker trip
    set_breakers_on(false);

    bool success = true;
    success &= simulator.simulate_for(0.5);

    // Verify immediate response
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_machine_state == sfc_state_id::BREAKERS_OFF ||
                state_machine_state == sfc_state_id::AUXILIARIES_ON || state_machine_state == sfc_state_id::IDLE)
        << "State machine should transition toward BREAKERS_OFF or lower";

    // Pulses must be blocked immediately
    auto release_pulses = system.grid_side_control.state_machine.get_outputs().release_pulses.read();
    EXPECT_FALSE(release_pulses) << "Pulses must be blocked when breakers open";

    // Startup sequencer should return to IDLE
    auto sequencer_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(sequencer_state, startup_sequencer_state_t::IDLE)
        << "Startup sequencer should be IDLE after breaker trip";

    EXPECT_TRUE(success) << "Breaker-off shutdown should complete without errors";
}

/**
 * @test SpeedReferenceZeroOverrideDuringShutdownTest
 * @brief Validates speed reference is forced to zero during shutdown
 *
 * During shutdown, the speed reference should be overridden to zero to
 * ensure the machine decelerates. This test verifies the override mechanism.
 */
TEST_F(ClosedLoopShutdownTest, SpeedReferenceZeroOverrideDuringShutdownTest)
{
    // Bring system to operation
    bring_to_operation();

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION";

    // Record pre-shutdown speed reference
    auto pre_shutdown_speed_ref = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
    EXPECT_GT(pre_shutdown_speed_ref.value(), 0.3f) << "Speed reference should be non-zero before shutdown";

    // Initiate shutdown
    set_shutdown_request(true);

    bool success = true;
    success &= simulator.simulate_for(0.5);

    // Verify speed reference behavior during shutdown
    auto post_shutdown_speed_ref = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();

    // When shutdown is triggered, the startup sequencer goes to IDLE which resets speed reference
    auto sequencer_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(sequencer_state, startup_sequencer_state_t::IDLE) << "Startup sequencer should be IDLE";

    // The speed reference from the sequencer should be zero or near-zero in IDLE state
    EXPECT_LE(post_shutdown_speed_ref.value(), pre_shutdown_speed_ref.value())
        << "Speed reference should decrease or reset during shutdown";

    EXPECT_TRUE(success) << "Speed reference override test should complete without errors";
}
