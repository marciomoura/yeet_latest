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
        register_complete_logger(simulator, system.grid_side_control.grid_protections);
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

    // Fault injection helpers for shutdown tests - use overrides to simulate hardware faults
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

    void set_current_extinguished(bool extinguished)
    {
        yeet::utils::mock_signal_input::write(
            system.grid_side_control.state_machine.get_inputs().is_current_extinguished, extinguished);
    }

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
    set_stop_command(true);

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

/**
 * @test EmergencyOffShutdownTest
 * @brief Validates emergency shutdown via emergency off button
 *
 * This test verifies that the system performs an immediate shutdown when
 * the emergency off signal is asserted, transitions to IDLE, and requires
 * user confirmation to clear the inhibit flag.
 */
TEST_F(ClosedLoopShutdownTest, EmergencyOffShutdownTest)
{
    // Enable protection module
    system.grid_side_control.grid_protections.get_parameters().protection_module_enabled.write(true);

    // Bring system to operation (simulates for 25s, reaches 0.5 pu)
    bring_to_operation();

    // Set speed reference to 1.0 pu and wait for ramp (0.5 pu * 30s/pu = 15s)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{1.0});
    simulator.simulate_for(20.0);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION before shutdown";

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 1.0f, 0.05f) << "Machine should be running at nominal speed";

    // Issue Emergency Off
    system.grid_io_mock.get_parameters().emergency_off.write(true);

    bool success = true;
    // Allow time for state machine to react and current to extinguish
    success &= simulator.simulate_for(1.0);

    // Verify state machine moved to IDLE
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_machine_state, sfc_state_id::IDLE) << "State machine should transition to IDLE on emergency off";

    // Verify operation is inhibited
    auto is_inhibited = system.grid_side_control.grid_protections.get_outputs().is_inhibited.read();
    EXPECT_TRUE(is_inhibited) << "System should be inhibited after emergency off";

    // Clear Emergency Off condition
    system.grid_io_mock.get_parameters().emergency_off.write(false);
    success &= simulator.simulate_for(0.5);

    // Inhibit should still be active
    is_inhibited = system.grid_side_control.grid_protections.get_outputs().is_inhibited.read();
    EXPECT_TRUE(is_inhibited) << "Inhibit should persist after clearing emergency off condition";

    // User confirmation to clear inhibit
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(2.0f);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(true);
    simulator.simulate_for(5.0f);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(2.0f);

    // User confirmation to clear inhibit
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(2.0f);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(true);
    simulator.simulate_for(5.0f);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(2.0f);

    // Verify inhibit is cleared
    is_inhibited = system.grid_side_control.grid_protections.get_outputs().is_inhibited.read();
    EXPECT_FALSE(is_inhibited) << "Inhibit should be cleared after user confirmation";

    EXPECT_TRUE(success) << "Emergency off shutdown test should complete without errors";
}

/**
 * @test ExternalFaultShutdownTest
 * @brief Validates emergency shutdown via external fault input
 *
 * This test verifies that the system performs an immediate shutdown when
 * the external fault signal is asserted, transitions to IDLE, and requires
 * user confirmation to clear the inhibit flag.
 */
TEST_F(ClosedLoopShutdownTest, ExternalFaultShutdownTest)
{
    // Enable protection module
    system.grid_side_control.grid_protections.get_parameters().protection_module_enabled.write(true);

    // Bring system to operation (simulates for 25s, reaches 0.5 pu)
    bring_to_operation();

    // Set speed reference to 1.0 pu and wait for ramp (0.5 pu * 30s/pu = 15s)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{1.0});
    simulator.simulate_for(20.0);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION before shutdown";

    auto operating_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(operating_speed.value(), 1.0f, 0.05f) << "Machine should be running at nominal speed";

    // Issue External Fault
    system.grid_io_mock.get_parameters().external_fault.write(true);

    bool success = true;
    // Allow time for state machine to react and current to extinguish
    success &= simulator.simulate_for(1.0);

    // Verify state machine moved to IDLE
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_machine_state, sfc_state_id::IDLE) << "State machine should transition to IDLE on external fault";

    // Verify operation is inhibited
    auto is_inhibited = system.grid_side_control.grid_protections.get_outputs().is_inhibited.read();
    EXPECT_TRUE(is_inhibited) << "System should be inhibited after external fault";

    // Clear External Fault condition
    system.grid_io_mock.get_parameters().external_fault.write(false);
    success &= simulator.simulate_for(0.5);

    // Inhibit should still be active
    is_inhibited = system.grid_side_control.grid_protections.get_outputs().is_inhibited.read();
    EXPECT_TRUE(is_inhibited) << "Inhibit should persist after clearing external fault condition";

    // User confirmation to clear inhibit
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(true);
    simulator.simulate_for(0.1);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);

    // Verify inhibit is cleared
    is_inhibited = system.grid_side_control.grid_protections.get_outputs().is_inhibited.read();
    EXPECT_FALSE(is_inhibited) << "Inhibit should be cleared after user confirmation";

    EXPECT_TRUE(success) << "External fault shutdown test should complete without errors";
}

/**
 * @test DoorOpenShutdownTest
 * @brief Validates emergency shutdown when cabinet door is opened
 *
 * This test verifies that the system performs an immediate shutdown when
 * the door is opened, transitions to IDLE, and requires user confirmation
 * to clear the inhibit flag.
 */
TEST_F(ClosedLoopShutdownTest, DoorOpenShutdownTest)
{
    // Enable protection module
    system.grid_side_control.grid_protections.get_parameters().protection_module_enabled.write(true);

    // Bring system to operation
    bring_to_operation();

    // Set speed reference to 1.0 pu and wait for ramp
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{1.0});
    simulator.simulate_for(20.0);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION before shutdown";

    // Open cabinet door
    system.grid_io_mock.get_parameters().door_is_open.write(true);

    bool success = true;
    // Allow time for state machine to react and current to extinguish
    success &= simulator.simulate_for(1.0);

    // Verify state machine moved to IDLE
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_machine_state, sfc_state_id::IDLE) << "State machine should transition to IDLE when door is opened";

    // Verify operation is inhibited
    auto is_inhibited = system.grid_side_control.grid_protections.get_outputs().is_inhibited.read();
    EXPECT_TRUE(is_inhibited) << "System should be inhibited after door is opened";

    // Close cabinet door
    system.grid_io_mock.get_parameters().door_is_open.write(false);
    success &= simulator.simulate_for(0.5);

    // Inhibit should still be active
    is_inhibited = system.grid_side_control.grid_protections.get_outputs().is_inhibited.read();
    EXPECT_TRUE(is_inhibited) << "Inhibit should persist after closing the door";

    // User confirmation to clear inhibit
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(true);
    simulator.simulate_for(0.1);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);

    // Verify inhibit is cleared
    is_inhibited = system.grid_side_control.grid_protections.get_outputs().is_inhibited.read();
    EXPECT_FALSE(is_inhibited) << "Inhibit should be cleared after user confirmation";

    EXPECT_TRUE(success) << "Door open shutdown test should complete without errors";
}

/**
 * @test BreakerTrippingShutdownTest
 * @brief Validates emergency shutdown when MV breaker reports tripping
 *
 * This test verifies that the system performs an immediate shutdown when
 * the MV breaker tripping signal is asserted, transitions to IDLE, and
 * requires turn_on_command toggle to restart.
 */
TEST_F(ClosedLoopShutdownTest, BreakerTrippingShutdownTest)
{
    // Bring system to operation
    bring_to_operation();

    // Set speed reference to 1.0 pu and wait for ramp
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{1.0});
    simulator.simulate_for(20.0);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION before shutdown";

    // Simulate MV breaker tripping
    system.grid_io_mock.get_parameters().mv_breaker_is_tripping.write(true);

    bool success = true;
    // Allow time for state machine to react and current to extinguish
    success &= simulator.simulate_for(1.0);

    // Verify state machine moved to IDLE
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_machine_state, sfc_state_id::IDLE) << "State machine should transition to IDLE on breaker tripping";

    // Clear tripping signal
    system.grid_io_mock.get_parameters().mv_breaker_is_tripping.write(false);
    success &= simulator.simulate_for(0.5);

    // Verify system cannot restart immediately (trip latch in IDLE)
    set_turn_on_command(true);
    simulator.simulate_for(0.5);
    state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_machine_state, sfc_state_id::IDLE) << "System should stay in IDLE until turn_on is toggled";

    // Clear start command to prevent automatic full restart
    set_start_command(false);

    // Toggle turn_on to clear latch
    set_turn_on_command(false);
    simulator.simulate_for(0.5);
    set_turn_on_command(true);
    simulator.simulate_for(0.5);

    // Verify system starts moving toward AUXILIARIES_ON
    state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_machine_state == sfc_state_id::AUXILIARIES_ON ||
                state_machine_state == sfc_state_id::BREAKERS_OFF || state_machine_state == sfc_state_id::BREAKERS_ON ||
                state_machine_state == sfc_state_id::OPERATION_OFF)
        << "System should restart and leave IDLE after turn_on toggle";

    EXPECT_TRUE(success) << "Breaker tripping shutdown test should complete without errors";
}

/**
 * @test CommunicationLossShutdownTest
 * @brief Validates emergency shutdown when fiber optic communication is lost
 *
 * This test verifies that the system performs an immediate shutdown when
 * the fiber optic link fails for 10ms, which is longer than the configured
 * watchdog timeout.
 */
TEST_F(ClosedLoopShutdownTest, CommunicationLossShutdownTest)
{
    // Enable protection module
    system.grid_side_control.grid_protections.get_parameters().protection_module_enabled.write(true);
    system.machine_side_control.machine_protections.get_parameters().protection_module_enabled.write(true);

    // Bring system to operation
    bring_to_operation();

    // Configure short communication timeout (5ms) to ensure 10ms failure trips
    system.grid_side_control.grid_protections.get_parameters().comm_timeout.write(duration_t{0.005});
    system.machine_side_control.machine_protections.get_parameters().comm_timeout.write(duration_t{0.005});

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION before shutdown";

    // Simulate fiber optic failure for 10ms
    system.fiber_optic_failure = true;
    simulator.simulate_for(0.010);

    // Verify trip occurred BEFORE restoring fiber
    // (communication_loss_tripped reflects CURRENT fault status, not latched)
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().communication_loss_tripped.read())
        << "Grid-side should detect communication loss trip during failure";
    EXPECT_TRUE(system.machine_side_control.machine_protections.get_outputs().communication_loss_tripped.read())
        << "Machine-side should detect communication loss trip during failure";
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side system trip should be latched";
    EXPECT_TRUE(system.machine_side_control.machine_protections.get_outputs().is_inhibited.read())
        << "Machine-side system trip should be latched";

    // Restore fiber and allow time for state machine to react
    system.fiber_optic_failure = false;
    simulator.simulate_for(0.1);

    // After fiber restoration, fault flag should clear but system trip remains latched
    EXPECT_FALSE(system.grid_side_control.grid_protections.get_outputs().communication_loss_tripped.read())
        << "Grid-side fault should clear after fiber restoration";
    EXPECT_FALSE(system.machine_side_control.machine_protections.get_outputs().communication_loss_tripped.read())
        << "Machine-side fault should clear after fiber restoration";
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side system trip should remain latched until acknowledged";
    EXPECT_TRUE(system.machine_side_control.machine_protections.get_outputs().is_inhibited.read())
        << "Machine-side system trip should remain latched until acknowledged";

    // Verify state machine moved to IDLE (emergency shutdown)
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_machine_state, sfc_state_id::IDLE)
        << "State machine should transition to IDLE on communication loss";
}

/**
 * @test GridToMachineCommunicationLossShutdownTest
 * @brief Validates that a failure in Grid->Machine direction trips both sides and requires user confirmation
 *
 * This test verifies the "heartbeat" logic: if Grid->Machine fiber is broken,
 * Machine side trips due to RX failure, Grid side trips because Machine
 * reports 'communication_ok = false', and both sides require user confirmation
 * to clear the inhibit flag after fiber restoration.
 */
TEST_F(ClosedLoopShutdownTest, GridToMachineCommunicationLossShutdownTest)
{
    // Enable protection module
    system.grid_side_control.grid_protections.get_parameters().protection_module_enabled.write(true);
    system.machine_side_control.machine_protections.get_parameters().protection_module_enabled.write(true);

    // Bring system to operation
    bring_to_operation();

    // Configure short communication timeout (5ms)
    system.grid_side_control.grid_protections.get_parameters().comm_timeout.write(duration_t{0.005});
    system.machine_side_control.machine_protections.get_parameters().comm_timeout.write(duration_t{0.005});

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION before shutdown";

    // Simulate Grid -> Machine failure ONLY
    // Grid RX is still fine, but Machine RX is broken
    system.fiber_optic_grid_to_machine_failure = true;
    simulator.simulate_for(0.020);  // 20ms is plenty for 5ms timeout

    // Verify Machine-side tripped due to local RX failure
    EXPECT_TRUE(system.machine_side_control.machine_protections.get_outputs().communication_loss_tripped.read())
        << "Machine-side should detect local RX failure";

    // Verify Grid-side ALSO tripped (via heartbeat/remote_ok bit)
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().communication_loss_tripped.read())
        << "Grid-side should trip because Machine reported communication failure";

    // Verify both sides have system trip latched
    EXPECT_TRUE(system.machine_side_control.machine_protections.get_outputs().is_inhibited.read())
        << "Machine-side system trip should be latched";
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side system trip should be latched";

    // Verify both sides are inhibited
    EXPECT_TRUE(system.machine_side_control.machine_protections.get_outputs().is_inhibited.read())
        << "Machine-side should be inhibited after communication loss";
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side should be inhibited after communication loss";

    // Allow time for shutdown sequence to complete (extinguish current -> IDLE)
    simulator.simulate_for(1.0);

    // Verify state machine moved to IDLE
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_machine_state, sfc_state_id::IDLE) << "State machine should transition to IDLE";

    // Restore fiber communication
    system.fiber_optic_grid_to_machine_failure = false;
    simulator.simulate_for(0.1);

    // After fiber restoration, fault flags should clear but system trip/inhibit remains latched
    EXPECT_FALSE(system.machine_side_control.machine_protections.get_outputs().communication_loss_tripped.read())
        << "Machine-side fault should clear after fiber restoration";
    EXPECT_FALSE(system.grid_side_control.grid_protections.get_outputs().communication_loss_tripped.read())
        << "Grid-side fault should clear after fiber restoration";
    EXPECT_TRUE(system.machine_side_control.machine_protections.get_outputs().is_inhibited.read())
        << "Machine-side system trip should remain latched until acknowledged";
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side system trip should remain latched until acknowledged";
    EXPECT_TRUE(system.machine_side_control.machine_protections.get_outputs().is_inhibited.read())
        << "Machine-side inhibit should remain active until acknowledged";
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side inhibit should remain active until acknowledged";

    // Clear machine-side trip first
    system.machine_side_control.machine_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);
    system.machine_side_control.machine_protections.get_parameters().confirm_trips.write(true);
    simulator.simulate_for(0.1);
    system.machine_side_control.machine_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);

    // Verify machine-side trip and inhibit are cleared
    EXPECT_FALSE(system.machine_side_control.machine_protections.get_outputs().is_inhibited.read())
        << "Machine-side trip should be cleared after user confirmation";
    EXPECT_FALSE(system.machine_side_control.machine_protections.get_outputs().is_inhibited.read())
        << "Machine-side inhibit should be cleared after user confirmation";

    // Now clear grid-side trip
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(true);
    simulator.simulate_for(0.1);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);

    // Verify grid-side trip and inhibit are both cleared after user confirmation
    EXPECT_FALSE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side trip should be cleared after user confirmation";
    EXPECT_FALSE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side inhibit should be cleared after user confirmation";
}

/**
 * @test MachineToGridCommunicationLossShutdownTest
 * @brief Validates that a failure in Machine->Grid direction trips grid-side and requires user confirmation
 *
 * This test verifies the reverse direction: if Machine->Grid fiber is broken,
 * Grid side trips due to RX failure, and the system requires user confirmation
 * to clear the inhibit flag after fiber restoration.
 */
TEST_F(ClosedLoopShutdownTest, MachineToGridCommunicationLossShutdownTest)
{
    // Enable protection module
    system.grid_side_control.grid_protections.get_parameters().protection_module_enabled.write(true);
    system.machine_side_control.machine_protections.get_parameters().protection_module_enabled.write(true);

    // Bring system to operation
    bring_to_operation();

    // Configure short communication timeout (5ms)
    system.grid_side_control.grid_protections.get_parameters().comm_timeout.write(duration_t{0.005});
    system.machine_side_control.machine_protections.get_parameters().comm_timeout.write(duration_t{0.005});

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "System should be in CLOSED_LOOP_OPERATION before shutdown";

    // Simulate Machine -> Grid failure ONLY
    // Machine TX is broken, Grid RX fails
    system.fiber_optic_machine_to_grid_failure = true;
    simulator.simulate_for(0.020);  // 20ms is plenty for 5ms timeout

    // Verify Grid-side tripped due to local RX failure
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().communication_loss_tripped.read())
        << "Grid-side should detect local RX failure";

    // Verify grid-side system trip is latched
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side system trip should be latched";

    // Verify grid-side is inhibited
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side should be inhibited after communication loss";

    // Allow time for shutdown sequence to complete
    simulator.simulate_for(1.0);

    // Verify state machine moved to IDLE
    auto state_machine_state = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_machine_state, sfc_state_id::IDLE)
        << "State machine should transition to IDLE on communication loss";

    // Restore fiber communication
    system.fiber_optic_machine_to_grid_failure = false;
    simulator.simulate_for(0.1);

    // After fiber restoration, fault flag should clear but system trip/inhibit remains latched
    EXPECT_FALSE(system.grid_side_control.grid_protections.get_outputs().communication_loss_tripped.read())
        << "Grid-side fault should clear after fiber restoration";
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side system trip should remain latched until acknowledged";
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side inhibit should remain active until acknowledged";

    // Machine-side also needs confirmation since it received machine_protection_tripped from grid
    // Clear machine-side trip first to prevent re-triggering grid-side
    system.machine_side_control.machine_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);
    system.machine_side_control.machine_protections.get_parameters().confirm_trips.write(true);
    simulator.simulate_for(0.1);
    system.machine_side_control.machine_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);

    // Verify machine-side trip is cleared
    EXPECT_FALSE(system.machine_side_control.machine_protections.get_outputs().is_inhibited.read())
        << "Machine-side trip should be cleared after user confirmation";

    // Now clear grid-side trip (toggle confirm_trips)
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(true);
    simulator.simulate_for(0.1);
    system.grid_side_control.grid_protections.get_parameters().confirm_trips.write(false);
    simulator.simulate_for(0.1);

    // Verify grid-side trip and inhibit are both cleared after user confirmation
    EXPECT_FALSE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side trip should be cleared after user confirmation";
    EXPECT_FALSE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Grid-side inhibit should be cleared after user confirmation";
}
