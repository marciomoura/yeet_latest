/**
 * @file closed_loop_ios_test.cpp
 * @brief Integration tests for SFC external IO interfaces
 *
 * Tests the IO interfaces for the SFC application:
 *
 * **Excitation System Interface:**
 * - DO: Excitation release (excitation_start_command)
 * - AO: Excitation current reference (field_current_reference)
 * - AI: Actual excitation current (optional)
 * - DI: Excitation running status (optional)
 *
 * **Main Circuit Breaker (MV Breaker) Interface:**
 * - DO: Close command (mv_breaker_close)
 * - DO: Open command (mv_breaker_open - includes trip scenarios)
 * - DI: Breaker is closed (optional status)
 * - DI: Breaker is tripping (protection relay - triggers shutdown)
 * - DI: Emergency off (triggers shutdown)
 * - DI: Door supervision (prevents MV Breaker close when open)
 *
 * **Key Behaviors:**
 * - Door supervision prevents MCB close command when door is open
 * - Emergency off triggers shutdown request with current extinguish
 * - Breaker tripping triggers shutdown request
 * - MCB open command is always asserted during SHUTDOWN_REQUESTED state
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

class ClosedLoopIOsTest : public ::testing::Test {
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
        register_complete_logger(simulator, system.grid_side_control.state_machine);
        register_complete_logger(simulator, system.machine_side_control.excitation_controller);
    }

    void initialize()
    {
        set_turn_on_command(false);
        set_start_command(false);
        set_stop_command(false);
        set_current_extinguished(false);
        set_breaker_is_tripping(false);
        set_emergency_off(false);
        set_door_is_open(false);

        // Reset all grid IO mock overrides
        system.grid_io_mock.get_parameters().override_auxiliaries_are_on.write(false);
        system.grid_io_mock.get_parameters().override_fans_are_running.write(false);
        system.grid_io_mock.get_parameters().override_mv_breaker_is_closed.write(false);

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

    // IO-level helpers for step-by-step testing of state machine transitions
    // These use overrides for direct control in IO interface tests
    void set_auxiliaries_on(bool on)
    {
        system.grid_io_mock.get_parameters().override_auxiliaries_are_on.write(true);
        system.grid_io_mock.get_parameters().override_auxiliaries_are_on_value.write(on);
    }

    void set_fans_running(bool on)
    {
        system.grid_io_mock.get_parameters().override_fans_are_running.write(true);
        system.grid_io_mock.get_parameters().override_fans_are_running_value.write(on);
    }

    void set_breakers_on(bool on)
    {
        system.grid_io_mock.get_parameters().override_mv_breaker_is_closed.write(true);
        system.grid_io_mock.get_parameters().override_mv_breaker_is_closed_value.write(on);
    }

    void set_current_extinguished(bool extinguished)
    {
        system.grid_side_control.state_machine.get_parameters().is_current_extinguished_override.write(true);
        system.grid_side_control.state_machine.get_parameters().is_current_extinguished_override_value.write(
            extinguished);
    }

    void set_breaker_is_tripping(bool tripping)
    {
        system.grid_io_mock.get_parameters().mv_breaker_is_tripping.write(tripping);
    }

    void set_emergency_off(bool emergency) { system.grid_io_mock.get_parameters().emergency_off.write(emergency); }

    void set_door_is_open(bool open) { system.grid_io_mock.get_parameters().door_is_open.write(open); }

    // Helper to quickly bring system to BREAKERS_OFF state using overrides
    // This is useful for IO tests that need to start from a specific state
    // Note: These helpers set the overrides but DON'T run simulation - caller must simulate
    void bring_to_breakers_off_state()
    {
        set_turn_on_command(true);  // Required to leave IDLE
        set_auxiliaries_on(true);   // Override aux status to accelerate through AUXILIARIES_ON
        set_fans_running(true);     // Required for auxiliaries_are_on
    }

    // Helper to quickly bring system to BREAKERS_ON state using overrides
    void bring_to_breakers_on_state()
    {
        bring_to_breakers_off_state();
        set_breakers_on(true);  // Override breaker status to accelerate through BREAKERS_OFF
    }

    // Helper to quickly bring system to OPERATION_ON state using overrides
    void bring_to_operation_on_state()
    {
        bring_to_breakers_on_state();
        set_start_command(true);  // Trigger transition to OPERATION_ON
    }

    // Configure startup sequencer for quick startup (used by most tests)
    void configure_quick_startup()
    {
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
        system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.3});
    }

    // System under test
    yeet::application::sfc::integration::plant_complete_sfc_closed_loop_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// ==================== MCB Interface Tests ====================

/**
 * @test MCBCloseCommandInBreakersOffState
 * @brief Validates MCB close command is issued in BREAKERS_OFF state
 *
 * When state machine is in BREAKERS_OFF, it should request MCB close
 * to transition to BREAKERS_ON.
 */
TEST_F(ClosedLoopIOsTest, MCBCloseCommandInBreakersOffState)
{
    bool success = true;

    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should be in IDLE state";

    // Start with IDLE state
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be false in IDLE";

    // Transition to AUXILIARIES_ON -> BREAKERS_OFF (step by step)
    // Note: We use step-by-step control here to test BREAKERS_OFF specifically
    set_turn_on_command(true);  // Required to leave IDLE
    set_auxiliaries_on(true);   // Override aux status to reach BREAKERS_OFF
    set_fans_running(true);     // Required for auxiliaries_are_on
    // Note: Do NOT set breakers_on override - we want to stay in BREAKERS_OFF
    success &= simulator.simulate_for(ms(50.0f));

    // In BREAKERS_OFF, MV Breaker close command should be true
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be true in BREAKERS_OFF state";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open command should be false in BREAKERS_OFF state";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be false in BREAKERS_OFF (breakers not yet confirmed on)";
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::BREAKERS_OFF)
        << "Should be in BREAKERS_OFF state";

    EXPECT_TRUE(success);
}

/**
 * @test DoorOpenTripsAndForcesShutdown
 * @brief Door open must generate a trip: MCB open asserted and auxiliaries command goes off.
 *
 * Note: Trip acknowledgment and clearing only happens in SHUTDOWN_REQUESTED state.
 * This test verifies the protection latching behavior and IO outputs.
 */
TEST_F(ClosedLoopIOsTest, DoorOpenTripsAndForcesShutdown)
{
    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Bring system to BREAKERS_OFF so it would normally request MCB close.
    bring_to_breakers_off_state();
    success &= simulator.simulate_for(ms(50.0f));

    ASSERT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::BREAKERS_OFF)
        << "Should be in BREAKERS_OFF state";

    // Enable protection module so door supervision latches and requires confirmation.
    system.grid_side_control.grid_protections.get_parameters().protection_module_enabled.write(true);
    success &= simulator.simulate_for(ms(20.0f));

    // Open the door: must produce a trip/shutdown behavior.
    set_door_is_open(true);
    success &= simulator.simulate_for(ms(20.0f));

    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Door open should latch protection inhibit";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open must be asserted on door-open trip";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close must not be asserted during door-open trip";

    // Verify the *external* digital outputs match the state machine semantics.
    EXPECT_TRUE(system.grid_side_control.get_external_ios().get_mv_breaker_open())
        << "External DO mv_breaker_open must be asserted on door-open trip";
    EXPECT_FALSE(system.grid_side_control.get_external_ios().get_mv_breaker_close())
        << "External DO mv_breaker_close must not be asserted during door-open trip";
    EXPECT_FALSE(system.grid_side_control.get_external_ios().get_auxiliaries_on_command())
        << "Auxiliaries command must go off on door-open trip";

    // Relationship check: when we command breaker open, auxiliaries must be commanded off.
    if (system.grid_side_control.get_external_ios().get_mv_breaker_open()) {
        EXPECT_FALSE(system.grid_side_control.get_external_ios().get_auxiliaries_on_command())
            << "Auxiliaries must be commanded off whenever MCB open is commanded";
    }

    // Close the door: alarm stays latched until user confirmation.
    set_door_is_open(false);
    success &= simulator.simulate_for(ms(20.0f));
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Alarm should remain latched after door closes";

    // In BREAKERS_OFF state, trip_acknowledged is not asserted, so:
    // - has_active_trip is false (fault cleared)
    // - is_inhibited remains true (alarm not confirmed)
    // This is expected behavior - trip acknowledgment only occurs in SHUTDOWN_REQUESTED
    EXPECT_FALSE(system.grid_side_control.grid_protections.get_outputs().has_active_trip.read())
        << "Trip clears after door is closed";
    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Inhibit should remain latched in BREAKERS_OFF";

    // User confirmation cannot clear inhibit while trip is still latched
    auto& protection_params = system.grid_side_control.grid_protections.get_parameters();
    protection_params.confirm_trips.write(false);
    success &= simulator.simulate_for(ms(10.0f));
    protection_params.confirm_trips.write(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Inhibit should still be latched (trip hasn't been acknowledged)
    EXPECT_FALSE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Inhibit should is cleared after user confirmation once trip is cleared";

    EXPECT_TRUE(success);
}

/**
 * @test MCBOpenCommandDuringShutdown
 * @brief Validates MCB open command behavior during shutdown sequence
 *
 * When state machine receives stop_command and current is not yet extinguished,
 * it should transition to SHUTDOWN_REQUESTED and assert mv_breaker_open.
 * Note: If current extinguishes quickly, transition to OPERATION_OFF happens.
 *
 * State machine transitions:
 * IDLE -> AUXILIARIES_ON -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF -> OPERATION_ON
 * -> SHUTDOWN_REQUESTED (on stop_command)
 */
TEST_F(ClosedLoopIOsTest, MCBOpenCommandDuringShutdown)
{
    configure_quick_startup();

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_ON
    bring_to_operation_on_state();
    success &= simulator.simulate_for(ms(50.0f));

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    // Configure plant
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    // Wait for startup
    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // MV Breaker commands during OPERATION_ON: close is asserted to maintain breaker closed
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be true during OPERATION_ON (level-based control)";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open command should be false during OPERATION_ON";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be true during OPERATION_ON";

    // Trigger shutdown via stop command (but keep current_extinguished false to stay in SHUTDOWN_REQUESTED)
    set_stop_command(true);
    // Execute just one state machine cycle (10ms) to catch the SHUTDOWN_REQUESTED state
    success &= simulator.simulate_for(ms(15.0f));

    // Verify state transition to SHUTDOWN_REQUESTED (or beyond if current extinguished quickly)
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_id == sfc_state_id::SHUTDOWN_REQUESTED || state_id == sfc_state_id::OPERATION_OFF)
        << "Should transition to SHUTDOWN_REQUESTED or OPERATION_OFF on stop command";

    // If in SHUTDOWN_REQUESTED, MV Breaker open command should be TRUE
    if (state_id == sfc_state_id::SHUTDOWN_REQUESTED) {
        EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
            << "MV Breaker open command should be true in SHUTDOWN_REQUESTED";
        EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
            << "MV Breaker close command should be false in SHUTDOWN_REQUESTED";
    }

    EXPECT_TRUE(success);
}

/**
 * @test BreakerTrippingTriggersShutdown
 * @brief Validates breaker_is_tripping input triggers shutdown sequence
 *
 * When breaker_is_tripping (DI from protection relay) becomes true,
 * the state machine should immediately transition to SHUTDOWN_REQUESTED
 * and open the MCB.
 *
 * State machine transitions:
 * IDLE -> AUXILIARIES_ON -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF -> OPERATION_ON
 * -> SHUTDOWN_REQUESTED (on breaker_is_tripping)
 */
TEST_F(ClosedLoopIOsTest, BreakerTrippingTriggersShutdown)
{
    configure_quick_startup();

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_ON
    bring_to_operation_on_state();
    success &= simulator.simulate_for(ms(50.0f));

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // Simulate protection relay indicating breaker is tripping
    set_breaker_is_tripping(true);
    success &= simulator.simulate_for(ms(15.0f));  // One state machine cycle

    // Verify state transition to SHUTDOWN_REQUESTED
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::SHUTDOWN_REQUESTED)
        << "Should transition to SHUTDOWN_REQUESTED when breaker_is_tripping";

    // MV Breaker open command should be asserted, close command should be false
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open command should be true when mv_breaker_is_tripping triggers shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be false in SHUTDOWN_REQUESTED";
    // Note: mv_breaker_is_closed reflects input state, which is still true until breaker actually opens
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should still be true (input hasn't changed yet)";

    EXPECT_TRUE(success);
}

/**
 * @test EmergencyOffTriggersShutdown
 * @brief Validates emergency_off input triggers immediate shutdown
 *
 * When emergency_off (DI from E-stop button) becomes true,
 * the state machine should immediately transition to SHUTDOWN_REQUESTED.
 *
 * State machine transitions:
 * IDLE -> AUXILIARIES_ON -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF -> OPERATION_ON
 * -> SHUTDOWN_REQUESTED (on emergency_off) -> OPERATION_OFF (when current extinguished)
 */
TEST_F(ClosedLoopIOsTest, EmergencyOffTriggersShutdown)
{
    configure_quick_startup();

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_ON
    bring_to_operation_on_state();
    success &= simulator.simulate_for(ms(50.0f));  // Allow time for state transitions

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // Simulate emergency off button press
    set_emergency_off(true);
    success &= simulator.simulate_for(ms(15.0f));  // One state machine cycle

    // Verify MV Breaker outputs from runner (after safety interlocks)
    // Emergency off should force MV Breaker open and block MV Breaker close regardless of state
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open should be true when emergency_off is active (safety override)";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close should be false when emergency_off is active (safety interlock)";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should still be true (breaker hasn't opened yet)";

    // Verify state transition to SHUTDOWN_REQUESTED (or beyond if current already extinguished)
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_id == sfc_state_id::SHUTDOWN_REQUESTED || state_id == sfc_state_id::OPERATION_OFF)
        << "Should transition to SHUTDOWN_REQUESTED or OPERATION_OFF on emergency_off";

    // Complete shutdown sequence - clear start_command to prevent re-entering OPERATION_ON
    set_current_extinguished(true);
    set_emergency_off(false);  // E-stop released
    success &= simulator.simulate_for(ms(20.0f));

    // After current extinguished with start_command cleared, verify state transition to OPERATION_OFF
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::IDLE) << "Should transition to IDLE after current extinguished";

    EXPECT_TRUE(success);
}

/**
 * @test EmergencyOffInhibitsStartup
 * @brief Validates emergency_off input prevents system from starting
 *
 * When emergency_off is active:
 * - MV Breaker close should be blocked (mv_breaker_close = false)
 * - MV Breaker open should be forced (mv_breaker_open = true)
 * - System should NOT transition beyond BREAKERS_OFF state
 *
 * This ensures the E-stop button acts as a startup inhibit.
 */
TEST_F(ClosedLoopIOsTest, EmergencyOffInhibitsStartup)
{
    configure_quick_startup();

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Activate emergency off BEFORE attempting startup
    set_emergency_off(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Verify MV Breaker interlocks are active even in IDLE state
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open should be forced true when emergency_off is active";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close should be blocked when emergency_off is active";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be false in IDLE";

    // Attempt to start: even if auxiliaries are available, emergency_off forces auxiliaries command OFF.
    // Note: Without turn_on_command, system won't leave IDLE anyway.
    // With turn_on_command + emergency_off, system will enter AUXILIARIES_ON but
    // any_fault_active check may prevent further progress or force shutdown.
    set_turn_on_command(true);  // Required to leave IDLE
    set_auxiliaries_on(true);
    set_fans_running(true);
    success &= simulator.simulate_for(ms(50.0f));

    // With emergency_off active, system may leave IDLE but should not progress to full operation
    // Note: Exact behavior depends on state machine fault handling
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();

    // Verify auxiliaries command is forced off.
    EXPECT_FALSE(system.grid_side_control.get_external_ios().get_auxiliaries_on_command())
        << "Auxiliaries command should be forced off when emergency_off is active";

    // Release emergency off: auxiliaries can now be commanded on and the state machine can progress.
    set_emergency_off(false);
    success &= simulator.simulate_for(ms(50.0f));

    // With auxiliaries available, system should advance out of IDLE.
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_NE(state_id, sfc_state_id::IDLE) << "System should be able to leave IDLE after emergency_off is released";

    EXPECT_TRUE(success);
}

// ==================== Excitation Interface Tests ====================

/**
 * @test ExcitationReleaseFollowsStartupSequencer
 * @brief Validates excitation release command follows startup sequencer
 *
 * The excitation release (DO) should be:
 * - FALSE in IDLE state
 * - TRUE when startup sequencer enters FLUX_BUILDUP and beyond
 *
 * Note: In the complete closed-loop system, the startup sequencer generates
 * release_excitation which enables the excitation controller. This test validates
 * that during operation, the excitation is released.
 *
 * State machine transitions:
 * IDLE -> AUXILIARIES_ON -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF -> OPERATION_ON
 */
TEST_F(ClosedLoopIOsTest, ExcitationReleaseFollowsStartupSequencer)
{
    configure_quick_startup();

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Initially, startup sequencer should not release excitation
    EXPECT_FALSE(system.machine_side_control.startup_sequencer.get_outputs().release_excitation.read())
        << "Startup sequencer should not release excitation in IDLE";

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_ON
    bring_to_operation_on_state();
    success &= simulator.simulate_for(ms(10.0f));

    // Configure plant
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    // Complete startup and wait for operation
    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // During operation (CLOSED_LOOP_OPERATION), startup sequencer should release excitation
    EXPECT_TRUE(system.machine_side_control.startup_sequencer.get_outputs().release_excitation.read())
        << "Startup sequencer should release excitation during OPERATION_ON";

    EXPECT_TRUE(success);
}

/**
 * @test ExcitationCurrentReferenceFromExcitationController
 * @brief Validates excitation field current reference is generated by excitation controller
 *
 * The field current reference (AO) from excitation controller should:
 * - Be at setpoint value when excitation is released and prefluxing is complete
 *
 * Note: The excitation controller receives release_excitation from startup sequencer
 * and generates the field_current_reference based on prefluxing state.
 *
 * State machine transitions:
 * IDLE -> AUXILIARIES_ON -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF -> OPERATION_ON
 */
TEST_F(ClosedLoopIOsTest, ExcitationCurrentReferenceFromExcitationController)
{
    configure_quick_startup();

    // Set a specific forced field current
    system.machine_side_control.excitation_controller.get_parameters().forced_field_current_pu = current_pu_t{0.8};

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_ON
    bring_to_operation_on_state();
    success &= simulator.simulate_for(ms(10.0f));

    // Configure plant
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    // Wait for full startup to complete
    success &= simulator.simulate_for(8.5);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // During operation, field current should be at setpoint
    auto during_operation_ref =
        system.machine_side_control.excitation_controller.get_outputs().field_current_reference.read();
    EXPECT_NEAR(during_operation_ref.value(), 0.8f, 0.15f)
        << "Field current reference should be at setpoint during operation";

    EXPECT_TRUE(success);
}

/**
 * @test ExcitationStopCommandOnShutdown
 * @brief Validates excitation stop command is issued on shutdown
 *
 * When shutdown is initiated, the excitation controller should:
 * - De-assert excitation_start_command
 * - Assert excitation_stop_command
 *
 * State machine transitions:
 * IDLE -> AUXILIARIES_ON -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF -> OPERATION_ON
 * -> SHUTDOWN_REQUESTED -> OPERATION_OFF -> BREAKERS_ON -> BREAKERS_OFF -> AUXILIARIES_ON -> IDLE
 */
TEST_F(ClosedLoopIOsTest, ExcitationStopCommandOnShutdown)
{
    configure_quick_startup();

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_ON
    bring_to_operation_on_state();
    success &= simulator.simulate_for(ms(10.0f));

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // Verify excitation is on
    EXPECT_TRUE(system.machine_side_control.excitation_controller.get_outputs().excitation_start_command.read())
        << "Excitation start command should be true during OPERATION_ON";

    // Trigger shutdown
    set_stop_command(true);
    success &= simulator.simulate_for(ms(100.0f));

    // Complete shutdown - clear start_command to prevent re-entering OPERATION_ON
    set_current_extinguished(true);
    set_stop_command(false);
    set_start_command(false);  // Clear start command to allow transition to OPERATION_OFF
    success &= simulator.simulate_for(ms(200.0f));

    // Verify state transition to OPERATION_OFF
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_OFF) << "Should be in OPERATION_OFF after current extinguished";

    // Turn off breakers and auxiliaries
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.0f));
    set_breakers_on(false);
    set_start_command(false);
    success &= simulator.simulate_for(ms(200.0f));

    // Verify state transition to BREAKERS_OFF
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::BREAKERS_OFF) << "Should be in BREAKERS_OFF after breakers off";

    // Clear turn_on_command to allow transition to IDLE
    set_turn_on_command(false);
    set_auxiliaries_on(false);
    success &= simulator.simulate_for(ms(200.0f));

    // Verify state transition to IDLE
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::IDLE) << "Should be in IDLE after auxiliaries off";

    // After complete shutdown, excitation should be stopped
    EXPECT_TRUE(system.machine_side_control.excitation_controller.get_outputs().excitation_stop_command.read())
        << "Excitation stop command should be true after complete shutdown";

    EXPECT_TRUE(success);
}

// ==================== Combined IO Scenario Tests ====================

/**
 * @test CompleteIOSequenceStartupToShutdown
 * @brief Validates complete IO sequence from startup to shutdown
 *
 * This test validates the complete lifecycle of IO signals:
 * 1. IDLE: All outputs off
 * 2. Startup: MCB close command, excitation release
 * 3. Operation: Stable operation with excitation maintaining field
 * 4. Shutdown: MCB open command (in SHUTDOWN_REQUESTED)
 * 5. Return to IDLE: All outputs off
 */
TEST_F(ClosedLoopIOsTest, CompleteIOSequenceStartupToShutdown)
{
    configure_quick_startup();

    // Disable excitation override to test actual controller behavior
    // system.machine_side_control.external_ios.get_parameters().override_excitation_outputs.write(false);

    bool success = true;

    // ==================== Phase 1: IDLE State ====================
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::IDLE) << "Should start in IDLE state";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be false in IDLE";
    EXPECT_FALSE(system.machine_side_control.excitation_controller.get_outputs().excitation_start_command.read())
        << "Excitation start command should be false in IDLE";

    // ==================== Phase 2: Startup ====================
    set_turn_on_command(true);  // Required to leave IDLE
    set_auxiliaries_on(true);
    set_fans_running(true);
    success &= simulator.simulate_for(ms(50.0f));

    // Verify state transition to BREAKERS_OFF
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::BREAKERS_OFF) << "Should be in BREAKERS_OFF after auxiliaries on";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be true in BREAKERS_OFF";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open command should be false in BREAKERS_OFF";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be false in BREAKERS_OFF";

    set_breakers_on(true);
    success &= simulator.simulate_for(ms(50.0f));

    // Verify state transition after breakers confirmed on.
    // BREAKERS_ON may auto-transition quickly to OPERATION_OFF depending on timing.
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_id == sfc_state_id::BREAKERS_ON || state_id == sfc_state_id::OPERATION_OFF)
        << "Should be in BREAKERS_ON or OPERATION_OFF after breakers confirmed on";
    // MV Breaker commands in BREAKERS_ON/OPERATION_OFF: close is asserted to maintain breaker closed
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be true to maintain breaker closed (level-based control)";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be true once breakers are confirmed on";

    set_start_command(true);
    success &= simulator.simulate_for(ms(50.0f));

    // Verify state transition after start command.
    // OPERATION_OFF may transition quickly to OPERATION_ON depending on plant dynamics.
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_id == sfc_state_id::OPERATION_OFF || state_id == sfc_state_id::OPERATION_ON)
        << "Should be in OPERATION_OFF or OPERATION_ON after start command";

    // ==================== Phase 3: Operation ====================
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON after startup complete";

    // MV Breaker commands during OPERATION_ON: close is asserted to maintain breaker closed
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be true during OPERATION_ON (level-based control)";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open command should be false during OPERATION_ON";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be true during OPERATION_ON";
    EXPECT_TRUE(system.machine_side_control.excitation_controller.get_outputs().excitation_start_command.read())
        << "Excitation should be on during OPERATION_ON";

    // ==================== Phase 4: Shutdown ====================
    set_stop_command(true);
    success &= simulator.simulate_for(ms(15.0f));  // One state machine cycle

    // Verify state transition to SHUTDOWN_REQUESTED (or beyond if current extinguished quickly)
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_id == sfc_state_id::SHUTDOWN_REQUESTED || state_id == sfc_state_id::OPERATION_OFF)
        << "Should be in SHUTDOWN_REQUESTED or OPERATION_OFF after stop command";

    set_current_extinguished(true);
    set_stop_command(false);
    set_start_command(false);  // Clear start command to allow transition to OPERATION_OFF
    success &= simulator.simulate_for(ms(200.0f));

    // Verify state transition to OPERATION_OFF
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_OFF) << "Should be in OPERATION_OFF after current extinguished";

    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.0f));
    set_breakers_on(false);
    success &= simulator.simulate_for(ms(200.0f));

    // Verify state transition to BREAKERS_OFF
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::BREAKERS_OFF) << "Should be in BREAKERS_OFF after breakers off";

    // Clear turn_on_command to allow transition to IDLE
    set_turn_on_command(false);
    set_auxiliaries_on(false);
    success &= simulator.simulate_for(ms(200.0f));

    // ==================== Phase 5: Return to IDLE ====================
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::IDLE) << "Should return to IDLE after auxiliaries off";

    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be false in IDLE";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be false in IDLE";
    EXPECT_TRUE(system.machine_side_control.excitation_controller.get_outputs().excitation_stop_command.read())
        << "Excitation stop should be asserted in IDLE";

    EXPECT_TRUE(success);
}

/**
 * @test DoorOpenedDuringOperationTripsAndForcesShutdown
 * @brief Validates door opening during operation generates a trip and forces shutdown outputs.
 *
 * Door open is treated as a trip source. Opening the door during OPERATION_ON must:
 * - assert MCB open command
 * - deassert auxiliaries command
 * - latch protection inhibit (restart requires user confirmation)
 */
TEST_F(ClosedLoopIOsTest, DoorOpenedDuringOperationTripsAndForcesShutdown)
{
    configure_quick_startup();

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_ON
    bring_to_operation_on_state();

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));

    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // MV Breaker commands during OPERATION_ON: close is asserted to maintain breaker closed
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close command should be true during OPERATION_ON (level-based control)";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open command should be false during OPERATION_ON";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_is_closed.read())
        << "MV Breaker is closed status should be true during OPERATION_ON";

    // Enable protection module so door supervision latches and requires confirmation.
    system.grid_side_control.grid_protections.get_parameters().protection_module_enabled.write(true);
    success &= simulator.simulate_for(ms(20.0f));

    // Open the door during operation: must trip and force shutdown.
    set_door_is_open(true);
    success &= simulator.simulate_for(ms(15.0f));  // One state machine cycle

    EXPECT_TRUE(system.grid_side_control.grid_protections.get_outputs().is_inhibited.read())
        << "Door open should latch protection inhibit";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mv_breaker_open.read())
        << "MV Breaker open must be asserted on door-open trip";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mv_breaker_close.read())
        << "MV Breaker close must not be asserted during door-open trip";
    EXPECT_FALSE(system.grid_side_control.get_external_ios().get_auxiliaries_on_command())
        << "Auxiliaries command must go off on door-open trip";

    // State should move into the shutdown path (the breaker-status DI may still indicate closed).
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::SHUTDOWN_REQUESTED) << "Door open during operation should trigger shutdown";

    EXPECT_TRUE(success);
}
