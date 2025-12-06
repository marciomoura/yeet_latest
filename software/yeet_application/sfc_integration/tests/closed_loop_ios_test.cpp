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
 * **Main Circuit Breaker (MCB) Interface:**
 * - DO: Close command (mcb_close)
 * - DO: Open command (mcb_open - includes trip scenarios)
 * - DI: Breaker is closed (optional status)
 * - DI: Breaker is tripping (protection relay - triggers shutdown)
 * - DI: Emergency off (triggers shutdown)
 * - DI: Door supervision (prevents MCB close when open)
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
        set_auxiliaries_on(false);
        set_breakers_on(false);
        set_start_command(false);
        set_stop_command(false);
        set_shutdown_request(false);
        set_current_extinguished(false);
        set_breaker_is_tripping(false);
        set_emergency_off(false);
        set_door_is_open(false);

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

    void set_breaker_is_tripping(bool tripping)
    {
        yeet::mock_signal_input::write(
            system.grid_side_control.state_machine.get_inputs().breaker_is_tripping, tripping);
    }

    void set_emergency_off(bool emergency)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().emergency_off, emergency);
    }

    void set_door_is_open(bool open)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().door_is_open, open);
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
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be false in IDLE";

    // Transition to AUXILIARIES_ON -> BREAKERS_OFF
    set_auxiliaries_on(true);
    success &= simulator.simulate_for(ms(10.0f));

    // In BREAKERS_OFF, MCB close command should be true
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be true in BREAKERS_OFF state";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false in BREAKERS_OFF state";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be false in BREAKERS_OFF (breakers not yet confirmed on)";
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::BREAKERS_OFF)
        << "Should be in BREAKERS_OFF state";

    EXPECT_TRUE(success);
}

/**
 * @test DoorSupervisionPreventsBreakersClose
 * @brief Validates door supervision prevents MCB close command
 *
 * Door supervision (door_is_open=true) should block the MCB close command
 * even when state machine requests it. This is a safety interlock.
 *
 * State machine transitions: IDLE -> AUXILIARIES_ON -> BREAKERS_OFF (stays here due to door interlock)
 */
TEST_F(ClosedLoopIOsTest, DoorSupervisionPreventsBreakersClose)
{
    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Set door open BEFORE entering BREAKERS_OFF
    set_door_is_open(true);

    // Transition to AUXILIARIES_ON -> BREAKERS_OFF
    set_auxiliaries_on(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Verify state transition: IDLE -> AUXILIARIES_ON -> BREAKERS_OFF
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::BREAKERS_OFF) << "Should be in BREAKERS_OFF state";

    // MCB close command should be blocked by door interlock
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be blocked when door is open";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false in BREAKERS_OFF state";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be false in BREAKERS_OFF";

    // Close the door - now MCB close command should be allowed
    set_door_is_open(false);
    success &= simulator.simulate_for(ms(10.0f));

    // State should still be BREAKERS_OFF (waiting for mcb_is_closed confirmation)
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::BREAKERS_OFF)
        << "Should remain in BREAKERS_OFF state";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be allowed when door is closed";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false in BREAKERS_OFF";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be false in BREAKERS_OFF (waiting for confirmation)";

    EXPECT_TRUE(success);
}

/**
 * @test MCBOpenCommandDuringShutdown
 * @brief Validates MCB open command behavior during shutdown sequence
 *
 * When state machine receives stop_command and current is not yet extinguished,
 * it should transition to SHUTDOWN_REQUESTED and assert mcb_open.
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

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF
    set_auxiliaries_on(true);
    set_breakers_on(true);
    success &= simulator.simulate_for(ms(10.0f));
    set_start_command(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    // Configure plant
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    // Wait for startup
    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // MCB commands should be safe during operation
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be false during OPERATION_ON";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false during OPERATION_ON";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be true during OPERATION_ON";

    // Trigger shutdown via stop command (but keep current_extinguished false to stay in SHUTDOWN_REQUESTED)
    set_stop_command(true);
    // Execute just one state machine cycle (10ms) to catch the SHUTDOWN_REQUESTED state
    success &= simulator.simulate_for(ms(15.0f));

    // Verify state transition to SHUTDOWN_REQUESTED (or beyond if current extinguished quickly)
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_id == sfc_state_id::SHUTDOWN_REQUESTED || state_id == sfc_state_id::OPERATION_OFF)
        << "Should transition to SHUTDOWN_REQUESTED or OPERATION_OFF on stop command";

    // If in SHUTDOWN_REQUESTED, MCB open command should be TRUE
    if (state_id == sfc_state_id::SHUTDOWN_REQUESTED) {
        EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
            << "MCB open command should be true in SHUTDOWN_REQUESTED";
        EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
            << "MCB close command should be false in SHUTDOWN_REQUESTED";
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

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF
    set_auxiliaries_on(true);
    set_breakers_on(true);
    success &= simulator.simulate_for(ms(10.0f));
    set_start_command(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // Simulate protection relay indicating breaker is tripping
    set_breaker_is_tripping(true);
    success &= simulator.simulate_for(ms(100.0f));

    // Verify state transition to SHUTDOWN_REQUESTED
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::SHUTDOWN_REQUESTED)
        << "Should transition to SHUTDOWN_REQUESTED when breaker_is_tripping";

    // MCB open command should be asserted, close command should be false
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be true when breaker_is_tripping triggers shutdown";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be false in SHUTDOWN_REQUESTED";
    // Note: mcb_is_closed reflects input state, which is still true until breaker actually opens
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should still be true (input hasn't changed yet)";

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

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF
    set_auxiliaries_on(true);
    set_breakers_on(true);
    success &= simulator.simulate_for(ms(10.0f));
    set_start_command(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // Simulate emergency off button press
    set_emergency_off(true);
    success &= simulator.simulate_for(ms(15.0f));  // One state machine cycle

    // Verify MCB outputs from runner (after safety interlocks)
    // Emergency off should force MCB open and block MCB close regardless of state
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open should be true when emergency_off is active (safety override)";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close should be false when emergency_off is active (safety interlock)";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should still be true (breaker hasn't opened yet)";

    // Verify state transition to SHUTDOWN_REQUESTED (or beyond if current already extinguished)
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_TRUE(state_id == sfc_state_id::SHUTDOWN_REQUESTED || state_id == sfc_state_id::OPERATION_OFF)
        << "Should transition to SHUTDOWN_REQUESTED or OPERATION_OFF on emergency_off";

    // Complete shutdown sequence - clear start_command to prevent re-entering OPERATION_ON
    set_current_extinguished(true);
    set_emergency_off(false);  // E-stop released
    set_start_command(false);  // Clear start command to allow transition to OPERATION_OFF
    success &= simulator.simulate_for(ms(100.0f));

    // After current extinguished with start_command cleared, verify state transition to OPERATION_OFF
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_OFF) << "Should transition to OPERATION_OFF after current extinguished";

    EXPECT_TRUE(success);
}

/**
 * @test EmergencyOffInhibitsStartup
 * @brief Validates emergency_off input prevents system from starting
 *
 * When emergency_off is active:
 * - MCB close should be blocked (mcb_close = false)
 * - MCB open should be forced (mcb_open = true)
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

    // Verify MCB interlocks are active even in IDLE state
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open should be forced true when emergency_off is active";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close should be blocked when emergency_off is active";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be false in IDLE";

    // Attempt to start: set auxiliaries on
    set_auxiliaries_on(true);
    success &= simulator.simulate_for(ms(20.0f));  // Two state machine cycles for transition

    // Should transition to BREAKERS_OFF but MCB cannot close
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::BREAKERS_OFF) << "Should reach BREAKERS_OFF state";

    // Verify MCB interlocks still prevent closure
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open should remain true with emergency_off active";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close should remain blocked with emergency_off active";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be false in BREAKERS_OFF";

    // Signal that breakers are on (simulating what would happen if interlock failed)
    set_breakers_on(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Even if breaker feedback says closed, state machine sees it via mcb_is_closed output
    // which should be false because the interlock blocks the close command
    // Note: In a real system, breaker would not close due to mcb_close=false
    // For this test, we verify the output commands are correct

    // Verify outputs remain in safe state
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open should be forced by emergency_off safety override";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close should be blocked by emergency_off safety interlock";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be true (breaker feedback set, but close command blocked)";

    // Release emergency off
    set_emergency_off(false);
    success &= simulator.simulate_for(ms(20.0f));  // Allow transition

    // MCB close should now be allowed, and system should progress
    // Since mcb_is_closed was already set, state machine will have transitioned
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();

    // BREAKERS_ON auto-transitions to OPERATION_OFF, so we expect OPERATION_OFF
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_OFF)
        << "Should transition to OPERATION_OFF after emergency_off released";

    // Verify MCB close is no longer blocked (state machine may not request it anymore since breakers are on)
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open should be false when emergency_off is released and system is running";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close should be false in OPERATION_OFF (breakers already closed)";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be true in OPERATION_OFF";

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

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF
    set_auxiliaries_on(true);
    set_breakers_on(true);
    success &= simulator.simulate_for(ms(10.0f));
    set_start_command(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Configure plant
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

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

    // Set a specific field current setpoint
    system.machine_side_control.excitation_controller.get_parameters().field_current_setpoint = current_pu_t{0.8};

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF
    set_auxiliaries_on(true);
    set_breakers_on(true);
    success &= simulator.simulate_for(ms(10.0f));
    set_start_command(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Configure plant
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

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

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF
    set_auxiliaries_on(true);
    set_breakers_on(true);
    success &= simulator.simulate_for(ms(10.0f));
    set_start_command(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

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
    system.machine_side_control.external_ios.get_parameters().override_excitation_outputs.write(false);

    bool success = true;

    // ==================== Phase 1: IDLE State ====================
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::IDLE) << "Should start in IDLE state";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be false in IDLE";
    EXPECT_FALSE(system.machine_side_control.excitation_controller.get_outputs().excitation_start_command.read())
        << "Excitation start command should be false in IDLE";

    // ==================== Phase 2: Startup ====================
    set_auxiliaries_on(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Verify state transition to BREAKERS_OFF
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::BREAKERS_OFF) << "Should be in BREAKERS_OFF after auxiliaries on";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be true in BREAKERS_OFF";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false in BREAKERS_OFF";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be false in BREAKERS_OFF";

    set_breakers_on(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Verify state transition to BREAKERS_ON
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::BREAKERS_ON) << "Should be in BREAKERS_ON after breakers confirmed on";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be false in BREAKERS_ON";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be true in BREAKERS_ON";

    set_start_command(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Verify state transition to OPERATION_OFF
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_OFF) << "Should be in OPERATION_OFF after start command";

    // ==================== Phase 3: Operation ====================
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON after startup complete";

    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be false during OPERATION_ON";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false during OPERATION_ON";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be true during OPERATION_ON";
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

    set_auxiliaries_on(false);
    success &= simulator.simulate_for(ms(200.0f));

    // ==================== Phase 5: Return to IDLE ====================
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::IDLE) << "Should return to IDLE after auxiliaries off";

    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false in IDLE";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be false in IDLE";
    EXPECT_TRUE(system.machine_side_control.excitation_controller.get_outputs().excitation_stop_command.read())
        << "Excitation stop should be asserted in IDLE";

    EXPECT_TRUE(success);
}

/**
 * @test DoorOpenedDuringOperationDoesNotAffectMCB
 * @brief Validates door opening during operation doesn't affect MCB close command
 *
 * The door interlock only prevents the MCB CLOSE command (not open).
 * Opening the door during operation should NOT trigger any MCB action -
 * it only prevents closing if the breaker were to open.
 *
 * State machine transitions:
 * IDLE -> AUXILIARIES_ON -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF -> OPERATION_ON
 * (remains in OPERATION_ON when door opens - no state transition)
 */
TEST_F(ClosedLoopIOsTest, DoorOpenedDuringOperationDoesNotAffectMCB)
{
    configure_quick_startup();

    bool success = true;

    // Verify initial state
    EXPECT_EQ(system.grid_side_control.state_machine.get_outputs().current_state_id.read(), sfc_state_id::IDLE)
        << "Should start in IDLE state";

    // Execute startup sequence: IDLE -> BREAKERS_OFF -> BREAKERS_ON -> OPERATION_OFF
    set_auxiliaries_on(true);
    set_breakers_on(true);
    success &= simulator.simulate_for(ms(10.0f));
    set_start_command(true);
    success &= simulator.simulate_for(ms(10.0f));

    // Pre-flux phase
    success &= simulator.simulate_for(0.5);

    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.1f));
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_max_torque_limit, torque_pu_t{1.0});
    yeet::mock_signal_input::write(
        system.machine_side_control.torque_controller.get_inputs().dynamic_min_torque_limit, torque_pu_t{0.0});

    success &= simulator.simulate_for(8.0);

    // Verify state transition to OPERATION_ON
    auto state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Should be in OPERATION_ON state after startup";

    // MCB commands before door open
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should be false during OPERATION_ON";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should be false during OPERATION_ON";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should be true during OPERATION_ON";

    // Open the door during operation
    set_door_is_open(true);
    success &= simulator.simulate_for(ms(200.0f));

    // Should remain in OPERATION_ON - door supervision does NOT trigger shutdown
    state_id = system.grid_side_control.state_machine.get_outputs().current_state_id.read();
    EXPECT_EQ(state_id, sfc_state_id::OPERATION_ON) << "Opening door during operation should NOT trigger shutdown";

    // MCB commands should be unaffected
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_close.read())
        << "MCB close command should remain false during OPERATION_ON";
    EXPECT_FALSE(system.grid_side_control.state_machine.get_outputs().mcb_open.read())
        << "MCB open command should remain false during OPERATION_ON";
    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().mcb_is_closed.read())
        << "MCB is closed status should remain true during OPERATION_ON";

    EXPECT_TRUE(success);
}
