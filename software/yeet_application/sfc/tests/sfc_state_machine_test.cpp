#include "sfc/sfc_state_machine.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

class SfcStateMachineTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        state_machine.reset();
        turn_on = false;
        auxiliaries_on = false;
        breakers_on = false;
        start = false;
        stop = false;
        current_extinguished = false;
        protection_not_inhibited = true;  // Protection not inhibited by default
        any_fault_active = false;
        breaker_is_tripping = false;
        emergency_off = false;
        door_is_open = false;
        shutdown_request = false;
    }

    void verify_state(sfc_state_id expected_state, const sfc_state_outputs& expected_outputs)
    {
        EXPECT_EQ(state_machine.get_current_state(), expected_state);
        const auto outputs = state_machine.get_outputs();
        EXPECT_EQ(outputs.pulse_releases, expected_outputs.pulse_releases);
    }

    // Helper: navigate to OPERATION_ON state
    void navigate_to_operation_on()
    {
        turn_on = true;
        auto inputs = create_default_inputs();
        state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

        auxiliaries_on = true;
        inputs = create_default_inputs();
        state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF

        breakers_on = true;
        inputs = create_default_inputs();
        state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON
        state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF

        start = true;
        inputs = create_default_inputs();
        state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON
    }

    // Helper members for computed event values (must be members to avoid dangling references)
    bool computed_any_fault() const { return emergency_off || breaker_is_tripping || any_fault_active; }
    bool computed_is_inhibited() const { return door_is_open || !protection_not_inhibited; }
    bool computed_protection_trip() const { return !protection_not_inhibited || any_fault_active; }

    sfc_state_inputs create_default_inputs()
    {
        sfc_state_inputs inputs{.turn_on_command = mock_event("turn_on", turn_on),
            .start_command = mock_event("start", start),
            .stop_command = mock_event("stop", stop),
            .shutdown_request = mock_event("shutdown_request", shutdown_request),
            .auxiliaries_are_on_event = mock_event("auxiliaries_on", auxiliaries_on),
            .mcb_is_closed_event = mock_event("breakers_on", breakers_on),
            .is_current_extinguished = mock_event("current_extinguished", current_extinguished),
            .any_fault_active_event = sfc_state_event("any_fault_active", [this]() { return computed_any_fault(); }),
            .is_inhibited_from_starting_event =
                sfc_state_event("is_inhibited", [this]() { return computed_is_inhibited(); }),
            .protection_trip_active_event =
                sfc_state_event("protection_trip", [this]() { return computed_protection_trip(); })};
        return inputs;
    }

    sfc_state_machine state_machine;
    bool turn_on;
    bool auxiliaries_on;
    bool breakers_on;
    bool start;
    bool stop;
    bool current_extinguished;
    bool protection_not_inhibited;
    bool any_fault_active;
    bool breaker_is_tripping;
    bool emergency_off;
    bool door_is_open;
    bool shutdown_request;
};

// Test initial state
TEST_F(SfcStateMachineTest, InitialState)
{
    sfc_state_outputs expected{};
    expected.pulse_releases = false;
    verify_state(sfc_state_id::IDLE, expected);
}

// Test IDLE -> AUXILIARIES_ON transition
TEST_F(SfcStateMachineTest, IdleToAuxiliariesOn)
{
    turn_on = true;  // Command to energize system
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    sfc_state_outputs expected{};
    expected.pulse_releases = false;
    verify_state(sfc_state_id::AUXILIARIES_ON, expected);
    EXPECT_TRUE(state_machine.has_transitioned());
}

// Test AUXILIARIES_ON -> BREAKERS_OFF -> BREAKERS_ON transition
TEST_F(SfcStateMachineTest, AuxiliariesOnToBreakersOn)
{
    // Navigate to AUXILIARIES_ON state
    turn_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    // Auxiliaries feedback confirms they are on -> BREAKERS_OFF
    auxiliaries_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);

    sfc_state_outputs expected{};
    expected.pulse_releases = false;
    verify_state(sfc_state_id::BREAKERS_OFF, expected);
    EXPECT_TRUE(state_machine.has_transitioned());

    // Breakers closed feedback -> BREAKERS_ON
    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    verify_state(sfc_state_id::BREAKERS_ON, expected);
    EXPECT_TRUE(state_machine.has_transitioned());
}

// Test BREAKERS_ON -> OPERATION_OFF transition
TEST_F(SfcStateMachineTest, BreakersOnToOperationOff)
{
    // Navigate to BREAKERS_ON state through proper sequence
    turn_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    auxiliaries_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    // Then auto-transition to OPERATION_OFF (no faults/inhibits)
    inputs = create_default_inputs();
    state_machine.update(inputs);

    sfc_state_outputs expected{};
    expected.pulse_releases = false;
    verify_state(sfc_state_id::OPERATION_OFF, expected);
    EXPECT_TRUE(state_machine.has_transitioned());
}

// Test OPERATION_OFF -> OPERATION_ON transition
TEST_F(SfcStateMachineTest, OperationOffToOperationOn)
{
    // Navigate to OPERATION_OFF state through proper sequence
    turn_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    auxiliaries_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF

    sfc_state_outputs expected{};
    expected.pulse_releases = false;
    verify_state(sfc_state_id::OPERATION_OFF, expected);

    // Now send start command
    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

    // Should be in OPERATION_ON due to our start command being active
    expected.pulse_releases = true;  // Only state that has pulses enabled
    verify_state(sfc_state_id::OPERATION_ON, expected);
}

// Test OPERATION_ON -> SHUTDOWN_REQUESTED transition
TEST_F(SfcStateMachineTest, OperationOnToShutdownRequested)
{
    // Navigate to OPERATION_ON state through proper sequence
    navigate_to_operation_on();

    sfc_state_outputs expected{};
    expected.pulse_releases = true;  // OPERATION_ON has pulses enabled
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // Then transition to SHUTDOWN_REQUESTED with stop command
    stop = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    expected.pulse_releases = true;
    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);
    EXPECT_TRUE(state_machine.has_transitioned());
}

// Test SHUTDOWN_REQUESTED -> OPERATION_OFF transition
TEST_F(SfcStateMachineTest, ShutdownRequestedToOperationOff)
{
    // Navigate to SHUTDOWN_REQUESTED state through proper sequence
    navigate_to_operation_on();

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    stop = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_ON -> SHUTDOWN_REQUESTED

    expected.pulse_releases = true;
    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);

    // Then transition back to OPERATION_OFF when current is extinguished
    // NOTE: Requires TWO cycles to transition (one to set acknowledgment flag, one to transition)
    current_extinguished = true;
    stop = false;  // Remove stop command
    inputs = create_default_inputs();

    // First cycle: conditions met, acknowledgment flag set, stays in SHUTDOWN_REQUESTED
    state_machine.update(inputs);
    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);

    // Second cycle: acknowledgment flag already set, now transitions to OPERATION_OFF
    state_machine.update(inputs);
    expected.pulse_releases = false;
    verify_state(sfc_state_id::OPERATION_OFF, expected);
    EXPECT_TRUE(state_machine.has_transitioned());
}

// Test complete startup sequence
TEST_F(SfcStateMachineTest, CompleteStartupSequence)
{
    // Start from IDLE
    sfc_state_outputs expected{};
    verify_state(sfc_state_id::IDLE, expected);

    // 1. Turn on system (command)
    turn_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);
    verify_state(sfc_state_id::AUXILIARIES_ON, expected);

    // 2. Auxiliaries feedback confirms they are on -> BREAKERS_OFF
    auxiliaries_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    verify_state(sfc_state_id::BREAKERS_OFF, expected);

    // 3. Breakers actually close -> BREAKERS_ON
    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    verify_state(sfc_state_id::BREAKERS_ON, expected);

    // 4. Auto-transition to OPERATION_OFF (no faults/inhibits)
    inputs = create_default_inputs();
    state_machine.update(inputs);
    verify_state(sfc_state_id::OPERATION_OFF, expected);

    // 5. Start command -> OPERATION_ON
    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);
}

// Test complete shutdown sequence
TEST_F(SfcStateMachineTest, CompleteShutdownSequence)
{
    // Start from OPERATION_ON through proper sequence
    navigate_to_operation_on();

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // 1. Request shutdown
    stop = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);
    expected.pulse_releases = true;
    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);

    // 2. Current extinguished (requires TWO cycles due to acknowledgment delay)
    current_extinguished = true;
    stop = false;  // Remove stop command
    inputs = create_default_inputs();

    // First cycle: acknowledgment flag set, stays in SHUTDOWN_REQUESTED
    state_machine.update(inputs);
    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);

    // Second cycle: transition to OPERATION_OFF
    state_machine.update(inputs);
    expected.pulse_releases = false;
    verify_state(sfc_state_id::OPERATION_OFF, expected);

    // 3. Turn off system (turn_on command goes false) -> regress through states
    turn_on = false;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_OFF -> BREAKERS_ON (due to turn_on=false)
    verify_state(sfc_state_id::BREAKERS_ON, expected);

    state_machine.update(inputs);  // BREAKERS_ON -> BREAKERS_OFF (due to turn_on=false)
    verify_state(sfc_state_id::BREAKERS_OFF, expected);

    state_machine.update(inputs);  // BREAKERS_OFF -> IDLE (due to turn_on=false)
    verify_state(sfc_state_id::IDLE, expected);
}

// Test reset functionality
TEST_F(SfcStateMachineTest, ResetFromAnyState)
{
    // Navigate to OPERATION_ON state through proper sequence
    navigate_to_operation_on();

    // Reset should return to IDLE
    state_machine.reset();
    sfc_state_outputs expected{};
    verify_state(sfc_state_id::IDLE, expected);
}

// ==================== Tests for extinguish_dc_current feature ====================

// Test extinguish request is false in non-shutdown states
TEST_F(SfcStateMachineTest, ExtinguishRequestFalseInIdleState)
{
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    const auto outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.request_extinguish_dc_current);
}

TEST_F(SfcStateMachineTest, ExtinguishRequestFalseInOperationOnState)
{
    // Navigate to OPERATION_ON state
    navigate_to_operation_on();

    const auto outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.request_extinguish_dc_current);
}

// Test extinguish request is true in SHUTDOWN_REQUESTED state
TEST_F(SfcStateMachineTest, ExtinguishRequestTrueInShutdownRequestedState)
{
    // Navigate to SHUTDOWN_REQUESTED state
    navigate_to_operation_on();

    stop = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_ON -> SHUTDOWN_REQUESTED

    const auto outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.request_extinguish_dc_current);
}

// Test extinguish request clears when exiting SHUTDOWN_REQUESTED state
TEST_F(SfcStateMachineTest, ExtinguishRequestClearsOnExitShutdownState)
{
    // Navigate to SHUTDOWN_REQUESTED state
    navigate_to_operation_on();

    stop = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_ON -> SHUTDOWN_REQUESTED

    auto outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.request_extinguish_dc_current);

    // Exit SHUTDOWN_REQUESTED by extinguishing current (requires TWO cycles)
    current_extinguished = true;
    stop = false;
    inputs = create_default_inputs();

    // First cycle: acknowledgment flag set, stays in SHUTDOWN_REQUESTED
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.request_extinguish_dc_current);  // Still in SHUTDOWN_REQUESTED

    // Second cycle: transition to OPERATION_OFF
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.request_extinguish_dc_current);  // Now in OPERATION_OFF
}

// Test shutdown sequence with extinguish output verification
TEST_F(SfcStateMachineTest, ShutdownSequenceWithExtinguishOutput)
{
    // Navigate to OPERATION_ON state
    navigate_to_operation_on();

    // Verify extinguish is false in normal operation
    auto outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.request_extinguish_dc_current);
    EXPECT_TRUE(outputs.pulse_releases);

    // Request shutdown
    stop = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_ON -> SHUTDOWN_REQUESTED

    // Verify extinguish is true during shutdown
    outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.request_extinguish_dc_current);
    EXPECT_TRUE(outputs.pulse_releases);      // Pulses stay active for controlled shutdown
    EXPECT_FALSE(outputs.freeze_integrator);  // Integrator not frozen

    // Complete shutdown (requires TWO cycles)
    current_extinguished = true;
    stop = false;
    inputs = create_default_inputs();

    // First cycle: acknowledgment flag set, stays in SHUTDOWN_REQUESTED
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.request_extinguish_dc_current);  // Still in SHUTDOWN_REQUESTED
    EXPECT_TRUE(outputs.pulse_releases);                 // Pulses still active
    EXPECT_FALSE(outputs.freeze_integrator);             // Integrator not frozen yet

    // Second cycle: transition to OPERATION_OFF
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.request_extinguish_dc_current);  // Now in OPERATION_OFF
    EXPECT_FALSE(outputs.pulse_releases);
    EXPECT_TRUE(outputs.freeze_integrator);
}

// ==================== Tests for MCB outputs ====================

// Test mv_breaker_close output is true in BREAKERS_OFF state
TEST_F(SfcStateMachineTest, McbCloseOutputInBreakersOffState)
{
    turn_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    auxiliaries_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF

    auto outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.mv_breaker_close);
    EXPECT_FALSE(outputs.mv_breaker_open);
}

// Test mv_breaker_open output is true in SHUTDOWN_REQUESTED state
TEST_F(SfcStateMachineTest, McbOpenOutputInShutdownRequestedState)
{
    navigate_to_operation_on();

    stop = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_ON -> SHUTDOWN_REQUESTED

    auto outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.mv_breaker_open);
    EXPECT_FALSE(outputs.mv_breaker_close);
}

// ==================== Tests for reset_controllers output ====================

// Test reset_controllers pulses on OPERATION_ON entry
TEST_F(SfcStateMachineTest, ResetControllersPulsesOnOperationOnEntry)
{
    navigate_to_operation_on();

    // First cycle in OPERATION_ON: reset_controllers should be true
    auto outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.reset_controllers);

    // Second update: reset_controllers should be false
    auto inputs = create_default_inputs();
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.reset_controllers);

    // Third update: still false
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.reset_controllers);
}

// Test reset_controllers pulses again on re-entry to OPERATION_ON
TEST_F(SfcStateMachineTest, ResetControllersPulsesOnReEntryToOperationOn)
{
    navigate_to_operation_on();

    // Clear the reset pulse
    auto inputs = create_default_inputs();
    state_machine.update(inputs);
    auto outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.reset_controllers);

    // Trigger shutdown
    stop = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_ON -> SHUTDOWN_REQUESTED

    // Complete shutdown
    current_extinguished = true;
    stop = false;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // First cycle (acknowledgment)
    state_machine.update(inputs);  // Second cycle -> OPERATION_OFF

    // Return to OPERATION_ON
    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

    // Reset pulse should appear again
    outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.reset_controllers);
}

// ==================== Tests for protection_not_inhibited condition ====================

// Test that OPERATION_OFF -> OPERATION_ON is blocked when protection is inhibited
TEST_F(SfcStateMachineTest, ProtectionInhibitedBlocksOperationOn)
{
    turn_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    auxiliaries_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF

    // Set start command but inhibit protection
    start = true;
    protection_not_inhibited = false;  // Protection inhibited (pending alarm confirmation)
    inputs = create_default_inputs();
    state_machine.update(inputs);

    // Should stay in OPERATION_OFF
    EXPECT_EQ(state_machine.get_current_state(), sfc_state_id::OPERATION_OFF);

    // Clear inhibit
    protection_not_inhibited = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);

    // Now should transition to OPERATION_ON
    EXPECT_EQ(state_machine.get_current_state(), sfc_state_id::OPERATION_ON);
}

// ==================== Tests for fault/trip during shutdown ====================

// Test that fault during shutdown goes directly to IDLE (trip path) and requires turn_on toggle
TEST_F(SfcStateMachineTest, FaultDuringShutdownGoesToIdleWithTripLatch)
{
    navigate_to_operation_on();

    // Trigger shutdown
    stop = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_ON -> SHUTDOWN_REQUESTED

    // Current extinguished but fault still active - should go to IDLE
    current_extinguished = true;
    any_fault_active = true;
    stop = false;
    inputs = create_default_inputs();
    state_machine.update(inputs);

    // Should be in IDLE (trip path)
    EXPECT_EQ(state_machine.get_current_state(), sfc_state_id::IDLE);

    // Trip latch should be active - turn_on still true but we should stay in IDLE
    any_fault_active = false;  // Clear fault
    inputs = create_default_inputs();
    state_machine.update(inputs);
    EXPECT_EQ(state_machine.get_current_state(), sfc_state_id::IDLE);

    // Release turn_on to clear trip latch
    turn_on = false;
    inputs = create_default_inputs();
    state_machine.update(inputs);

    // Trip latch should be cleared, but still in IDLE (turn_on is false)
    EXPECT_EQ(state_machine.get_current_state(), sfc_state_id::IDLE);

    // Now turn_on again - should transition to AUXILIARIES_ON
    turn_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    EXPECT_EQ(state_machine.get_current_state(), sfc_state_id::AUXILIARIES_ON);
}

// ==================== Tests for emergency triggers ====================

// Test emergency_off triggers immediate shutdown
TEST_F(SfcStateMachineTest, EmergencyOffTriggersShutdown)
{
    navigate_to_operation_on();

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // Emergency off
    emergency_off = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);
}

// Test breaker_is_tripping triggers immediate shutdown
TEST_F(SfcStateMachineTest, BreakerTrippingTriggersShutdown)
{
    navigate_to_operation_on();

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // Breaker tripping
    breaker_is_tripping = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);
}

// Test shutdown_request triggers shutdown from OPERATION_ON
TEST_F(SfcStateMachineTest, ShutdownRequestTriggersShutdown)
{
    navigate_to_operation_on();

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // Shutdown request
    shutdown_request = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);
}

// ==================== Tests for start command loss ====================

// Test losing start_command triggers shutdown
TEST_F(SfcStateMachineTest, LostStartCommandTriggersShutdown)
{
    navigate_to_operation_on();

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // Remove start command
    start = false;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);
}

// Test losing mcb_is_closed triggers shutdown
TEST_F(SfcStateMachineTest, LostMcbClosedTriggersShutdown)
{
    navigate_to_operation_on();

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // MCB opens unexpectedly
    breakers_on = false;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);
}

// Test losing auxiliaries triggers shutdown
TEST_F(SfcStateMachineTest, LostAuxiliariesTriggersShutdown)
{
    navigate_to_operation_on();

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // Auxiliaries lost
    auxiliaries_on = false;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);
}

// ==================== Tests for freeze_integrator output ====================

// Test freeze_integrator is true in pre-operation states
TEST_F(SfcStateMachineTest, FreezeIntegratorInPreOperationStates)
{
    // IDLE
    auto outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.freeze_integrator);  // IDLE has no freeze

    // AUXILIARIES_ON
    turn_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.freeze_integrator);

    // BREAKERS_OFF
    auxiliaries_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.freeze_integrator);

    // BREAKERS_ON
    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.freeze_integrator);

    // OPERATION_OFF
    inputs = create_default_inputs();
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.freeze_integrator);

    // OPERATION_ON - freeze should be false
    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.freeze_integrator);
}

}  // namespace yeet::application::sfc
