#include "sfc/sfc_state_machine.hpp"

#include <gtest/gtest.h>

namespace yeet::application::sfc {

class SfcStateMachineTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        state_machine.reset();
        auxiliaries_on = false;
        breakers_on = false;
        start = false;
        stop = false;
        current_extinguished = false;
        protection_not_inhibited = true;  // Protection not inhibited by default
    }

    void verify_state(sfc_state_id expected_state, const sfc_state_outputs& expected_outputs)
    {
        EXPECT_EQ(state_machine.get_current_state(), expected_state);
        const auto outputs = state_machine.get_outputs();
        EXPECT_EQ(outputs.pulse_releases, expected_outputs.pulse_releases);
    }

    sfc_state_inputs create_default_inputs()
    {
        bool shutdown_request_false = false;
        bool any_fault_false = false;
        bool breaker_is_tripping_false = false;
        bool emergency_off_false = false;
        bool door_is_open_false = false;
        sfc_state_inputs inputs{.auxiliaries_are_on_event = mock_event("auxiliaries_on", auxiliaries_on),
            .mcb_is_closed_event = mock_event("breakers_on", breakers_on),
            .start_command = mock_event("start", start),
            .stop_command = mock_event("stop", stop),
            .shutdown_request = mock_event("shutdown_request", shutdown_request_false),
            .is_current_extinguished = mock_event("current_extinguished", current_extinguished),
            .protection_not_inhibited_event = mock_event("protection_not_inhibited", protection_not_inhibited),
            .any_protection_fault_active_event = mock_event("any_fault_active", any_fault_false),
            .breaker_is_tripping_event = mock_event("breaker_is_tripping", breaker_is_tripping_false),
            .emergency_off_event = mock_event("emergency_off", emergency_off_false),
            .door_is_open_event = mock_event("door_is_open", door_is_open_false)};
        return inputs;
    }

    sfc_state_machine state_machine;
    bool auxiliaries_on;
    bool breakers_on;
    bool start;
    bool stop;
    bool current_extinguished;
    bool protection_not_inhibited;
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
    auxiliaries_on = true;
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
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);

    // First transition to BREAKERS_OFF when breakers command is given
    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);

    sfc_state_outputs expected{};
    expected.pulse_releases = false;
    verify_state(sfc_state_id::BREAKERS_OFF, expected);
    EXPECT_TRUE(state_machine.has_transitioned());

    // Then transition to BREAKERS_ON when breakers actually close
    state_machine.update(inputs);
    verify_state(sfc_state_id::BREAKERS_ON, expected);
    EXPECT_TRUE(state_machine.has_transitioned());
}

// Test BREAKERS_ON -> OPERATION_OFF transition
TEST_F(SfcStateMachineTest, BreakersOnToOperationOff)
{
    // Navigate to BREAKERS_ON state through proper sequence
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    // Then transition to OPERATION_OFF with start command
    start = true;
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
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF

    sfc_state_outputs expected{};
    expected.pulse_releases = false;
    verify_state(sfc_state_id::OPERATION_OFF, expected);

    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

    // Should be in OPERATION_ON due to our start command being active
    expected.pulse_releases = true;  // Only state that has pulses enabled
    verify_state(sfc_state_id::OPERATION_ON, expected);
}

// Test OPERATION_ON -> SHUTDOWN_REQUESTED transition
TEST_F(SfcStateMachineTest, OperationOnToShutdownRequested)
{
    // Navigate to OPERATION_ON state through proper sequence
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON
    sfc_state_outputs expected{};
    expected.pulse_releases = true;  // OPERATION_ON has pulses enabled
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // Then transition to SHUTDOWN_REQUESTED with stop command
    stop = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);

    expected.pulse_releases = true;
    verify_state(sfc_state_id::SHUTDOWN_REQUESTED, expected);
    EXPECT_TRUE(state_machine.has_transitioned());
}

// Test SHUTDOWN_REQUESTED -> OPERATION_OFF transition
TEST_F(SfcStateMachineTest, ShutdownRequestedToOperationOff)
{
    // Navigate to SHUTDOWN_REQUESTED state through proper sequence
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    stop = true;
    inputs = create_default_inputs();
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

    // 1. Turn on auxiliaries
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);
    verify_state(sfc_state_id::AUXILIARIES_ON, expected);

    // 2. Command breakers on -> BREAKERS_OFF
    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    verify_state(sfc_state_id::BREAKERS_OFF, expected);

    // 3. Breakers actually close -> BREAKERS_ON
    state_machine.update(inputs);
    verify_state(sfc_state_id::BREAKERS_ON, expected);

    // 4. Start operation -> OPERATION_OFF
    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);
    verify_state(sfc_state_id::OPERATION_OFF, expected);

    // 5. Continue to OPERATION_ON
    state_machine.update(inputs);
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);
}

// Test complete shutdown sequence
TEST_F(SfcStateMachineTest, CompleteShutdownSequence)
{
    // Start from OPERATION_ON through proper sequence
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

    sfc_state_outputs expected{};
    expected.pulse_releases = true;
    verify_state(sfc_state_id::OPERATION_ON, expected);

    // 1. Request shutdown
    stop = true;
    inputs = create_default_inputs();
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

    // 3. Turn off breakers
    breakers_on = false;
    start = false;  // Remove start command
    inputs = create_default_inputs();
    state_machine.update(inputs);
    verify_state(sfc_state_id::BREAKERS_ON, expected);

    // 4. Breakers actually open -> BREAKERS_OFF
    state_machine.update(inputs);
    verify_state(sfc_state_id::BREAKERS_OFF, expected);

    // 5. Turn off auxiliaries, in this case we go directly to idle
    auxiliaries_on = false;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_OFF -> IDLE
    state_machine.update(inputs);  // AUXILIARIES_ON -> IDLE
    verify_state(sfc_state_id::IDLE, expected);
}

// Test reset functionality
TEST_F(SfcStateMachineTest, ResetFromAnyState)
{
    // Navigate to OPERATION_ON state through proper sequence
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

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
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

    const auto outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.request_extinguish_dc_current);
}

// Test extinguish request is true in SHUTDOWN_REQUESTED state
TEST_F(SfcStateMachineTest, ExtinguishRequestTrueInShutdownRequestedState)
{
    // Navigate to SHUTDOWN_REQUESTED state
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

    stop = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // OPERATION_ON -> SHUTDOWN_REQUESTED

    const auto outputs = state_machine.get_outputs();
    EXPECT_TRUE(outputs.request_extinguish_dc_current);
}

// Test extinguish request clears when exiting SHUTDOWN_REQUESTED state
TEST_F(SfcStateMachineTest, ExtinguishRequestClearsOnExitShutdownState)
{
    // Navigate to SHUTDOWN_REQUESTED state
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

    stop = true;
    inputs = create_default_inputs();
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
    auxiliaries_on = true;
    auto inputs = create_default_inputs();
    state_machine.update(inputs);  // IDLE -> AUXILIARIES_ON

    breakers_on = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // AUXILIARIES_ON -> BREAKERS_OFF
    state_machine.update(inputs);  // BREAKERS_OFF -> BREAKERS_ON

    start = true;
    inputs = create_default_inputs();
    state_machine.update(inputs);  // BREAKERS_ON -> OPERATION_OFF
    state_machine.update(inputs);  // OPERATION_OFF -> OPERATION_ON

    // Verify extinguish is false in normal operation
    auto outputs = state_machine.get_outputs();
    EXPECT_FALSE(outputs.request_extinguish_dc_current);
    EXPECT_TRUE(outputs.pulse_releases);

    // Request shutdown
    stop = true;
    inputs = create_default_inputs();
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

}  // namespace yeet::application::sfc
