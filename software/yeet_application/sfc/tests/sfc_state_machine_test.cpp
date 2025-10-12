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
        sfc_state_inputs inputs{.auxiliaries_are_on_event = mock_event("auxiliaries_on", auxiliaries_on),
            .breakers_are_on_event = mock_event("breakers_on", breakers_on),
            .start_command = mock_event("start", start),
            .stop_command = mock_event("stop", stop),
            .shutdown_request = mock_event("shutdown_request", shutdown_request_false),
            .is_current_estinguished = mock_event("is_current_estinguished", current_extinguished),
            .is_excitation_ready = mock_event("is_excitation_ready", excitation_ready)};
        return inputs;
    }

    sfc_state_machine state_machine;
    bool auxiliaries_on;
    bool breakers_on;
    bool start;
    bool stop;
    bool current_extinguished;
    bool excitation_ready;
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
    current_extinguished = true;
    stop = false;  // Remove stop command
    inputs = create_default_inputs();
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

    // 2. Current extinguished
    current_extinguished = true;
    expected.pulse_releases = false;
    stop = false;  // Remove stop command
    inputs = create_default_inputs();
    state_machine.update(inputs);
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

}  // namespace yeet::application::sfc
