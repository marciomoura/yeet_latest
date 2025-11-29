#include "yeet_control/thyristor_bridge.hpp"

#include <gtest/gtest.h>

#include <algorithm>

#include "yeet_control/units.hpp"

using namespace yeet;
using namespace yeet::control;

// Test fixture for thyristor bridge tests
class ThyristorBridgeTest : public ::testing::Test {
protected:
    thyristor_bridge bridge;

    // Helper function to check if all thyristors are OFF
    bool all_thyristors_off(const std::array<bool, 6>& states)
    {
        return std::none_of(states.begin(), states.end(), [](bool s) { return s; });
    }

    // Helper function to count conducting thyristors
    int count_conducting(const std::array<bool, 6>& states) { return std::count(states.begin(), states.end(), true); }
};

// Test that all thyristors start in the OFF state
TEST_F(ThyristorBridgeTest, InitialStateAllOff)
{
    auto states = bridge.get_conduction_states();
    EXPECT_TRUE(all_thyristors_off(states));
}

// Test reset functionality
TEST_F(ThyristorBridgeTest, ResetTurnsAllThyristorsOff)
{
    // First, turn on some thyristors
    abc<voltage_t> v_ac{voltage_t{100.0}, voltage_t{-50.0}, voltage_t{-50.0}};  // Phase A positive
    voltage_t v_dc_pos{0.0};
    voltage_t v_dc_neg{0.0};
    thyristor_pulses pulses{true, true, true, true, true, true};

    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, pulses);

    // Verify at least one thyristor is on
    EXPECT_GT(count_conducting(bridge.get_conduction_states()), 0);

    // Reset and verify all are off
    bridge.reset();
    EXPECT_TRUE(all_thyristors_off(bridge.get_conduction_states()));
}

// Test that thyristors turn on only with both gate pulse AND forward bias
TEST_F(ThyristorBridgeTest, TurnOnRequiresPulseAndForwardBias)
{
    abc<voltage_t> v_ac{voltage_t{100.0}, voltage_t{-50.0}, voltage_t{-50.0}};  // Phase A at +100V, others negative
    voltage_t v_dc_pos{0.0};
    voltage_t v_dc_neg{0.0};

    // Test 1: No gate pulse, even with forward bias
    thyristor_pulses no_pulses{false, false, false, false, false, false};
    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, no_pulses);
    EXPECT_TRUE(all_thyristors_off(bridge.get_conduction_states()));

    // Test 2: Gate pulse on T1 (upper thyristor, phase A)
    // T1: Anode = v_ac[0] = +100V, Cathode = v_dc[0] = 0V -> Forward biased
    bridge.reset();
    thyristor_pulses pulse_t1{true, false, false, false, false, false};
    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, pulse_t1);

    auto states = bridge.get_conduction_states();
    EXPECT_TRUE(states[0]);   // T1 should be ON
    EXPECT_FALSE(states[1]);  // Others should be OFF
    EXPECT_FALSE(states[2]);
    EXPECT_FALSE(states[3]);
    EXPECT_FALSE(states[4]);
    EXPECT_FALSE(states[5]);
}

// Test that thyristor does NOT turn on with gate pulse but reverse bias
TEST_F(ThyristorBridgeTest, NoTurnOnWithReverseBias)
{
    // Set DC rail positive, AC negative -> reverse bias for upper thyristors
    abc<voltage_t> v_ac{voltage_t{-100.0}, voltage_t{-50.0}, voltage_t{-50.0}};
    voltage_t v_dc_pos{50.0};
    voltage_t v_dc_neg{0.0};
    thyristor_pulses pulse_t1{true, false, false, false, false, false};

    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, pulse_t1);

    // T1: Anode = -100V, Cathode = +50V -> Reverse biased, should NOT turn on
    EXPECT_TRUE(all_thyristors_off(bridge.get_conduction_states()));
}

// Test latching behavior: once on, thyristor stays on even if pulse removed
TEST_F(ThyristorBridgeTest, LatchingBehavior)
{
    abc<voltage_t> v_ac{voltage_t{100.0}, voltage_t{-50.0}, voltage_t{-50.0}};
    voltage_t v_dc_pos{0.0};
    voltage_t v_dc_neg{0.0};

    // Turn on T1 with pulse
    thyristor_pulses pulse_on{true, false, false, false, false, false};
    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, pulse_on);
    EXPECT_TRUE(bridge.get_conduction_states()[0]);

    // Remove pulse, thyristor should stay on
    thyristor_pulses pulse_off{false, false, false, false, false, false};
    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, pulse_off);
    EXPECT_TRUE(bridge.get_conduction_states()[0]);  // Still ON
}

// Test natural commutation: upper thyristor turns off when current goes negative
TEST_F(ThyristorBridgeTest, NaturalCommutationUpperThyristor)
{
    // First turn on T1 (upper thyristor, phase A)
    abc<voltage_t> v_ac{voltage_t{100.0}, voltage_t{-50.0}, voltage_t{-50.0}};
    voltage_t v_dc_pos{0.0};
    voltage_t v_dc_neg{0.0};
    thyristor_pulses pulse_t1{true, false, false, false, false, false};

    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, pulse_t1);
    EXPECT_TRUE(bridge.get_conduction_states()[0]);

    // Now simulate current reversal: phase A current becomes negative
    abc<current_t> i_ac_negative{current_t{-10.0}, current_t{5.0}, current_t{5.0}};  // Phase A negative
    current_t i_dc{10.0};                                                            // DC current still positive

    bridge.force_natural_turn_off(i_ac_negative, i_dc);
    EXPECT_FALSE(bridge.get_conduction_states()[0]);  // T1 should turn OFF
}

// Test natural commutation: lower thyristor turns off when current goes positive
TEST_F(ThyristorBridgeTest, NaturalCommutationLowerThyristor)
{
    // Turn on T4 (lower thyristor, phase A)
    // T4: Anode = v_dc[0], Cathode = v_ac[0]
    // For forward bias: v_dc[0] > v_ac[0]
    abc<voltage_t> v_ac{voltage_t{-100.0}, voltage_t{50.0}, voltage_t{50.0}};  // Phase A negative
    voltage_t v_dc_pos{0.0};
    voltage_t v_dc_neg{0.0};
    thyristor_pulses pulse_t4{false, false, false, true, false, false};

    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, pulse_t4);
    EXPECT_TRUE(bridge.get_conduction_states()[3]);  // T4 should be ON

    // Simulate current reversal: phase A current becomes positive
    abc<current_t> i_ac_positive{current_t{10.0}, current_t{-5.0}, current_t{-5.0}};  // Phase A positive
    current_t i_dc{10.0};

    bridge.force_natural_turn_off(i_ac_positive, i_dc);
    EXPECT_FALSE(bridge.get_conduction_states()[3]);  // T4 should turn OFF
}

// Test that all thyristors turn off when DC current becomes negative
TEST_F(ThyristorBridgeTest, AllOffWhenDCCurrentNegative)
{
    // Turn on thyristors
    abc<voltage_t> v_ac{voltage_t{100.0}, voltage_t{-50.0}, voltage_t{-50.0}};
    voltage_t v_dc_pos{0.0};
    voltage_t v_dc_neg{0.0};
    thyristor_pulses pulses{true, false, true, false, true, false};

    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, pulses);
    EXPECT_GT(count_conducting(bridge.get_conduction_states()), 0);

    // Set DC current to negative
    abc<current_t> i_ac{current_t{-5.0}, current_t{2.5}, current_t{2.5}};
    current_t i_dc{-1.0};

    bridge.force_natural_turn_off(i_ac, i_dc);
    EXPECT_TRUE(all_thyristors_off(bridge.get_conduction_states()));
}

// Test multiple thyristors conducting simultaneously
TEST_F(ThyristorBridgeTest, MultipleConductingThyristors)
{
    // Create conditions where T1 (phase A upper) and T2 (phase C lower) can conduct
    abc<voltage_t> v_ac{voltage_t{100.0}, voltage_t{0.0}, voltage_t{-100.0}};  // A positive, C negative
    voltage_t v_dc_pos{0.0};
    voltage_t v_dc_neg{0.0};
    thyristor_pulses pulses{true, true, false, false, false, false};

    bridge.update_conduction_state(v_ac, v_dc_pos, v_dc_neg, pulses);

    auto states = bridge.get_conduction_states();
    EXPECT_TRUE(states[0]);  // T1 should be ON (A upper, forward biased)
    EXPECT_TRUE(states[1]);  // T2 should be ON (C lower, forward biased)
    EXPECT_EQ(count_conducting(states), 2);
}

// Test thyristor topology: verify correct phase connections
TEST_F(ThyristorBridgeTest, ThyristorPhaseConnections)
{
    // Test each thyristor individually with appropriate conditions
    voltage_t v_dc_pos{0.0};
    voltage_t v_dc_neg{0.0};

    // T1: Upper thyristor, phase A (index 0)
    bridge.reset();
    abc<voltage_t> v_ac_t1{voltage_t{100.0}, voltage_t{-50.0}, voltage_t{-50.0}};
    thyristor_pulses pulse_t1{true, false, false, false, false, false};
    bridge.update_conduction_state(v_ac_t1, v_dc_pos, v_dc_neg, pulse_t1);
    EXPECT_TRUE(bridge.get_conduction_states()[0]);

    // T3: Upper thyristor, phase B (index 1, but connected to v_ac[1] which is phase B)
    bridge.reset();
    abc<voltage_t> v_ac_t3{voltage_t{-50.0}, voltage_t{100.0}, voltage_t{-50.0}};
    thyristor_pulses pulse_t3{false, false, true, false, false, false};
    bridge.update_conduction_state(v_ac_t3, v_dc_pos, v_dc_neg, pulse_t3);
    EXPECT_TRUE(bridge.get_conduction_states()[2]);

    // T5: Upper thyristor, phase C (index 2)
    bridge.reset();
    abc<voltage_t> v_ac_t5{voltage_t{-50.0}, voltage_t{-50.0}, voltage_t{100.0}};
    thyristor_pulses pulse_t5{false, false, false, false, true, false};
    bridge.update_conduction_state(v_ac_t5, v_dc_pos, v_dc_neg, pulse_t5);
    EXPECT_TRUE(bridge.get_conduction_states()[4]);
}
