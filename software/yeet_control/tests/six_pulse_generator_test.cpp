#include "yeet_control/six_pulse_generator.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet;
using namespace yeet::control;

// Test that when the generator is disabled, all output pulses are false.
TEST(SixPulseGeneratorTest, PulsesAreOffWhenDisabled) {
    six_pulse_generator gen(six_pulse_generator::reference_type::line);
    auto pulses = gen.update(angle<zero_to_two_pi>(0.0), angle<zero_to_two_pi>(0.0), false);
    EXPECT_FALSE(pulses.p1 || pulses.p2 || pulses.p3 || pulses.p4 || pulses.p5 || pulses.p6);
}

// Test the basic firing sequence with zero firing angle and a line reference.
// Each pulse should fire exactly at its natural commutation angle (0, 60, 120, etc.).
TEST(SixPulseGeneratorTest, BasicFiringSequenceLineReference) {
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle<zero_to_two_pi>(deg_to_rad(1.0)));
    const angle<zero_to_two_pi> zero_alpha(0.0);

    // Test firing of pulse 1 at its natural angle (0 deg)
    auto p1 = gen.update(angle<zero_to_two_pi>(deg_to_rad(0.0)), zero_alpha, true);
    EXPECT_TRUE(p1.p1);
    EXPECT_FALSE(p1.p2);

    // Test firing of pulse 2 at its natural angle (60 deg)
    auto p2 = gen.update(angle<zero_to_two_pi>(deg_to_rad(60.0)), zero_alpha, true);
    EXPECT_FALSE(p2.p1);
    EXPECT_TRUE(p2.p2);

    // Test firing of pulse 3 at its natural angle (120 deg)
    auto p3 = gen.update(angle<zero_to_two_pi>(deg_to_rad(120.0)), zero_alpha, true);
    EXPECT_TRUE(p3.p3);

    // Test firing of pulse 4 at its natural angle (180 deg)
    auto p4 = gen.update(angle<zero_to_two_pi>(deg_to_rad(180.0)), zero_alpha, true);
    EXPECT_TRUE(p4.p4);

    // Test firing of pulse 5 at its natural angle (240 deg)
    auto p5 = gen.update(angle<zero_to_two_pi>(deg_to_rad(240.0)), zero_alpha, true);
    EXPECT_TRUE(p5.p5);

    // Test firing of pulse 6 at its natural angle (300 deg)
    auto p6 = gen.update(angle<zero_to_two_pi>(deg_to_rad(300.0)), zero_alpha, true);
    EXPECT_TRUE(p6.p6);
}

// Test the firing sequence with a phase reference, which should be offset by -30 degrees.
TEST(SixPulseGeneratorTest, BasicFiringSequencePhaseReference) {
    // Default constructor uses phase reference
    six_pulse_generator gen(six_pulse_generator::reference_type::phase, angle<zero_to_two_pi>(deg_to_rad(10.0)));
    const angle<zero_to_two_pi> zero_alpha(0.0);

    // For phase reference, natural commutation is at +30 degrees from the reference angle.
    // So, to fire at 0 degrees (line), the phase reference must be at 30 degrees from the phase reference
    auto p1 = gen.update(angle<zero_to_two_pi>(deg_to_rad(30.0 + 0.0)), zero_alpha, true);
    EXPECT_TRUE(p1.p1);

    // To fire at 60 degrees (line), phase ref must be at 30 degrees.
    auto p2 = gen.update(angle<zero_to_two_pi>(deg_to_rad(30.0 + 65.0)), zero_alpha, true);
    EXPECT_TRUE(p2.p2);
}

// Test that the firing_angle correctly delays the pulse with a line reference.
TEST(SixPulseGeneratorTest, FiringAngleDelayP1) {
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle<zero_to_two_pi>(deg_to_rad(10.0)));

    // Firing window for P1 is [firing_angle + 0°, firing_angle + 0° + width)

    // Reference = 0°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    angle<zero_to_two_pi> firing_angle(deg_to_rad(0.0));
    auto pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(0.0)), firing_angle, true);
    EXPECT_TRUE(pulses.p1);

    // Reference = 0°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(5.0)), firing_angle, true);
    EXPECT_TRUE(pulses.p1);

    // Reference = 10°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(10.001)), firing_angle, true);
    EXPECT_FALSE(pulses.p1);

    // Reference = 10.01°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(10.01)), firing_angle, true);
    EXPECT_FALSE(pulses.p1);

    // Reference = 20°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(20.0)), firing_angle, true);
    EXPECT_FALSE(pulses.p1);

    // Now firing angle is 30°.
    firing_angle = angle<zero_to_two_pi>(deg_to_rad(30.0));
    // Reference = 0°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(0.0)), firing_angle, true);
    EXPECT_FALSE(pulses.p1);

    // Reference = 30°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(30.0)), firing_angle, true);
    EXPECT_TRUE(pulses.p1);

    // Reference = 35°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(35.0)), firing_angle, true);
    EXPECT_TRUE(pulses.p1);

    // Reference = 40°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(40.0)), firing_angle, true);
    EXPECT_FALSE(pulses.p1);

    // Reference = 50°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(50.0)), firing_angle, true);
    EXPECT_FALSE(pulses.p1);

    // Now firing angle is 60°.
    firing_angle = angle<zero_to_two_pi>(deg_to_rad(60.0));
    // Reference = 0°, pulse width = 10°, firing angle = 60°. Range is: [60°, 70°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(0.0)), firing_angle, true);
    EXPECT_FALSE(pulses.p1);

    // Reference = 60°, pulse width = 10°, firing angle = 60°. Range is: [60°, 70°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(60.0)), firing_angle, true);
    EXPECT_TRUE(pulses.p1);

    // Reference = 65°, pulse width = 10°, firing angle = 60°. Range is: [60°, 70°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(65.0)), firing_angle, true);
    EXPECT_TRUE(pulses.p1);

    // Reference = 70°, pulse width = 10°, firing angle = 60°. Range is: [60°, 70°).
    pulses = gen.update(angle<zero_to_two_pi>(deg_to_rad(70.0)), firing_angle, true);
    EXPECT_FALSE(pulses.p1);
}

// Test that the pulse remains active for its specified width.
TEST(SixPulseGeneratorTest, PulseWidth) {
    // Use a 10 degree pulse width for easier testing.
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle<zero_to_two_pi>(deg_to_rad(10.0)));
    const angle<zero_to_two_pi> zero_alpha(0.0);

    // Pulse 1 firing window is [0°, 10°).
    // It should be ON at the very start of the window.
    auto p1_on_start = gen.update(angle<zero_to_two_pi>(deg_to_rad(0.0)), zero_alpha, true);
    EXPECT_TRUE(p1_on_start.p1);

    // It should still be ON just before the end of the window.
    auto p1_on_end = gen.update(angle<zero_to_two_pi>(deg_to_rad(9.9)), zero_alpha, true);
    EXPECT_TRUE(p1_on_end.p1);

    // It should be OFF exactly at the end of the window (range is exclusive of the end).
    auto p1_off_end = gen.update(angle<zero_to_two_pi>(deg_to_rad(10.001)), zero_alpha, true);
    EXPECT_FALSE(p1_off_end.p1);
}

// This test verifies that a pulse whose active window wraps around the 0/360 degree
// boundary behaves correctly.
TEST(SixPulseGeneratorTest, AngleWrapping) {
    // Use a 5 degree pulse width.
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle<zero_to_two_pi>(deg_to_rad(5.0)));

    // To test wrapping, we need a pulse window that crosses the 0° line.
    // Let's use pulse 6, which starts naturally at 300°.
    // We'll apply a firing angle of 58° to shift its window.
    // Firing window for P6 is now [300°+58°, 300°+58°+5°) = [358°, 363°).
    // This wraps to become the union of [358°, 360°) and [0°, 3°).
    const angle<zero_to_two_pi> alpha58(deg_to_rad(58.0));

    // Test point just outside the start of the window (should be OFF)
    auto p_before = gen.update(angle<zero_to_two_pi>(deg_to_rad(357.9)), alpha58, true);
    EXPECT_FALSE(p_before.p6);

    // Test point just inside the start of the window (should be ON)
    auto p_start = gen.update(angle<zero_to_two_pi>(deg_to_rad(358.0)), alpha58, true);
    EXPECT_TRUE(p_start.p6);

    // Test point in the middle of the first part of the window (should be ON)
    auto p_mid1 = gen.update(angle<zero_to_two_pi>(deg_to_rad(359.0)), alpha58, true);
    EXPECT_TRUE(p_mid1.p6);

    // Test point in the middle of the wrapped part of the window (should be ON)
    auto p_mid2 = gen.update(angle<zero_to_two_pi>(deg_to_rad(1.0)), alpha58, true);
    EXPECT_TRUE(p_mid2.p6);

    // Test point just before the end of the window (should be ON)
    auto p_end = gen.update(angle<zero_to_two_pi>(deg_to_rad(2.9)), alpha58, true);
    EXPECT_TRUE(p_end.p6);

    // Test point exactly at the end of the window (should be OFF)
    auto p_after = gen.update(angle<zero_to_two_pi>(deg_to_rad(3.0001)), alpha58, true);
    EXPECT_FALSE(p_after.p6);
}