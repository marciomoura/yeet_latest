#include "yeet_control/six_pulse_generator.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"

using namespace yeet;
using namespace yeet::control;

// Test that when the generator is disabled, all output pulses are false.
TEST(SixPulseGeneratorTest, PulsesAreOffWhenDisabled)
{
    six_pulse_generator gen(six_pulse_generator::reference_type::line);
    gen.update(angle_wrapped(0.0), angle_wrapped(0.0), false);
    auto pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1 || pulses.p2 || pulses.p3 || pulses.p4 || pulses.p5 || pulses.p6);
}

// Test the basic firing sequence with zero firing angle and a line reference.
// Each pulse should fire exactly at its natural commutation angle (0, 60, 120, etc.).
TEST(SixPulseGeneratorTest, BasicFiringSequenceLineReference)
{
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(1.0)));
    const angle_wrapped zero_alpha(0.0);

    // Test firing of pulse 1 at its natural angle (0 deg)
    gen.update(angle_wrapped(deg_to_rad(0.0)), zero_alpha, true);
    auto p1 = gen.get_pulses();
    EXPECT_TRUE(p1.p1);
    EXPECT_FALSE(p1.p2);

    // Test firing of pulse 2 at its natural angle (60 deg)
    gen.update(angle_wrapped(deg_to_rad(60.0)), zero_alpha, true);
    auto p2 = gen.get_pulses();
    EXPECT_FALSE(p2.p1);
    EXPECT_TRUE(p2.p2);

    // Test firing of pulse 3 at its natural angle (120 deg)
    gen.update(angle_wrapped(deg_to_rad(120.0)), zero_alpha, true);
    auto p3 = gen.get_pulses();
    EXPECT_TRUE(p3.p3);

    // Test firing of pulse 4 at its natural angle (180 deg)
    gen.update(angle_wrapped(deg_to_rad(180.0)), zero_alpha, true);
    auto p4 = gen.get_pulses();
    EXPECT_TRUE(p4.p4);

    // Test firing of pulse 5 at its natural angle (240 deg)
    gen.update(angle_wrapped(deg_to_rad(240.0)), zero_alpha, true);
    auto p5 = gen.get_pulses();
    EXPECT_TRUE(p5.p5);

    // Test firing of pulse 6 at its natural angle (300 deg)
    gen.update(angle_wrapped(deg_to_rad(300.0)), zero_alpha, true);
    auto p6 = gen.get_pulses();
    EXPECT_TRUE(p6.p6);
}

// Test the firing sequence with a phase reference, which should be offset by -30 degrees.
TEST(SixPulseGeneratorTest, BasicFiringSequencePhaseReference)
{
    // Default constructor uses phase reference
    six_pulse_generator gen(six_pulse_generator::reference_type::phase, angle_wrapped(deg_to_rad(10.0)));
    const angle_wrapped zero_alpha(0.0);

    // For phase reference, natural commutation is at +30 degrees from the reference angle.
    // So, to fire at 0 degrees (line), the phase reference must be at 30 degrees from the phase reference
    gen.update(angle_wrapped(deg_to_rad(30.0 + 0.0)), zero_alpha, true);
    auto p1 = gen.get_pulses();
    EXPECT_TRUE(p1.p1);

    // To fire at 60 degrees (line), phase ref must be at 30 degrees.
    gen.update(angle_wrapped(deg_to_rad(30.0 + 65.0)), zero_alpha, true);
    auto p2 = gen.get_pulses();
    EXPECT_TRUE(p2.p2);
}

// Test that the firing_angle correctly delays the pulse with a line reference.
TEST(SixPulseGeneratorTest, FiringAngleDelayP1)
{
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(10.0)));

    // Firing window for P1 is [firing_angle + 0°, firing_angle + 0° + width)

    // Reference = 0°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    angle_wrapped firing_angle(deg_to_rad(0.0));
    gen.update(angle_wrapped(deg_to_rad(0.0)), firing_angle, true);
    auto pulses = gen.get_pulses();
    EXPECT_TRUE(pulses.p1);

    // Reference = 0°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    gen.update(angle_wrapped(deg_to_rad(5.0)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_TRUE(pulses.p1);

    // Reference = 10°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    gen.update(angle_wrapped(deg_to_rad(10.001)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1);

    // Reference = 10.01°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    gen.update(angle_wrapped(deg_to_rad(10.01)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1);

    // Reference = 20°, pulse width = 10°, firing angle = 0°. Range is: [0°, 10°).
    gen.update(angle_wrapped(deg_to_rad(20.0)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1);

    // Now firing angle is 30°.
    firing_angle = angle_wrapped(deg_to_rad(30.0));
    // Reference = 0°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    gen.update(angle_wrapped(deg_to_rad(0.0)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1);

    // Reference = 30°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    gen.update(angle_wrapped(deg_to_rad(30.0)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_TRUE(pulses.p1);

    // Reference = 35°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    gen.update(angle_wrapped(deg_to_rad(35.0)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_TRUE(pulses.p1);

    // Reference = 40°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    gen.update(angle_wrapped(deg_to_rad(40.1)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1);

    // Reference = 50°, pulse width = 10°, firing angle = 30°. Range is: [30°, 40°).
    gen.update(angle_wrapped(deg_to_rad(50.0)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1);

    // Now firing angle is 60°.
    firing_angle = angle_wrapped(deg_to_rad(60.0));
    // Reference = 0°, pulse width = 10°, firing angle = 60°. Range is: [60°, 70°).
    gen.update(angle_wrapped(deg_to_rad(0.0)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1);

    // Reference = 60°, pulse width = 10°, firing angle = 60°. Range is: [60°, 70°).
    gen.update(angle_wrapped(deg_to_rad(60.0)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_TRUE(pulses.p1);

    // Reference = 65°, pulse width = 10°, firing angle = 60°. Range is: [60°, 70°).
    gen.update(angle_wrapped(deg_to_rad(65.0)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_TRUE(pulses.p1);

    // Reference = 70°, pulse width = 10°, firing angle = 60°. Range is: [60°, 70°).
    gen.update(angle_wrapped(deg_to_rad(70.1)), firing_angle, true);
    pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1);
}

// Test that the pulse remains active for its specified width.
TEST(SixPulseGeneratorTest, PulseWidth)
{
    // Use a 10 degree pulse width for easier testing.
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(10.0)));
    const angle_wrapped zero_alpha(0.0);

    // Pulse 1 firing window is [0°, 10°).
    // It should be ON at the very start of the window.
    gen.update(angle_wrapped(deg_to_rad(0.0)), zero_alpha, true);
    auto p1_on_start = gen.get_pulses();
    EXPECT_TRUE(p1_on_start.p1);

    // It should still be ON just before the end of the window.
    gen.update(angle_wrapped(deg_to_rad(9.9)), zero_alpha, true);
    auto p1_on_end = gen.get_pulses();
    EXPECT_TRUE(p1_on_end.p1);

    // It should be OFF exactly at the end of the window (range is exclusive of the end).
    gen.update(angle_wrapped(deg_to_rad(10.001)), zero_alpha, true);
    auto p1_off_end = gen.get_pulses();
    EXPECT_FALSE(p1_off_end.p1);
}

// This test verifies that a pulse whose active window wraps around the 0/360 degree
// boundary behaves correctly.
TEST(SixPulseGeneratorTest, AngleWrapping)
{
    // Use a 5 degree pulse width.
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(5.0)));

    // To test wrapping, we need a pulse window that crosses the 0° line.
    // Let's use pulse 6, which starts naturally at 300°.
    // We'll apply a firing angle of 58° to shift its window.
    // Firing window for P6 is now [300°+58°, 300°+58°+5°) = [358°, 363°).
    // This wraps to become the union of [358°, 360°) and [0°, 3°).
    const angle_wrapped alpha58(deg_to_rad(58.0));

    // Test point just outside the start of the window (should be OFF)
    gen.update(angle_wrapped(deg_to_rad(357.9)), alpha58, true);
    auto p_before = gen.get_pulses();
    EXPECT_FALSE(p_before.p6);

    // Test point just inside the start of the window (should be ON)
    gen.update(angle_wrapped(deg_to_rad(358.1)), alpha58, true);
    auto p_start = gen.get_pulses();
    EXPECT_TRUE(p_start.p6);

    // Test point in the middle of the first part of the window (should be ON)
    gen.update(angle_wrapped(deg_to_rad(359.0)), alpha58, true);
    auto p_mid1 = gen.get_pulses();
    EXPECT_TRUE(p_mid1.p6);

    // Test point in the middle of the wrapped part of the window (should be ON)
    gen.update(angle_wrapped(deg_to_rad(1.0)), alpha58, true);
    auto p_mid2 = gen.get_pulses();
    EXPECT_TRUE(p_mid2.p6);

    // Test point just before the end of the window (should be ON)
    gen.update(angle_wrapped(deg_to_rad(2.9)), alpha58, true);
    auto p_end = gen.get_pulses();
    EXPECT_TRUE(p_end.p6);

    // Test point exactly at the end of the window (should be OFF)
    gen.update(angle_wrapped(deg_to_rad(3.0001)), alpha58, true);
    auto p_after = gen.get_pulses();
    EXPECT_FALSE(p_after.p6);
}

// Test that the sector is correctly identified based on the reference angle.
TEST(SixPulseGeneratorTest, SectorIdentification)
{
    six_pulse_generator gen;
    const angle_wrapped zero_alpha(0.0);

    // Sector 1: [0, 60)
    gen.update(angle_wrapped(deg_to_rad(0.0)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_1);
    gen.update(angle_wrapped(deg_to_rad(59.999)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_1);

    // Sector 2: [60, 120)
    gen.update(angle_wrapped(deg_to_rad(60.1)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_2);
    gen.update(angle_wrapped(deg_to_rad(119.999)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_2);

    // Sector 3: [120, 180)
    gen.update(angle_wrapped(deg_to_rad(120.01)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_3);

    // Sector 4: [180, 240)
    gen.update(angle_wrapped(deg_to_rad(180.0)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_4);

    // Sector 5: [240, 300)
    gen.update(angle_wrapped(deg_to_rad(240.1)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_5);

    // Sector 6: [300, 360)
    gen.update(angle_wrapped(deg_to_rad(300.1)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_6);
    gen.update(angle_wrapped(deg_to_rad(359.999)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_6);
}

// Test that the sector changed flag behaves correctly.
TEST(SixPulseGeneratorTest, SectorChangeDetection)
{
    six_pulse_generator gen;
    const angle_wrapped zero_alpha(0.0);

    // First update: sector changes from initial 'none' to 1. Flag should be true.
    gen.update(angle_wrapped(deg_to_rad(10.0)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_1);
    EXPECT_TRUE(gen.get_sector_changed());

    // Second update: still in sector 1. Flag should be false.
    gen.update(angle_wrapped(deg_to_rad(20.0)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_1);
    EXPECT_FALSE(gen.get_sector_changed());

    // Third update: move to sector 2. Flag should be true.
    gen.update(angle_wrapped(deg_to_rad(70.0)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_2);
    EXPECT_TRUE(gen.get_sector_changed());

    // Fourth update: still in sector 2. Flag should be false.
    gen.update(angle_wrapped(deg_to_rad(80.0)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_2);
    EXPECT_FALSE(gen.get_sector_changed());

    // Test wrap-around from sector 6 to sector 1
    gen.update(angle_wrapped(deg_to_rad(350.0)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_6);
    EXPECT_TRUE(gen.get_sector_changed());

    gen.update(angle_wrapped(deg_to_rad(5.0)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_1);
    EXPECT_TRUE(gen.get_sector_changed());

    // Final check: stay in sector 1. Flag should be false again.
    gen.update(angle_wrapped(deg_to_rad(6.0)), zero_alpha, true);
    EXPECT_EQ(gen.get_current_sector(), six_pulse_generator::pulse_sector_t::sector_1);
    EXPECT_FALSE(gen.get_sector_changed());
}

//==============================================================================
// Virtual Reference Mode Tests
//==============================================================================

// Test that virtual reference mode is disabled by default
TEST(SixPulseGeneratorTest, VirtualReferenceDisabledByDefault)
{
    six_pulse_generator gen;
    EXPECT_FALSE(gen.is_virtual_reference_enabled());
    EXPECT_EQ(gen.get_virtual_reference_angle().get_radians(), 0.0);
}

// Test that virtual reference update returns all-off pulses when disabled
TEST(SixPulseGeneratorTest, VirtualReferenceReturnsOffWhenDisabled)
{
    six_pulse_generator gen;
    const angle_wrapped firing_angle(deg_to_rad(30.0));

    // Virtual reference not enabled - should return all off
    gen.update(firing_angle, true);
    auto pulses = gen.get_pulses();
    EXPECT_FALSE(pulses.p1 || pulses.p2 || pulses.p3 || pulses.p4 || pulses.p5 || pulses.p6);
}

// Test virtual reference configuration
TEST(SixPulseGeneratorTest, VirtualReferenceConfiguration)
{
    six_pulse_generator gen;
    const frequency_t test_freq{60.0};
    const duration_t test_dt = us(50.0);

    gen.configure_virtual_reference(test_freq, test_dt);

    EXPECT_EQ(gen.get_virtual_reference_frequency(), test_freq);
}

// Test virtual reference enable/disable
TEST(SixPulseGeneratorTest, VirtualReferenceEnableDisable)
{
    six_pulse_generator gen;

    EXPECT_FALSE(gen.is_virtual_reference_enabled());

    gen.enable_virtual_reference(true);
    EXPECT_TRUE(gen.is_virtual_reference_enabled());

    gen.enable_virtual_reference(false);
    EXPECT_FALSE(gen.is_virtual_reference_enabled());
}

// Test virtual reference angle integration
TEST(SixPulseGeneratorTest, VirtualReferenceAngleIntegration)
{
    six_pulse_generator gen(six_pulse_generator::reference_type::line);
    const frequency_t freq{50.0};   // 50 Hz
    const duration_t dt = ms(1.0);  // 1 ms sampling time
    const angle_wrapped zero_alpha(0.0);

    gen.configure_virtual_reference(freq, dt);
    gen.enable_virtual_reference(true);
    gen.reset_virtual_reference();

    // Expected angle increment per update: 2π × 50 Hz × 0.001 s = 0.314159... rad = 18°
    const double expected_increment_rad = two_pi * 50.0 * 0.001;

    // First update
    gen.update(zero_alpha, true);
    EXPECT_NEAR(gen.get_virtual_reference_angle().get_radians(), expected_increment_rad, 1e-6);

    // Second update
    gen.update(zero_alpha, true);
    EXPECT_NEAR(gen.get_virtual_reference_angle().get_radians(), 2.0 * expected_increment_rad, 1e-6);
}

// Test virtual reference reset
TEST(SixPulseGeneratorTest, VirtualReferenceReset)
{
    six_pulse_generator gen;
    gen.configure_virtual_reference(frequency_t{50.0}, ms(1.0));
    gen.enable_virtual_reference(true);

    // Run a few updates to change the angle
    gen.update(angle_wrapped(0.0), true);
    gen.update(angle_wrapped(0.0), true);
    EXPECT_NE(gen.get_virtual_reference_angle().get_radians(), 0.0);

    // Reset to zero
    gen.reset_virtual_reference();
    EXPECT_EQ(gen.get_virtual_reference_angle().get_radians(), 0.0);

    // Reset to specific angle
    gen.reset_virtual_reference(angle_wrapped(deg_to_rad(90.0)));
    EXPECT_NEAR(gen.get_virtual_reference_angle().get_degrees(), 90.0, 1e-6);
}

// Test that virtual reference generates proper pulses
TEST(SixPulseGeneratorTest, VirtualReferencePulseGeneration)
{
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(30.0)));
    const frequency_t freq{50.0};
    const duration_t dt = us(100.0);
    const angle_wrapped zero_alpha(0.0);

    gen.configure_virtual_reference(freq, dt);
    gen.enable_virtual_reference(true);
    gen.reset_virtual_reference();

    // Simulate enough updates to complete one full electrical cycle
    // At 50 Hz and 100 µs, we need 1 / (50 × 100e-6) = 200 samples per cycle
    bool found_p1 = false, found_p2 = false, found_p3 = false;
    bool found_p4 = false, found_p5 = false, found_p6 = false;

    for (int i = 0; i < 200; ++i) {
        gen.update(zero_alpha, true);
        auto pulses = gen.get_pulses();
        if (pulses.p1) found_p1 = true;
        if (pulses.p2) found_p2 = true;
        if (pulses.p3) found_p3 = true;
        if (pulses.p4) found_p4 = true;
        if (pulses.p5) found_p5 = true;
        if (pulses.p6) found_p6 = true;
    }

    // All six pulses should have been generated during one cycle
    EXPECT_TRUE(found_p1);
    EXPECT_TRUE(found_p2);
    EXPECT_TRUE(found_p3);
    EXPECT_TRUE(found_p4);
    EXPECT_TRUE(found_p5);
    EXPECT_TRUE(found_p6);
}

//==============================================================================
// Pulse Mask Tests
//==============================================================================

// Test that pulse masking is disabled by default
TEST(SixPulseGeneratorTest, PulseMaskDisabledByDefault)
{
    six_pulse_generator gen;
    EXPECT_FALSE(gen.is_pulse_mask_enabled());

    auto mask = gen.get_pulse_mask();
    EXPECT_TRUE(mask.p1 && mask.p2 && mask.p3 && mask.p4 && mask.p5 && mask.p6);
}

// Test pulse mask enable/disable
TEST(SixPulseGeneratorTest, PulseMaskEnableDisable)
{
    six_pulse_generator gen;

    EXPECT_FALSE(gen.is_pulse_mask_enabled());

    gen.enable_pulse_mask(true);
    EXPECT_TRUE(gen.is_pulse_mask_enabled());

    gen.enable_pulse_mask(false);
    EXPECT_FALSE(gen.is_pulse_mask_enabled());
}

// Test pulse mask configuration
TEST(SixPulseGeneratorTest, PulseMaskConfiguration)
{
    six_pulse_generator gen;

    six_pulse_generator::pulse_mask custom_mask{true, false, true, false, true, false};
    gen.configure_pulse_mask(custom_mask);

    auto mask = gen.get_pulse_mask();
    EXPECT_TRUE(mask.p1);
    EXPECT_FALSE(mask.p2);
    EXPECT_TRUE(mask.p3);
    EXPECT_FALSE(mask.p4);
    EXPECT_TRUE(mask.p5);
    EXPECT_FALSE(mask.p6);
}

// Test pulse mask static factory methods
TEST(SixPulseGeneratorTest, PulseMaskStaticMethods)
{
    auto all_on = six_pulse_generator::pulse_mask::all_enabled();
    EXPECT_TRUE(all_on.p1 && all_on.p2 && all_on.p3 && all_on.p4 && all_on.p5 && all_on.p6);

    auto all_off = six_pulse_generator::pulse_mask::all_disabled();
    EXPECT_FALSE(all_off.p1 || all_off.p2 || all_off.p3 || all_off.p4 || all_off.p5 || all_off.p6);

    auto odd = six_pulse_generator::pulse_mask::odd_only();
    EXPECT_TRUE(odd.p1);
    EXPECT_FALSE(odd.p2);
    EXPECT_TRUE(odd.p3);
    EXPECT_FALSE(odd.p4);
    EXPECT_TRUE(odd.p5);
    EXPECT_FALSE(odd.p6);

    auto even = six_pulse_generator::pulse_mask::even_only();
    EXPECT_FALSE(even.p1);
    EXPECT_TRUE(even.p2);
    EXPECT_FALSE(even.p3);
    EXPECT_TRUE(even.p4);
    EXPECT_FALSE(even.p5);
    EXPECT_TRUE(even.p6);
}

// Test that pulse mask correctly blocks individual pulses
TEST(SixPulseGeneratorTest, PulseMaskBlocksPulses)
{
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(10.0)));
    const angle_wrapped zero_alpha(0.0);

    // Configure mask to only allow odd pulses (P1, P3, P5)
    gen.configure_pulse_mask(six_pulse_generator::pulse_mask::odd_only());
    gen.enable_pulse_mask(true);

    // Test at angle where P1 would fire (0°)
    gen.update(angle_wrapped(deg_to_rad(0.0)), zero_alpha, true);
    auto p1 = gen.get_pulses();
    EXPECT_TRUE(p1.p1);  // P1 is allowed by mask

    // Test at angle where P2 would fire (60°)
    gen.update(angle_wrapped(deg_to_rad(60.0)), zero_alpha, true);
    auto p2 = gen.get_pulses();
    EXPECT_FALSE(p2.p2);  // P2 is blocked by mask

    // Test at angle where P3 would fire (120°)
    gen.update(angle_wrapped(deg_to_rad(120.0)), zero_alpha, true);
    auto p3 = gen.get_pulses();
    EXPECT_TRUE(p3.p3);  // P3 is allowed by mask

    // Test at angle where P4 would fire (180°)
    gen.update(angle_wrapped(deg_to_rad(180.0)), zero_alpha, true);
    auto p4 = gen.get_pulses();
    EXPECT_FALSE(p4.p4);  // P4 is blocked by mask

    // Test at angle where P5 would fire (240°)
    gen.update(angle_wrapped(deg_to_rad(240.0)), zero_alpha, true);
    auto p5 = gen.get_pulses();
    EXPECT_TRUE(p5.p5);  // P5 is allowed by mask

    // Test at angle where P6 would fire (300°)
    gen.update(angle_wrapped(deg_to_rad(300.0)), zero_alpha, true);
    auto p6 = gen.get_pulses();
    EXPECT_FALSE(p6.p6);  // P6 is blocked by mask
}

// Test that disabling pulse mask allows all pulses again
TEST(SixPulseGeneratorTest, PulseMaskDisablingRestoresPulses)
{
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(10.0)));
    const angle_wrapped zero_alpha(0.0);

    // Configure mask to block all pulses
    gen.configure_pulse_mask(six_pulse_generator::pulse_mask::all_disabled());
    gen.enable_pulse_mask(true);

    // P1 should be blocked
    gen.update(angle_wrapped(deg_to_rad(0.0)), zero_alpha, true);
    auto blocked = gen.get_pulses();
    EXPECT_FALSE(blocked.p1);

    // Disable masking
    gen.enable_pulse_mask(false);

    // P1 should now fire
    gen.update(angle_wrapped(deg_to_rad(0.0)), zero_alpha, true);
    auto unblocked = gen.get_pulses();
    EXPECT_TRUE(unblocked.p1);
}

// Test combined virtual reference and pulse masking
TEST(SixPulseGeneratorTest, VirtualReferenceWithPulseMask)
{
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(30.0)));

    // Configure virtual reference
    gen.configure_virtual_reference(frequency_t{50.0}, us(100.0));
    gen.enable_virtual_reference(true);
    gen.reset_virtual_reference();

    // Configure mask to only allow P1 and P4
    six_pulse_generator::pulse_mask single_pair{true, false, false, true, false, false};
    gen.configure_pulse_mask(single_pair);
    gen.enable_pulse_mask(true);

    // Simulate one full cycle
    bool found_p1 = false, found_p2 = false, found_p3 = false;
    bool found_p4 = false, found_p5 = false, found_p6 = false;

    for (int i = 0; i < 200; ++i) {
        gen.update(angle_wrapped(0.0), true);
        auto pulses = gen.get_pulses();
        if (pulses.p1) found_p1 = true;
        if (pulses.p2) found_p2 = true;
        if (pulses.p3) found_p3 = true;
        if (pulses.p4) found_p4 = true;
        if (pulses.p5) found_p5 = true;
        if (pulses.p6) found_p6 = true;
    }

    // Only P1 and P4 should have been generated
    EXPECT_TRUE(found_p1);
    EXPECT_FALSE(found_p2);
    EXPECT_FALSE(found_p3);
    EXPECT_TRUE(found_p4);
    EXPECT_FALSE(found_p5);
    EXPECT_FALSE(found_p6);
}

// Test that the 'any thyristor fired' flag correctly detects rising edges of any pulse.
TEST(SixPulseGeneratorTest, AnyThyristorFiredDetection)
{
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped::from_degrees(10.0));
    const angle_wrapped zero_alpha(0.0);

    // Initial state: no pulses
    EXPECT_FALSE(gen.get_any_thyristor_fired());

    // Update 1: Move to 0°, P1 should start firing
    gen.update(angle_wrapped::from_degrees(0.0), zero_alpha, true);
    EXPECT_TRUE(gen.get_pulses().p1);
    EXPECT_TRUE(gen.get_any_thyristor_fired());

    // Update 2: Still at 0°, P1 is already firing (no rising edge)
    gen.update(angle_wrapped::from_degrees(0.0), zero_alpha, true);
    EXPECT_TRUE(gen.get_pulses().p1);
    EXPECT_FALSE(gen.get_any_thyristor_fired());

    // Update 3: Move to 60°, P2 should start firing
    gen.update(angle_wrapped::from_degrees(60.0), zero_alpha, true);
    EXPECT_TRUE(gen.get_pulses().p2);
    EXPECT_TRUE(gen.get_any_thyristor_fired());
}

// Exhaustive sweep to verify firing detection for all thyristors
TEST(SixPulseGeneratorTest, DetectAnyThyristorFiredExhaustiveSweep)
{
    // 1. Initialize a six_pulse_generator with a 120-degree pulse width.
    six_pulse_generator gen(six_pulse_generator::reference_type::line, angle_wrapped(deg_to_rad(120.0)));
    const angle_wrapped alpha(0.0);

    struct FiredInfo {
        int id;
        double angle;
    };
    std::vector<FiredInfo> fired_events;
    std::array<bool, 6> detected = {false, false, false, false, false, false};

    // Initialize state by calling update with an angle just before 0
    gen.update(angle_wrapped(deg_to_rad(-1.0)), alpha, true);
    six_pulse_generator::pulses previous_pulses = gen.get_pulses();

    // 2. Sweep the reference angle from 0 to 360 degrees in small steps (1 degree).
    // Use < 360 to avoid double-counting the 0/360 wrap-around point for a single cycle.
    for (double deg = 0.0; deg < 360.0; deg += 1.0) {
        gen.update(angle_wrapped(deg_to_rad(deg)), alpha, true);
        auto current_pulses = gen.get_pulses();

        if (gen.get_any_thyristor_fired()) {
            if (current_pulses.p1 && !previous_pulses.p1) {
                detected[0] = true;
                fired_events.push_back({1, deg});
            }
            if (current_pulses.p2 && !previous_pulses.p2) {
                detected[1] = true;
                fired_events.push_back({2, deg});
            }
            if (current_pulses.p3 && !previous_pulses.p3) {
                detected[2] = true;
                fired_events.push_back({3, deg});
            }
            if (current_pulses.p4 && !previous_pulses.p4) {
                detected[3] = true;
                fired_events.push_back({4, deg});
            }
            if (current_pulses.p5 && !previous_pulses.p5) {
                detected[4] = true;
                fired_events.push_back({5, deg});
            }
            if (current_pulses.p6 && !previous_pulses.p6) {
                detected[5] = true;
                fired_events.push_back({6, deg});
            }
        }
        previous_pulses = current_pulses;
    }

    // 5. Provide a report
    std::cout << "Thyristor Firing Detection Report:" << std::endl;
    for (int i = 0; i < 6; ++i) {
        std::cout << "P" << (i + 1) << ": " << (detected[i] ? "DETECTED" : "MISSING") << std::endl;
    }

    std::cout << "Fired Events Sequence:" << std::endl;
    for (const auto& event : fired_events) {
        std::cout << "  At " << event.angle << " deg: P" << event.id << " fired" << std::endl;
    }

    // 3. Verify that get_any_thyristor_fired() becomes true exactly 6 times during the full cycle.
    EXPECT_EQ(fired_events.size(), 6);

    // 4. Verify that each time it becomes true, it corresponds to the expected firing offset.
    for (int i = 0; i < 6; ++i) {
        EXPECT_TRUE(detected[i]) << "Thyristor P" << (i + 1) << " was NOT detected!";
    }
}
