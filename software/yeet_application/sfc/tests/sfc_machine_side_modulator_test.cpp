#include "sfc/sfc_machine_side_modulator.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

namespace {

// Test fixture for machine-side modulator tests
class SfcMachineSideModulatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Create modulator with 100µs sampling time
        modulator = std::make_unique<sfc_machine_side_modulator>(duration_t(100e-6));
    }

    std::unique_ptr<sfc_machine_side_modulator> modulator;

    // Common test parameters
    static constexpr duration_t sampling_time{100e-6};  // 100µs
    static constexpr angle_t test_lut_angle = deg_to_rad(45.0);
    static constexpr angle_t custom_pulse_angle = deg_to_rad(120.0);
};

}  // namespace

// ========================================
// Constructor and Configuration Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, ConstructorInitializesToSafeState)
{
    // After construction, modulator should be in safe default state
    EXPECT_EQ(modulator->get_current_operating_mode(), sfc_operating_mode_t::pulse_mode);
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);  // Safe default angle
}

TEST_F(SfcMachineSideModulatorTest, ConfigurePulseModeAngle)
{
    // Configure custom pulse mode firing angle
    modulator->configure_pulse_mode_firing_angle_rad(custom_pulse_angle);

    // Enable system in pulse mode and verify the configured angle is used
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, angle_t(0.0), test_lut_angle);

    // The firing angle should be calculated based on rotor position (sector 0) + configured offset
    const real_t expected_angle = 0.0 + custom_pulse_angle.value();  // Sector 0 base + offset
    EXPECT_NEAR(modulator->get_firing_angle_rad().value(), expected_angle, 1e-6);
}

TEST_F(SfcMachineSideModulatorTest, ResetFunctionRestoresInitialState)
{
    // Put modulator in a non-initial state
    modulator->update(true, sfc_operating_mode_t::load_commutated, true, deg_to_rad(100.0), test_lut_angle);

    // Reset and verify initial state is restored
    modulator->reset();

    EXPECT_EQ(modulator->get_current_operating_mode(), sfc_operating_mode_t::pulse_mode);
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);
}

// ========================================
// Operating Mode Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, LoadCommutatedModePassesThroughLutAngle)
{
    // Test that in load_commutated mode, the firing angle from lookup table is passed through
    modulator->update(true, sfc_operating_mode_t::load_commutated, false, deg_to_rad(45.0), test_lut_angle);

    EXPECT_EQ(modulator->get_current_operating_mode(), sfc_operating_mode_t::load_commutated);
    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_lut_angle.value());
    EXPECT_TRUE(modulator->get_release_pulses());              // Always release pulses in natural commutation
    EXPECT_FALSE(modulator->get_request_force_commutation());  // Never request commutation
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
}

TEST_F(SfcMachineSideModulatorTest, PulseModeCalculatesPositionBasedAngle)
{
    // Test that in pulse_mode, the firing angle is calculated based on rotor position
    modulator->update(
        true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(30.0), test_lut_angle);  // Rotor in sector 0

    EXPECT_EQ(modulator->get_current_operating_mode(), sfc_operating_mode_t::pulse_mode);
    // Firing angle should be sector base (0°) + default pulse offset (90°)
    EXPECT_NEAR(modulator->get_firing_angle_rad().value(), deg_to_rad(90.0).value(), 1e-6);
}

// ========================================
// System Enable/Disable Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, DisabledSystemOutputsSafeDefaults)
{
    // When system is disabled, should output safe values regardless of inputs
    modulator->update(false, sfc_operating_mode_t::load_commutated, true, deg_to_rad(100.0), test_lut_angle);

    EXPECT_EQ(modulator->get_firing_angle_rad().value(), 0.0);  // Safe default angle
    EXPECT_FALSE(modulator->get_release_pulses());              // Block pulses when disabled
    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
}

TEST_F(SfcMachineSideModulatorTest, EnabledSystemProcessesInputsNormally)
{
    // When system is enabled, should process inputs normally
    modulator->update(true, sfc_operating_mode_t::load_commutated, false, deg_to_rad(45.0), test_lut_angle);

    EXPECT_EQ(modulator->get_firing_angle_rad().value(), test_lut_angle.value());
    EXPECT_TRUE(modulator->get_release_pulses());
}

// ========================================
// Rotor Sector Tracking Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, RotorSectorCalculationIsCorrect)
{
    // Test sector boundaries (each sector is 60 degrees)
    const struct {
        real_t rotor_degrees;
        int expected_sector;
    } test_cases[] = {{0.0, 0},               // Sector 0: [0°, 60°)
        {30.0, 0}, {59.999, 0}, {60.0, 1},    // Sector 1: [60°, 120°)
        {90.0, 1}, {119.999, 1}, {120.0, 2},  // Sector 2: [120°, 180°)
        {180.0, 3},                           // Sector 3: [180°, 240°)
        {240.0, 4},                           // Sector 4: [240°, 300°)
        {300.0, 5},                           // Sector 5: [300°, 360°)
        {359.999, 5}};

    for (const auto& test_case : test_cases) {
        modulator->update(
            true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(test_case.rotor_degrees), test_lut_angle);

        // Calculate expected firing angle: sector_base + pulse_offset
        const real_t sector_base_rad = (test_case.expected_sector * 60.0) * pi / 180.0;
        const real_t expected_firing_angle = sector_base_rad + deg_to_rad(90.0).value();

        EXPECT_NEAR(modulator->get_firing_angle_rad().value(), expected_firing_angle, 1e-5)
            << "Failed for rotor angle: " << test_case.rotor_degrees
            << "°, expected sector: " << test_case.expected_sector;
    }
}

// ========================================
// Forced Commutation Handshake Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, SectorChangeInitiatesCommutationRequest)
{
    // Start in sector 0
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(30.0), test_lut_angle);

    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
    EXPECT_FALSE(modulator->get_request_force_commutation());

    // Move to sector 1 - should initiate commutation request
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(70.0), test_lut_angle);

    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_FALSE(modulator->get_release_pulses());            // Pulses should be blocked
    EXPECT_TRUE(modulator->get_request_force_commutation());  // Request should be asserted
}

TEST_F(SfcMachineSideModulatorTest, AcknowledgmentCompletesHandshake)
{
    // Move to sector change to initiate commutation
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(30.0), test_lut_angle);
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(70.0), test_lut_angle);

    // Verify we're waiting for acknowledgment
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
    EXPECT_TRUE(modulator->get_request_force_commutation());
    EXPECT_FALSE(modulator->get_release_pulses());

    // Provide acknowledgment from grid-side
    modulator->update(true, sfc_operating_mode_t::pulse_mode, true, deg_to_rad(70.0), test_lut_angle);

    // Handshake should be complete
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_FALSE(modulator->get_request_force_commutation());  // Request should be cleared
    EXPECT_TRUE(modulator->get_release_pulses());              // Pulses should be released
}

TEST_F(SfcMachineSideModulatorTest, RequestRemainsActiveUntilAcknowledged)
{
    // Initiate commutation request
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(30.0), test_lut_angle);
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(70.0), test_lut_angle);

    // Request should remain active across multiple update cycles
    for (int i = 0; i < 10; ++i) {
        modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(70.0), test_lut_angle);

        EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
        EXPECT_TRUE(modulator->get_request_force_commutation());
        EXPECT_FALSE(modulator->get_release_pulses());
    }
}

TEST_F(SfcMachineSideModulatorTest, MultipleSectorChangesHandledCorrectly)
{
    // Test sequence: Sector 0 -> 1 -> 2
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(30.0), test_lut_angle);  // Sector 0

    // Move to sector 1
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(70.0), test_lut_angle);  // Sector 1
    EXPECT_TRUE(modulator->get_request_force_commutation());

    // Acknowledge and complete first handshake
    modulator->update(true, sfc_operating_mode_t::pulse_mode, true, deg_to_rad(70.0), test_lut_angle);
    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);

    // Move to sector 2 - should initiate new commutation
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(130.0), test_lut_angle);  // Sector 2
    EXPECT_TRUE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
}

// ========================================
// Edge Case Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, RotorPositionWraparoundHandledCorrectly)
{
    // Test wraparound from sector 5 to sector 0
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(330.0), test_lut_angle);  // Sector 5

    // Move to sector 0 (wraparound case)
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(10.0), test_lut_angle);  // Sector 0

    EXPECT_TRUE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::waiting_for_ack);
}

TEST_F(SfcMachineSideModulatorTest, NoCommutationRequestInLoadCommutatedMode)
{
    // Sector changes in load_commutated mode should not generate commutation requests
    modulator->update(
        true, sfc_operating_mode_t::load_commutated, false, deg_to_rad(30.0), test_lut_angle);  // Sector 0

    modulator->update(
        true, sfc_operating_mode_t::load_commutated, false, deg_to_rad(70.0), test_lut_angle);  // Sector 1

    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
}

TEST_F(SfcMachineSideModulatorTest, ModeTransitionClearsCommutationState)
{
    // Start commutation sequence in pulse mode
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(30.0), test_lut_angle);
    modulator->update(true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(70.0), test_lut_angle);

    EXPECT_TRUE(modulator->get_request_force_commutation());

    // Switch to load_commutated mode - should clear commutation state
    modulator->update(true, sfc_operating_mode_t::load_commutated, false, deg_to_rad(70.0), test_lut_angle);

    EXPECT_FALSE(modulator->get_request_force_commutation());
    EXPECT_EQ(modulator->get_commutation_state(), machine_side_commutation_state_t::running);
    EXPECT_TRUE(modulator->get_release_pulses());
}

// ========================================
// Boundary Condition Tests
// ========================================

TEST_F(SfcMachineSideModulatorTest, ExactSectorBoundaryPositions)
{
    // Test exact sector boundary positions
    const real_t sector_boundaries[] = {0.0, 60.0, 120.0, 180.0, 240.0, 300.0};

    for (size_t i = 0; i < sizeof(sector_boundaries) / sizeof(sector_boundaries[0]); ++i) {
        modulator->reset();  // Reset for clean test
        modulator->update(
            true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(sector_boundaries[i]), test_lut_angle);

        // Should be in the expected sector
        const real_t sector_base_rad = sector_boundaries[i] * pi / 180.0;
        const real_t expected_firing_angle = sector_base_rad + deg_to_rad(90.0).value();

        EXPECT_NEAR(modulator->get_firing_angle_rad().value(), expected_firing_angle, 1e-5)
            << "Failed for boundary angle: " << sector_boundaries[i] << "°";
    }
}

TEST_F(SfcMachineSideModulatorTest, LargeRotorAngleInputsHandledCorrectly)
{
    // Test with angles beyond 360°
    modulator->update(
        true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(450.0), test_lut_angle);  // 450° = 90° (sector 1)

    // Should be equivalent to 90° (sector 1)
    const real_t expected_firing_angle = deg_to_rad(60.0).value() + deg_to_rad(90.0).value();
    EXPECT_NEAR(modulator->get_firing_angle_rad().value(), expected_firing_angle, 1e-5);
}

TEST_F(SfcMachineSideModulatorTest, NegativeRotorAngleInputsHandledCorrectly)
{
    // Test with negative angles
    modulator->update(
        true, sfc_operating_mode_t::pulse_mode, false, deg_to_rad(-30.0), test_lut_angle);  // -30° = 330° (sector 5)

    // Should be equivalent to 330° (sector 5)
    const real_t expected_firing_angle = deg_to_rad(300.0).value() + deg_to_rad(90.0).value();
    EXPECT_NEAR(modulator->get_firing_angle_rad().value(), expected_firing_angle, 1e-5);
}