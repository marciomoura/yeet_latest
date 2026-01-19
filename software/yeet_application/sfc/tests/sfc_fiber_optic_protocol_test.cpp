/**
 * @file sfc_fiber_optic_protocol_test.cpp
 * @brief Unit tests for fiber-optic communication protocol encoding/decoding
 *
 * Tests verify:
 * - Grid → Machine message encoding/decoding
 * - Machine → Grid message encoding/decoding
 * - Round-trip encoding/decoding integrity
 * - Payload length validation
 * - Float value transmission accuracy
 * - Bit field packing/unpacking correctness
 * - Edge cases (zero values, max values, boundary conditions)
 */

#include "sfc/sfc_fiber_optic_protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

using namespace yeet::application::sfc::fiber_optic;
using namespace yeet::application::sfc;

/* ============================================================================
 * Grid → Machine Protocol Tests
 * ============================================================================ */

class GridToMachineProtocolTest : public ::testing::Test {
protected:
    std::array<std::uint32_t, max_payload_words> payload{};

    void SetUp() override { payload.fill(0); }
};

// --- Basic Encoding/Decoding Tests ---

TEST_F(GridToMachineProtocolTest, EncodeReturnsCorrectPayloadLength)
{
    grid_to_machine_message msg{};
    std::uint8_t length = encode_grid_to_machine(msg, payload.data());
    EXPECT_EQ(length, grid_to_machine_payload_length);
    EXPECT_EQ(length, 3);
}

TEST_F(GridToMachineProtocolTest, DecodeRejectsInsufficientPayloadLength)
{
    auto result = decode_grid_to_machine(payload.data(), 0);
    EXPECT_FALSE(result.has_value());

    result = decode_grid_to_machine(payload.data(), 1);
    EXPECT_FALSE(result.has_value());
}

TEST_F(GridToMachineProtocolTest, DecodeAcceptsSufficientPayloadLength)
{
    auto result = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);
    EXPECT_TRUE(result.has_value());

    result = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length + 1);
    EXPECT_TRUE(result.has_value());
}

// --- Float Value Transmission Tests ---

TEST_F(GridToMachineProtocolTest, RoundTripZeroDCLinkCurrent)
{
    grid_to_machine_message original{};
    original.dc_link_current = yeet::current_pu_t{0.0f};

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->dc_link_current.value(), 0.0f);
}

TEST_F(GridToMachineProtocolTest, RoundTripPositiveDCLinkCurrent)
{
    grid_to_machine_message original{};
    original.dc_link_current = yeet::current_pu_t{0.75f};

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->dc_link_current.value(), 0.75f);
}

TEST_F(GridToMachineProtocolTest, RoundTripNegativeDCLinkCurrent)
{
    grid_to_machine_message original{};
    original.dc_link_current = yeet::current_pu_t{-0.5f};

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->dc_link_current.value(), -0.5f);
}

TEST_F(GridToMachineProtocolTest, RoundTripMaximumDCLinkCurrent)
{
    grid_to_machine_message original{};
    original.dc_link_current = yeet::current_pu_t{1.0f};

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->dc_link_current.value(), 1.0f);
}

// --- Forced Commutation Flag Tests ---

TEST_F(GridToMachineProtocolTest, RoundTripForcedCommutationFalse)
{
    grid_to_machine_message original{};
    original.is_forced_commutation = false;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FALSE(decoded->is_forced_commutation);
}

TEST_F(GridToMachineProtocolTest, RoundTripForcedCommutationTrue)
{
    grid_to_machine_message original{};
    original.is_forced_commutation = true;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->is_forced_commutation);
}

// --- Boolean Flag Tests ---

TEST_F(GridToMachineProtocolTest, RoundTripAllFlagsFalse)
{
    grid_to_machine_message original{};
    original.force_commutation_acknowledged = false;
    original.release_pulses = false;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FALSE(decoded->force_commutation_acknowledged);
    EXPECT_FALSE(decoded->release_pulses);
}

TEST_F(GridToMachineProtocolTest, RoundTripAllFlagsTrue)
{
    grid_to_machine_message original{};
    original.force_commutation_acknowledged = true;
    original.release_pulses = true;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->force_commutation_acknowledged);
    EXPECT_TRUE(decoded->release_pulses);
}

TEST_F(GridToMachineProtocolTest, RoundTripIndividualFlagForceCommAck)
{
    grid_to_machine_message original{};
    original.force_commutation_acknowledged = true;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->force_commutation_acknowledged);
    EXPECT_FALSE(decoded->release_pulses);
}

TEST_F(GridToMachineProtocolTest, RoundTripIndividualFlagReleasePulses)
{
    grid_to_machine_message original{};
    original.release_pulses = true;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FALSE(decoded->force_commutation_acknowledged);
    EXPECT_TRUE(decoded->release_pulses);
}

// --- State ID Tests ---

TEST_F(GridToMachineProtocolTest, RoundTripStateIDZero)
{
    grid_to_machine_message original{};
    original.state_id = 0;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->state_id, 0);
}

TEST_F(GridToMachineProtocolTest, RoundTripStateIDMidRange)
{
    grid_to_machine_message original{};
    original.state_id = 42;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->state_id, 42);
}

TEST_F(GridToMachineProtocolTest, RoundTripStateIDMaximum)
{
    grid_to_machine_message original{};
    original.state_id = 127;  // 7-bit maximum

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->state_id, 127);
}

// --- Complete Message Round-Trip Tests ---

TEST_F(GridToMachineProtocolTest, RoundTripCompleteMessageAllFieldsSet)
{
    grid_to_machine_message original{};
    original.is_forced_commutation = true;
    original.dc_link_current = yeet::current_pu_t{0.85f};
    original.grid_voltage_magnitude = yeet::voltage_pu_t{0.95f};
    original.force_commutation_acknowledged = true;
    original.release_pulses = false;
    original.state_id = 15;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->is_forced_commutation);
    EXPECT_FLOAT_EQ(decoded->dc_link_current.value(), 0.85f);
    EXPECT_FLOAT_EQ(decoded->grid_voltage_magnitude.value(), 0.95f);
    EXPECT_TRUE(decoded->force_commutation_acknowledged);
    EXPECT_FALSE(decoded->release_pulses);
    EXPECT_EQ(decoded->state_id, 15);
}

TEST_F(GridToMachineProtocolTest, RoundTripCompleteMessageMixedFields)
{
    grid_to_machine_message original{};
    original.is_forced_commutation = false;
    original.dc_link_current = yeet::current_pu_t{-0.25f};
    original.grid_voltage_magnitude = yeet::voltage_pu_t{1.05f};
    original.force_commutation_acknowledged = false;
    original.release_pulses = true;
    original.state_id = 100;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FALSE(decoded->is_forced_commutation);
    EXPECT_FLOAT_EQ(decoded->dc_link_current.value(), -0.25f);
    EXPECT_FLOAT_EQ(decoded->grid_voltage_magnitude.value(), 1.05f);
    EXPECT_FALSE(decoded->force_commutation_acknowledged);
    EXPECT_TRUE(decoded->release_pulses);
    EXPECT_EQ(decoded->state_id, 100);
}

TEST_F(GridToMachineProtocolTest, RoundTripCommunicationOK)
{
    grid_to_machine_message original{};
    original.communication_ok = true;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->communication_ok);
    // Verify other flags remain false
    EXPECT_FALSE(decoded->grid_release_protections);
    EXPECT_FALSE(decoded->machine_release_protections);
}

TEST_F(GridToMachineProtocolTest, RoundTripReleaseProtections)
{
    grid_to_machine_message original{};
    original.grid_release_protections = true;
    original.machine_release_protections = true;

    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->grid_release_protections);
    EXPECT_TRUE(decoded->machine_release_protections);
    EXPECT_FALSE(decoded->communication_ok);
}

/* ============================================================================
 * Machine → Grid Protocol Tests
 * ============================================================================ */

class MachineToGridProtocolTest : public ::testing::Test {
protected:
    std::array<std::uint32_t, max_payload_words> payload{};

    void SetUp() override { payload.fill(0); }
};

// --- Basic Encoding/Decoding Tests ---

TEST_F(MachineToGridProtocolTest, EncodeReturnsCorrectPayloadLength)
{
    machine_to_grid_message msg{};
    std::uint8_t length = encode_machine_to_grid(msg, payload.data());
    EXPECT_EQ(length, machine_to_grid_payload_length);
    EXPECT_EQ(length, 4);
}

TEST_F(MachineToGridProtocolTest, DecodeRejectsInsufficientPayloadLength)
{
    auto result = decode_machine_to_grid(payload.data(), 0);
    EXPECT_FALSE(result.has_value());

    result = decode_machine_to_grid(payload.data(), 3);
    EXPECT_FALSE(result.has_value());
}

TEST_F(MachineToGridProtocolTest, DecodeAcceptsSufficientPayloadLength)
{
    auto result = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);
    EXPECT_TRUE(result.has_value());

    result = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length + 1);
    EXPECT_TRUE(result.has_value());
}

// --- Float Value Transmission Tests ---

TEST_F(MachineToGridProtocolTest, RoundTripMeasuredSpeedZero)
{
    machine_to_grid_message original{};
    original.measured_speed = yeet::speed_pu_t{0.0f};

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->measured_speed.value(), 0.0f);
}

TEST_F(MachineToGridProtocolTest, RoundTripMeasuredSpeedPositive)
{
    machine_to_grid_message original{};
    original.measured_speed = yeet::speed_pu_t{0.95f};

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->measured_speed.value(), 0.95f);
}

TEST_F(MachineToGridProtocolTest, RoundTripDCLinkCurrentReferenceZero)
{
    machine_to_grid_message original{};
    original.dc_link_current_reference = yeet::current_pu_t{0.0f};

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->dc_link_current_reference.value(), 0.0f);
}

TEST_F(MachineToGridProtocolTest, RoundTripDCLinkCurrentReferencePositive)
{
    machine_to_grid_message original{};
    original.dc_link_current_reference = yeet::current_pu_t{0.8f};

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->dc_link_current_reference.value(), 0.8f);
}

TEST_F(MachineToGridProtocolTest, RoundTripMachineVoltageMagnitudeZero)
{
    machine_to_grid_message original{};
    original.machine_voltage_magnitude = yeet::voltage_pu_t{0.0f};

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->machine_voltage_magnitude.value(), 0.0f);
}

TEST_F(MachineToGridProtocolTest, RoundTripMachineVoltageMagnitudePositive)
{
    machine_to_grid_message original{};
    original.machine_voltage_magnitude = yeet::voltage_pu_t{1.0f};

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->machine_voltage_magnitude.value(), 1.0f);
}

TEST_F(MachineToGridProtocolTest, RoundTripAllFloatValues)
{
    machine_to_grid_message original{};
    original.measured_speed = yeet::speed_pu_t{0.5f};
    original.dc_link_current_reference = yeet::current_pu_t{0.75f};
    original.machine_voltage_magnitude = yeet::voltage_pu_t{0.9f};

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->measured_speed.value(), 0.5f);
    EXPECT_FLOAT_EQ(decoded->dc_link_current_reference.value(), 0.75f);
    EXPECT_FLOAT_EQ(decoded->machine_voltage_magnitude.value(), 0.9f);
}

// --- Boolean Flag Tests ---

TEST_F(MachineToGridProtocolTest, RoundTripAllFlagsFalse)
{
    machine_to_grid_message original{};
    original.force_commutation_request = false;

    original.machine_protection_tripped = false;

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FALSE(decoded->force_commutation_request);

    EXPECT_FALSE(decoded->machine_protection_tripped);
}

TEST_F(MachineToGridProtocolTest, RoundTripAllFlagsTrue)
{
    machine_to_grid_message original{};
    original.force_commutation_request = true;

    original.machine_protection_tripped = true;

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->force_commutation_request);

    EXPECT_TRUE(decoded->machine_protection_tripped);
}

TEST_F(MachineToGridProtocolTest, RoundTripIndividualFlagForceCommutationRequest)
{
    machine_to_grid_message original{};
    original.force_commutation_request = true;

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->force_commutation_request);

    EXPECT_FALSE(decoded->machine_protection_tripped);
}

TEST_F(MachineToGridProtocolTest, RoundTripIndividualFlagMachineProtectionTripped)
{
    machine_to_grid_message original{};
    original.machine_protection_tripped = true;

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FALSE(decoded->force_commutation_request);

    EXPECT_TRUE(decoded->machine_protection_tripped);
}

// --- Complete Message Round-Trip Tests ---

TEST_F(MachineToGridProtocolTest, RoundTripCompleteMessageAllFieldsSet)
{
    machine_to_grid_message original{};
    original.measured_speed = yeet::speed_pu_t{0.65f};
    original.dc_link_current_reference = yeet::current_pu_t{0.88f};
    original.machine_voltage_magnitude = yeet::voltage_pu_t{0.95f};
    original.force_commutation_request = true;

    original.machine_protection_tripped = true;

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->measured_speed.value(), 0.65f);
    EXPECT_FLOAT_EQ(decoded->dc_link_current_reference.value(), 0.88f);
    EXPECT_FLOAT_EQ(decoded->machine_voltage_magnitude.value(), 0.95f);
    EXPECT_TRUE(decoded->force_commutation_request);

    EXPECT_TRUE(decoded->machine_protection_tripped);
}

TEST_F(MachineToGridProtocolTest, RoundTripCompleteMessageMixedFields)
{
    machine_to_grid_message original{};
    original.measured_speed = yeet::speed_pu_t{0.0f};
    original.dc_link_current_reference = yeet::current_pu_t{0.0f};
    original.machine_voltage_magnitude = yeet::voltage_pu_t{0.1f};
    original.force_commutation_request = false;

    original.machine_protection_tripped = false;

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->measured_speed.value(), 0.0f);
    EXPECT_FLOAT_EQ(decoded->dc_link_current_reference.value(), 0.0f);
    EXPECT_FLOAT_EQ(decoded->machine_voltage_magnitude.value(), 0.1f);
    EXPECT_FALSE(decoded->force_commutation_request);

    EXPECT_FALSE(decoded->machine_protection_tripped);
}

TEST_F(MachineToGridProtocolTest, RoundTripCommunicationOK)
{
    machine_to_grid_message original{};
    original.communication_ok = true;

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->communication_ok);
    EXPECT_FALSE(decoded->protection_inhibit);
}

TEST_F(MachineToGridProtocolTest, RoundTripProtectionInhibit)
{
    machine_to_grid_message original{};
    original.protection_inhibit = true;

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->protection_inhibit);
    EXPECT_FALSE(decoded->communication_ok);
}

TEST_F(MachineToGridProtocolTest, RoundTripAllFlagsAndFloats)
{
    machine_to_grid_message original{};
    original.measured_speed = yeet::speed_pu_t{0.123f};
    original.dc_link_current_reference = yeet::current_pu_t{0.456f};
    original.machine_voltage_magnitude = yeet::voltage_pu_t{0.789f};
    original.force_commutation_request = true;
    original.machine_protection_tripped = true;
    original.protection_inhibit = true;
    original.communication_ok = true;

    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->measured_speed.value(), 0.123f);
    EXPECT_FLOAT_EQ(decoded->dc_link_current_reference.value(), 0.456f);
    EXPECT_FLOAT_EQ(decoded->machine_voltage_magnitude.value(), 0.789f);
    EXPECT_TRUE(decoded->force_commutation_request);
    EXPECT_TRUE(decoded->machine_protection_tripped);
    EXPECT_TRUE(decoded->protection_inhibit);
    EXPECT_TRUE(decoded->communication_ok);
}

/* ============================================================================
 * Edge Case and Robustness Tests
 * ============================================================================ */

TEST(FiberOpticProtocolEdgeCasesTest, GridToMachineHandlesNegativeFloatValues)
{
    grid_to_machine_message original{};
    original.dc_link_current = yeet::current_pu_t{-0.99f};

    std::array<std::uint32_t, max_payload_words> payload{};
    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->dc_link_current.value(), -0.99f);
}

TEST(FiberOpticProtocolEdgeCasesTest, MachineToGridHandlesNegativeFloatValues)
{
    machine_to_grid_message original{};
    original.measured_speed = yeet::speed_pu_t{-0.5f};
    original.dc_link_current_reference = yeet::current_pu_t{-0.3f};
    original.machine_voltage_magnitude = yeet::voltage_pu_t{-0.1f};

    std::array<std::uint32_t, max_payload_words> payload{};
    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->measured_speed.value(), -0.5f);
    EXPECT_FLOAT_EQ(decoded->dc_link_current_reference.value(), -0.3f);
    EXPECT_FLOAT_EQ(decoded->machine_voltage_magnitude.value(), -0.1f);
}

TEST(FiberOpticProtocolEdgeCasesTest, GridToMachineHandlesVerySmallFloatValues)
{
    grid_to_machine_message original{};
    original.dc_link_current = yeet::current_pu_t{1e-6f};

    std::array<std::uint32_t, max_payload_words> payload{};
    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->dc_link_current.value(), 1e-6f);
}

TEST(FiberOpticProtocolEdgeCasesTest, MachineToGridHandlesVerySmallFloatValues)
{
    machine_to_grid_message original{};
    original.measured_speed = yeet::speed_pu_t{1e-7f};

    std::array<std::uint32_t, max_payload_words> payload{};
    [[maybe_unused]] auto length = encode_machine_to_grid(original, payload.data());
    auto decoded = decode_machine_to_grid(payload.data(), machine_to_grid_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FLOAT_EQ(decoded->measured_speed.value(), 1e-7f);
}

TEST(FiberOpticProtocolEdgeCasesTest, GridToMachinePreservesStateIDWithAllFlagsSet)
{
    // Test that state ID bits don't interfere with flag bits
    grid_to_machine_message original{};
    original.state_id = 127;  // Maximum 7-bit value
    original.force_commutation_acknowledged = true;
    original.release_pulses = true;

    std::array<std::uint32_t, max_payload_words> payload{};
    [[maybe_unused]] auto length = encode_grid_to_machine(original, payload.data());
    auto decoded = decode_grid_to_machine(payload.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->state_id, 127);
    EXPECT_TRUE(decoded->force_commutation_acknowledged);
    EXPECT_TRUE(decoded->release_pulses);
}

TEST(FiberOpticProtocolEdgeCasesTest, MultipleSequentialEncodingsIndependent)
{
    // Verify that encoding one message doesn't affect subsequent encodings
    std::array<std::uint32_t, max_payload_words> payload1{};
    std::array<std::uint32_t, max_payload_words> payload2{};

    grid_to_machine_message msg1{};
    msg1.dc_link_current = yeet::current_pu_t{0.5f};
    msg1.state_id = 10;

    grid_to_machine_message msg2{};
    msg2.dc_link_current = yeet::current_pu_t{0.9f};
    msg2.state_id = 20;

    [[maybe_unused]] auto length1 = encode_grid_to_machine(msg1, payload1.data());
    [[maybe_unused]] auto length2 = encode_grid_to_machine(msg2, payload2.data());

    auto decoded1 = decode_grid_to_machine(payload1.data(), grid_to_machine_payload_length);
    auto decoded2 = decode_grid_to_machine(payload2.data(), grid_to_machine_payload_length);

    ASSERT_TRUE(decoded1.has_value());
    ASSERT_TRUE(decoded2.has_value());

    EXPECT_FLOAT_EQ(decoded1->dc_link_current.value(), 0.5f);
    EXPECT_EQ(decoded1->state_id, 10);

    EXPECT_FLOAT_EQ(decoded2->dc_link_current.value(), 0.9f);
    EXPECT_EQ(decoded2->state_id, 20);
}
