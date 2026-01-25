/**
 * @file fiber_optic_protocol.cpp
 * @brief Fiber-optic communication protocol encoding/decoding implementation
 */

#include "sfc/sfc_fiber_optic_protocol.hpp"

#include <cstring>  // for memcpy (until C++20 bit_cast is available)

namespace yeet::application::sfc::fiber_optic {

/* ============================================================================
 * Grid → Machine Protocol Implementation
 * ============================================================================ */

[[nodiscard]] std::uint8_t encode_grid_to_machine(const grid_to_machine_message& msg, std::uint32_t* payload) noexcept
{
    // Word 0: DC link current measurement [pu] (IEEE 754 float bits)
    // Word 1: Packed control flags and state
    //   Bit 0:      Is forced commutation mode
    //   Bit 1:      Force commutation acknowledged
    //   Bit 2:      Release pulses
    //   Bit 3:      Grid release protections
    //   Bit 4:      Machine release protections
    //   Bits 5:     Communication OK
    //   Bits 6:     Freeze integrator
    //   Bits 7-24:  Reserved for future use
    //   Bits 25-31: State machine state ID (0-127)

    // Encode DC link current float value (extract from typed unit)
    const float current_value = msg.dc_link_current.value();
    std::memcpy(&payload[0], &current_value, sizeof(float));

    // Pack control flags into word 1
    std::uint32_t packed_flags = 0;

    // Encode boolean flags at the beginning (bits 0-5)
    if (msg.is_forced_commutation) {
        packed_flags |= (1U << grid_to_machine_message::is_forced_commutation_bit_pos);
    }
    if (msg.force_commutation_acknowledged) {
        packed_flags |= (1U << grid_to_machine_message::force_comm_ack_bit_pos);
    }
    if (msg.release_pulses) {
        packed_flags |= (1U << grid_to_machine_message::release_pulses_bit_pos);
    }
    if (msg.grid_release_protections) {
        packed_flags |= (1U << grid_to_machine_message::grid_release_protections_bit_pos);
    }
    if (msg.machine_release_protections) {
        packed_flags |= (1U << grid_to_machine_message::machine_release_protections_bit_pos);
    }
    if (msg.communication_ok) {
        packed_flags |= (1U << grid_to_machine_message::communication_ok_bit_pos);
    }
    if (msg.freeze_integrator) {
        packed_flags |= (1U << grid_to_machine_message::freeze_integrator_bit_pos);
    }

    // Encode state ID at the end (bits 25-31)
    packed_flags |= (static_cast<std::uint32_t>(msg.state_id) & grid_to_machine_message::state_id_mask)
                    << grid_to_machine_message::state_id_shift;

    // Write packed word to payload
    payload[1] = packed_flags;

    return grid_to_machine_payload_length;
}

[[nodiscard]] std::optional<grid_to_machine_message> decode_grid_to_machine(
    const std::uint32_t* payload, std::uint8_t payload_length) noexcept
{
    // Validate payload length
    if (payload_length < grid_to_machine_payload_length) {
        return std::nullopt;
    }

    grid_to_machine_message msg;

    // Decode DC link current float value and construct typed unit
    float current_value;
    std::memcpy(&current_value, &payload[0], sizeof(float));
    msg.dc_link_current = yeet::current_pu_t{current_value};

    // Extract packed word 1
    const std::uint32_t packed_flags = payload[1];

    // Decode boolean flags from the beginning (bits 0-5)
    msg.is_forced_commutation = (packed_flags & (1U << grid_to_machine_message::is_forced_commutation_bit_pos)) != 0;
    msg.force_commutation_acknowledged = (packed_flags & (1U << grid_to_machine_message::force_comm_ack_bit_pos)) != 0;
    msg.release_pulses = (packed_flags & (1U << grid_to_machine_message::release_pulses_bit_pos)) != 0;
    msg.grid_release_protections =
        (packed_flags & (1U << grid_to_machine_message::grid_release_protections_bit_pos)) != 0;
    msg.machine_release_protections =
        (packed_flags & (1U << grid_to_machine_message::machine_release_protections_bit_pos)) != 0;
    msg.communication_ok = (packed_flags & (1U << grid_to_machine_message::communication_ok_bit_pos)) != 0;
    msg.freeze_integrator = (packed_flags & (1U << grid_to_machine_message::freeze_integrator_bit_pos)) != 0;

    // Decode state ID from the end (bits 25-31)
    msg.state_id = static_cast<std::uint8_t>(
        (packed_flags >> grid_to_machine_message::state_id_shift) & grid_to_machine_message::state_id_mask);

    return msg;
}

/* ============================================================================
 * Machine → Grid Protocol Implementation
 * ============================================================================ */

[[nodiscard]] std::uint8_t encode_machine_to_grid(const machine_to_grid_message& msg, std::uint32_t* payload) noexcept
{
    // Word 0: Measured speed [pu] (IEEE 754 float bits)
    // Word 1: DC link current reference [pu] (IEEE 754 float bits)
    // Word 2: Machine voltage magnitude [pu] (IEEE 754 float bits)
    // Word 3: Packed control flags

    // Encode float values using standards-compliant type punning (extract from typed units)
    // Note: C++20 would use std::bit_cast, but memcpy is standards-compliant for C++17
    const float speed_value = msg.measured_speed.value();
    std::memcpy(&payload[0], &speed_value, sizeof(float));

    const float current_value = msg.dc_link_current_reference.value();
    std::memcpy(&payload[1], &current_value, sizeof(float));

    const float voltage_value = msg.machine_voltage_magnitude.value();
    std::memcpy(&payload[2], &voltage_value, sizeof(float));

    // Pack control flags into word 3
    std::uint32_t packed_flags = 0;
    if (msg.force_commutation_request) {
        packed_flags |= (1U << machine_to_grid_message::force_comm_req_bit_pos);
    }
    if (msg.machine_protection_tripped) {
        packed_flags |= (1U << machine_to_grid_message::machine_protection_tripped_bit_pos);
    }
    if (msg.protection_inhibit) {
        packed_flags |= (1U << machine_to_grid_message::protection_inhibit_bit_pos);
    }
    if (msg.communication_ok) {
        packed_flags |= (1U << machine_to_grid_message::communication_ok_bit_pos);
    }
    payload[3] = packed_flags;

    return machine_to_grid_payload_length;
}

[[nodiscard]] std::optional<machine_to_grid_message> decode_machine_to_grid(
    const std::uint32_t* payload, std::uint8_t payload_length) noexcept
{
    // Validate payload length
    if (payload_length < machine_to_grid_payload_length) {
        return std::nullopt;
    }

    machine_to_grid_message msg;

    // Decode float values using standards-compliant type punning and construct typed units
    // Note: C++20 would use std::bit_cast, but memcpy is standards-compliant for C++17
    float speed_value;
    std::memcpy(&speed_value, &payload[0], sizeof(float));
    msg.measured_speed = yeet::speed_pu_t{speed_value};

    float current_value;
    std::memcpy(&current_value, &payload[1], sizeof(float));
    msg.dc_link_current_reference = yeet::current_pu_t{current_value};

    float voltage_value;
    std::memcpy(&voltage_value, &payload[2], sizeof(float));
    msg.machine_voltage_magnitude = yeet::voltage_pu_t{voltage_value};

    // Decode packed control flags from word 3
    const std::uint32_t packed_flags = payload[3];
    msg.force_commutation_request = (packed_flags & (1U << machine_to_grid_message::force_comm_req_bit_pos)) != 0;
    msg.machine_protection_tripped =
        (packed_flags & (1U << machine_to_grid_message::machine_protection_tripped_bit_pos)) != 0;
    msg.protection_inhibit = (packed_flags & (1U << machine_to_grid_message::protection_inhibit_bit_pos)) != 0;
    msg.communication_ok = (packed_flags & (1U << machine_to_grid_message::communication_ok_bit_pos)) != 0;

    return msg;
}

}  // namespace yeet::application::sfc::fiber_optic
