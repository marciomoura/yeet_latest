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
    //   Bits 0-7:   Operating mode (enum, 0-255)
    //   Bit 8:      Is current zero
    //   Bit 9:      Current controller is saturated
    //   Bit 10:     Force commutation acknowledged
    //   Bit 11:     Pulse releases
    //   Bits 12-18: State machine state ID (0-127)
    //   Bit 19:     Freeze integrator (unified)
    //   Bit 20:     Is tripped (grid-side trip status)
    //   Bit 21:     Preset request
    //   Bits 22-31: Reserved for future use

    // Encode DC link current float value
    std::memcpy(&payload[0], &msg.dc_link_current_pu, sizeof(float));

    // Pack control flags into word 1
    std::uint32_t packed_flags = 0;

    // Encode operating mode (bits 0-7)
    packed_flags |= (static_cast<std::uint32_t>(msg.mode) & detail::g2m_operating_mode_mask)
                    << detail::g2m_operating_mode_shift;

    // Encode boolean flags
    if (msg.is_current_zero) {
        packed_flags |= detail::g2m_is_current_zero_bit;
    }
    if (msg.is_saturated) {
        packed_flags |= detail::g2m_current_controller_saturated_bit;
    }
    if (msg.force_commutation_acknowledged) {
        packed_flags |= detail::g2m_force_comm_ack_bit;
    }
    if (msg.pulse_releases) {
        packed_flags |= detail::g2m_pulse_releases_bit;
    }

    // Encode state ID (bits 12-18)
    packed_flags |= (static_cast<std::uint32_t>(msg.state_id) & 0x7FU) << detail::g2m_state_id_shift;

    // Encode sync control flags
    if (msg.freeze_integrator) {
        packed_flags |= detail::g2m_freeze_integrator_bit;
    }
    if (msg.is_tripped) {
        packed_flags |= detail::g2m_is_tripped_bit;
    }
    if (msg.preset_request) {
        packed_flags |= detail::g2m_preset_request_bit;
    }

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

    // Decode DC link current float value
    std::memcpy(&msg.dc_link_current_pu, &payload[0], sizeof(float));

    // Extract packed word 1
    const std::uint32_t packed_flags = payload[1];

    // Decode operating mode (bits 0-7)
    msg.mode = static_cast<yeet::application::sfc::sfc_operating_mode_t>(
        (packed_flags >> detail::g2m_operating_mode_shift) & detail::g2m_operating_mode_mask);

    // Decode boolean flags
    msg.is_current_zero = (packed_flags & detail::g2m_is_current_zero_bit) != 0;
    msg.is_saturated = (packed_flags & detail::g2m_current_controller_saturated_bit) != 0;
    msg.force_commutation_acknowledged = (packed_flags & detail::g2m_force_comm_ack_bit) != 0;
    msg.pulse_releases = (packed_flags & detail::g2m_pulse_releases_bit) != 0;

    // Decode state ID (bits 12-18)
    msg.state_id = static_cast<std::uint8_t>((packed_flags >> detail::g2m_state_id_shift) & 0x7FU);

    // Decode sync control flags
    msg.freeze_integrator = (packed_flags & detail::g2m_freeze_integrator_bit) != 0;
    msg.is_tripped = (packed_flags & detail::g2m_is_tripped_bit) != 0;
    msg.preset_request = (packed_flags & detail::g2m_preset_request_bit) != 0;

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

    // Encode float values using standards-compliant type punning
    // Note: C++20 would use std::bit_cast, but memcpy is standards-compliant for C++17
    std::memcpy(&payload[0], &msg.measured_speed_pu, sizeof(float));
    std::memcpy(&payload[1], &msg.dc_link_current_reference_pu, sizeof(float));
    std::memcpy(&payload[2], &msg.machine_voltage_magnitude_pu, sizeof(float));

    // Pack control flags into word 3
    std::uint32_t packed_flags = 0;
    if (msg.force_commutation_request) {
        packed_flags |= detail::m2g_force_comm_req_bit;
    }
    if (msg.is_tripped) {
        packed_flags |= detail::m2g_is_tripped_bit;
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

    // Decode float values using standards-compliant type punning
    // Note: C++20 would use std::bit_cast, but memcpy is standards-compliant for C++17
    std::memcpy(&msg.measured_speed_pu, &payload[0], sizeof(float));
    std::memcpy(&msg.dc_link_current_reference_pu, &payload[1], sizeof(float));
    std::memcpy(&msg.machine_voltage_magnitude_pu, &payload[2], sizeof(float));

    // Decode packed control flags from word 3
    const std::uint32_t packed_flags = payload[3];
    msg.force_commutation_request = (packed_flags & detail::m2g_force_comm_req_bit) != 0;
    msg.is_tripped = (packed_flags & detail::m2g_is_tripped_bit) != 0;

    return msg;
}

}  // namespace yeet::application::sfc::fiber_optic
