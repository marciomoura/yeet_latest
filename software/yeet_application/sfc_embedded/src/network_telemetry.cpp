/**
 * @file network_telemetry.cpp
 * @brief Network telemetry protocol implementation
 */

#include "sfc_embedded/network_telemetry.hpp"

#include <cstring>

namespace yeet::application::sfc::telemetry {

std::size_t calculate_payload_size(std::uint32_t flags) noexcept
{
    std::size_t size = 0;

    if (flags & telemetry_flags::measurements) {
        size += sizeof(grid_measurements_group);
    }
    if (flags & telemetry_flags::observer) {
        size += sizeof(grid_observer_group);
    }
    if (flags & telemetry_flags::modulator) {
        size += sizeof(grid_modulator_group);
    }
    if (flags & telemetry_flags::controller) {
        size += sizeof(grid_controller_group);
    }
    if (flags & telemetry_flags::state_machine) {
        size += sizeof(state_machine_group);
    }

    return size;
}

bool validate_header(const telemetry_packet_header& header) noexcept
{
    // Check protocol version
    if (header.protocol_version != protocol_version) {
        return false;
    }

    // Check packet type is valid
    if (header.packet_type != static_cast<std::uint8_t>(packet_type::grid_side_telemetry) &&
        header.packet_type != static_cast<std::uint8_t>(packet_type::machine_side_telemetry) &&
        header.packet_type != static_cast<std::uint8_t>(packet_type::system_status)) {
        return false;
    }

    // Verify payload length matches flags
    std::size_t expected_size = calculate_payload_size(header.flags);
    if (header.payload_length != expected_size) {
        return false;
    }

    return true;
}

std::size_t encode_grid_side_telemetry(const grid_side_telemetry_packet& packet, std::uint8_t* buffer) noexcept
{
    std::size_t offset = 0;

    // Create a mutable copy of header to set fields
    telemetry_packet_header header = packet.header;
    header.packet_type = static_cast<std::uint8_t>(packet_type::grid_side_telemetry);
    header.protocol_version = protocol_version;
    header.payload_length = static_cast<std::uint16_t>(calculate_payload_size(header.flags));

    // Encode header (16 bytes)
    std::memcpy(buffer + offset, &header, sizeof(telemetry_packet_header));
    offset += sizeof(telemetry_packet_header);

    // Encode signal groups (only if flag is set)
    if (header.flags & telemetry_flags::measurements) {
        std::memcpy(buffer + offset, &packet.measurements, sizeof(grid_measurements_group));
        offset += sizeof(grid_measurements_group);
    }

    if (header.flags & telemetry_flags::observer) {
        std::memcpy(buffer + offset, &packet.observer, sizeof(grid_observer_group));
        offset += sizeof(grid_observer_group);
    }

    if (header.flags & telemetry_flags::modulator) {
        std::memcpy(buffer + offset, &packet.modulator, sizeof(grid_modulator_group));
        offset += sizeof(grid_modulator_group);
    }

    if (header.flags & telemetry_flags::controller) {
        std::memcpy(buffer + offset, &packet.controller, sizeof(grid_controller_group));
        offset += sizeof(grid_controller_group);
    }

    if (header.flags & telemetry_flags::state_machine) {
        std::memcpy(buffer + offset, &packet.state_machine, sizeof(state_machine_group));
        offset += sizeof(state_machine_group);
    }

    return offset;
}

bool decode_grid_side_telemetry(
    const std::uint8_t* buffer, std::size_t buffer_size, grid_side_telemetry_packet& packet) noexcept
{
    // Minimum size check (must have at least header)
    if (buffer_size < sizeof(telemetry_packet_header)) {
        return false;
    }

    std::size_t offset = 0;

    // Decode header
    std::memcpy(&packet.header, buffer + offset, sizeof(telemetry_packet_header));
    offset += sizeof(telemetry_packet_header);

    // Validate header
    if (!validate_header(packet.header)) {
        return false;
    }

    // Check total size matches
    if (buffer_size < sizeof(telemetry_packet_header) + packet.header.payload_length) {
        return false;
    }

    // Decode signal groups (only if flag is set)
    if (packet.header.flags & telemetry_flags::measurements) {
        std::memcpy(&packet.measurements, buffer + offset, sizeof(grid_measurements_group));
        offset += sizeof(grid_measurements_group);
    }

    if (packet.header.flags & telemetry_flags::observer) {
        std::memcpy(&packet.observer, buffer + offset, sizeof(grid_observer_group));
        offset += sizeof(grid_observer_group);
    }

    if (packet.header.flags & telemetry_flags::modulator) {
        std::memcpy(&packet.modulator, buffer + offset, sizeof(grid_modulator_group));
        offset += sizeof(grid_modulator_group);
    }

    if (packet.header.flags & telemetry_flags::controller) {
        std::memcpy(&packet.controller, buffer + offset, sizeof(grid_controller_group));
        offset += sizeof(grid_controller_group);
    }

    if (packet.header.flags & telemetry_flags::state_machine) {
        std::memcpy(&packet.state_machine, buffer + offset, sizeof(state_machine_group));
        offset += sizeof(state_machine_group);
    }

    return true;
}

}  // namespace yeet::application::sfc::telemetry
