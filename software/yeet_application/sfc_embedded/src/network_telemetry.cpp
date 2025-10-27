/**
 * @file network_telemetry.cpp
 * @brief Network telemetry protocol implementation
 */

#include "sfc_embedded/network_telemetry.hpp"

#include <cstring>

namespace yeet::application::sfc::telemetry {

std::size_t encode_grid_side_telemetry(const grid_side_telemetry_packet& packet, std::uint8_t* buffer) noexcept
{
    std::size_t offset = 0;

    // Encode header (16 bytes) - explicit byte-by-byte for portability
    const auto& header = packet.get_header();

    buffer[offset++] = header.packet_type;
    buffer[offset++] = header.protocol_version;
    buffer[offset++] = static_cast<std::uint8_t>(header.signal_count & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.signal_count >> 8) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>(header.timestamp_ms & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.timestamp_ms >> 8) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.timestamp_ms >> 16) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.timestamp_ms >> 24) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>(header.sequence_number & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.sequence_number >> 8) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.sequence_number >> 16) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.sequence_number >> 24) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>(header.reserved & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.reserved >> 8) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.reserved >> 16) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((header.reserved >> 24) & 0xFF);

    // Encode signal array (only the signals that were pushed)
    const auto& signals = packet.get_signals();
    const std::uint16_t signal_count = packet.get_signal_count();

    for (std::uint16_t i = 0; i < signal_count; ++i) {
        // Encode float as 4 bytes (little-endian IEEE 754)
        const float value = signals[i];
        const std::uint32_t* value_ptr = reinterpret_cast<const std::uint32_t*>(&value);
        const std::uint32_t raw = *value_ptr;

        buffer[offset++] = static_cast<std::uint8_t>(raw & 0xFF);
        buffer[offset++] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
        buffer[offset++] = static_cast<std::uint8_t>((raw >> 16) & 0xFF);
        buffer[offset++] = static_cast<std::uint8_t>((raw >> 24) & 0xFF);
    }

    return offset;
}

}  // namespace yeet::application::sfc::telemetry
