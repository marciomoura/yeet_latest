/**
 * @file network_telemetry.cpp
 * @brief Network telemetry protocol implementation
 */

#include "yeet/network_telemetry.hpp"

#include <cstring>

namespace yeet::telemetry {

namespace {

/**
 * @brief Calculate CRC32 checksum using standard polynomial
 *
 * Implements CRC-32 (IEEE 802.3) algorithm for packet integrity validation.
 * This matches Python's zlib.crc32() implementation.
 *
 * @param data Pointer to data buffer
 * @param length Number of bytes to checksum
 * @return 32-bit CRC checksum
 */
std::uint32_t calculate_crc32(const std::uint8_t* data, std::size_t length) noexcept
{
    // CRC-32 (IEEE 802.3) polynomial: 0xEDB88320
    static constexpr std::uint32_t polynomial = 0xEDB88320;

    std::uint32_t crc = 0xFFFFFFFF;  // Initial value

    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ (-(static_cast<int>(crc) & 1) & polynomial);
        }
    }

    return ~crc;  // Final XOR
}

}  // anonymous namespace

/**
 * @brief Generic implementation for encoding telemetry packets
 *
 * This implementation works for all packet types (grid-side, machine-side, etc.).
 * The packet type is already determined when finalize() was called.
 */
std::size_t encode_telemetry_packet(const telemetry_packet& packet, std::uint8_t* buffer) noexcept
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
    // Signals are stored as raw uint32_t values (floats are bit-cast)
    const auto& signals = packet.get_signals();
    const std::uint16_t signal_count = packet.get_signal_count();

    for (std::uint16_t i = 0; i < signal_count; ++i) {
        // Encode uint32_t as 4 bytes (little-endian)
        const std::uint32_t raw = signals[i];

        buffer[offset++] = static_cast<std::uint8_t>(raw & 0xFF);
        buffer[offset++] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
        buffer[offset++] = static_cast<std::uint8_t>((raw >> 16) & 0xFF);
        buffer[offset++] = static_cast<std::uint8_t>((raw >> 24) & 0xFF);
    }

    // Calculate CRC32 over header + signals (everything encoded so far)
    const std::uint32_t crc = calculate_crc32(buffer, offset);

    // Append CRC32 at end of packet (little-endian)
    buffer[offset++] = static_cast<std::uint8_t>(crc & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((crc >> 8) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((crc >> 16) & 0xFF);
    buffer[offset++] = static_cast<std::uint8_t>((crc >> 24) & 0xFF);

    return offset;
}

std::size_t encode_sync_frame(std::uint8_t* buffer) noexcept
{
    // Encode sync magic number 0xDEADBEEF in little-endian
    // This creates the byte sequence: 0xEF 0xBE 0xAD 0xDE
    const std::uint32_t magic = sync_magic;

    buffer[0] = static_cast<std::uint8_t>(magic & 0xFF);
    buffer[1] = static_cast<std::uint8_t>((magic >> 8) & 0xFF);
    buffer[2] = static_cast<std::uint8_t>((magic >> 16) & 0xFF);
    buffer[3] = static_cast<std::uint8_t>((magic >> 24) & 0xFF);

    return 4;  // Sync frame is always 4 bytes
}

}  // namespace yeet::telemetry
