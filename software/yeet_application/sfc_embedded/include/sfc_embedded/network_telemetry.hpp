/**
 * @file network_telemetry.hpp
 * @brief Network telemetry protocol for transmitting real-time control data
 *
 * This module provides a flexible, efficient protocol for streaming control system
 * telemetry data over TCP/IP. Designed for both grid-side and machine-side controllers.
 *
 * Design Features:
 * - Variable-length array-based packets (send only what you need)
 * - Simple float array payload (no complex structs or padding)
 * - Extensible: add signals by extending indices, no protocol changes
 * - Bandwidth efficient: transmit only populated signals
 * - Type-safe C++17 implementation with std::array
 * - Generic packet class supports both grid-side and machine-side via packet_type parameter
 *
 * Protocol Structure:
 * - Header: Packet type, version, signal count, timestamp, sequence number (16 bytes)
 * - Payload: Variable-length array of float signals (N * 4 bytes)
 * - CRC32: Integrity checksum over header + payload (4 bytes)
 * - Signal mapping: Documented index → semantic meaning
 *
 * Example Usage:
 * @code
 * // Create telemetry packet
 * telemetry_packet_t packet;
 * packet.push_signal(signal_1);
 * packet.push_signal(signal_2);
 * packet.finalize(get_time_ms(), sequence_num);
 *
 * // Encode and transmit
 * std::array<std::uint8_t, max_packet_size> buffer;
 * auto size = encode_telemetry_packet(packet, buffer.data());
 * tcp_send(buffer.data(), size);
 *
 * // The packet type is determined when finalize() is called,
 * // or can be controlled via template parameter.
 * @endcode
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>

namespace yeet::application::sfc::telemetry {

/* ============================================================================
 * Protocol Constants
 * ============================================================================ */

/** @brief Protocol version (increment when changing packet structure) */
static constexpr std::uint8_t protocol_version = 1;

/** @brief Synchronization magic number (0xDEADBEEF) sent at stream start */
static constexpr std::uint32_t sync_magic = 0xDEADBEEF;

/** @brief Maximum number of signals per packet */
static constexpr std::size_t max_signals = 256;

/** @brief CRC32 size in bytes */
static constexpr std::size_t crc_size = 4;

/** @brief Maximum encoded packet size (header + max signals + CRC32) */
static constexpr std::size_t max_packet_size = 16 + (max_signals * sizeof(float)) + crc_size;

/* ============================================================================
 * Packet Type Enumeration
 * ============================================================================ */

/**
 * @brief Telemetry packet types
 *
 * Identifies which controller side generated the packet.
 */
enum class packet_type : std::uint8_t {
    sync_frame = 0x00,                     /**< Stream synchronization marker (magic: 0xDEADBEEF) */
    grid_side_telemetry = 0x01,            /**< Grid-side control telemetry */
    machine_side_telemetry = 0x02,         /**< Machine-side control telemetry */
    system_status = 0x10,                  /**< System health and diagnostics */
    configuration = 0x20,                  /**< Configuration parameters */
    command = 0x30,                        /**< Command from host to controller */
    parameter_set_request = 0x40,          /**< Host → Controller: Set parameter */
    parameter_set_response = 0x41,         /**< Controller → Host: Confirm/reject SET */
    parameter_get_request = 0x42,          /**< Host → Controller: Query parameter value */
    parameter_get_response = 0x43,         /**< Controller → Host: Parameter value */
    parameter_count_request = 0x46,        /**< Host → Controller: Get parameter count */
    parameter_count_response = 0x47,       /**< Controller → Host: Total parameter count */
    parameter_get_by_index_request = 0x48, /**< Host → Controller: Get parameter by index */
    parameter_get_by_index_response = 0x49 /**< Controller → Host: Parameter metadata by index */
};

/* ============================================================================
 * Generic Packet Header (for parameter/command packets)
 * ============================================================================ */

/**
 * @brief Generic packet header (16 bytes fixed)
 *
 * Used for parameter GET/SET and other command/response packets.
 * This is a simplified header compatible with the old telemetry_packet_header
 * used by parameter_handler.
 */
struct telemetry_packet_header {
    std::uint8_t packet_type;      /**< Packet type identifier */
    std::uint8_t protocol_version; /**< Protocol version for compatibility */
    std::uint16_t payload_length;  /**< Total payload size in bytes (after header) */
    std::uint32_t flags;           /**< Bit flags for packet options */
    std::uint32_t timestamp_ms;    /**< Controller timestamp in milliseconds */
    std::uint32_t sequence_number; /**< Incrementing sequence number (detect drops) */
};
static_assert(sizeof(telemetry_packet_header) == 16, "Header must be 16 bytes");

/* ============================================================================
 * Telemetry Packet Structure (Generic)
 * ============================================================================ */

/**
 * @brief Generic telemetry packet with variable-length signal array
 *
 * A truly generic packet structure that works with any packet type.
 * The packet type is determined at runtime when finalize() is called,
 * making this a single implementation for all controller types.
 *
 * Simple, extensible packet structure:
 * - Header contains metadata (timestamp, signal count, packet type)
 * - Payload is a variable-length array of float signals
 * - Only signals that are pushed are transmitted
 * - Signal indices map to documented semantic meanings
 *
 * The same packet class is used for grid-side, machine-side, or any
 * other telemetry source. No templates, no type aliases needed.
 *
 * Usage:
 * @code
 * // Create packet and push signals
 * telemetry_packet packet;
 * packet.push_signal(value1);
 * packet.push_signal(value2);
 * packet.push_signal(value3);
 *
 * // Finalize sets the packet type and other header fields
 * packet.finalize(packet_type::grid_side_telemetry, timestamp_ms, sequence_num);
 *
 * // Encode and transmit
 * std::array<std::uint8_t, max_packet_size> buffer;
 * size_t encoded_size = encode_telemetry_packet(packet, buffer.data());
 * network_send(buffer.data(), encoded_size);
 * @endcode
 */
class telemetry_packet {
public:
    /**
     * @brief Packet header structure (16 bytes fixed)
     */
    struct header_t {
        std::uint8_t packet_type;      /**< Packet type (grid/machine side) */
        std::uint8_t protocol_version; /**< Protocol version for compatibility */
        std::uint16_t signal_count;    /**< Number of signals in payload */
        std::uint32_t timestamp_ms;    /**< Controller timestamp in milliseconds */
        std::uint32_t sequence_number; /**< Incrementing sequence (detect drops) */
        std::uint32_t reserved;        /**< Reserved for future use (padding to 16 bytes) */
    };
    static_assert(sizeof(header_t) == 16, "Header must be 16 bytes");

    /**
     * @brief Default constructor - initializes empty packet
     */
    telemetry_packet() noexcept : _header{}, _signal_count(0), _signals{} {}

    /**
     * @brief Push a signal value into the packet
     *
     * Adds a signal to the payload array. Signals are added sequentially
     * in the order they are pushed. Index mapping is documented externally.
     *
     * @param value Signal value to add
     * @return true if signal was added, false if packet is full
     */
    bool push_signal(float value) noexcept
    {
        if (_signal_count >= max_signals) {
            return false;
        }
        _signals[_signal_count++] = value;
        return true;
    }

    /**
     * @brief Add a signal at a specific index in the packet
     *
     * Adds a signal at the specified index position. This allows
     * explicit control over signal placement rather than sequential ordering.
     * The signal count is automatically updated if needed.
     *
     * @param index Signal index (0-based position in payload)
     * @param value Signal value to add
     * @return true if signal was added, false if index is out of range
     */
    bool add_signal(std::uint16_t index, float value) noexcept
    {
        if (index >= max_signals) {
            return false;
        }
        _signals[index] = value;
        // Update signal count if this index extends beyond current count
        if (index >= _signal_count) {
            _signal_count = index + 1;
        }
        return true;
    }

    /**
     * @brief Finalize packet header before transmission
     *
     * Sets header fields and prepares packet for encoding.
     * Call this after all signals have been pushed.
     *
     * @param type Packet type (grid_side_telemetry, machine_side_telemetry, etc.)
     * @param timestamp_ms Current system timestamp
     * @param sequence_number Packet sequence number
     */
    void finalize(packet_type type, std::uint32_t timestamp_ms, std::uint32_t sequence_number) noexcept
    {
        _header.packet_type = static_cast<std::uint8_t>(type);
        _header.protocol_version = telemetry::protocol_version;
        _header.signal_count = _signal_count;
        _header.timestamp_ms = timestamp_ms;
        _header.sequence_number = sequence_number;
        _header.reserved = 0;
    }

    /**
     * @brief Reset packet for reuse
     *
     * Clears signal count without zeroing memory (for efficiency).
     * Call this before reusing a packet object.
     */
    void reset() noexcept { _signal_count = 0; }

    /**
     * @brief Get number of signals currently in packet
     */
    std::uint16_t get_signal_count() const noexcept { return _signal_count; }

    /**
     * @brief Get packet header (const)
     */
    const header_t& get_header() const noexcept { return _header; }

    /**
     * @brief Get signal array (const)
     */
    const std::array<float, max_signals>& get_signals() const noexcept { return _signals; }

    /**
     * @brief Calculate encoded size of this packet
     *
     * @return Total bytes needed for transmission (header + signals + CRC32)
     */
    std::size_t get_encoded_size() const noexcept
    {
        return sizeof(header_t) + (_signal_count * sizeof(float)) + crc_size;
    }

private:
    header_t _header;                        /**< Packet header (16 bytes) */
    std::uint16_t _signal_count;             /**< Current number of signals */
    std::array<float, max_signals> _signals; /**< Signal payload array */
};

/* ============================================================================
 * Encoding Functions (Generic)
 * ============================================================================ */

/**
 * @brief Encode telemetry packet to binary buffer (generic for all packet types)
 *
 * Serializes header + signal array + CRC32 to a byte buffer for transmission.
 * Only encodes the signals that were pushed (based on signal_count).
 * Works with any telemetry packet type (grid-side, machine-side, etc.).
 *
 * The CRC32 is calculated over the entire packet (header + signals) using
 * the IEEE 802.3 polynomial, matching Python's zlib.crc32() implementation.
 *
 * @param packet Source packet with populated data
 * @param buffer Destination buffer (min packet.get_encoded_size() bytes)
 * @return Number of bytes written to buffer (includes header + signals + CRC32)
 *
 * Example:
 * @code
 * telemetry_packet packet;
 * packet.push_signal(voltage_pu);
 * packet.finalize(packet_type::grid_side_telemetry, get_time_ms(), seq_num);
 *
 * std::array<std::uint8_t, max_packet_size> buffer;
 * size_t size = encode_telemetry_packet(packet, buffer.data());
 * tcp_send(buffer.data(), size);
 * @endcode
 */
std::size_t encode_telemetry_packet(const telemetry_packet& packet, std::uint8_t* buffer) noexcept;
/**
 * @brief Encode synchronization frame (magic number 0xDEADBEEF)
 *
 * Sends a distinctive marker at the beginning of telemetry streams to help
 * the client resynchronize if it connects mid-stream. The sync frame is 4 bytes:
 * 0xEF 0xBE 0xAD 0xDE (little-endian 0xDEADBEEF).
 *
 * This is sent once when START command is received, before any telemetry packets.
 *
 * @param buffer Destination buffer (min 4 bytes)
 * @return Number of bytes written (always 4)
 *
 * Example:
 * @code
 * // When START command received
 * std::uint8_t buffer[4];
 * encode_sync_frame(buffer);
 * tcp_send(buffer, 4);
 *
 * // Then start sending telemetry packets normally
 * // Client will recognize 0xDEADBEEF and know stream has started
 * @endcode
 */
std::size_t encode_sync_frame(std::uint8_t* buffer) noexcept;

}  // namespace yeet::application::sfc::telemetry
