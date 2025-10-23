/**
 * @file network_telemetry.hpp
 * @brief Network telemetry protocol for transmitting real-time control data
 *
 * This module provides a scalable, efficient protocol for streaming control system
 * telemetry data over TCP/IP. Designed for both grid-side and machine-side controllers.
 *
 * Design Features:
 * - Fixed-size binary packets for deterministic parsing
 * - Hierarchical signal grouping (measurements, observers, controllers, etc.)
 * - Bit flags to indicate which groups are present in a packet
 * - Extensible design: add new groups without breaking existing parsers
 * - Type-safe C++17 implementation
 *
 * Protocol Structure:
 * - Header: Packet type, version, flags, timestamp
 * - Payload: Variable number of signal groups (indicated by flags)
 * - Each group: Fixed-size struct with relevant signals
 *
 * Example Usage:
 * @code
 * // Populate telemetry packet
 * grid_side_telemetry_packet packet;
 * packet.header.timestamp_ms = get_time_ms();
 * packet.measurements = {grid_voltage_pu, dc_current_pu, ...};
 * packet.observer = {frequency_pu, angle_rad, ...};
 * packet.header.flags = TELEM_FLAG_MEASUREMENTS | TELEM_FLAG_OBSERVER;
 *
 * // Encode and transmit
 * uint8_t buffer[512];
 * size_t size = encode_telemetry_packet(packet, buffer);
 * tcp_send(buffer, size);
 * @endcode
 */

#pragma once

#include <cstdint>
#include <cstring>

namespace yeet::application::sfc::telemetry {

/* ============================================================================
 * Protocol Constants
 * ============================================================================ */

/** @brief Protocol version (increment when changing packet structure) */
static constexpr std::uint8_t protocol_version = 1;

/** @brief Maximum packet size (fits in single TCP segment) */
static constexpr std::size_t max_packet_size = 1024;

/* ============================================================================
 * Packet Type Enumeration
 * ============================================================================ */

/**
 * @brief Telemetry packet types
 *
 * Identifies which controller side generated the packet and what data it contains.
 */
enum class packet_type : std::uint8_t {
    grid_side_telemetry = 0x01,    /**< Grid-side control telemetry */
    machine_side_telemetry = 0x02, /**< Machine-side control telemetry */
    system_status = 0x10,          /**< System health and diagnostics */
    configuration = 0x20,          /**< Configuration parameters */
    command = 0x30                 /**< Command from host to controller */
};

/* ============================================================================
 * Signal Group Flags (Bit Masks)
 * ============================================================================ */

/**
 * @brief Flags indicating which signal groups are present in packet
 *
 * Each bit represents a specific signal group. Multiple groups can be
 * combined in a single packet by OR-ing flags together.
 */
namespace telemetry_flags {
static constexpr std::uint32_t measurements = (1U << 0);    /**< Voltage/current measurements */
static constexpr std::uint32_t observer = (1U << 1);        /**< Observer estimates (freq, angle) */
static constexpr std::uint32_t modulator = (1U << 2);       /**< Modulator outputs (firing angle) */
static constexpr std::uint32_t controller = (1U << 3);      /**< Controller state (ref, measured, mode) */
static constexpr std::uint32_t state_machine = (1U << 4);   /**< State machine status */
static constexpr std::uint32_t pulse_generator = (1U << 5); /**< Pulse generator outputs */
static constexpr std::uint32_t excitation = (1U << 6);      /**< Excitation controller (machine-side) */
static constexpr std::uint32_t speed_control = (1U << 7);   /**< Speed controller (machine-side) */
static constexpr std::uint32_t diagnostics = (1U << 8);     /**< Diagnostic counters and flags */
static constexpr std::uint32_t reserved_9 = (1U << 9);      /**< Reserved for future use */
static constexpr std::uint32_t reserved_10 = (1U << 10);    /**< Reserved for future use */
// ... up to 32 groups
}  // namespace telemetry_flags

/* ============================================================================
 * Common Packet Header
 * ============================================================================ */

/**
 * @brief Common header for all telemetry packets
 *
 * Fixed-size header (16 bytes) present in all packets for quick parsing.
 */
struct telemetry_packet_header {
    std::uint8_t packet_type;      /**< Packet type (see packet_type enum) */
    std::uint8_t protocol_version; /**< Protocol version (for compatibility checking) */
    std::uint16_t payload_length;  /**< Total payload size in bytes (after header) */
    std::uint32_t flags;           /**< Bit flags indicating which groups are present */
    std::uint32_t timestamp_ms;    /**< Controller timestamp in milliseconds */
    std::uint32_t sequence_number; /**< Incrementing sequence number (detect drops) */
};
static_assert(sizeof(telemetry_packet_header) == 16, "Header must be 16 bytes");

/* ============================================================================
 * Grid-Side Signal Groups
 * ============================================================================ */

/**
 * @brief Grid and DC link measurement signals (per-unit)
 *
 * Group: TELEM_FLAG_MEASUREMENTS
 * Size: 40 bytes (9 floats + 1 bool with padding)
 */
struct grid_measurements_group {
    float grid_voltage_magnitude_pu; /**< Grid voltage magnitude [pu] */
    float grid_voltage_a_pu;         /**< Phase A voltage [pu] */
    float grid_voltage_b_pu;         /**< Phase B voltage [pu] */
    float grid_voltage_c_pu;         /**< Phase C voltage [pu] */
    float grid_current_magnitude_pu; /**< Grid current magnitude [pu] */
    float grid_current_a_pu;         /**< Phase A current [pu] */
    float grid_current_b_pu;         /**< Phase B current [pu] */
    float grid_current_c_pu;         /**< Phase C current [pu] */
    float dc_link_current_pu;        /**< DC link current [pu] */
    bool is_commuting;               /**< Thyristor commutation status */
};
static_assert(sizeof(grid_measurements_group) == 40, "Must be fixed size");

/**
 * @brief Grid-side observer estimates
 *
 * Group: TELEM_FLAG_OBSERVER
 * Size: 12 bytes (3 floats)
 */
struct grid_observer_group {
    float estimated_voltage_magnitude_pu; /**< Estimated grid voltage magnitude [pu] */
    float estimated_frequency_pu;         /**< Estimated grid frequency [pu] */
    float estimated_angle_rad;            /**< Estimated grid angle [rad] */
};
static_assert(sizeof(grid_observer_group) == 12, "Must be fixed size");

/**
 * @brief Grid-side modulator outputs
 *
 * Group: TELEM_FLAG_MODULATOR
 * Size: 16 bytes (2 bools + 2 floats with padding)
 */
struct grid_modulator_group {
    bool is_enabled;              /**< Modulator enable status */
    float firing_angle_input_pu;  /**< Commanded firing angle [pu] */
    float firing_angle_output_pu; /**< Actual firing angle (after limits) [pu] */
    bool force_commutation_ack;   /**< Force commutation acknowledgment flag */
};
static_assert(sizeof(grid_modulator_group) == 16, "Must be fixed size");

/**
 * @brief Current controller state
 *
 * Group: TELEM_FLAG_CONTROLLER
 * Size: 20 bytes (4 floats + 4 uint8_t)
 */
struct grid_controller_group {
    float dc_link_current_reference;   /**< Control reference [pu] */
    float dc_link_current_measured;    /**< Control measured [pu] */
    float current_error_pu;            /**< Control error [pu] */
    float controller_output_pu;        /**< PI controller output [pu] */
    std::uint8_t operating_mode;       /**< 0=forced, 1=natural commutation */
    std::uint8_t controller_saturated; /**< Saturation flag */
    std::uint8_t freeze_integrator;    /**< Integrator freeze flag */
    std::uint8_t reserved;
};
static_assert(sizeof(grid_controller_group) == 20, "Must be fixed size");

/**
 * @brief State machine status
 *
 * Group: TELEM_FLAG_STATE_MACHINE
 * Size: 9 bytes (1 uint8_t + 8 bools, packed)
 */
struct state_machine_group {
    std::uint8_t current_state;   /**< Current state enum value */
    bool pulse_releases;          /**< Pulses enabled flag */
    bool auxiliaries_are_on;      /**< Auxiliaries status */
    bool breakers_are_on;         /**< Breakers status */
    bool start_command;           /**< Start command status */
    bool stop_command;            /**< Stop command status */
    bool is_current_extinguished; /**< Current extinguished status */
    bool is_shutdown_requested;   /**< Shutdown requested flag */
};
static_assert(sizeof(state_machine_group) == 8, "Must be fixed size");

/* ============================================================================
 * Complete Packet Structures
 * ============================================================================ */

/**
 * @brief Complete grid-side telemetry packet
 *
 * Contains header + all possible signal groups.
 * Only groups indicated by header.flags are populated and transmitted.
 */
struct grid_side_telemetry_packet {
    telemetry_packet_header header;
    grid_measurements_group measurements;
    grid_observer_group observer;
    grid_modulator_group modulator;
    grid_controller_group controller;
    state_machine_group state_machine;
};

/* ============================================================================
 * Encoding/Decoding Functions
 * ============================================================================ */

/**
 * @brief Encode grid-side telemetry packet to binary buffer
 *
 * Serializes only the signal groups indicated by packet.header.flags.
 * This allows efficient transmission of partial telemetry data.
 *
 * @param packet Source packet with populated data
 * @param buffer Destination buffer (min max_packet_size bytes)
 * @return Number of bytes written to buffer
 *
 * Example:
 * @code
 * grid_side_telemetry_packet packet;
 * packet.header.flags = TELEM_FLAG_MEASUREMENTS | TELEM_FLAG_CONTROLLER;
 * // ... populate measurement and controller data ...
 * uint8_t buffer[512];
 * size_t size = encode_grid_side_telemetry(packet, buffer);
 * tcp_send(buffer, size);
 * @endcode
 */
std::size_t encode_grid_side_telemetry(const grid_side_telemetry_packet& packet, std::uint8_t* buffer) noexcept;

/**
 * @brief Decode grid-side telemetry packet from binary buffer
 *
 * Parses binary data and populates packet structure.
 * Only signal groups indicated by header.flags are parsed.
 *
 * @param buffer Source buffer containing received data
 * @param buffer_size Size of received data in bytes
 * @param packet Destination packet structure
 * @return true if decode successful, false if invalid data
 *
 * Example:
 * @code
 * grid_side_telemetry_packet packet;
 * if (decode_grid_side_telemetry(rx_buffer, rx_size, packet)) {
 *     if (packet.header.flags & TELEM_FLAG_MEASUREMENTS) {
 *         printf("Grid voltage: %.3f pu\n", packet.measurements.grid_voltage_magnitude_pu);
 *     }
 * }
 * @endcode
 */
bool decode_grid_side_telemetry(
    const std::uint8_t* buffer, std::size_t buffer_size, grid_side_telemetry_packet& packet) noexcept;

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Calculate expected payload size based on flags
 *
 * @param flags Bit flags indicating which groups are present
 * @return Expected payload size in bytes (excluding header)
 */
std::size_t calculate_payload_size(std::uint32_t flags) noexcept;

/**
 * @brief Validate packet header
 *
 * @param header Header to validate
 * @return true if valid (correct version, reasonable flags), false otherwise
 */
bool validate_header(const telemetry_packet_header& header) noexcept;

}  // namespace yeet::application::sfc::telemetry
