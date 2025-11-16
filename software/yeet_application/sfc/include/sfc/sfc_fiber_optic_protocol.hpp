/**
 * @file fiber_optic_protocol.hpp
 * @brief Fiber-optic communication protocol encoding/decoding
 *
 * This module provides type-safe, efficient encoding and decoding functions for the
 * inter-board fiber-optic communication protocol between grid-side and machine-side controllers.
 *
 * Protocol Features:
 * - Bit-packed format for minimal bandwidth usage
 * - Standards-compliant float transmission via bit_cast
 * - Extensible design with reserved bits for future expansion
 * - Clear separation of concerns (encoding/decoding logic isolated from application)
 * - Modern C++17 implementation with type safety
 *
 * Protocol Specifications:
 * - Grid → Machine: 1 uint32_t word (4 bytes) - 75% bandwidth reduction!
 * - Machine → Grid: 4 uint32_t words (16 bytes)
 * - Total bidirectional: 20 bytes per cycle (37.5% reduction from original 32 bytes)
 *
 * @see fiber_optic_protocol.md for complete protocol documentation
 */

#pragma once

#include <array>
#include <bitset>
#include <cstdint>
#include <cstring>
#include <optional>

#include "sfc/sfc_types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::fiber_optic {

/* ============================================================================
 * Protocol Constants
 * ============================================================================ */

/** @brief Maximum payload size for fiber-optic packets (uint32_t words) */
static constexpr std::size_t max_payload_words = 5;

/** @brief Grid → Machine payload length (uint32_t words) */
static constexpr std::size_t grid_to_machine_payload_length = 3;

/** @brief Machine → Grid payload length (uint32_t words) */
static constexpr std::size_t machine_to_grid_payload_length = 4;

/* ============================================================================
 * Grid → Machine Protocol Structures
 * ============================================================================ */

/**
 * @brief Grid → Machine decoded message
 *
 * Contains all signals transmitted from grid-side to machine-side controller.
 * Includes both status/measurements and control synchronization signals.
 */
struct grid_to_machine_message {
    yeet::current_pu_t dc_link_current{0.0f};        /**< DC link current measurement [pu] */
    yeet::voltage_pu_t grid_voltage_magnitude{0.0f}; /**< Grid voltage magnitude [pu] */
    bool is_forced_commutation{false};               /**< System operating in forced commutation mode */
    bool force_commutation_acknowledged{false};      /**< Force commutation request acknowledged */
    bool release_pulses{false};                      /**< Thyristor pulses are enabled */
    bool grid_release_protections{false};            /**< Grid-side protection release (from state machine) */
    bool grid_trip_acknowledged{false};              /**< Grid-side trip acknowledgment (from state machine) */
    std::uint8_t state_id{0};                        /**< Current grid-side state machine state */

    // Bit field layout for Word 2 encoding
    // Boolean flags occupy bits 0-4, state_id occupies bits 25-31, unused bits 5-24
    enum bit_positions : std::uint8_t {
        is_forced_commutation_bit_pos = 0,     // Bit 0
        force_comm_ack_bit_pos = 1,            // Bit 1
        release_pulses_bit_pos = 2,            // Bit 2
        grid_release_protections_bit_pos = 3,  // Bit 3
        grid_trip_acknowledged_bit_pos = 4,    // Bit 4
        unused_bits_start = 5,                 // Bits 5-24 reserved
        unused_bits_end = 24
    };

    enum bit_shifts : std::uint8_t {
        state_id_shift = 25  // Bits 25-31 (7 bits for state_id, 0-127)
    };

    enum bit_masks : std::uint32_t {
        state_id_mask = 0x7FU  // 7-bit mask for state_id (0-127)
    };
};

/**
 * @brief Encode Grid → Machine message into packed uint32_t payload
 *
 * Packs all grid-side status, measurements, and control synchronization signals
 * into 3 uint32_t words for transmission over fiber-optic link:
 * - Word 0: DC link current measurement [pu] (IEEE 754 float bits)
 * - Word 1: Grid voltage magnitude [pu] (IEEE 754 float bits)
 * - Word 2: Packed control flags and state
 *   - Bit 0:      Is forced commutation mode
 *   - Bit 1:      Force commutation acknowledged
 *   - Bit 2:      Release pulses
 *   - Bit 3:      Grid release protections
 *   - Bit 4:      Grid trip acknowledged
 *   - Bits 5-24:  Reserved for future use
 *   - Bits 25-31: State machine state ID (0-127)
 *
 * @param[in]  msg      Message structure to encode
 * @param[out] payload  Array to receive encoded data (min 3 words)
 * @return Number of uint32_t words written to payload (always 3)
 *
 * @note Float values are transmitted as IEEE 754 bit patterns using bit_cast
 *
 * Example:
 * @code
 * grid_to_machine_message msg{
 *     .dc_link_current = yeet::current_pu_t{0.75f},
 *     .grid_voltage_magnitude = yeet::voltage_pu_t{1.0f},
 *     .is_forced_commutation = false,
 *     .force_commutation_acknowledged = true,
 *     .release_pulses = true,
 *     .state_id = 5
 * };
 * std::array<std::uint32_t, max_payload_words> payload;
 * encode_grid_to_machine(msg, payload);
 * @endcode
 */
[[nodiscard]] std::uint8_t encode_grid_to_machine(const grid_to_machine_message& msg, std::uint32_t* payload) noexcept;

/**
 * @brief Decode Grid → Machine message from packed uint32_t payload
 *
 * Unpacks grid-side status, measurements, and control synchronization signals
 * from 3 uint32_t words received over fiber-optic link:
 * - Word 0: DC link current measurement [pu] (IEEE 754 float bits)
 * - Word 1: Grid voltage magnitude [pu] (IEEE 754 float bits)
 * - Word 2: Packed control flags and state
 *   - Bit 0:      Is forced commutation mode
 *   - Bit 1:      Force commutation acknowledged
 *   - Bit 2:      Release pulses
 *   - Bit 3:      Grid release protections
 *   - Bit 4:      Grid trip acknowledged
 *   - Bits 5-24:  Reserved (unused)
 *   - Bits 25-31: State machine state ID (0-127)
 *
 * @param[in]  payload        Array containing received data
 * @param[in]  payload_length Number of uint32_t words in payload
 * @return Decoded message if successful, std::nullopt if payload_length insufficient
 *
 * @note Float values are reconstructed from IEEE 754 bit patterns using bit_cast
 *
 * Example:
 * @code
 * if (auto msg = decode_grid_to_machine(rx_payload, rx_length)) {
 *     // Use decoded signals
 *     process_grid_state(msg->state_id);
 * }
 * @endcode
 */
[[nodiscard]] std::optional<grid_to_machine_message> decode_grid_to_machine(
    const std::uint32_t* payload, std::uint8_t payload_length) noexcept;

/* ============================================================================
 * Machine → Grid Protocol Structures
 * ============================================================================ */

/**
 * @brief Machine → Grid decoded message
 *
 * Contains all signals transmitted from machine-side to grid-side controller.
 */
struct machine_to_grid_message {
    yeet::speed_pu_t measured_speed{0.0f};              /**< Measured rotor speed [pu] */
    yeet::current_pu_t dc_link_current_reference{0.0f}; /**< DC link current reference [pu] */
    yeet::voltage_pu_t machine_voltage_magnitude{0.0f}; /**< Machine voltage magnitude [pu] */
    bool force_commutation_request{false};              /**< Force commutation requested by machine side */
    bool shutdown_request{false};           /**< Shutdown request from machine side (brings current to zero) */
    bool machine_protection_tripped{false}; /**< Machine-side protections have tripped */
    bool protection_inhibit{false};         /**< Protection system inhibited (awaiting user acknowledgment) */
    bool machine_trip_acknowledged{false};  /**< Machine-side trip acknowledgment */

    // Bit field layout for Word 3 encoding
    enum bit_positions : std::uint8_t {
        force_comm_req_bit_pos = 0,              // Bit 0
        shutdown_request_bit_pos = 1,            // Bit 1
        machine_protection_tripped_bit_pos = 2,  // Bit 2
        protection_inhibit_bit_pos = 3,          // Bit 3
        machine_trip_acknowledged_bit_pos = 4    // Bit 4
    };
};

/**
 * @brief Encode Machine → Grid message into packed uint32_t payload
 *
 * Packs all machine-side measurements and control signals into 4 uint32_t words
 * for transmission over fiber-optic link:
 * - Word 0: Measured speed [pu] (IEEE 754 float bits)
 * - Word 1: DC link current reference [pu] (IEEE 754 float bits)
 * - Word 2: Machine voltage magnitude [pu] (IEEE 754 float bits)
 * - Word 3: Packed control flags
 *   - Bit 0: Force commutation request
 *   - Bit 1: Shutdown request
 *   - Bit 2: Machine protection tripped
 *   - Bit 3: Protection inhibit (awaiting acknowledgment)
 *   - Bit 4: Machine trip acknowledged
 *   - Bits 5-31: Reserved for future use
 *
 * @param[in]  msg      Message structure to encode
 * @param[out] payload  Array to receive encoded data (min 4 words)
 * @return Number of uint32_t words written to payload (always 4)
 *
 * @note Float values are transmitted as IEEE 754 bit patterns using bit_cast
 *
 * Example:
 * @code
 * machine_to_grid_message msg{
 *     .measured_speed = yeet::speed_pu_t{0.5f},
 *     .dc_link_current_reference = yeet::current_pu_t{0.8f},
 *     .machine_voltage_magnitude = yeet::voltage_pu_t{1.0f},
 *     .force_commutation_request = false
 * };
 * std::array<std::uint32_t, max_payload_words> payload;
 * encode_machine_to_grid(msg, payload);
 * @endcode
 */
[[nodiscard]] std::uint8_t encode_machine_to_grid(const machine_to_grid_message& msg, std::uint32_t* payload) noexcept;

/**
 * @brief Decode Machine → Grid message from packed uint32_t payload
 *
 * Unpacks machine-side measurements and control signals from 4 uint32_t words
 * received over fiber-optic link:
 * - Word 0: Measured speed [pu] (IEEE 754 float bits)
 * - Word 1: DC link current reference [pu] (IEEE 754 float bits)
 * - Word 2: Machine voltage magnitude [pu] (IEEE 754 float bits)
 * - Word 3: Packed control flags
 *   - Bit 0: Force commutation request
 *   - Bit 1: Shutdown request
 *   - Bit 2: Machine protection tripped
 *   - Bit 3: Protection inhibit (awaiting acknowledgment)
 *   - Bit 4: Machine trip acknowledged
 *   - Bits 5-31: Reserved for future use
 *
 * @param[in]  payload        Array containing received data
 * @param[in]  payload_length Number of uint32_t words in payload
 * @return Decoded message if successful, std::nullopt if payload_length insufficient
 *
 * @note Float values are reconstructed from IEEE 754 bit patterns using bit_cast
 *
 * Example:
 * @code
 * if (auto msg = decode_machine_to_grid(rx_payload, rx_length)) {
 *     // Use decoded msg->measured_speed, msg->dc_link_current_reference, etc.
 * }
 * @endcode
 */
[[nodiscard]] std::optional<machine_to_grid_message> decode_machine_to_grid(
    const std::uint32_t* payload, std::uint8_t payload_length) noexcept;

}  // namespace yeet::application::sfc::fiber_optic
