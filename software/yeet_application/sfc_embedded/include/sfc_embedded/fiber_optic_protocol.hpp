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

namespace yeet::application::sfc::fiber_optic {

/* ============================================================================
 * Protocol Constants
 * ============================================================================ */

/** @brief Maximum payload size for fiber-optic packets (uint32_t words) */
static constexpr std::size_t max_payload_words = 4;

/** @brief Grid → Machine payload length (uint32_t words) */
static constexpr std::size_t grid_to_machine_payload_length = 1;

/** @brief Machine → Grid payload length (uint32_t words) */
static constexpr std::size_t machine_to_grid_payload_length = 4;

/* ============================================================================
 * Grid → Machine Protocol Structures
 * ============================================================================ */

/**
 * @brief Operating mode enumeration
 *
 * Represents the current commutation mode of the converter system.
 * Values are encoded in bits 0-7 of the packed status word.
 */
enum class operating_mode : std::uint8_t {
    forced_commutation = 0,  /**< Forced commutation (low speed) */
    natural_commutation = 1, /**< Natural load commutation (normal operation) */
    synchronization = 2,     /**< Synchronization mode (future use) */
    fault = 3                /**< Fault state (future use) */
};

/**
 * @brief Grid → Machine decoded message
 *
 * Contains all signals transmitted from grid-side to machine-side controller.
 */
struct grid_to_machine_message {
    operating_mode mode{operating_mode::forced_commutation}; /**< Current operating mode */
    bool is_current_zero{false};                             /**< DC link current is zero */
    bool force_commutation_acknowledged{false};              /**< Force commutation request acknowledged */
    bool pulse_releases{false};                              /**< Thyristor pulses are enabled */
};

/**
 * @brief Encode Grid → Machine message into packed uint32_t payload
 *
 * Packs all grid-side status and control signals into a single uint32_t word
 * for transmission over fiber-optic link. Uses bit-packing to minimize bandwidth:
 * - Bits 0-7:   Operating mode (enum, 0-255)
 * - Bit 8:      Is current zero
 * - Bit 9:      Force commutation acknowledged
 * - Bit 10:     Pulse releases
 * - Bits 11-31: Reserved for future use
 *
 * @param[in]  msg      Message structure to encode
 * @param[out] payload  Array to receive encoded data (min 1 word)
 * @return Number of uint32_t words written to payload (always 1)
 *
 * Example:
 * @code
 * grid_to_machine_message msg{
 *     .mode = operating_mode::natural_commutation,
 *     .is_current_zero = false,
 *     .force_commutation_acknowledged = true,
 *     .pulse_releases = true
 * };
 * std::array<std::uint32_t, max_payload_words> payload;
 * encode_grid_to_machine(msg, payload);
 * @endcode
 */
[[nodiscard]] std::uint8_t encode_grid_to_machine(const grid_to_machine_message& msg, std::uint32_t* payload) noexcept;

/**
 * @brief Decode Grid → Machine message from packed uint32_t payload
 *
 * Unpacks grid-side status and control signals from a single uint32_t word
 * received over fiber-optic link. Extracts bit-packed fields:
 * - Bits 0-7:   Operating mode (enum)
 * - Bit 8:      Is current zero
 * - Bit 9:      Force commutation acknowledged
 * - Bit 10:     Pulse releases
 *
 * @param[in]  payload        Array containing received data
 * @param[in]  payload_length Number of uint32_t words in payload
 * @return Decoded message if successful, std::nullopt if payload_length insufficient
 *
 * Example:
 * @code
 * if (auto msg = decode_grid_to_machine(rx_payload, rx_length)) {
 *     // Use decoded msg->mode, msg->pulse_releases, etc.
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
    float measured_speed_pu{0.0f};            /**< Measured rotor speed [pu] */
    float dc_link_current_reference_pu{0.0f}; /**< DC link current reference [pu] */
    float machine_voltage_magnitude_pu{0.0f}; /**< Machine voltage magnitude [pu] */
    bool force_commutation_request{false};    /**< Force commutation requested by machine side */
};

/**
 * @brief Encode Machine → Grid message into packed uint32_t payload
 *
 * Packs all machine-side measurements and control signals into 4 uint32_t words
 * for transmission over fiber-optic link:
 * - Word 0: Measured speed [pu] (IEEE 754 float bits)
 * - Word 1: DC link current reference [pu] (IEEE 754 float bits)
 * - Word 2: Machine voltage magnitude [pu] (IEEE 754 float bits)
 * - Word 3: Packed control flags (bit 0 = force_commutation_request)
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
 *     .measured_speed_pu = 0.5f,
 *     .dc_link_current_reference_pu = 0.8f,
 *     .machine_voltage_magnitude_pu = 1.0f,
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
 * - Word 3: Packed control flags (bit 0 = force_commutation_request)
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
 *     // Use decoded msg->measured_speed_pu, msg->dc_link_current_reference_pu, etc.
 * }
 * @endcode
 */
[[nodiscard]] std::optional<machine_to_grid_message> decode_machine_to_grid(
    const std::uint32_t* payload, std::uint8_t payload_length) noexcept;

/* ============================================================================
 * Bit Field Definitions (for reference and testing)
 * ============================================================================ */

namespace detail {
// Grid → Machine bit field layout (Word 0)
static constexpr std::uint32_t g2m_operating_mode_mask = 0x000000FFU;  // Bits 0-7
static constexpr std::uint8_t g2m_operating_mode_shift = 0;
static constexpr std::uint32_t g2m_is_current_zero_bit = (1U << 8);  // Bit 8
static constexpr std::uint32_t g2m_force_comm_ack_bit = (1U << 9);   // Bit 9
static constexpr std::uint32_t g2m_pulse_releases_bit = (1U << 10);  // Bit 10

// Machine → Grid bit field layout (Word 3)
static constexpr std::uint32_t m2g_force_comm_req_bit = (1U << 0);  // Bit 0
}  // namespace detail

}  // namespace yeet::application::sfc::fiber_optic
