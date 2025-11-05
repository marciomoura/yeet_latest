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

namespace yeet::application::sfc::fiber_optic {

/* ============================================================================
 * Protocol Constants
 * ============================================================================ */

/** @brief Maximum payload size for fiber-optic packets (uint32_t words) */
static constexpr std::size_t max_payload_words = 5;

/** @brief Grid → Machine payload length (uint32_t words) */
static constexpr std::size_t grid_to_machine_payload_length = 2;

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
    yeet::application::sfc::sfc_operating_mode_t mode{
        yeet::application::sfc::sfc_operating_mode_t::natural_commutation}; /**< Current operating mode */
    float dc_link_current_pu{0.0f};                                         /**< DC link current measurement [pu] */
    bool current_is_zero{false};                                            /**< DC link current is zero */
    bool is_saturated{false};                                               /**< Current controller is saturated */
    bool force_commutation_acknowledged{false}; /**< Force commutation request acknowledged */
    bool pulse_releases{false};                 /**< Thyristor pulses are enabled */
    std::uint8_t state_id{0};                   /**< Current grid-side state machine state */
    bool freeze_integrator{false};              /**< Unified freeze signal for machine-side integrator */
    bool is_tripped{false};                     /**< Grid-side trip status */
    bool preset_request{false};                 /**< Preset request to machine-side controllers */
};

/**
 * @brief Encode Grid → Machine message into packed uint32_t payload
 *
 * Packs all grid-side status, measurements, and control synchronization signals
 * into 2 uint32_t words for transmission over fiber-optic link:
 * - Word 0: DC link current measurement [pu] (IEEE 754 float bits)
 * - Word 1: Packed control flags and state
 *   - Bits 0-7:   Operating mode (enum, 0-255)
 *   - Bit 8:      Is current zero
 *   - Bit 9:      Current controller is saturated
 *   - Bit 10:     Force commutation acknowledged
 *   - Bit 11:     Pulse releases
 *   - Bits 12-18: State machine state ID (0-127)
 *   - Bit 19:     Freeze integrator (unified)
 *   - Bit 20:     Is tripped (grid-side trip status)
 *   - Bit 21:     Preset request
 *   - Bits 22-31: Reserved for future use
 *
 * @param[in]  msg      Message structure to encode
 * @param[out] payload  Array to receive encoded data (min 2 words)
 * @return Number of uint32_t words written to payload (always 2)
 *
 * @note Float values are transmitted as IEEE 754 bit patterns using bit_cast
 *
 * Example:
 * @code
 * grid_to_machine_message msg{
 *     .mode = operating_mode::natural_commutation,
 *     .dc_link_current_pu = 0.75f,
 *     .current_is_zero = false,
 *     .is_saturated = false,
 *     .force_commutation_acknowledged = true,
 *     .pulse_releases = true,
 *     .state_id = 5,
 *     .freeze_integrator = false,
 *     .is_tripped = false,
 *     .preset_request = false
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
 * from 2 uint32_t words received over fiber-optic link:
 * - Word 0: DC link current measurement [pu] (IEEE 754 float bits)
 * - Word 1: Packed control flags and state
 *   - Bits 0-7:   Operating mode (enum)
 *   - Bit 8:      Is current zero
 *   - Bit 9:      Current controller is saturated
 *   - Bit 10:     Force commutation acknowledged
 *   - Bit 11:     Pulse releases
 *   - Bits 12-18: State machine state ID (0-127)
 *   - Bit 19:     Freeze integrator (unified)
 *   - Bit 20:     Is tripped (grid-side trip status)
 *   - Bit 21:     Preset request
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
 *     // Use decoded signals - all 9 fields are now available
 *     process_grid_state(msg->state_id);
 *     handle_integrator_freeze(msg->freeze_integrator);
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
    bool is_tripped{false};                   /**< Machine-side trip status */
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
// Grid → Machine bit field layout (Word 1)
static constexpr std::uint32_t g2m_operating_mode_mask = 0x000000FFU;  // Bits 0-7
static constexpr std::uint8_t g2m_operating_mode_shift = 0;
static constexpr std::uint32_t g2m_is_current_zero_bit = (1U << 8);               // Bit 8
static constexpr std::uint32_t g2m_current_controller_saturated_bit = (1U << 9);  // Bit 9
static constexpr std::uint32_t g2m_force_comm_ack_bit = (1U << 10);               // Bit 10
static constexpr std::uint32_t g2m_pulse_releases_bit = (1U << 11);               // Bit 11
static constexpr std::uint32_t g2m_state_id_mask = 0x0007F000U;                   // Bits 12-18
static constexpr std::uint8_t g2m_state_id_shift = 12;
static constexpr std::uint32_t g2m_freeze_integrator_bit = (1U << 19);  // Bit 19
static constexpr std::uint32_t g2m_is_tripped_bit = (1U << 20);         // Bit 20
static constexpr std::uint32_t g2m_preset_request_bit = (1U << 21);     // Bit 21

// Machine → Grid bit field layout (Word 3)
static constexpr std::uint32_t m2g_force_comm_req_bit = (1U << 0);  // Bit 0
static constexpr std::uint32_t m2g_is_tripped_bit = (1U << 1);      // Bit 1
}  // namespace detail

}  // namespace yeet::application::sfc::fiber_optic
