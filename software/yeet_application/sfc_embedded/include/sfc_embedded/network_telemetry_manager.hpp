/**
 * @file network_telemetry_manager.hpp
 * @brief Telemetry streaming manager for network interface
 *
 * This module manages telemetry packet construction and transmission scheduling.
 * It handles rate control (decimation), packet building, and transmission state.
 *
 * Design Features:
 * - Stateful: Tracks streaming state, timestamps, decimation counters
 * - Non-blocking: Never overwrites pending transmission buffers
 * - Configurable: Transmission rate and signal groups can be adjusted
 * - Efficient: Only builds packets when transmission is allowed
 *
 * Usage Pattern:
 * 1. Create manager with control system reference
 * 2. Call start() / stop() to control streaming
 * 3. Call update() periodically (e.g., from network task)
 * 4. update() returns packet data when transmission is due
 */

#pragma once

#include <cstdint>

#include "sfc_embedded/network_telemetry.hpp"
#include "sfc_embedded/network_telemetry_builder.hpp"
#include "sfc_integration/sfc_grid_side_control.hpp"

namespace yeet::application::sfc::network {

/**
 * @brief Telemetry packet ready for transmission
 *
 * Contains encoded binary data and metadata for transmission.
 */
struct telemetry_transmission {
    const std::uint8_t* data = nullptr; /**< Pointer to encoded packet data */
    std::size_t size = 0;               /**< Size of encoded packet in bytes */

    /**
     * @brief Check if transmission is ready
     * @return true if valid data is available for transmission
     */
    [[nodiscard]] constexpr bool is_ready() const noexcept { return data != nullptr && size > 0; }
};

/**
 * @brief Telemetry streaming manager
 *
 * Manages telemetry packet construction and transmission scheduling.
 * Handles rate control, packet building, and state management.
 */
class telemetry_manager {
public:
    /**
     * @brief Construct telemetry manager
     * @param control Reference to grid-side control system
     * @param decimation_factor Transmission rate divisor (default: 10 = 100 Hz from 1 kHz)
     *
     * Example:
     * @code
     * telemetry_manager manager(control, 10);  // 100 Hz transmission
     * @endcode
     */
    explicit telemetry_manager(const integration::sfc_grid_side_control& control, std::uint32_t decimation_factor = 10)
        : _control(control), _decimation_factor(decimation_factor)
    {
    }

    /**
     * @brief Start telemetry streaming
     *
     * Resets internal counters and enables packet transmission.
     */
    void start() noexcept;

    /**
     * @brief Stop telemetry streaming
     *
     * Disables packet transmission.
     */
    void stop() noexcept;

    /**
     * @brief Check if streaming is active
     * @return true if telemetry streaming is enabled
     */
    [[nodiscard]] constexpr bool is_streaming() const noexcept { return _is_streaming; }

    /**
     * @brief Update telemetry manager (call periodically)
     *
     * This method should be called at a fixed rate (e.g., 1 kHz from network task).
     * It handles decimation, packet building, and returns data when transmission is due.
     *
     * @param previous_tx_pending true if previous transmission is still pending
     * @return Telemetry packet ready for transmission (or empty if not ready)
     *
     * Example:
     * @code
     * void network_task() {
     *     auto tx = manager.update(outputs->network_tx.transmit_request);
     *     if (tx.is_ready()) {
     *         memcpy(outputs->network_tx.tx_buffer, tx.data, tx.size);
     *         outputs->network_tx.tx_length = tx.size;
     *         outputs->network_tx.transmit_request = true;
     *     }
     * }
     * @endcode
     */
    [[nodiscard]] telemetry_transmission update(bool previous_tx_pending) noexcept;

    /**
     * @brief Set transmission rate decimation factor
     * @param factor Decimation factor (e.g., 10 = transmit every 10th call)
     */
    void set_decimation_factor(std::uint32_t factor) noexcept { _decimation_factor = factor; }

    /**
     * @brief Get current decimation factor
     * @return Decimation factor
     */
    [[nodiscard]] constexpr std::uint32_t get_decimation_factor() const noexcept { return _decimation_factor; }

private:
    const integration::sfc_grid_side_control& _control; /**< Reference to control system */
    bool _is_streaming = false;                         /**< Streaming active flag */
    std::uint32_t _timestamp_ms = 0;                    /**< System timestamp in milliseconds */
    std::uint32_t _decimation_counter = 0;              /**< Counter for rate control */
    std::uint32_t _decimation_factor = 10;              /**< Transmit every Nth call */

    // Internal buffer for encoded packet (allocated on stack of this object)
    alignas(std::uint32_t) std::uint8_t _packet_buffer[telemetry::max_packet_size];
};

}  // namespace yeet::application::sfc::network
