/**
 * @file network_telemetry_manager.cpp
 * @brief Implementation of telemetry streaming manager
 */

#include "yeet/network_telemetry_manager.hpp"

#include <cstring>

namespace yeet::telemetry {

void telemetry_manager::start() noexcept
{
    _is_streaming = true;
    _decimation_counter = 0;
    _timestamp_ms = 0;
}

void telemetry_manager::stop() noexcept { _is_streaming = false; }

telemetry_transmission telemetry_manager::update(bool previous_tx_pending) noexcept
{
    // Don't build packet if not streaming
    if (!_is_streaming) {
        return {nullptr, 0};
    }

    // Don't overwrite pending transmission
    if (previous_tx_pending) {
        return {nullptr, 0};
    }

    // Decimation: Only transmit every Nth call
    _decimation_counter++;
    if (_decimation_counter < _decimation_factor) {
        return {nullptr, 0};  // Not yet time to transmit
    }
    _decimation_counter = 0;

    // Build telemetry packet from control system
    auto packet = _builder.build_packet(_timestamp_ms);

    // Encode packet to binary buffer
    std::size_t encoded_size = telemetry::encode_telemetry_packet(packet, _packet_buffer);

    if (encoded_size > 0) {
        // Increment timestamp (approximately 10 ms per call if called at 1 kHz with decimation=10)
        // Note: In production, use actual hardware timer for accurate timestamps
        const std::uint32_t time_increment_ms = _decimation_factor;  // Approximate time per decimated cycle
        _timestamp_ms += time_increment_ms;

        // Return packet data for transmission
        return {_packet_buffer, encoded_size};
    }

    // Encoding failed
    return {nullptr, 0};
}

}  // namespace yeet::telemetry
