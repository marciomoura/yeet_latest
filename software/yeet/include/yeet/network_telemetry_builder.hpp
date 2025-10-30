/**
 * @file network_telemetry_builder.hpp
 * @brief Helper classes to populate telemetry packets from control system
 *
 * This module provides convenient builders to extract telemetry data from
 * the control system components and populate network telemetry packets.
 *
 * The builder encapsulates signal extraction and packet population logic,
 * providing a clean interface to push signals from datamodel outputs into
 * the telemetry packet buffer. Both grid-side and machine-side builders
 * use the same generic interface with type-safe packet templates.
 */

#pragma once

#include "network_telemetry.hpp"

namespace yeet::telemetry {

/**
 * @brief Interface for telemetry packet builders
 *
 * Pure virtual interface that all telemetry builders must implement.
 * Allows telemetry_manager to work with any telemetry source without
 * knowing the concrete type (grid-side, machine-side, etc.).
 *
 * Usage:
 * @code
 * telemetry_builder* builder = &my_grid_side_builder;
 * auto packet = builder->build_packet(timestamp_ms);
 * @endcode
 */
class telemetry_builder {
public:
    virtual ~telemetry_builder() = default;

    /**
     * @brief Build telemetry packet with all available signals
     *
     * Pure virtual method that must be implemented by concrete builders.
     * Extracts signals from control system and populates packet.
     *
     * @param timestamp_ms Current system time in milliseconds
     * @return Complete telemetry packet ready for encoding
     */
    [[nodiscard]] virtual telemetry::telemetry_packet build_packet(std::uint32_t timestamp_ms) noexcept = 0;
};

}  // namespace yeet::telemetry
