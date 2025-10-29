/**
 * @file network_telemetry_builder.hpp
 * @brief Helper classes to populate telemetry packets from control system
 *
 * This module provides convenient builders to extract telemetry data from
 * the control system components and populate network telemetry packets.
 *
 * The builder encapsulates signal extraction and packet population logic,
 * providing a clean interface to push signals from datamodel outputs into
 * the telemetry packet buffer.
 */

#pragma once

#include "sfc_embedded/network_telemetry.hpp"
#include "sfc_integration/sfc_grid_side_control.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"

namespace yeet::application::sfc::telemetry {

/**
 * @brief Builder for grid-side telemetry packets
 *
 * Extracts data from grid-side control system and populates telemetry packets.
 * Provides a simple interface to build packets by pushing signals sequentially.
 *
 * The builder maintains internal state (sequence number) and provides helper
 * methods to extract signals from control system outputs and push them into
 * the packet buffer.
 *
 * Usage:
 * @code
 * grid_side_telemetry_builder builder(control);
 * auto packet = builder.build_packet(get_time_ms());
 * // Packet contains all signals from control system
 * @endcode
 */
class grid_side_telemetry_builder {
public:
    /**
     * @brief Construct builder with reference to grid-side control
     * @param control Grid-side control system to extract data from
     */
    explicit grid_side_telemetry_builder(const integration::sfc_grid_side_control& control)
        : _control(control), _sequence_number(0)
    {
    }

    /**
     * @brief Build telemetry packet with all available signals
     *
     * Extracts all signals from control system and populates packet.
     * This method handles all signal extraction and packet finalization.
     *
     * @param timestamp_ms Current system time in milliseconds
     * @return Complete telemetry packet ready for encoding
     */
    grid_side_telemetry_packet build_packet(std::uint32_t timestamp_ms) noexcept;

private:
    const integration::sfc_grid_side_control& _control;
    std::uint32_t _sequence_number;
};

}  // namespace yeet::application::sfc::telemetry
