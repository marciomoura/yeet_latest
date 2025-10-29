/**
 * @file machine_side_network_telemetry_builder.hpp
 * @brief Machine-side telemetry packet builder
 *
 * Provides the machine-side implementation of the telemetry builder interface.
 * Extracts signals from the machine-side control system and populates network
 * telemetry packets for transmission to the host.
 */

#pragma once

#include "sfc_embedded/grid_side_network_telemetry_builder.hpp"
#include "sfc_embedded/network_telemetry.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"

namespace yeet::application::sfc::telemetry {

/**
 * @brief Builder for machine-side telemetry packets
 *
 * Extracts data from machine-side control system and populates telemetry packets.
 * Provides a simple interface to build packets by pushing signals sequentially.
 *
 * The builder maintains internal state (sequence number) and provides helper
 * methods to extract signals from control system outputs and push them into
 * the packet buffer.
 *
 * Usage:
 * @code
 * machine_side_telemetry_builder builder(control);
 * auto packet = builder.build_packet(get_time_ms());
 * // Packet contains all signals from control system
 * @endcode
 */
class machine_side_telemetry_builder : public telemetry_builder {
public:
    /**
     * @brief Construct builder with reference to machine-side control
     * @param control Machine-side control system to extract data from
     */
    explicit machine_side_telemetry_builder(const integration::sfc_machine_side_control& control)
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
    [[nodiscard]] telemetry_packet build_packet(std::uint32_t timestamp_ms) noexcept override;

private:
    const integration::sfc_machine_side_control& _control;
    mutable std::uint32_t _sequence_number;
};

}  // namespace yeet::application::sfc::telemetry
