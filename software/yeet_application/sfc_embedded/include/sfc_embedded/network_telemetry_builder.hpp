/**
 * @file network_telemetry_builder.hpp
 * @brief Helper classes to populate telemetry packets from control system
 *
 * This module provides convenient builders to extract telemetry data from
 * the control system components and populate network telemetry packets.
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
 * Provides flexible control over which signal groups to include.
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
     * @brief Build complete telemetry packet with all available signal groups
     *
     * @param timestamp_ms Current system time in milliseconds
     * @return Complete telemetry packet ready for encoding
     */
    grid_side_telemetry_packet build_full_packet(std::uint32_t timestamp_ms) noexcept;

    /**
     * @brief Build telemetry packet with specific signal groups
     *
     * @param timestamp_ms Current system time in milliseconds
     * @param flags Bit flags indicating which groups to include
     * @return Telemetry packet with selected groups
     *
     * Example:
     * @code
     * auto packet = builder.build_packet(get_time_ms(),
     *     TELEM_FLAG_MEASUREMENTS | TELEM_FLAG_CONTROLLER);
     * @endcode
     */
    grid_side_telemetry_packet build_packet(std::uint32_t timestamp_ms, std::uint32_t flags) noexcept;

    /**
     * @brief Populate measurement group from control system
     */
    void populate_measurements(grid_measurements_group& group) const noexcept;

    /**
     * @brief Populate observer group from control system
     */
    void populate_observer(grid_observer_group& group) const noexcept;

    /**
     * @brief Populate modulator group from control system
     */
    void populate_modulator(grid_modulator_group& group) const noexcept;

    /**
     * @brief Populate controller group from control system
     */
    void populate_controller(grid_controller_group& group) const noexcept;

    /**
     * @brief Populate state machine group from control system
     */
    void populate_state_machine(state_machine_group& group) const noexcept;

private:
    const integration::sfc_grid_side_control& _control;
    std::uint32_t _sequence_number;
};

}  // namespace yeet::application::sfc::telemetry
