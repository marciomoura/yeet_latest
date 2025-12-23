#pragma once

#include "yeet_control/edge_detector.hpp"

namespace yeet::application::sfc {

/**
 * @brief Shared trip + inhibit confirmation handshake used by SFC protection runners.
 *
 * Responsibilities:
 * - Track `has_active_trip` which reflects whether any trip is currently active (clears automatically when fault
 * clears)
 * - Latch `is_inhibited` whenever a trip has occurred (system cannot restart until confirmed)
 * - Clear `is_inhibited` only on a rising edge of `confirm_trips` AND only when no active trips
 *
 * The key behavior change:
 * - `has_active_trip` is NOT latched - it reflects the real-time trip state
 * - `is_inhibited` IS latched - it requires user confirmation to clear
 *
 * This block is allocation-free and intended for hard real-time usage.
 */
class sfc_protections_trip_handshake {
public:
    /**
     * @brief Reset internal states to a safe initial condition.
     */
    void reset() noexcept;

    /**
     * @brief Execute one cycle of the handshake.
     *
     * @param any_trip_active True if any protection (severity-routed) currently has an active trip.
     * @param confirm_trips Operator/supervisor confirmation input (rising edge clears inhibit when no trips active).
     */
    void update(bool any_trip_active, bool confirm_trips) noexcept;

    /**
     * @brief Returns true if any trip is currently active.
     *
     * This is NOT latched - it reflects the real-time state of trip conditions.
     * Clears automatically when the fault condition clears.
     */
    bool get_has_active_trip() const noexcept;

    /**
     * @brief Latched inhibit output (confirmation required before restart).
     *
     * Set when a trip occurs, cleared only by rising edge of confirm_trips
     * when no trips are active.
     */
    bool get_is_inhibited() const noexcept;

private:
    bool _has_active_trip{false};
    bool _is_inhibited{false};

    yeet::control::edge_detector _confirm_trips_edge;
};

}  // namespace yeet::application::sfc
