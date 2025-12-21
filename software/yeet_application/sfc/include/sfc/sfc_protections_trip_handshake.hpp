#pragma once

#include "yeet_control/edge_detector.hpp"

namespace yeet::application::sfc {

/**
 * @brief Shared trip + alarm confirmation handshake used by SFC protection runners.
 *
 * Responsibilities:
 * - Latch `is_tripped` when any protection reports a trip.
 * - Latch `is_inhibited` (alarm pending) whenever a trip has occurred.
 * - Clear `is_tripped` only on a rising edge of `confirm_alarms` AND only when the fault is gone.
 * - Clear `is_inhibited` only on a rising edge of `confirm_alarms` AND only when not tripped.
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
     * @param any_trip_request True if any protection (severity-routed) requests a trip.
     * @param any_fault_condition_active True if the underlying fault condition is still active.
     * @param confirm_alarms Operator/supervisor confirmation input (rising edge clears inhibit).
     */
    void update(bool any_trip_request, bool any_fault_condition_active, bool confirm_alarms) noexcept;

    /**
     * @brief Latched trip output.
     */
    bool get_is_tripped() const noexcept;

    /**
     * @brief Latched inhibit output (alarm pending / confirmation required).
     */
    bool get_is_inhibited() const noexcept;

    /**
     * @brief True for exactly one cycle when the latched trip is cleared in update().
     */
    bool get_trip_cleared_pulse() const noexcept;

private:
    bool _trip_latched{false};
    bool _alarm_pending{false};
    bool _trip_cleared_pulse{false};

    yeet::control::edge_detector _confirm_alarms_edge;
};

}  // namespace yeet::application::sfc
