#include "sfc/sfc_protections_trip_handshake.hpp"

namespace yeet::application::sfc {

void sfc_protections_trip_handshake::reset() noexcept
{
    _has_active_trip = false;
    _is_inhibited = false;

    // No explicit reset on edge detector; a reset is equivalent to forgetting history.
    _confirm_trips_edge = yeet::control::edge_detector{};
}

void sfc_protections_trip_handshake::update(bool any_trip_active, bool confirm_trips) noexcept
{
    // Track active trips - this is NOT latched, reflects real-time state
    _has_active_trip = any_trip_active;

    // Latch inhibit when a trip occurs (system cannot restart until confirmed)
    if (any_trip_active) {
        _is_inhibited = true;
    }

    const bool confirm_edge = _confirm_trips_edge.update(confirm_trips);

    // Inhibit clears only on rising edge of confirm AND when no trips are active
    if (confirm_edge && !any_trip_active) {
        _is_inhibited = false;
    }
}

bool sfc_protections_trip_handshake::get_has_active_trip() const noexcept { return _has_active_trip; }

bool sfc_protections_trip_handshake::get_is_inhibited() const noexcept { return _is_inhibited; }

}  // namespace yeet::application::sfc
