#include "sfc/sfc_protections_trip_handshake.hpp"

namespace yeet::application::sfc {

void sfc_protections_trip_handshake::configure_trip_clear_policy(sfc_trip_clear_policy policy) noexcept
{
    _trip_clear_policy = policy;
}

void sfc_protections_trip_handshake::reset() noexcept
{
    _trip_latched = false;
    _alarm_pending = false;
    _trip_cleared_pulse = false;

    // No explicit reset on edge detector; a reset is equivalent to forgetting history.
    _confirm_alarms_edge = yeet::control::edge_detector{};
}

void sfc_protections_trip_handshake::update(
    bool any_trip_request, bool any_fault_condition_active, bool confirm_alarms) noexcept
{
    _trip_cleared_pulse = false;

    // Latch trip and alarm_pending when a trip is requested
    if (any_trip_request) {
        _trip_latched = true;
        _alarm_pending = true;
    }

    // Trip latch clears based on configured policy
    bool should_clear = false;

    if (_trip_latched) {
        switch (_trip_clear_policy) {
            case sfc_trip_clear_policy::auto_clear:
                // Clear automatically when fault is gone (no acknowledgement required)
                if (!any_fault_condition_active) {
                    should_clear = true;
                }
                break;

            default:
                // Only auto_clear is supported
                break;
        }
    }

    if (should_clear) {
        _trip_latched = false;
        _trip_cleared_pulse = true;
    }

    const bool confirm_edge = _confirm_alarms_edge.update(confirm_alarms);

    // Alarm confirmation (user acknowledgment): rising edge on confirm_alarms clears inhibit
    // ONLY when no active fault conditions exist (trip must be cleared first)
    if (confirm_edge && !_trip_latched && !any_fault_condition_active) {
        _alarm_pending = false;
    }
}

bool sfc_protections_trip_handshake::get_is_tripped() const noexcept { return _trip_latched; }

bool sfc_protections_trip_handshake::get_is_inhibited() const noexcept { return _alarm_pending; }

bool sfc_protections_trip_handshake::get_trip_cleared_pulse() const noexcept { return _trip_cleared_pulse; }

}  // namespace yeet::application::sfc
