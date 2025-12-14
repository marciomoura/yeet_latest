#pragma once

#include <cstdint>

namespace yeet::application::sfc::integration {

/**
 * @brief Severity routing for a protection condition.
 *
 * When a protection condition becomes active, it can either:
 * - raise an alarm only (no trip / shutdown request), or
 * - raise a trip (participates in combined is_tripped / shutdown path).
 */
enum class protection_severity : std::uint8_t {
    alarm_only = 0,
    trip = 1,
};

}  // namespace yeet::application::sfc::integration
