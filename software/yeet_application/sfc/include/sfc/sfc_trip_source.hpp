#pragma once

#include <cstdint>

namespace yeet::application::sfc {

/**
 * @brief Identifies the source of a protection trip on the Grid-Side.
 */
enum class grid_trip_source : uint32_t {
    none = 0,
    overcurrent = 1,
    overvoltage = 2,
    undervoltage = 3,
    communication_loss = 4,
    door_open = 5,
    external_fault = 6,
    emergency_off = 7,
    machine_protection_feedback = 8
};

/**
 * @brief Identifies the source of a protection trip on the Machine-Side.
 */
enum class machine_trip_source : uint32_t {
    none = 0,
    over_flux = 1,
    under_flux = 2,
    stall = 3,
    communication_loss = 4,
    overspeed = 5,
    machine_overvoltage = 6,
    loss_of_excitation = 7,
    external_fault = 8
};

}  // namespace yeet::application::sfc
