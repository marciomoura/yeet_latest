#pragma once

namespace yeet::application::sfc::integration {

/**
 * @brief Digital IO polarity configuration
 *
 * Defines the electrical active state of a digital input or output signal.
 * - ACTIVE_HIGH (0): Signal is active when voltage is high (default, most common)
 * - ACTIVE_LOW (1): Signal is active when voltage is low (inverted logic)
 *
 * This allows flexible hardware interfacing without code changes.
 * The polarity inversion is applied in the datamodel layer when reading/writing IOs.
 */
enum class io_polarity_t {
    ACTIVE_HIGH = 0,  ///< Signal is true when electrical level is high
    ACTIVE_LOW = 1    ///< Signal is true when electrical level is low (inverted)
};

}  // namespace yeet::application::sfc::integration
