#pragma once

namespace yeet::application::sfc {

/**
 * @brief Operating modes for the SFC system.
 *
 * This enumeration defines the two primary operating modes for Load-Commutated Inverter
 * (LCI) systems. The mode selection is critical for proper system operation across
 * different speed ranges.
 */
enum class sfc_operating_mode_t {
    /**
     * @brief Normal operation with natural commutation.
     *
     * Used when machine speed is sufficient to generate adequate back-EMF for
     * natural thyristor commutation. This is the high-efficiency operating mode
     * for speeds typically above 10% of nominal.
     */
    natural_commutation,

    /**
     * @brief Startup operation with forced commutation.
     *
     * Used during startup and low-speed operation when back-EMF is insufficient
     * for natural commutation. The system uses forced commutation techniques
     * to control thyristor switching.
     */
    forced_commutation
};

}  // namespace yeet::application::sfc