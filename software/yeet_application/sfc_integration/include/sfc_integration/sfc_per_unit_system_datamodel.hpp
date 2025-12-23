/**
 * @file sfc_per_unit_system_datamodel.hpp
 * @brief Per-unit system parameters as a datamodel
 *
 * Provides a datamodel interface for the SFC per-unit system base quantities,
 * allowing these fundamental system parameters to be accessed, monitored, and
 * optionally tuned through the network parameter interface.
 *
 * The per-unit system is used throughout the SFC control system to normalize
 * all signals and parameters. These base values are essential for understanding
 * the per-unit scales used in all other components.
 */

#pragma once

#include <memory>

#include "sfc/sfc_per_unit_system.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/parameter.hpp"
#include "yeet/parameter_handler.hpp"
#include "yeet/signal.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Per-unit system datamodel for network parameter access
 *
 * This datamodel exposes the base quantities used in the SFC per-unit system
 * to the network parameter interface. All values are read-only after initialization
 * but are registered so they can be queried via the parameter protocol.
 *
 * The per-unit system defines base quantities for:
 * - Grid-side AC quantities (voltage, current, frequency)
 * - DC link quantities (current base)
 * - Machine-side AC quantities (voltage, frequency)
 * - Machine mechanical parameters (pole pairs)
 */
class sfc_per_unit_system_datamodel {
public:
    /**
     * @brief Parameters structure containing all per-unit system base values
     */
    struct parameters {
        /// Grid line-to-line RMS base voltage [V]
        parameter<voltage_t> grid_voltage_base_rms{voltage_t{2090.0}};

        /// Grid base frequency [Hz]
        parameter<frequency_t> grid_frequency_base{frequency_t{50.0}};

        /// DC link base current [A]
        parameter<current_t> dc_link_current_base{current_t{1044.0}};

        /// Machine line-to-line RMS base voltage [V]
        parameter<voltage_t> machine_voltage_base_rms{voltage_t{1980.0}};

        /// Machine base frequency [Hz]
        parameter<frequency_t> machine_frequency_base{frequency_t{50.0}};

        /// Machine rated power [VA]
        parameter<power_t> machine_power_base{power_t{2950e3}};

        /// Number of machine poles
        parameter<int> machine_num_poles{2};
    };

    /**
     * @brief Get parameters structure (const)
     * @return Const reference to parameters
     */
    const parameters& get_parameters() const { return _parameters; }

    /**
     * @brief Register datamodel with model registry
     *
     * Registers all per-unit system parameters with the network parameter interface,
     * making them accessible via the parameter protocol (GET, GET_BY_INDEX, etc.).
     *
     * Component name: "per_unit_system"
     * Parameters registered:
     * - grid_voltage_base_rms
     * - grid_frequency_base
     * - grid_current_base
     * - dc_link_current_base
     * - machine_voltage_base_rms
     * - machine_frequency_base
     * - machine_power_base
     * - machine_num_poles
     *
     * @param registry Reference to the model registry
     */
    void register_datamodel(
        yeet::telemetry::parameter_handler& handler, std::shared_ptr<sfc_per_unit_system> pu_system);

private:
    sfc_per_unit_system get_sfc_per_unit_system();

    parameters _parameters;  ///< Per-unit system base quantity parameters
};

}  // namespace yeet::application::sfc::integration
