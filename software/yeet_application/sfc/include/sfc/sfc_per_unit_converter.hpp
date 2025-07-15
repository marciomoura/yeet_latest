#pragma once

#include "sfc_per_unit_system.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/reference_frame.hpp"

namespace yeet::application::sfc {

/**
 * @brief Converts raw SI measurements to the per-unit system for the SFC.
 *
 * This class takes raw voltage, current, and angle measurements from the plant
 * model and converts them into per-unit quantities in the dq reference frame.
 */
class sfc_per_unit_converter {
public:
    /**
     * @brief Constructs the per-unit converter.
     * @param pu_system The per-unit system definition to use for conversions.
     */
    explicit sfc_per_unit_converter(const sfc_per_unit_system& pu_system);

    /**
     * @brief Updates all per-unit measurements from raw SI values.
     *
     * @param grid_voltages_si Raw grid-side three-phase voltages.
     * @param grid_currents_si Raw grid-side three-phase currents.
     * @param machine_voltages_si Raw machine-side three-phase voltages.
     * @param machine_currents_si Raw machine-side three-phase currents.
     * @param dc_link_current_si Raw DC-link current.
     */
    void update(const control::abc<double>& grid_voltages_si, const control::abc<double>& grid_currents_si,
                const control::abc<double>& machine_voltages_si, const control::abc<double>& machine_currents_si,
                double dc_link_current_si);

    // Per-unit measurement accessors
    const control::abc<double>& get_grid_voltages_pu() const { return _grid_voltages_pu; }
    const control::abc<double>& get_grid_currents_pu() const { return _grid_currents_pu; }
    const control::abc<double>& get_machine_voltages_pu() const { return _machine_voltages_pu; }
    const control::abc<double>& get_machine_currents_pu() const { return _machine_currents_pu; }
    double get_dc_link_current_pu() const { return _dc_link_current_pu; }

private:
    const sfc_per_unit_system& _pu_system;

    // Per-unit values
    control::abc<double> _grid_voltages_pu;
    control::abc<double> _grid_currents_pu;
    control::abc<double> _machine_voltages_pu;
    control::abc<double> _machine_currents_pu;
    double _dc_link_current_pu{};
};

}  // namespace yeet::application::sfc