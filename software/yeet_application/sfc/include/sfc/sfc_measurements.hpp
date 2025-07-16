#pragma once

#include "sfc_per_unit_system.hpp"
#include "yeet_control/reference_frame.hpp"

namespace yeet::application::sfc {

/**
 * @brief Handles measurement conversion for the grid side of the SFC.
 *
 * This class takes raw three-phase voltage and current measurements and
 * converts them into the per-unit system defined for the grid side.
 * It does not perform any filtering.
 */
class sfc_grid_side_measurements {
public:
    /**
     * @brief Updates the per-unit values from new raw measurements.
     * @param bases The per-unit base values for the grid side.
     * @param raw_voltages The raw three-phase voltage measurements [V].
     * @param raw_currents The raw three-phase current measurements [A].
     */
    void update(sfc_per_unit_system::grid_side_bases const& bases, const control::abc<double>& raw_voltages,
                const control::abc<double>& raw_currents);

    const control::abc<double>& get_voltage_pu() const { return _voltage_pu; }
    const control::abc<double>& get_current_pu() const { return _current_pu; }
    const control::alphabeta<double>& get_voltage_alphabeta_pu() const { return _voltage_alphabeta_pu; }
    double get_voltage_magnitude_pu() const { return _voltage_magnitude_pu; }

private:
    control::abc<double> _voltage_pu{};
    control::abc<double> _current_pu{};
    control::alphabeta<double> _voltage_alphabeta_pu{};
    double _voltage_magnitude_pu{};
};

/**
 * @brief Handles measurement conversion for the DC link of the SFC.
 *
 * This class takes raw DC link voltage and current measurements and
 * converts them into the per-unit system defined for the DC link.
 * It does not perform any filtering.
 */
class sfc_dc_link_measurements {
public:
    /**
     * @brief Updates the per-unit values from new raw measurements.
     * @param bases The per-unit base values for the DC link.
     * @param raw_voltage The raw DC link voltage measurement [V].
     * @param raw_current The raw DC link current measurement [A].
     */
    void update(sfc_per_unit_system::dc_link_bases const& bases, double raw_voltage, double raw_current);

    double get_voltage_pu() const { return _voltage_pu; }
    double get_current_pu() const { return _current_pu; }

private:
    double _voltage_pu{};
    double _current_pu{};
};

/**
 * @brief Handles measurement conversion for the machine side of the SFC.
 *
 * This class takes raw three-phase voltage and current measurements and
 * converts them into the per-unit system defined for the machine side.
 * It does not perform any filtering.
 */
class sfc_machine_side_measurements {
public:
    /**
     * @brief Updates the per-unit values from new raw measurements.
     * @param bases The per-unit base values for the machine side.
     * @param raw_voltages The raw three-phase voltage measurements [V].
     * @param raw_currents The raw three-phase current measurements [A].
     */
    void update(sfc_per_unit_system::machine_side_bases const& bases, const control::abc<double>& raw_voltages,
                const control::abc<double>& raw_currents);

    const control::abc<double>& get_voltage_pu() const { return _voltage_pu; }
    const control::abc<double>& get_current_pu() const { return _current_pu; }
    const control::alphabeta<double>& get_voltage_alphabeta_pu() const { return _voltage_alphabeta_pu; }
    double get_voltage_magnitude_pu() const { return _voltage_magnitude_pu; }

private:
    control::abc<double> _voltage_pu{};
    control::abc<double> _current_pu{};
    control::alphabeta<double> _voltage_alphabeta_pu{};
    double _voltage_magnitude_pu{};
};

}  // namespace yeet::application::sfc