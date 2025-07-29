#pragma once

#include "sfc_per_unit_system.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

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
    void update(sfc_per_unit_system::machine_side_bases const& bases, const abc<voltage_t>& raw_voltages,
                const abc<current_t>& raw_currents);

    const abc<voltage_pu_t>& get_voltage_pu() const { return _voltage_pu; }
    const abc<current_pu_t>& get_current_pu() const { return _current_pu; }
    const alphabeta<voltage_pu_t>& get_voltage_alphabeta_pu() const { return _voltage_alphabeta_pu; }
    double get_voltage_magnitude_pu() const { return _voltage_magnitude_pu; }

private:
    abc<voltage_pu_t> _voltage_pu{};
    abc<current_pu_t> _current_pu{};
    alphabeta<voltage_pu_t> _voltage_alphabeta_pu{};
    voltage_pu_t _voltage_magnitude_pu{};
};

}  // namespace yeet::application::sfc