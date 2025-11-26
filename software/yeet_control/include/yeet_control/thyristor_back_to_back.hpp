#pragma once

#include "yeet_control/thyristor_bridge.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Models a complete LCI back-to-back converter system.
 *
 * This class connects two 6-pulse thyristor bridges via a DC link.
 * - Grid-Side Converter (Rectifier): Controls DC link current.
 * - Machine-Side Converter (Inverter): Commutates current to the machine.
 * - DC Link: Models the inductor and resistance connecting the two bridges.
 *
 * The model solves the differential equation:
 * L_total * di_dc/dt = V_rect_dc + V_inv_dc - R_total * i_dc
 *
 * Where V_inv_dc is typically negative during motoring (opposing current flow).
 */
class thyristor_back_to_back {
public:
    /**
     * @brief Construct a new thyristor back to back object
     *
     * @param grid_res Grid-side source resistance (per phase)
     * @param grid_ind Grid-side source inductance (per phase)
     * @param dc_res DC link resistance
     * @param dc_ind DC link inductance
     * @param mach_res Machine-side source resistance (per phase)
     * @param mach_ind Machine-side source inductance (per phase)
     * @param ts Sampling time
     */
    thyristor_back_to_back(resistance_t grid_res,
        inductance_t grid_ind,
        resistance_t dc_res,
        inductance_t dc_ind,
        resistance_t mach_res,
        inductance_t mach_ind,
        duration_t ts);

    /**
     * @brief Update the state of the back-to-back converter.
     *
     * @param v_grid Grid phase voltages (a, b, c)
     * @param v_machine Machine phase voltages (a, b, c) - Back-EMF
     * @param p_rect Firing pulses for the rectifier bridge
     * @param p_inv Firing pulses for the inverter bridge
     */
    void update(const abc<voltage_t>& v_grid,
        const abc<voltage_t>& v_machine,
        const thyristor_pulses& p_rect,
        const thyristor_pulses& p_inv);

    /**
     * @brief Reset internal states (currents, voltages) to zero.
     */
    void reset();

    // --- Getters ---

    /**
     * @brief Get the DC Link Current
     */
    current_t get_dc_current() const { return _i_dc; }

    /**
     * @brief Get the Rectifier DC Voltage (Average or Instantaneous depending on implementation)
     */
    voltage_t get_rectifier_voltage() const { return _v_rect_dc; }

    /**
     * @brief Get the Inverter DC Voltage
     */
    voltage_t get_inverter_voltage() const { return _v_inv_dc; }

    /**
     * @brief Get the Grid Side AC Currents
     */
    abc<current_t> get_grid_currents() const;

    /**
     * @brief Get the Machine Side AC Currents
     */
    abc<current_t> get_machine_currents() const;

    /**
     * @brief Get the Rectifier Bridge object (for inspection)
     */
    const thyristor_bridge& get_rectifier_bridge() const { return _rect_bridge; }

    /**
     * @brief Get the Inverter Bridge object (for inspection)
     */
    const thyristor_bridge& get_inverter_bridge() const { return _inv_bridge; }

    /**
     * @brief Get the Rectifier Terminal Voltages (after source impedance drop)
     */
    abc<voltage_t> get_rectifier_terminal_voltages() const { return _v_term_rect; }

    /**
     * @brief Get the Inverter Terminal Voltages (after source impedance drop)
     */
    abc<voltage_t> get_inverter_terminal_voltages() const { return _v_term_inv; }

    /**
     * @brief Get the Rectifier DC Voltage
     */
    voltage_t get_rectifier_dc_voltage() const { return _v_rect_dc; }

    /**
     * @brief Get the Inverter DC Voltage
     */
    voltage_t get_inverter_dc_voltage() const { return _v_inv_dc; }

private:
    // System Parameters
    resistance_t _grid_resistance;
    inductance_t _grid_inductance;
    resistance_t _dc_resistance;
    inductance_t _dc_inductance;
    resistance_t _machine_resistance;
    inductance_t _machine_inductance;
    duration_t _sampling_time;

    // Sub-components
    thyristor_bridge _rect_bridge;
    thyristor_bridge _inv_bridge;

    // State Variables
    current_t _i_dc{0.0};
    voltage_t _v_rect_dc{0.0};
    voltage_t _v_inv_dc{0.0};
    abc<voltage_t> _v_term_rect{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
    abc<voltage_t> _v_term_inv{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
};

}  // namespace yeet::control
