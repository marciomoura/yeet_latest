#pragma once

#include "yeet_control/reference_frame.hpp"
#include "yeet_control/thyristor_bridge.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Complete dynamic model of a 6-pulse thyristor rectifier system.
 *
 * Models a three-phase AC voltage source → grid RL impedance → thyristor bridge → DC RL load with back-EMF.
 *
 * System topology:
 *   V_source_abc → [R_grid, L_grid] → Thyristor Bridge → [R_load, L_load, V_dc_back_emf]
 *
 * State variables (integrated over time):
 *   - i_dc: DC link current through the load
 *   - i_commutating: Current through the incoming thyristor during commutation overlap
 *
 * Commutation model:
 *   - Normally: One upper + one lower thyristor conduct, each carries i_dc
 *   - During commutation: Incoming thyristor current rises from 0 → i_dc
 *                         Outgoing thyristor current = i_dc - i_commutating
 *   - After commutation: i_commutating resets to 0, new thyristor carries full i_dc
 *
 * AC phase currents = difference between upper and lower thyristor currents in that phase.
 *
 * Architecture:
 *   - Call update() each time step with AC voltages, gate pulses, and DC back-EMF
 *   - Internal Runge-Kutta 4th order integration for accurate dynamics
 *   - Natural commutation handled automatically based on current transfer completion
 */
class thyristor_rectifier {
public:
    /**
     * @brief Constructs the thyristor rectifier model.
     * @param grid_resistance Grid equivalent series resistance per phase
     * @param grid_inductance Grid equivalent series inductance per phase
     * @param load_resistance DC load resistance
     * @param load_inductance DC load inductance
     * @param sampling_time Integration time step
     */
    thyristor_rectifier(resistance_t grid_resistance,
        inductance_t grid_inductance,
        resistance_t load_resistance,
        inductance_t load_inductance,
        duration_t sampling_time);

    /**
     * @brief Updates the rectifier model for one time step.
     *
     * @param v_ac_source Three-phase source voltages (line-to-neutral)
     * @param pulses Gate signals for thyristors T1-T6
     * @param v_dc_back_emf DC back-voltage from load (motor EMF, battery, etc.)
     */
    void update(const abc<voltage_t>& v_ac_source, const thyristor_pulses& pulses, voltage_t v_dc_back_emf);

    /**
     * @brief Resets all state variables to zero.
     */
    void reset();

    /**
     * @brief Gets the three-phase grid currents.
     * @return Grid currents in abc frame
     */
    abc<current_t> get_grid_currents() const noexcept;

    /**
     * @brief Gets the DC load current.
     * @return DC current
     */
    current_t get_dc_current() const noexcept;

    /**
     * @brief Gets the DC voltage output from the thyristor bridge.
     * @return DC voltage
     */
    voltage_t get_dc_voltage() const noexcept { return _v_dc_output; }

    /**
     * @brief Gets the line-to-line voltages at the bridge AC terminals.
     * @return Array [v_ab, v_bc, v_ca] showing voltage notch during commutation
     */
    abc<voltage_t> get_ac_line_voltages() const noexcept { return _bridge.get_ac_line_voltages(); }

    const thyristor_bridge& bridge() const { return _bridge; }

    /**
     * @brief Gets the phase voltages at the bridge AC terminals.
     * @return Array [v_a, v_b, v_c] showing voltage notch during commutation
     */
    abc<voltage_t> get_ac_phase_voltages() const noexcept { return _bridge.get_ac_phase_voltages(); }

    /**
     * @brief Gets the thyristor bridge conduction states.
     * @return Array of 6 booleans (true = conducting)
     */
    const std::array<bool, 6>& get_conduction_states() const noexcept { return _bridge.get_conduction_states(); }

    /**
     * @brief Gets individual thyristor currents.
     * @return Array of 6 currents for T1-T6
     */
    const std::array<current_t, 6>& get_thyristor_currents() const noexcept { return _bridge.get_thyristor_currents(); }

private:
    // Circuit parameters
    resistance_t _grid_resistance;
    inductance_t _grid_inductance;
    resistance_t _load_resistance;
    inductance_t _load_inductance;
    duration_t _sampling_time;

    // State variables
    current_t _i_dc{0.0};  // DC link current through load

    // Cached outputs
    voltage_t _v_dc_output{0.0};  // DC voltage from bridge

    // Internal components
    thyristor_bridge _bridge;
};

}  // namespace yeet::control
