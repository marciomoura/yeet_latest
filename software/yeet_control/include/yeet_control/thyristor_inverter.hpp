#pragma once

#include "yeet_control/reference_frame.hpp"
#include "yeet_control/thyristor_bridge.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Dynamic model of a 6-pulse thyristor inverter (Machine-Side Converter).
 *
 * Models a DC voltage source driving a thyristor bridge connected to a three-phase AC system (e.g., Synchronous
 * Machine).
 *
 * System topology:
 *   V_dc_source → [R_dc, L_dc] → Thyristor Bridge → [R_ac, L_ac] → V_ac_emf
 *
 * Power Flow:
 *   - Typically DC to AC (Motoring): V_dc_source > 0, Alpha > 90 deg.
 *   - Bridge produces negative DC voltage (Counter-EMF).
 *   - V_dc_source overcomes this Counter-EMF to drive positive current.
 *
 * State variables:
 *   - i_dc: DC link current
 *   - Bridge commutation states (handled internally by thyristor_bridge)
 */
class thyristor_inverter {
public:
    /**
     * @brief Constructs the thyristor inverter model.
     * @param dc_link_resistance Resistance of the DC link inductor/source
     * @param dc_link_inductance Inductance of the DC link inductor
     * @param ac_resistance AC side series resistance (e.g., Stator resistance)
     * @param ac_inductance AC side series inductance (e.g., Stator leakage)
     * @param sampling_time Integration time step
     */
    thyristor_inverter(resistance_t dc_link_resistance,
        inductance_t dc_link_inductance,
        resistance_t ac_resistance,
        inductance_t ac_inductance,
        duration_t sampling_time);

    /**
     * @brief Updates the inverter model for one time step.
     *
     * @param v_dc_source The DC source voltage driving the link (must be > 0 for motoring)
     * @param pulses Gate signals for thyristors T1-T6
     * @param v_ac_emf Three-phase AC Back-EMF voltages (line-to-neutral)
     */
    void update(voltage_t v_dc_source, const thyristor_pulses& pulses, const abc<voltage_t>& v_ac_emf);

    /**
     * @brief Resets all state variables to zero.
     */
    void reset();

    /**
     * @brief Sets the DC link current directly (for initialization/testing).
     * @param i_dc The current to set.
     */
    void set_dc_current(current_t i_dc) { _i_dc = i_dc; }

    /**
     * @brief Gets the three-phase AC currents flowing OUT of the bridge.
     * @return AC currents in abc frame
     */
    abc<current_t> get_ac_currents() const noexcept;

    /**
     * @brief Gets the DC link current.
     * @return DC current
     */
    current_t get_dc_current() const noexcept;

    /**
     * @brief Gets the DC voltage at the bridge terminals.
     * Note: In inverter mode, this will typically be negative.
     * @return DC voltage (V_pos - V_neg)
     */
    voltage_t get_bridge_dc_voltage() const noexcept { return _v_bridge_dc; }

    const thyristor_bridge& bridge() const { return _bridge; }

    /**
     * @brief Gets the phase voltages at the bridge AC terminals.
     * Includes the voltage drop across AC impedance: V_term = V_emf + Z * I
     * @return Array [v_a, v_b, v_c]
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
    resistance_t _dc_link_resistance;
    inductance_t _dc_link_inductance;
    resistance_t _ac_resistance;
    inductance_t _ac_inductance;
    duration_t _sampling_time;

    // State variables
    current_t _i_dc{0.0};

    // Cached outputs
    voltage_t _v_bridge_dc{0.0};

    // Internal components
    thyristor_bridge _bridge;
};

}  // namespace yeet::control
