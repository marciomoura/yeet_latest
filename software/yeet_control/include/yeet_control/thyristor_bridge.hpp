#pragma once

#include <array>
#include <cstdint>

#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Gate pulse signals for a 6-pulse thyristor bridge.
 *
 * Provides access to individual thyristor gate pulses using named members (t1-t6)
 * or indexed access via operator[] (0-5).
 */
struct thyristor_pulses {
    bool t1{false};  ///< Gate pulse for thyristor T1 (upper, phase A)
    bool t2{false};  ///< Gate pulse for thyristor T2 (lower, phase C)
    bool t3{false};  ///< Gate pulse for thyristor T3 (upper, phase B)
    bool t4{false};  ///< Gate pulse for thyristor T4 (lower, phase A)
    bool t5{false};  ///< Gate pulse for thyristor T5 (upper, phase C)
    bool t6{false};  ///< Gate pulse for thyristor T6 (lower, phase B)

    /**
     * @brief Array-style access to pulses by index [0-5].
     * @param idx Index: 0=T1, 1=T2, 2=T3, 3=T4, 4=T5, 5=T6
     */
    constexpr bool operator[](size_t idx) const
    {
        switch (idx) {
            case 0:
                return t1;
            case 1:
                return t2;
            case 2:
                return t3;
            case 3:
                return t4;
            case 4:
                return t5;
            case 5:
                return t6;
            default:
                return false;
        }
    }

    /**
     * @brief Mutable array-style access to pulses by index [0-5].
     * @param idx Index: 0=T1, 1=T2, 2=T3, 3=T4, 4=T5, 5=T6
     */
    bool& operator[](size_t idx)
    {
        switch (idx) {
            case 0:
                return t1;
            case 1:
                return t2;
            case 2:
                return t3;
            case 3:
                return t4;
            case 4:
                return t5;
            case 5:
                return t6;
            default:
                return t1;  // Should never reach here if used correctly
        }
    }
};

/**
 * @brief Models a 6-pulse three-phase thyristor bridge.
 *
 * This class represents the switching behavior of a standard 6-thyristor bridge
 * used in Load Commutated Inverter (LCI) systems. It models:
 * - Turn-on conditions: gate pulse AND forward bias (anode > cathode voltage)
 * - Natural (load) commutation: turn-off when current reverses
 * - Latching behavior: once triggered, thyristors remain on until commutation
 *
 * Thyristor numbering follows standard convention:
 * - T1, T3, T5: Upper thyristors connected to phases A, C, B respectively
 * - T4, T6, T2: Lower thyristors connected to phases A, C, B respectively
 */
class thyristor_bridge {
public:
    thyristor_bridge() = default;

    /**
     * @brief Updates the conduction state of all 6 thyristors.
     *
     * This is the core logic function. It checks for turn-on conditions (gate pulse AND
     * forward bias) and must be called before any other calculation in a time step.
     *
     * @param v_ac_abc       Instantaneous three-phase AC voltages at the bridge terminals.
     * @param v_dc_positive  Voltage on the positive DC rail.
     * @param v_dc_negative  Voltage on the negative DC rail.
     * @param pulses         Gate signals for thyristors T1-T6.
     */
    void update_conduction_state(const abc<voltage_t>& v_ac_abc,
        voltage_t v_dc_positive,
        voltage_t v_dc_negative,
        const thyristor_pulses& pulses);

    /**
     * @brief Forcibly turns off thyristors whose current has become negative.
     *
     * In a simulation, after an integration step, the current through a thyristor
     * might become negative. This method identifies and turns off such thyristors
     * to simulate natural commutation.
     *
     * @param i_ac_abc       The calculated AC phase currents.
     * @param i_dc           The calculated DC link current.
     */
    void force_natural_turn_off(const abc<current_t>& i_ac_abc, current_t i_dc);

    /**
     * @brief Gets the current conduction state of the bridge's thyristors.
     * @return std::array<bool, 6> An array of booleans for T1-T6 (true if ON).
     */
    const std::array<bool, 6>& get_conduction_states() const { return _conducting_states; }
    std::array<bool, 6>& get_conduction_states_mutable() { return _conducting_states; }

    /**
     * @brief Resets all thyristors to the OFF state.
     */
    void reset() { _conducting_states.fill(false); }

    /**
     * @brief Calculates the AC phase currents based on conducting thyristors and DC current.
     *
     * This is a simplified model assuming fixed DC current. Upper thyristors source current
     * to their phases (positive), lower thyristors sink current (negative).
     *
     * @param i_dc_fixed The fixed DC link current.
     * @return abc<current_t> Calculated AC phase currents.
     */
    abc<current_t> calculate_ac_currents(current_t i_dc_fixed) const;

    /**
     * @brief Calculates the DC output voltage based on conducting thyristors and AC voltages.
     *
     * Determines which AC phase voltages are connected to the DC rails through conducting
     * thyristors and calculates the resulting DC voltage.
     *
     * @param v_ac_abc The three-phase AC voltages.
     * @param v_dc_positive The positive DC rail voltage.
     * @param v_dc_negative The negative DC rail voltage.
     * @return voltage_t The calculated DC output voltage.
     */
    voltage_t calculate_dc_voltage(
        const abc<voltage_t>& v_ac_abc, voltage_t v_dc_positive, voltage_t v_dc_negative) const;

    /**
     * @brief Gets the line-to-line voltages at the AC terminals of the bridge.
     *
     * Returns the three line-to-line voltages: [v_ab, v_bc, v_ca].
     * During commutation overlap, when two phases are shorted through conducting thyristors,
     * the corresponding line-to-line voltage drops to near zero (voltage notch).
     *
     * @return abc<voltage_t> Line-to-line voltages at AC terminals.
     */
    const abc<voltage_t>& get_ac_line_voltages() const { return _v_line_abc; }

    /**
     * @brief Gets the actual phase voltages at the AC terminals of the bridge.
     *
     * Returns the three phase voltages: [v_a, v_b, v_c] measured at the bridge terminals.
     * During commutation overlap, when two phases are connected through conducting thyristors
     * on the same DC rail, those phases are electrically shorted and their terminal voltages
     * collapse to the average value, creating a voltage notch in both phase voltages.
     *
     * @return abc<voltage_t> Phase voltages at AC terminals.
     */
    const abc<voltage_t>& get_ac_phase_voltages() const { return _v_phase_abc; }

    std::int32_t get_commutation_outgoing_thyristor_idx() const { return _commutation_outgoing_thyristor_idx; }

    std::int32_t get_commutation_incoming_thyristor_idx() const { return _commutation_incoming_thyristor_idx; }

    /**
     * @brief Updates the commutation logic (ODE) for the incoming thyristor.
     *
     * Solves the differential equation: L_comm * di_comm/dt = V_incoming - V_outgoing
     * using Forward Euler integration.
     *
     * @param i_dc The current DC link current.
     * @param commutation_inductance The loop inductance involved in commutation (2*L_grid).
     * @param dt The time step.
     * @param v_ac_abc The instantaneous AC phase voltages.
     */
    void update_commutation_state(
        current_t i_dc, inductance_t commutation_inductance, duration_t dt, const abc<voltage_t>& v_ac_abc);

    /**
     * @brief Updates the individual thyristor currents based on the DC current and commutation state.
     *
     * @param i_dc The total DC link current.
     */
    void update_thyristor_currents(current_t i_dc);

    /**
     * @brief Gets the individual currents for all 6 thyristors.
     * @return std::array<current_t, 6> Currents for T1-T6.
     */
    const std::array<current_t, 6>& get_thyristor_currents() const { return _thyristor_currents; }

    /**
     * @brief Calculates the instantaneous DC voltage at the bridge terminals.
     *
     * Accounts for commutation overlap (voltage averaging) if a commutation is in progress.
     *
     * @param v_ac_abc The AC phase voltages.
     * @return voltage_t The DC output voltage (V_pos - V_neg).
     */
    voltage_t get_instantaneous_dc_voltage(const abc<voltage_t>& v_ac_abc) const;

private:
    std::array<voltage_t, 6> _anode_voltages{};
    std::array<voltage_t, 6> _cathode_voltages{};
    std::array<bool, 6> _conducting_states{false};   ///< ON/OFF state for each thyristor T1-T6.
    std::array<current_t, 6> _thyristor_currents{};  ///< Individual thyristor currents.
    abc<voltage_t> _v_line_abc{};                    ///< Cached line-to-line voltages [v_ab, v_bc, v_ca].
    abc<voltage_t> _v_phase_abc{};                   ///< Cached phase voltages [v_a, v_b, v_c] at terminals.
    abc<voltage_t> _v_ac_source{};                   ///< Source voltages for notch calculation.

    current_t _commutation_current{0.0};
    std::int32_t _commutation_outgoing_thyristor_idx{-1};  ///< Thyristor turning-off during commutation overlap
    std::int32_t _commutation_incoming_thyristor_idx{-1};  ///< Thyristor turning-on during commutation overlap
};

}  // namespace yeet::control