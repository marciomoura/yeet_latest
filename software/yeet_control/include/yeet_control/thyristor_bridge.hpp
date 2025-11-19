#pragma once
#include <array>

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
     * @param v_ac_abc       Instantaneous three-phase AC voltages at the bridge terminals [a, b, c].
     * @param v_dc_rails     The voltages on the positive and negative DC rails [v_p, v_n].
     * @param pulses         Gate signals for thyristors T1-T6.
     */
    void update_conduction_state(const std::array<double, 3>& v_ac_abc,
        const std::array<double, 2>& v_dc_rails,
        const std::array<bool, 6>& pulses);

    /**
     * @brief Forcibly turns off thyristors whose current has become negative.
     *
     * In a simulation, after an integration step, the current through a thyristor
     * might become negative. This method identifies and turns off such thyristors
     * to simulate natural commutation.
     *
     * @param i_ac_abc       The calculated AC phase currents [i_a, i_b, i_c].
     * @param i_dc           The calculated DC link current.
     */
    void force_natural_turn_off(const std::array<double, 3>& i_ac_abc, double i_dc);

    /**
     * @brief Gets the current conduction state of the bridge's thyristors.
     * @return std::array<bool, 6> An array of booleans for T1-T6 (true if ON).
     */
    const std::array<bool, 6>& get_conduction_states() const { return _conducting_states_; }
    std::array<bool, 6>& get_conduction_states_mutable() { return _conducting_states_; }

    /**
     * @brief Resets all thyristors to the OFF state.
     */
    void reset() { _conducting_states_.fill(false); }

    /**
     * @brief Calculates the AC phase currents based on conducting thyristors and DC current.
     *
     * This is a simplified model assuming fixed DC current. Upper thyristors source current
     * to their phases (positive), lower thyristors sink current (negative).
     *
     * @param i_dc_fixed The fixed DC link current.
     * @return std::array<double, 3> Calculated AC phase currents [i_a, i_b, i_c].
     */
    std::array<double, 3> calculate_ac_currents(double i_dc_fixed) const;

    /**
     * @brief Calculates the DC output voltage based on conducting thyristors and AC voltages.
     *
     * Determines which AC phase voltages are connected to the DC rails through conducting
     * thyristors and calculates the resulting DC voltage.
     *
     * @param v_ac_abc The three-phase AC voltages [v_a, v_b, v_c].
     * @param v_dc_rails The DC rail voltages [v_positive, v_negative].
     * @return double The calculated DC output voltage.
     */
    double calculate_dc_voltage(const std::array<double, 3>& v_ac_abc, const std::array<double, 2>& v_dc_rails) const;

private:
    std::array<bool, 6> _conducting_states_{false};  ///< ON/OFF state for each thyristor T1-T6.
};