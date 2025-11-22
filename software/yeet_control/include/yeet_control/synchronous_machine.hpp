#pragma once

#include <array>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/types.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Represents the time derivatives of the synchronous machine state.
 */
struct synchronous_machine_derivatives {
    real_t psi_fD{};   ///< d(psi_fD)/dt [V]
    real_t psi_kD{};   ///< d(psi_kD)/dt [V]
    real_t psi_kQ{};   ///< d(psi_kQ)/dt [V]
    real_t omega_r{};  ///< d(omega_r)/dt [rad/s^2]
    real_t theta_r{};  ///< d(theta_r)/dt [rad/s] (This is omega_r)

    synchronous_machine_derivatives operator+(const synchronous_machine_derivatives& other) const
    {
        return {psi_fD + other.psi_fD, psi_kD + other.psi_kD, psi_kQ + other.psi_kQ, omega_r + other.omega_r,
            theta_r + other.theta_r};
    }

    synchronous_machine_derivatives operator*(real_t scalar) const
    {
        return {psi_fD * scalar, psi_kD * scalar, psi_kQ * scalar, omega_r * scalar, theta_r * scalar};
    }
};

/**
 * @brief Represents the dynamic state of the 5th-order synchronous machine model.
 *
 * Holds the variables that are integrated over time: rotor flux linkages and mechanical states.
 * This state structure is compatible with the RK4 integrator (range_kutta_derivative).
 */
struct synchronous_machine_state {
    real_t psi_fD{};          ///< Field winding flux linkage [Wb]
    real_t psi_kD{};          ///< d-axis damper winding flux linkage [Wb]
    real_t psi_kQ{};          ///< q-axis damper winding flux linkage [Wb]
    real_t omega_r{};         ///< Rotor mechanical speed [rad/s]
    angle_wrapped theta_r{};  ///< Rotor mechanical angle [rad]

    // Addition operator for RK4 integration (State + Derivative)
    synchronous_machine_state operator+(const synchronous_machine_derivatives& delta) const
    {
        synchronous_machine_state next = *this;
        next.psi_fD += delta.psi_fD;
        next.psi_kD += delta.psi_kD;
        next.psi_kQ += delta.psi_kQ;
        next.omega_r += delta.omega_r;
        next.theta_r += angle_t(delta.theta_r);
        return next;
    }
};

/**
 * @brief Bundles the d- and q-axis subtransient inductances.
 */
struct synchronous_machine_subtransient_inductances {
    inductance_t d_axis{0.0};
    inductance_t q_axis{0.0};
};

/**
 * @brief Provides both inductance and impedance information for subtransient behavior.
 */
struct synchronous_machine_subtransient_impedance {
    synchronous_machine_subtransient_inductances inductances{};
    impedance_t d_axis{0.0};
    impedance_t q_axis{0.0};
};

/**
 * @brief A 5th-order wound-rotor synchronous machine model in the dq reference frame.
 *
 * This class implements a current-fed synchronous machine model suitable for LCI (Load Commutated
 * Inverter) simulation. The model neglects stator flux dynamics (d(psi_s)/dt), which is valid for
 * electrical frequencies above ~5 Hz and typical for thyristor-fed drive applications.
 *
 * Key features:
 * - 5th-order model: 3 rotor flux states + 2 mechanical states
 * - Current-source interface (ideal for LCI topology)
 * - Provides back-EMF feedback for thyristor commutation analysis
 * - Compatible with range_kutta_derivative for RK4 integration
 *
 * Model assumptions:
 * - Stator flux dynamics neglected (valid for τ_s << τ_r)
 * - Linear magnetic circuit (no saturation)
 * - Balanced three-phase operation
 * - Damper windings are short-circuited (v_kD = v_kQ = 0)
 */
class synchronous_machine {
public:
    /**
     * @brief Constructs a synchronous machine model.
     *
     * @param R_s   Stator resistance [Ω]
     * @param R_fD  Field winding resistance [Ω]
     * @param R_kD  d-axis damper winding resistance [Ω]
     * @param R_kQ  q-axis damper winding resistance [Ω]
     * @param L_d   d-axis synchronous inductance [H]
     * @param L_q   q-axis synchronous inductance [H]
     * @param L_ad  d-axis mutual inductance [H]
     * @param L_aq  q-axis mutual inductance [H]
     * @param L_fD  Field winding self-inductance [H]
     * @param L_kD  d-axis damper self-inductance [H]
     * @param L_kQ  q-axis damper self-inductance [H]
     * @param J     Moment of inertia [kg·m²]
     * @param P     Number of poles
     */
    synchronous_machine(resistance_t R_s,
        resistance_t R_fD,
        resistance_t R_kD,
        resistance_t R_kQ,
        inductance_t L_d,
        inductance_t L_q,
        inductance_t L_ad,
        inductance_t L_aq,
        inductance_t L_fD,
        inductance_t L_kD,
        inductance_t L_kQ,
        moment_of_inertia_t J,
        int P);

    /**
     * @brief Calculates derivatives for a CURRENT-FED machine (e.g., from an LCI).
     *
     * This is the primary interface for LCI simulation. Given the stator currents from the
     * thyristor inverter, it computes the time derivatives of all state variables.
     *
     * @param current_state The current state of the machine
     * @param i_abc         The three-phase stator currents from the inverter [A]
     * @param v_field       The voltage applied to the field winding [V]
     * @param T_load        The mechanical load torque on the shaft [N·m]
     * @return synchronous_machine_derivatives containing the calculated derivatives
     */
    synchronous_machine_derivatives calculate_derivatives_current_fed(const synchronous_machine_state& current_state,
        const abc<current_t>& i_abc,
        voltage_t v_field,
        torque_t T_load) const;

    /**
     * @brief Calculates derivatives for a VOLTAGE-FED machine (e.g., from a VSI).
     *
     * Given the stator voltages from a voltage-source inverter, it computes the time
     * derivatives of all state variables. This method first solves for stator currents
     * algebraically from the voltage equations.
     *
     * @param current_state The current state of the machine
     * @param v_abc         The three-phase stator voltages from the inverter [V]
     * @param v_field       The voltage applied to the field winding [V]
     * @param T_load        The mechanical load torque on the shaft [N·m]
     * @return synchronous_machine_derivatives containing the calculated derivatives
     */
    synchronous_machine_derivatives calculate_derivatives_voltage_fed(const synchronous_machine_state& current_state,
        const abc<voltage_t>& v_abc,
        voltage_t v_field,
        torque_t T_load) const;

    /**
     * @brief Calculates terminal voltages (back-EMF).
     *
     * Essential for current-fed simulation feedback. Provides the machine terminal voltages
     * that the thyristor bridge must overcome for commutation.
     *
     * @param current_state The current state of the machine
     * @param i_abc         The three-phase stator currents
     * @return abc<voltage_t> The resulting terminal voltages [Va, Vb, Vc]
     */
    abc<voltage_t> get_terminal_voltages_abc(
        const synchronous_machine_state& current_state, const abc<current_t>& i_abc) const;

    /**
     * @brief Calculates terminal currents.
     *
     * Essential for voltage-fed simulation feedback. Provides the machine terminal currents
     * resulting from applied voltages.
     *
     * @param current_state The current state of the machine
     * @param v_abc         The three-phase stator voltages
     * @return abc<current_t> The resulting terminal currents [Ia, Ib, Ic]
     */
    abc<current_t> get_terminal_currents_abc(
        const synchronous_machine_state& current_state, const abc<voltage_t>& v_abc) const;

    /**
     * @brief Resets the machine model to initial conditions.
     *
     * Sets all state variables to zero. Typically called before starting a new simulation.
     */
    void reset();

    // Getters for machine parameters
    resistance_t get_stator_resistance() const { return _resistance_stator; }
    inductance_t get_d_axis_inductance() const { return _inductance_d; }
    inductance_t get_q_axis_inductance() const { return _inductance_q; }
    inductance_t get_mutual_inductance_d() const { return _inductance_mutual_d; }
    inductance_t get_field_inductance() const { return _inductance_field; }

    /**
     * @brief Returns the d and q axis subtransient inductances (frequency independent).
     */
    synchronous_machine_subtransient_inductances get_subtransient_inductances() const noexcept;

    /**
     * @brief Calculates the subtransient impedance at a given electrical frequency.
     */
    synchronous_machine_subtransient_impedance calculate_subtransient_impedance(
        frequency_t electrical_frequency) const noexcept;

private:
    // Parameters (SI units)
    resistance_t _resistance_stator;
    resistance_t _resistance_field;
    resistance_t _resistance_damper_d;
    resistance_t _resistance_damper_q;
    inductance_t _inductance_d;
    inductance_t _inductance_q;
    inductance_t _inductance_mutual_d;
    inductance_t _inductance_mutual_q;
    inductance_t _inductance_field;
    inductance_t _inductance_damper_d;
    inductance_t _inductance_damper_q;
    moment_of_inertia_t _inertia;
    real_t _machine_poles_div_2;  // P/2 for electrical angle calculations
};

}  // namespace yeet::control
