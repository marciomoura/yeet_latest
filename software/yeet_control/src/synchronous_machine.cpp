#include "yeet_control/synchronous_machine.hpp"

#include <algorithm>
#include <cmath>

#include "yeet_control/constants.hpp"

namespace yeet::control {

synchronous_machine::synchronous_machine(resistance_t R_s,
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
    int P)
    : _resistance_stator(R_s),
      _resistance_field(R_fD),
      _resistance_damper_d(R_kD),
      _resistance_damper_q(R_kQ),
      _inductance_d(L_d),
      _inductance_q(L_q),
      _inductance_mutual_d(L_ad),
      _inductance_mutual_q(L_aq),
      _inductance_field(L_fD),
      _inductance_damper_d(L_kD),
      _inductance_damper_q(L_kQ),
      _inertia(J),
      _machine_poles_div_2(static_cast<real_t>(P) / 2.0)
{
}

synchronous_machine_derivatives synchronous_machine::calculate_derivatives_current_fed(
    const synchronous_machine_state& current_state,
    const abc<current_t>& i_abc,
    voltage_t v_field,
    torque_t T_load) const
{
    // 1. Calculate electrical angle from mechanical angle
    angle_wrapped theta_e = current_state.theta_r * _machine_poles_div_2;

    // 2. Transform stator currents to dq frame
    dq<current_t> i_dq = i_abc.to_dq(theta_e);
    current_t i_d = i_dq.d();
    current_t i_q = i_dq.q();

    // 3. Calculate rotor currents from flux linkages and stator currents
    // Rotor inductance matrix L_rr in dq frame:
    // L_rr = [(L_fD + L_ad)    L_ad           0        ]
    //        [L_ad             (L_kD + L_ad)  0        ]
    //        [0                0              (L_kQ + L_aq)]
    //
    // Stator-rotor mutual inductance matrix L_sr:
    // L_sr = [L_ad  0   ]
    //        [L_ad  0   ]
    //        [0     L_aq]
    //
    // Rotor currents: i_r = L_rr^-1 * (psi_r - L_sr * i_s)

    // For efficiency, we solve the system analytically for the 3x3 block-diagonal structure
    // d-axis subsystem (2x2):
    real_t inductance_rotor_d_determinant = (_inductance_field.value() + _inductance_mutual_d.value()) *
                                                (_inductance_damper_d.value() + _inductance_mutual_d.value()) -
                                            _inductance_mutual_d.value() * _inductance_mutual_d.value();

    real_t flux_field_residual = current_state.psi_fD - _inductance_mutual_d.value() * i_d.value();
    real_t flux_damper_d_residual = current_state.psi_kD - _inductance_mutual_d.value() * i_d.value();

    current_t current_field =
        current_t(((_inductance_damper_d.value() + _inductance_mutual_d.value()) * flux_field_residual -
                      _inductance_mutual_d.value() * flux_damper_d_residual) /
                  inductance_rotor_d_determinant);
    current_t current_damper_d =
        current_t((-_inductance_mutual_d.value() * flux_field_residual +
                      (_inductance_field.value() + _inductance_mutual_d.value()) * flux_damper_d_residual) /
                  inductance_rotor_d_determinant);

    // q-axis subsystem (1x1):
    real_t flux_damper_q_residual = current_state.psi_kQ - _inductance_mutual_q.value() * i_q.value();
    current_t current_damper_q =
        current_t(flux_damper_q_residual / (_inductance_damper_q.value() + _inductance_mutual_q.value()));

    // 4. Calculate rotor flux derivatives: d(psi_r)/dt = v_r - R_r * i_r
    synchronous_machine_derivatives derivatives{};
    derivatives.psi_fD = (v_field - _resistance_field * current_field).value();
    derivatives.psi_kD = (voltage_t(0.0) - _resistance_damper_d * current_damper_d).value();  // Damper short-circuited
    derivatives.psi_kQ = (voltage_t(0.0) - _resistance_damper_q * current_damper_q).value();  // Damper short-circuited

    // 5. Calculate stator fluxes for torque calculation
    // psi_s = L_s * i_s + L_sr^T * i_r
    flux_t flux_stator_d = _inductance_d * i_d + _inductance_mutual_d * (current_field + current_damper_d);
    flux_t flux_stator_q = _inductance_q * i_q + _inductance_mutual_q * current_damper_q;

    // 6. Calculate electromagnetic torque
    // T_e = (3/2) * (P/2) * (psi_d * i_q - psi_q * i_d)
    torque_t torque_electromagnetic = torque_t(
        1.5 * _machine_poles_div_2 * (flux_stator_d.value() * i_q.value() - flux_stator_q.value() * i_d.value()));

    // 7. Calculate mechanical derivatives
    // J * d(omega)/dt = T_e - T_load
    derivatives.omega_r = ((torque_electromagnetic - T_load) / _inertia).value();

    // d(theta)/dt = omega
    derivatives.theta_r = current_state.omega_r;

    return derivatives;
}

synchronous_machine_derivatives synchronous_machine::calculate_derivatives_voltage_fed(
    const synchronous_machine_state& current_state,
    const abc<voltage_t>& v_abc,
    voltage_t v_field,
    torque_t T_load) const
{
    // For voltage-fed operation, we must first solve for the stator currents algebraically
    // from the stator voltage equations, then proceed with the current-fed calculation

    abc<current_t> i_abc = get_terminal_currents_abc(current_state, v_abc);

    // Now that we have currents, use the current-fed derivative calculation
    return calculate_derivatives_current_fed(current_state, i_abc, v_field, T_load);
}

abc<voltage_t> synchronous_machine::get_terminal_voltages_abc(
    const synchronous_machine_state& current_state, const abc<current_t>& i_abc) const
{
    // 1. Calculate electrical angle and frequency
    angle_wrapped theta_e = current_state.theta_r * _machine_poles_div_2;
    angular_frequency_t omega_e = angular_frequency_t(current_state.omega_r * _machine_poles_div_2);

    // 2. Transform stator currents to dq frame
    dq<current_t> i_dq = i_abc.to_dq(theta_e);
    current_t i_d = i_dq.d();
    current_t i_q = i_dq.q();

    // 3. Calculate rotor currents (same as in derivatives calculation)
    real_t inductance_rotor_d_determinant = (_inductance_field.value() + _inductance_mutual_d.value()) *
                                                (_inductance_damper_d.value() + _inductance_mutual_d.value()) -
                                            _inductance_mutual_d.value() * _inductance_mutual_d.value();

    real_t flux_field_residual = current_state.psi_fD - _inductance_mutual_d.value() * i_d.value();
    real_t flux_damper_d_residual = current_state.psi_kD - _inductance_mutual_d.value() * i_d.value();

    current_t current_field =
        current_t(((_inductance_damper_d.value() + _inductance_mutual_d.value()) * flux_field_residual -
                      _inductance_mutual_d.value() * flux_damper_d_residual) /
                  inductance_rotor_d_determinant);
    current_t current_damper_d =
        current_t((-_inductance_mutual_d.value() * flux_field_residual +
                      (_inductance_field.value() + _inductance_mutual_d.value()) * flux_damper_d_residual) /
                  inductance_rotor_d_determinant);

    real_t flux_damper_q_residual = current_state.psi_kQ - _inductance_mutual_q.value() * i_q.value();
    current_t current_damper_q =
        current_t(flux_damper_q_residual / (_inductance_damper_q.value() + _inductance_mutual_q.value()));

    // 4. Calculate stator flux linkages
    flux_t flux_stator_d = _inductance_d * i_d + _inductance_mutual_d * (current_field + current_damper_d);
    flux_t flux_stator_q = _inductance_q * i_q + _inductance_mutual_q * current_damper_q;

    // DEBUG: Log field current calculation
    static int debug_counter2 = 0;
    if (debug_counter2 < 3) {
        std::cout << "[MACHINE DEBUG2] psi_fD=" << current_state.psi_fD << ", i_f_calc=" << current_field.value()
                  << ", i_kD=" << current_damper_d.value() << ", psi_stator_d=" << flux_stator_d.value() << std::endl;
        debug_counter2++;
    }

    // 5. Calculate stator voltages in dq frame (neglecting d(psi_s)/dt)
    // v_d = R_s*i_d - omega_e*psi_q
    // v_q = R_s*i_q + omega_e*psi_d
    voltage_t voltage_stator_d = _resistance_stator * i_d - voltage_t(omega_e.value() * flux_stator_q.value());
    voltage_t voltage_stator_q = _resistance_stator * i_q + voltage_t(omega_e.value() * flux_stator_d.value());

    // DEBUG: Log voltage calculation (remove after debugging)
    static int debug_counter = 0;
    if (debug_counter < 3) {
        std::cout << "[MACHINE DEBUG] i_d=" << i_d.value() << ", i_q=" << i_q.value()
                  << ", psi_d=" << flux_stator_d.value() << ", psi_q=" << flux_stator_q.value()
                  << ", omega_e=" << omega_e.value() << " -> v_d=" << voltage_stator_d.value()
                  << ", v_q=" << voltage_stator_q.value() << std::endl;
        debug_counter++;
    }

    // 6. Transform back to abc frame
    dq<voltage_t> voltage_stator_dq(voltage_stator_d, voltage_stator_q);
    return voltage_stator_dq.to_abc(theta_e);
}

abc<current_t> synchronous_machine::get_terminal_currents_abc(
    const synchronous_machine_state& current_state, const abc<voltage_t>& v_abc) const
{
    // 1. Calculate electrical angle and frequency
    angle_wrapped theta_e(current_state.theta_r * _machine_poles_div_2);
    angular_frequency_t omega_e = angular_frequency_t(current_state.omega_r * _machine_poles_div_2);

    // 2. Transform stator voltages to dq frame
    dq<voltage_t> v_dq = v_abc.to_dq(theta_e);
    voltage_t voltage_stator_d = v_dq.d();
    voltage_t voltage_stator_q = v_dq.q();

    // 3. Solve for stator currents from voltage equations
    // The stator voltage equations in dq frame (neglecting d(psi_s)/dt) are:
    // v_d = R_s*i_d - omega_e*psi_q
    // v_q = R_s*i_q + omega_e*psi_d
    //
    // where psi_d and psi_q depend on i_d, i_q and rotor fluxes.
    // This forms a coupled algebraic system. We solve iteratively or use approximation.
    //
    // For simplicity, we use the rotor flux contribution as the dominant term:
    // psi_d ≈ L_ad * i_field
    // psi_q ≈ 0 (assuming no q-axis rotor flux initially)

    // First, estimate rotor currents from current rotor flux states
    // Approximate field current from field flux: i_field ≈ psi_fD / L_field
    current_t current_field_approx = current_t(current_state.psi_fD / _inductance_field.value());
    current_t current_damper_d_approx = current_t(current_state.psi_kD / _inductance_damper_d.value());
    current_t current_damper_q_approx = current_t(current_state.psi_kQ / _inductance_damper_q.value());

    // Estimate flux contributions from rotor
    flux_t flux_from_rotor_d = _inductance_mutual_d * (current_field_approx + current_damper_d_approx);
    flux_t flux_from_rotor_q = _inductance_mutual_q * current_damper_q_approx;

    // Solve for stator currents algebraically
    // v_d = R_s*i_d - omega_e*(L_q*i_q + flux_from_rotor_q)
    // v_q = R_s*i_q + omega_e*(L_d*i_d + flux_from_rotor_d)
    //
    // Rearranging:
    // R_s*i_d + omega_e*L_q*i_q = v_d + omega_e*flux_from_rotor_q
    // -omega_e*L_d*i_d + R_s*i_q = v_q - omega_e*flux_from_rotor_d
    //
    // Matrix form: [R_s        omega_e*L_q  ] [i_d] = [v_d + omega_e*flux_from_rotor_q]
    //              [-omega_e*L_d    R_s     ] [i_q]   [v_q - omega_e*flux_from_rotor_d]

    real_t a11 = _resistance_stator.value();
    real_t a12 = omega_e.value() * _inductance_q.value();
    real_t a21 = -omega_e.value() * _inductance_d.value();
    real_t a22 = _resistance_stator.value();

    real_t b1 = voltage_stator_d.value() + omega_e.value() * flux_from_rotor_q.value();
    real_t b2 = voltage_stator_q.value() - omega_e.value() * flux_from_rotor_d.value();

    // Solve 2x2 system using Cramer's rule
    real_t determinant = a11 * a22 - a12 * a21;

    current_t i_d = current_t((b1 * a22 - b2 * a12) / determinant);
    current_t i_q = current_t((a11 * b2 - a21 * b1) / determinant);

    // 4. Transform back to abc frame
    dq<current_t> current_stator_dq(i_d, i_q);
    return current_stator_dq.to_abc(theta_e);
}

void synchronous_machine::reset()
{
    // State is external to this class - this method is here for future extensions
    // where internal cached states might need clearing
}

synchronous_machine_subtransient_inductances synchronous_machine::get_subtransient_inductances() const noexcept
{
    synchronous_machine_subtransient_inductances result{};

    const real_t Ld = _inductance_d.value();
    const real_t Lad = _inductance_mutual_d.value();
    const real_t Lfd_total = _inductance_field.value() + Lad;
    const real_t Lkd_total = _inductance_damper_d.value() + Lad;

    const real_t determinant = (Lfd_total * Lkd_total) - (Lad * Lad);
    if (determinant > 0.0 && Lad > 0.0) {
        const real_t inv00 = Lkd_total / determinant;
        const real_t inv11 = Lfd_total / determinant;
        const real_t inv01 = -Lad / determinant;
        const real_t coupling = Lad * Lad * (inv00 + inv11 + 2.0 * inv01);
        const real_t Ld_double_prime = std::max(real_t{0.0}, Ld - coupling);
        result.d_axis = inductance_t(Ld_double_prime);
    }
    else {
        result.d_axis = _inductance_d;
    }

    const real_t Lq = _inductance_q.value();
    const real_t Laq = _inductance_mutual_q.value();
    const real_t Lkq_total = _inductance_damper_q.value() + Laq;
    if (Lkq_total > 0.0 && Laq > 0.0) {
        const real_t Lq_double_prime = std::max(real_t{0.0}, Lq - (Laq * Laq / Lkq_total));
        result.q_axis = inductance_t(Lq_double_prime);
    }
    else {
        result.q_axis = _inductance_q;
    }

    return result;
}

synchronous_machine_subtransient_impedance synchronous_machine::calculate_subtransient_impedance(
    frequency_t electrical_frequency) const noexcept
{
    synchronous_machine_subtransient_impedance result{};
    result.inductances = get_subtransient_inductances();

    const real_t omega = static_cast<real_t>(2.0 * pi * electrical_frequency.value());
    result.d_axis = impedance_t(omega * result.inductances.d_axis.value());
    result.q_axis = impedance_t(omega * result.inductances.q_axis.value());

    return result;
}

}  // namespace yeet::control
