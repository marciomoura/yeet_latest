#pragma once

#include <cassert>

#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/derivative.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Salient-pole synchronous machine dq electrical model with one field winding (3rd order).
 *
 * Purpose:
 *  - Consume stator phase currents (abc), field voltage, and electrical speed (per-unit).
 *  - Internally integrate rotor electrical angle.
 *  - Evolve field flux state ψ_f.
 *  - Produce induced (transformer + speed) phase back-EMF E_abc (no Rs drop).
 *  - Provide electromagnetic torque per-unit.
 *
 * Assumptions:
 *  - Per-unit system (all inductances, resistances, voltages, currents in pu).
 *  - Balanced system (zero-sequence neglected).
 *  - Stator resistance handled externally if needed: v_phase = R_s * i_phase + E_phase.
 *  - Electrical speed per-unit is provided (mechanical speed * pole_pairs if mechanical base used).
 *
 * Core equations:
 *   ψ_f = L_md i_d + L_f i_f
 *   i_f = (ψ_f - L_md i_d)/L_f
 *   ψ_d = (L_d - L_md^2/L_f) i_d + (L_md/L_f) ψ_f = L_d' i_d + (L_md/L_f) ψ_f
 *   ψ_q = L_q i_q
 *   dψ_f/dt = v_f - R_f i_f
 *   e_d = dψ_d/dt - ω_e ψ_q
 *   e_q = dψ_q/dt + ω_e ψ_d
 *   T_e_pu = 1.5 * p * (ψ_d i_q - ψ_q i_d)
 *
 * Discretization: Forward Euler for ψ_f and simple finite difference for dψ/dt.
 */
class synchronous_machine_dq {
public:
    /**
     * @brief Constructs the synchronous machine dq model.
     * @param sampling_time The simulation sampling time [s].
     */
    explicit synchronous_machine_dq(real_t sampling_time);

    /**
     * @brief Configures the stator inductances.
     * @param d_axis_inductance_pu D-axis stator inductance [pu]. Must be positive.
     * @param q_axis_inductance_pu Q-axis stator inductance [pu]. Must be positive.
     */
    void configure_stator_inductances(inductance_pu_t d_axis_inductance_pu, inductance_pu_t q_axis_inductance_pu);

    /**
     * @brief Configures the field winding coupling parameters.
     * @param field_inductance_pu Field winding inductance [pu]. Must be positive.
     * @param mutual_inductance_pu Mutual inductance between field and d-axis [pu]. Must be positive.
     * @param field_resistance_pu Field winding resistance [pu]. Must be non-negative.
     */
    void configure_field_coupling(
        inductance_pu_t field_inductance_pu, inductance_pu_t mutual_inductance_pu, resistance_pu_t field_resistance_pu);

    /**
     * @brief Configures the number of pole pairs.
     * @param pole_pairs Number of pole pairs. Must be positive.
     */
    void configure_pole_pairs(unsigned pole_pairs);

    /**
     * @brief Configures the base electrical frequency used for per-unit speed conversion.
     * @param base_frequency_hz The base electrical frequency [Hz].
     */
    void configure_base_electrical_frequency_hz(real_t base_frequency_hz);

    void configure_stator_resistance(resistance_pu_t stator_resistance_pu);

    void reset(magnetic_flux_pu_t initial_field_flux_linkage_pu = magnetic_flux_pu_t{0.0f},
        current_pu_t initial_d_axis_current_pu = current_pu_t{0.0f},
        current_pu_t initial_q_axis_current_pu = current_pu_t{0.0f});

    torque_pu_t update(const abc<current_pu_t>& phase_currents_abc_pu,
        voltage_pu_t field_voltage_pu,
        speed_pu_t electrical_speed_pu,
        const angle_wrapped& electrical_angle) noexcept;

    abc<voltage_pu_t> get_back_emf_abc() const noexcept;
    dq<voltage_pu_t> get_induced_voltage_dq() const noexcept;
    torque_pu_t get_electromagnetic_torque_pu() const noexcept;
    current_pu_t get_field_current_pu() const noexcept;
    magnetic_flux_pu_t get_field_flux_linkage_pu() const noexcept;

private:
    // ---------------------------------------------------------------------
    // configuration (static) parameters
    // ---------------------------------------------------------------------
    real_t _sampling_time{0.0f};                                              // simulation/sample period [s]
    dq<inductance_pu_t> _l_dq{inductance_pu_t{1.0f}, inductance_pu_t{1.0f}};  // stator Ld, Lq
    inductance_pu_t _lf{1.0f};                                                // field inductance Lf
    inductance_pu_t _lmd{0.8f};                                               // mutual inductance Lmd
    resistance_pu_t _rf{0.0f};                                                // field resistance Rf
    resistance_pu_t _rs{0.0f};  // stator resistance Rs (not used internally, exposed for external network)
    unsigned _pole_pairs{1};    // p
    real_t _base_electrical_angular_frequency_rad_per_second{0.0f};  // base ω_e (rad/s)

    // ---------------------------------------------------------------------
    // derived / precomputed from configuration
    // ---------------------------------------------------------------------
    real_t _torque_scale{1.5f};       // 1.5 * pole_pairs
    inductance_pu_t _ld_prime{1.0f};  // Ld' = Ld - Lmd^2 / Lf
    real_t _coupling_gain{0.0f};      // Lmd / Lf

    // ---------------------------------------------------------------------
    // dynamic state variables
    // ---------------------------------------------------------------------
    magnetic_flux_pu_t _psi_f{0.0f};  // field flux linkage ψ_f
    derivative _psi_d_deriv;          // dψ_d/dt (through derivative block)
    derivative _psi_q_deriv;          // dψ_q/dt
    current_pu_t _i_f{0.0f};          // field current i_f (computed each update, kept for inspection)

    // ---------------------------------------------------------------------
    // calculated electrical outputs
    // ---------------------------------------------------------------------
    dq<voltage_pu_t> _e_dq{};    // induced dq voltages (back-emf)
    abc<voltage_pu_t> _e_abc{};  // induced phase back-emf
    torque_pu_t _te{0.0f};       // electromagnetic torque

    void recompute_reduced_d_axis_inductance();
    dq<magnetic_flux_pu_t> calculate_flux_linkages(const dq<current_pu_t>& currents_dq) const;
};

}  // namespace yeet::control