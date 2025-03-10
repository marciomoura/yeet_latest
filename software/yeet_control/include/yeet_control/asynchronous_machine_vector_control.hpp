#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

#include "first_order_low_pass_filter.hpp"
#include "pi_controller.hpp"
#include "reference_frame.hpp"

namespace yeet::control {

/**
 * @brief Vector control implementation for asynchronous (induction) machines
 *
 * Implements indirect field-oriented control (IFOC) with:
 * - d-axis current control for rotor flux
 * - q-axis current control for torque
 * - Slip calculation for proper field orientation
 */
template <typename T>
class asynchronous_machine_vector_controller {
public:
    using type = fundamental_type_t<T>;

    explicit asynchronous_machine_vector_controller(type sampling_period)
        : _ts(sampling_period),
          _speed_controller(),
          _d_current_controller(),
          _q_current_controller(),
          _flux_controller(),
          _speed_ref_filter(sampling_period),
          _flux_ref_filter(sampling_period) {}

    /**
     * @brief Configure current controllers based on machine parameters (per-unit)
     *
     * Uses Internal Model Control (IMC) tuning for first-order response
     *
     * @param stator_resistance    Stator resistance [p.u.]
     * @param rotor_resistance     Rotor resistance [p.u.]
     * @param stator_inductance    Stator inductance [p.u.]
     * @param rotor_inductance     Rotor inductance [p.u.]
     * @param mutual_inductance    Mutual inductance [p.u.]
     * @param response_time        Desired closed-loop time constant [s]
     * @param voltage_limit        Maximum voltage limit [p.u.]
     */
    void configure_current_controllers(type stator_resistance, type rotor_resistance, type stator_inductance,
                                       type rotor_inductance, type mutual_inductance, type response_time,
                                       type voltage_limit) {
        // Store machine parameters for control calculations
        _Rs = stator_resistance;
        _Rr = rotor_resistance;
        _Ls = stator_inductance;
        _Lr = rotor_inductance;
        _Lm = mutual_inductance;

        // Calculate leakage inductances
        _sigma = 1.0 - (_Lm * _Lm) / (_Ls * _Lr);

        // Calculate rotor time constant
        _Tr = _Lr / _Rr;

        // IMC tuning for d-axis controller (flux)
        const type kp_d = _sigma * _Ls / response_time;
        const type ti_d = _sigma * _Ls / _Rs;

        // IMC tuning for q-axis controller (torque)
        const type kp_q = _sigma * _Ls / response_time;
        const type ti_q = _sigma * _Ls / _Rs;

        _d_current_controller.configure_with_ti(kp_d, ti_d, _ts);
        _q_current_controller.configure_with_ti(kp_q, ti_q, _ts);

        // Set voltage limits
        _d_current_controller.set_output_limits(-voltage_limit, voltage_limit);
        _q_current_controller.set_output_limits(-voltage_limit, voltage_limit);
        _voltage_limit = voltage_limit;
    }

    /**
     * @brief Configure flux controller for magnetizing the machine
     *
     * @param rotor_time_constant  Rotor time constant (Lr/Rr) [s]
     * @param response_time        Desired flux response time [s]
     * @param nominal_flux         Nominal magnetizing flux [p.u.]
     */
    void configure_flux_controller(type rotor_time_constant, type response_time, type nominal_flux) {
        _Tr = rotor_time_constant;  // Store again for redundancy
        _nominal_flux = nominal_flux;

        // Flux controller tuning - first order response
        const type kp = 1.0 / response_time;
        const type ti = 4.0 * response_time;  // Conservative integral time

        _flux_controller.configure_with_ti(kp, ti, _ts);
        _flux_controller.set_output_limits(0.0, 10.0);  // Typical range for d-axis current

        // Set default flux reference to nominal
        _flux_reference = nominal_flux;
        _flux_ref_filter.configure(2.0);  // 2 Hz cutoff (slow flux changes)
    }

    /**
     * @brief Configure speed controller based on mechanical parameters.
     * Uses simplified inertia-based tuning.
     *
     * @param inertia           System inertia [p.u.]
     * @param desired_bandwidth Desired closed-loop bandwidth [rad/s]
     * @param damping_ratio     Desired damping ratio
     * @param torque_limit      Maximum torque limit [p.u.]
     */
    void configure_speed_controller(type inertia, type desired_bandwidth, type damping_ratio, type torque_limit) {
        // Store inertia for future reference
        _inertia = inertia;

        // For specified bandwidth and damping:
        // Kp = J * (2*zeta*wn)
        // Ki = J * wn^2
        const type kp = inertia * (type(2.0) * damping_ratio * desired_bandwidth);
        const type ti = type(2.0) * damping_ratio / desired_bandwidth;

        _speed_controller.configure_with_ti(kp, ti, _ts);

        // For induction machine, torque limit maps to q-current limit
        // with some margin for flux variations
        _speed_controller.set_output_limits(-torque_limit * 1.2, torque_limit * 1.2);

        _torque_limit = torque_limit;
    }

    /**
     * @brief Configure speed reference filter.
     * Typically set to 1/5 to 1/10 of the speed control bandwidth.
     *
     * @param cutoff_freq Filter cutoff frequency [Hz]
     */
    void configure_speed_filter(type cutoff_freq) { _speed_ref_filter.configure(cutoff_freq); }

    /**
     * @brief Set flux reference for field weakening or efficiency optimization
     *
     * @param flux_ref Flux reference [p.u.] (1.0 = nominal flux)
     */
    void set_flux_reference(T flux_ref) { _flux_reference = flux_ref; }

    /**
     * @brief Force specific current references, bypassing speed/flux controllers
     * Useful for testing current control loops directly
     *
     * @param id_ref d-axis current reference [p.u.]
     * @param iq_ref q-axis current reference [p.u.]
     */
    void force_current_reference(T id_ref, T iq_ref) {
        _force_current_ref = true;
        _forced_current_d_ref = id_ref;
        _forced_current_q_ref = iq_ref;
    }

    /**
     * @brief Disable forced current reference mode
     */
    void disable_forced_current() { _force_current_ref = false; }

    /**
     * @brief Main update function for the controller
     *
     * @param speed_ref Speed reference [p.u.]
     * @param speed_measured Measured speed [p.u.]
     * @param current_measured Measured stator currents in dq-frame [p.u.]
     * @param rotor_flux_magnitude Measured/estimated rotor flux magnitude [p.u.]
     * @return dq<T> Voltage command in dq-frame [p.u.]
     */
    dq<T> update(T speed_ref, T speed_measured, const dq<T>& current_measured, T rotor_flux_magnitude) {
        // Filter speed reference
        _speed_ref_filtered = _speed_ref_filter.update(speed_ref);

        // Filter flux reference
        T flux_ref_filtered = _flux_ref_filter.update(_flux_reference);

        // Current references calculation
        dq<T> current_ref;

        // 1. Calculate d-axis current reference (flux control)
        if (_force_current_ref) {
            current_ref.d() = _forced_current_d_ref;
        } else {
            // Flux error to d-axis current reference
            T flux_error = flux_ref_filtered - rotor_flux_magnitude;
            current_ref.d() = _flux_controller.update(flux_error);
        }

        // 2. Calculate q-axis current reference (torque control)
        if (_force_current_ref) {
            current_ref.q() = _forced_current_q_ref;
        } else {
            // Speed error to torque command to q-axis current
            T speed_error = _speed_ref_filtered - speed_measured;
            T torque_ref = _speed_controller.update(speed_error);

            // For induction machine: Torque = K * rotor_flux * iq
            // Normalize by flux magnitude to maintain consistent torque response
            T effective_flux = std::max(rotor_flux_magnitude, T(0.1));  // Avoid division by small values
            current_ref.q() = torque_ref / effective_flux;
        }

        // Calculate slip frequency for field orientation
        // slip = Rr*isq/(Lr*isd) for IFOC
        T slip_frequency;
        if (std::abs(current_measured.d()) < T(0.05)) {
            slip_frequency = T(0.0);  // Avoid division by small values
        } else {
            slip_frequency = (_Rr * current_measured.q()) / (_Lr * current_measured.d());
        }

        // Store slip frequency for external access
        _slip_frequency = slip_frequency;

        // Calculate electrical frequency = rotor speed + slip
        T electrical_freq = speed_measured + slip_frequency;

        // Current control with cross-coupling compensation
        dq<T> voltage_command = current_control(current_ref, current_measured, electrical_freq);

        return voltage_command;
    }

    /**
     * @brief Get the calculated slip frequency
     * @return T Slip frequency [p.u.]
     */
    T get_slip_frequency() const { return _slip_frequency; }

    /**
     * @brief Get the current flux reference
     * @return T Flux reference [p.u.]
     */
    T get_flux_reference() const { return _flux_reference; }

private:
    /**
     * @brief Current control loop with cross-coupling compensation
     */
    dq<T> current_control(const dq<T>& current_ref, const dq<T>& current_measured, T electrical_freq) {
        // Current errors
        T id_error = current_ref.d() - current_measured.d();
        T iq_error = current_ref.q() - current_measured.q();

        // PI controllers
        T vd_pi = _d_current_controller.update(id_error);
        T vq_pi = _q_current_controller.update(iq_error);

        // Cross-coupling terms for induction machine
        // v_d = v_d_pi - ω_e*σ*L_s*i_q
        // v_q = v_q_pi + ω_e*σ*L_s*i_d + ω_e*L_m²/L_r*ψ_r

        T vd_cc = -electrical_freq * _sigma * _Ls * current_measured.q();
        T vq_cc = electrical_freq * _sigma * _Ls * current_measured.d();

        // Add back EMF term (only approximation since we don't directly have rotor flux angle)
        // This is a simplified term as the accurate compensation requires rotor flux position
        T rotor_flux_effect = electrical_freq * (_Lm / _Lr) * (_Lm * current_measured.d());
        vq_cc += rotor_flux_effect;

        // Total voltage commands
        dq<T> voltage_ref;
        voltage_ref.d() = vd_pi + vd_cc;
        voltage_ref.q() = vq_pi + vq_cc;

        // Voltage limitation (circle limit)
        T v_mag_squared = voltage_ref.d() * voltage_ref.d() + voltage_ref.q() * voltage_ref.q();
        if (v_mag_squared > _voltage_limit * _voltage_limit) {
            T scale = _voltage_limit / std::sqrt(v_mag_squared);
            voltage_ref.d() *= scale;
            voltage_ref.q() *= scale;
        }

        return voltage_ref;
    }

    // Sampling period
    type _ts;

    // Controllers
    pi_controller<T> _speed_controller;
    pi_controller<T> _d_current_controller;
    pi_controller<T> _q_current_controller;
    pi_controller<T> _flux_controller;

    // Filters
    first_order_low_pass_filter<T> _speed_ref_filter;
    first_order_low_pass_filter<T> _flux_ref_filter;

    // Filtered references
    T _speed_ref_filtered{T(0)};

    // Machine parameters (per-unit)
    T _Rs{T(0)};       // Stator resistance
    T _Rr{T(0)};       // Rotor resistance
    T _Ls{T(0)};       // Stator inductance
    T _Lr{T(0)};       // Rotor inductance
    T _Lm{T(0)};       // Mutual inductance
    T _sigma{T(0)};    // Leakage factor (1-Lm²/(Ls*Lr))
    T _Tr{T(0)};       // Rotor time constant (Lr/Rr)
    T _inertia{T(0)};  // Machine inertia

    // Control limits
    T _voltage_limit{T(1.0)};
    T _torque_limit{T(1.5)};

    // Flux control
    T _nominal_flux{T(1.0)};
    T _flux_reference{T(1.0)};

    // Calculated values for monitoring
    T _slip_frequency{T(0.0)};

    // Forced current reference control
    bool _force_current_ref{false};
    T _forced_current_d_ref{T(0)};
    T _forced_current_q_ref{T(0)};
};

}  // namespace yeet::control