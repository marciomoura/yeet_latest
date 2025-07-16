#pragma once

#include <algorithm>
#include <cmath>

#include "pi_controller.hpp"
#include "reference_frame.hpp"

namespace yeet::control {

/**
 * @brief Current controller for electric machine control
 *
 * Handles d-axis and q-axis current control with cross-coupling compensation
 * for different machine types. Operates in per-unit.
 */
class machine_current_controller {
public:
    /**
     * @brief Enumeration for different machine types
     */
    enum class machine_type {
        permanent_magnet,   // PMSM: common cross-coupling with PM flux
        asynchronous,       // IM: cross-coupling with slip frequency
        externally_excited  // EESM: cross-coupling similar to PMSM
    };

    /**
     * @brief Constructor
     *
     * @param sampling_period Sampling period [s]
     * @param type Machine type
     */
    explicit machine_current_controller(double sampling_period, machine_type type = machine_type::permanent_magnet)
        : _sampling_period(sampling_period),
          _machine_type(type),
          _d_current_controller(sampling_period),
          _q_current_controller(sampling_period) {}

    /**
     * @brief Configure for permanent magnet synchronous machine
     *
     * @param stator_resistance Stator resistance [p.u.]
     * @param d_axis_inductance D-axis inductance [p.u.]
     * @param q_axis_inductance Q-axis inductance [p.u.]
     * @param pm_flux_linkage PM flux linkage [p.u.]
     * @param response_time Desired closed-loop response time [s]
     * @param voltage_limit Maximum voltage limit [p.u.]
     */
    void configure_for_pmsm(double stator_resistance, double d_axis_inductance, double q_axis_inductance,
                            double pm_flux_linkage, double response_time, double voltage_limit) {
        _machine_type = machine_type::permanent_magnet;

        _stator_resistance = stator_resistance;
        _d_axis_inductance = d_axis_inductance;
        _q_axis_inductance = q_axis_inductance;
        _pm_flux_linkage = pm_flux_linkage;
        _voltage_limit = voltage_limit;

        // IMC tuning for d-axis controller
        double kp_d = d_axis_inductance / response_time;
        double ti_d = d_axis_inductance / stator_resistance;

        // IMC tuning for q-axis controller
        double kp_q = q_axis_inductance / response_time;
        double ti_q = q_axis_inductance / stator_resistance;

        _d_current_controller.configure_with_ti(kp_d, ti_d);
        _q_current_controller.configure_with_ti(kp_q, ti_q);

        // Set voltage limits
        _d_current_controller.set_output_limits(-voltage_limit, voltage_limit);
        _q_current_controller.set_output_limits(-voltage_limit, voltage_limit);
    }

    /**
     * @brief Configure for induction machine
     *
     * @param stator_resistance Stator resistance [p.u.]
     * @param rotor_resistance Rotor resistance [p.u.]
     * @param stator_inductance Stator inductance [p.u.]
     * @param rotor_inductance Rotor inductance [p.u.]
     * @param mutual_inductance Mutual inductance [p.u.]
     * @param response_time Desired closed-loop response time [s]
     * @param voltage_limit Maximum voltage limit [p.u.]
     */
    void configure_for_induction_machine(double stator_resistance, double rotor_resistance, double stator_inductance,
                                         double rotor_inductance, double mutual_inductance, double response_time,
                                         double voltage_limit) {
        _machine_type = machine_type::asynchronous;

        _stator_resistance = stator_resistance;
        _rotor_resistance = rotor_resistance;
        _stator_inductance = stator_inductance;
        _rotor_inductance = rotor_inductance;
        _mutual_inductance = mutual_inductance;
        _voltage_limit = voltage_limit;

        // Calculate leakage factor
        _leakage_factor = 1.0 - (mutual_inductance * mutual_inductance) / (stator_inductance * rotor_inductance);

        // Transient inductance for current control
        double transient_inductance = _leakage_factor * stator_inductance;

        // IMC tuning for current controllers (using transient inductance)
        double kp = transient_inductance / response_time;
        double ti = transient_inductance / stator_resistance;

        _d_current_controller.configure_with_ti(kp, ti);
        _q_current_controller.configure_with_ti(kp, ti);

        // Set voltage limits
        _d_current_controller.set_output_limits(-voltage_limit, voltage_limit);
        _q_current_controller.set_output_limits(-voltage_limit, voltage_limit);
    }

    /**
     * @brief Configure for externally excited synchronous machine
     *
     * @param stator_resistance Stator resistance [p.u.]
     * @param d_axis_inductance D-axis inductance [p.u.]
     * @param q_axis_inductance Q-axis inductance [p.u.]
     * @param mutual_inductance_md Mutual inductance between d-axis and field [p.u.]
     * @param response_time Desired closed-loop response time [s]
     * @param voltage_limit Maximum voltage limit [p.u.]
     */
    void configure_for_eesm(double stator_resistance, double d_axis_inductance, double q_axis_inductance,
                            double mutual_inductance_md, double response_time, double voltage_limit) {
        _machine_type = machine_type::externally_excited;

        _stator_resistance = stator_resistance;
        _d_axis_inductance = d_axis_inductance;
        _q_axis_inductance = q_axis_inductance;
        _mutual_inductance_md = mutual_inductance_md;
        _voltage_limit = voltage_limit;

        // IMC tuning for d-axis controller
        double kp_d = d_axis_inductance / response_time;
        double ti_d = d_axis_inductance / stator_resistance;

        // IMC tuning for q-axis controller
        double kp_q = q_axis_inductance / response_time;
        double ti_q = q_axis_inductance / stator_resistance;

        _d_current_controller.configure_with_ti(kp_d, ti_d);
        _q_current_controller.configure_with_ti(kp_q, ti_q);

        // Set voltage limits
        _d_current_controller.set_output_limits(-voltage_limit, voltage_limit);
        _q_current_controller.set_output_limits(-voltage_limit, voltage_limit);
    }

    /**
     * @brief Update the current controller for PMSM or EESM
     *
     * @param current_reference Current reference in dq-frame [p.u.]
     * @param current_measured Measured current in dq-frame [p.u.]
     * @param electrical_speed Electrical speed [p.u.]
     * @param field_current Field current for EESM (ignored for PMSM) [p.u.]
     * @return dq<double> Voltage command in dq-frame [p.u.]
     */
    dq<double> update_synchronous_machine(const dq<double>& current_reference, const dq<double>& current_measured,
                                          double electrical_speed, double field_current = 0.0) {
        // Current errors
        double id_error = current_reference.d() - current_measured.d();
        double iq_error = current_reference.q() - current_measured.q();

        // PI controllers
        double vd_pi = _d_current_controller.update(id_error);
        double vq_pi = _q_current_controller.update(iq_error);

        // Cross-coupling compensation terms
        double vd_cc = -electrical_speed * _q_axis_inductance * current_measured.q();
        double vq_cc = electrical_speed * _d_axis_inductance * current_measured.d();

        // Add back-EMF term depending on machine type
        if (_machine_type == machine_type::permanent_magnet) {
            // For PMSM: back-EMF from permanent magnet
            vq_cc += electrical_speed * _pm_flux_linkage;
        } else if (_machine_type == machine_type::externally_excited) {
            // For EESM: back-EMF from field current
            vq_cc += electrical_speed * _mutual_inductance_md * field_current;
        }

        // Total voltage commands
        dq<double> voltage_command;
        voltage_command.d() = vd_pi + vd_cc;
        voltage_command.q() = vq_pi + vq_cc;

        // Apply voltage limitation
        apply_voltage_limitation(voltage_command);

        return voltage_command;
    }

    /**
     * @brief Update the current controller for induction machine
     *
     * @param current_reference Current reference in dq-frame [p.u.]
     * @param current_measured Measured current in dq-frame [p.u.]
     * @param electrical_speed Electrical speed (rotor speed + slip) [p.u.]
     * @param rotor_flux_magnitude Rotor flux magnitude [p.u.]
     * @return dq<double> Voltage command in dq-frame [p.u.]
     */
    dq<double> update_induction_machine(const dq<double>& current_reference, const dq<double>& current_measured,
                                        double electrical_speed, double rotor_flux_magnitude) {
        // Current errors
        double id_error = current_reference.d() - current_measured.d();
        double iq_error = current_reference.q() - current_measured.q();

        // PI controllers
        double vd_pi = _d_current_controller.update(id_error);
        double vq_pi = _q_current_controller.update(iq_error);

        // Cross-coupling terms for induction machine
        double transient_inductance = _leakage_factor * _stator_inductance;

        double vd_cc = -electrical_speed * transient_inductance * current_measured.q();
        double vq_cc = electrical_speed * transient_inductance * current_measured.d();

        // Add rotor flux effect (back-EMF equivalent)
        // This is simplified - exact term depends on rotor flux angle
        double magnetizing_effect =
            electrical_speed * (_mutual_inductance / _rotor_inductance) * (_mutual_inductance * current_measured.d());
        vq_cc += magnetizing_effect;

        // Total voltage commands
        dq<double> voltage_command;
        voltage_command.d() = vd_pi + vd_cc;
        voltage_command.q() = vq_pi + vq_cc;

        // Apply voltage limitation
        apply_voltage_limitation(voltage_command);

        return voltage_command;
    }

    /**
     * @brief Generic update method that dispatches to the appropriate machine-specific method
     *
     * @param current_reference Current reference in dq-frame [p.u.]
     * @param current_measured Measured current in dq-frame [p.u.]
     * @param electrical_speed Electrical speed [p.u.]
     * @param auxiliary_value Machine-specific value (field current or rotor flux) [p.u.]
     * @return dq<double> Voltage command in dq-frame [p.u.]
     */
    dq<double> update(const dq<double>& current_reference, const dq<double>& current_measured, double electrical_speed,
                      double auxiliary_value = 0.0) {
        switch (_machine_type) {
            case machine_type::asynchronous:
                return update_induction_machine(current_reference, current_measured, electrical_speed, auxiliary_value);

            case machine_type::permanent_magnet:
                return update_synchronous_machine(current_reference, current_measured, electrical_speed);

            case machine_type::externally_excited:
                return update_synchronous_machine(current_reference, current_measured, electrical_speed,
                                                  auxiliary_value);

            default:
                // Default to PMSM behavior
                return update_synchronous_machine(current_reference, current_measured, electrical_speed);
        }
    }

    /**
     * @brief Reset controller states
     */
    void reset() {
        _d_current_controller.reset();
        _q_current_controller.reset();
    }

    /**
     * @brief Set output voltage limits
     *
     * @param limit Maximum voltage magnitude [p.u.]
     */
    void set_voltage_limit(double limit) {
        _voltage_limit = limit;
        _d_current_controller.set_output_limits(-limit, limit);
        _q_current_controller.set_output_limits(-limit, limit);
    }

private:
    /**
     * @brief Apply voltage limitation (circle limit)
     *
     * @param voltage_command Voltage command to be limited [p.u.]
     */
    void apply_voltage_limitation(dq<double>& voltage_command) {
        double v_mag_squared = voltage_command.d() * voltage_command.d() + voltage_command.q() * voltage_command.q();

        if (v_mag_squared > _voltage_limit * _voltage_limit) {
            double scale = _voltage_limit / std::sqrt(v_mag_squared);
            voltage_command.d() *= scale;
            voltage_command.q() *= scale;
        }
    }

    // Basic configuration
    double _sampling_period;
    machine_type _machine_type;

    // Controllers
    pi_controller<double> _d_current_controller;
    pi_controller<double> _q_current_controller;

    // Common parameters
    double _stator_resistance{0.05};
    double _voltage_limit{1.0};

    // PMSM and EESM parameters
    double _d_axis_inductance{0.05};
    double _q_axis_inductance{0.05};
    double _pm_flux_linkage{0.9};       // For PMSM
    double _mutual_inductance_md{1.0};  // For EESM

    // Induction machine parameters
    double _rotor_resistance{0.05};
    double _stator_inductance{0.10};
    double _rotor_inductance{0.10};
    double _mutual_inductance{0.09};
    double _leakage_factor{0.1};
};

}  // namespace yeet::control