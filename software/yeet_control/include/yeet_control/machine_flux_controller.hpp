#pragma once

#include <algorithm>
#include <cmath>

#include "first_order_low_pass_filter.hpp"
#include "pi_controller.hpp"

namespace yeet::control {

/**
 * @brief Controller for machine flux regulation with field weakening capability
 *
 * This class provides flux control for various machine types:
 * - For induction machines:
 *     - Controls d-axis current to establish and maintain rotor flux
 *     - Calculates optimal slip frequency for field orientation
 *     - Includes feedforward terms for faster response
 * - For synchronous machines:
 *     - Provides field weakening capability at high speeds
 *     - Calculates optimal d-axis current to maximize operating range
 *     - Implements MTPA (Maximum Torque Per Ampere) strategy
 * - For externally-excited synchronous machines:
 *     - Controls field current to establish desired flux
 *     - Provides voltage commands for field excitation circuit
 */
class machine_flux_controller {
public:
    /**
     * @brief Enumeration for different flux control strategies
     */
    enum class control_mode {
        induction_machine,  // Direct rotor flux control for IM
        permanent_magnet,   // Field weakening for PMSM
        externally_excited  // Field current control for EESM
    };

    /**
     * @brief Constructor
     * @param sampling_period Sampling period [s]
     * @param mode Control mode based on machine type
     */
    explicit machine_flux_controller(double sampling_period, control_mode mode = control_mode::induction_machine)
        : _sampling_period(sampling_period),
          _control_mode(mode),
          _flux_ref_filter(sampling_period),
          _flux_controller(sampling_period) {}

    /**
     * @brief Configure for induction machine flux control
     *
     * @param stator_resistance Stator resistance [p.u.]
     * @param rotor_resistance Rotor resistance [p.u.]
     * @param stator_inductance Stator inductance [p.u.]
     * @param rotor_inductance Rotor inductance [p.u.]
     * @param mutual_inductance Mutual inductance [p.u.]
     * @param response_time Desired flux response time [s]
     * @param nominal_flux Nominal flux [p.u.]
     * @param max_magnetizing_current Maximum magnetizing current [p.u.]
     */
    void configure_for_induction_machine(double stator_resistance, double rotor_resistance, double stator_inductance,
                                         double rotor_inductance, double mutual_inductance, double response_time,
                                         double nominal_flux, double max_magnetizing_current = 2.0) {
        _control_mode = control_mode::induction_machine;

        _stator_resistance = stator_resistance;
        _rotor_resistance = rotor_resistance;
        _stator_inductance = stator_inductance;
        _rotor_inductance = rotor_inductance;
        _mutual_inductance = mutual_inductance;

        // Calculate rotor time constant
        _rotor_time_constant = rotor_inductance / rotor_resistance;

        // Calculate leakage factor sigma
        _leakage_factor = 1.0 - (mutual_inductance * mutual_inductance) / (stator_inductance * rotor_inductance);

        // Store nominal flux and calculate nominal magnetizing current
        _nominal_flux = nominal_flux;
        _nominal_magnetizing_current = nominal_flux * (rotor_inductance / mutual_inductance);

        // Configure flux PI controller with enhanced tuning
        double kp = 2.0 / response_time;  // Higher gain for better response
        double ti = 1.5 * response_time;  // Shorter integral time for faster convergence

        _flux_controller.configure_with_ti(kp, ti);
        _flux_controller.set_output_limits(0.0, max_magnetizing_current);

        // Configure flux reference filter (slow changes to avoid instability)
        _flux_ref_filter.configure(1.0);  // 1 Hz cutoff

        // Default to nominal flux
        set_flux_reference(nominal_flux);
    }

    /**
     * @brief Configure for permanent magnet machine field weakening
     *
     * @param d_axis_inductance D-axis inductance [p.u.]
     * @param q_axis_inductance Q-axis inductance [p.u.]
     * @param pm_flux_linkage PM flux linkage [p.u.]
     * @param max_negative_d_current Maximum negative d-axis current [p.u.]
     * @param voltage_limit Maximum voltage [p.u.]
     */
    void configure_for_pmsm_field_weakening(double d_axis_inductance, double q_axis_inductance, double pm_flux_linkage,
                                            double max_negative_d_current, double voltage_limit) {
        _control_mode = control_mode::permanent_magnet;

        _d_axis_inductance = d_axis_inductance;
        _q_axis_inductance = q_axis_inductance;
        _pm_flux_linkage = pm_flux_linkage;
        _max_negative_d_current = max_negative_d_current;
        _voltage_limit = voltage_limit;

        // Configure field weakening with conservative defaults
        enable_field_weakening(true, 0.95);
    }

    /**
     * @brief Configure for externally excited synchronous machine
     *
     * @param field_resistance Field winding resistance [p.u.]
     * @param field_inductance Field winding inductance [p.u.]
     * @param mutual_inductance_md Mutual inductance between d-axis and field [p.u.]
     * @param response_time Desired field current response time [s]
     * @param nominal_field_current Nominal field current [p.u.]
     * @param field_voltage_limit Maximum field voltage [p.u.]
     */
    void configure_for_external_excitation(double field_resistance, double field_inductance,
                                           double mutual_inductance_md, double response_time,
                                           double nominal_field_current, double field_voltage_limit) {
        _control_mode = control_mode::externally_excited;

        _field_resistance = field_resistance;
        _field_inductance = field_inductance;
        _mutual_inductance_md = mutual_inductance_md;
        _nominal_field_current = nominal_field_current;
        _field_voltage_limit = field_voltage_limit;

        // Configure field current controller
        double kp = field_inductance / response_time;
        double ti = field_inductance / field_resistance;

        _flux_controller.configure_with_ti(kp, ti);
        _flux_controller.set_output_limits(-field_voltage_limit, field_voltage_limit);

        // Configure field current reference filter
        _flux_ref_filter.configure(2.0);  // 2 Hz cutoff

        // Default to nominal field current
        set_flux_reference(nominal_field_current);
    }

    /**
     * @brief Set the flux reference
     *
     * @param flux_ref Flux reference [p.u.] or field current reference [p.u.] for EESM
     */
    void set_flux_reference(double flux_ref) { _flux_reference = flux_ref; }

    /**
     * @brief Enable or disable field weakening
     *
     * @param enable Enable field weakening
     * @param threshold Speed threshold for field weakening activation [p.u.]
     * @param voltage_reserve Voltage reserve margin [p.u.]
     */
    void enable_field_weakening(bool enable, double threshold = 1.0, double voltage_reserve = 0.05) {
        _field_weakening_enabled = enable;
        _field_weakening_threshold = threshold;
        _voltage_reserve = voltage_reserve;
    }

    /**
     * @brief Update flux control for induction machine
     *
     * @param measured_flux Measured rotor flux [p.u.]
     * @return double D-axis current reference for magnetization [p.u.]
     */
    double update_induction_machine(double measured_flux) {
        // Filter flux reference
        double flux_ref_filtered = _flux_ref_filter.update(_flux_reference);

        // Calculate flux error
        double flux_error = flux_ref_filtered - measured_flux;

        // PI control + feedforward for faster response
        double current_d_ref = _nominal_magnetizing_current + _flux_controller.update(flux_error);

        return current_d_ref;
    }

    /**
     * @brief Update field weakening for permanent magnet machine
     *
     * @param speed_pu Machine speed [p.u.]
     * @param current_q_ref Q-axis current reference [p.u.]
     * @return double D-axis current reference for field weakening [p.u.]
     */
    double update_pmsm_field_weakening(double speed_pu, double current_q_ref) {
        // No field weakening below threshold speed
        if (!_field_weakening_enabled || std::abs(speed_pu) <= _field_weakening_threshold) {
            return 0.0;
        }

        // Basic field weakening strategy: id ∝ -speed
        double base_field_weakening = (std::abs(speed_pu) - _field_weakening_threshold) / 0.5;
        double field_weakening_factor = std::min(1.0, base_field_weakening);

        // Take into account q-axis current to ensure circle voltage limit
        double max_d_current = std::sqrt(std::max(0.0, 1.0 - current_q_ref * current_q_ref));

        // Apply field weakening with limit
        double id_ref = -field_weakening_factor * _max_negative_d_current * max_d_current;

        // Clamp to maximum negative current
        return std::max(id_ref, -_max_negative_d_current);
    }

    /**
     * @brief Update field current control for externally excited machine
     *
     * @param measured_field_current Measured field current [p.u.]
     * @return double Field voltage command [p.u.]
     */
    double update_external_excitation(double measured_field_current) {
        // Filter reference
        double field_current_ref = _flux_ref_filter.update(_flux_reference);

        // Calculate error and apply PI control
        double field_current_error = field_current_ref - measured_field_current;
        double field_voltage = _flux_controller.update(field_current_error);

        return field_voltage;
    }

    /**
     * @brief Calculate optimum q-axis current for maximum torque
     */
    double calculate_optimum_q_current(double speed_pu, double torque_ref) {
        // Placeholder implementation
        return torque_ref / speed_pu;
    }

    /**
     * @brief Calculate optimum q-axis current for maximum torque per ampere (MTPA)
     *
     * @param torque_ref Torque reference [p.u.]
     * @param id_ref D-axis current reference [p.u.]
     * @return double Optimum q-axis current [p.u.]
     */
    double calculate_mtpa_current(double torque_ref, double id_ref = 0.0) {
        double iq_ref = 0.0;

        switch (_control_mode) {
            case control_mode::induction_machine:
                // For IM: T = k * ψr * iq (simplified)
                // Use measured or estimated flux or default to reference
                iq_ref = torque_ref / std::max(_flux_reference, 0.1);
                break;

            case control_mode::permanent_magnet: {
                // For PMSM with reluctance torque: T = k * (ψpm * iq + (Ld-Lq)*id*iq)
                double reluctance_factor = _d_axis_inductance - _q_axis_inductance;
                if (std::abs(reluctance_factor) < 0.01) {
                    // Surface-mounted PMSM (no reluctance torque)
                    iq_ref = torque_ref / _pm_flux_linkage;
                } else {
                    // Interior PMSM with reluctance torque
                    iq_ref = torque_ref / (_pm_flux_linkage + reluctance_factor * id_ref);
                }
                break;
            }

            case control_mode::externally_excited:
                // For EESM: T ∝ Lmd * if * iq (simplified)
                iq_ref = torque_ref / (_mutual_inductance_md * std::max(_nominal_field_current, 0.1));
                break;
        }

        return iq_ref;
    }

private:
    double _sampling_period;
    control_mode _control_mode;

    // Induction machine parameters
    double _stator_resistance{};
    double _rotor_resistance{};
    double _stator_inductance{};
    double _rotor_inductance{};
    double _mutual_inductance{};
    double _rotor_time_constant{};
    double _leakage_factor{};
    double _nominal_flux{};
    double _nominal_magnetizing_current{};

    // PMSM parameters
    double _d_axis_inductance{};
    double _q_axis_inductance{};
    double _pm_flux_linkage{};
    double _max_negative_d_current{};
    double _voltage_limit{};

    // EESM parameters
    double _field_resistance{};
    double _field_inductance{};
    double _mutual_inductance_md{};
    double _nominal_field_current{};
    double _field_voltage_limit{};

    // Control parameters
    double _flux_reference{};
    bool _field_weakening_enabled{};
    double _field_weakening_threshold{};
    double _voltage_reserve{};

    // Controller objects
    pi_controller<double> _flux_controller;
    first_order_low_pass_filter<double> _flux_ref_filter;
};

}  // namespace yeet::control