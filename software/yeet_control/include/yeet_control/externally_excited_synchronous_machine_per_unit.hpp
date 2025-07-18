#pragma once

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/externally_excited_synchronous_machine.hpp"
#include "yeet_control/machine_per_unit_system.hpp"
#include "yeet_control/reference_frame.hpp"

namespace yeet::control {

class externally_excited_synchronous_machine_per_unit {
public:
    // Constructor that takes a per_unit_definition struct
    explicit externally_excited_synchronous_machine_per_unit(const per_unit_definition& definition, double dt = 1e-4)
        : _definition(definition), _base_values(definition), _machine(dt) {}

    // For backward compatibility - constructs using peak phase voltage directly
    explicit externally_excited_synchronous_machine_per_unit(double peak_phase_voltage, double nominal_power,
                                                             double nominal_frequency, double dt = 1e-4)
        : externally_excited_synchronous_machine_per_unit(
              per_unit_definition::from_peak_phase_voltage(peak_phase_voltage, nominal_power, nominal_frequency), dt) {}

    /**
     * @brief Parametrize the machine with per-unit values.
     *
     * @param Rs_pu         Stator resistance [p.u.]
     * @param Rf_pu         Field resistance [p.u.]
     * @param Ld_pu         d-axis inductance [p.u.]
     * @param Lq_pu         q-axis inductance [p.u.]
     * @param Lf_pu         Field inductance [p.u.]
     * @param Lmd_pu        Mutual inductance between d-axis and field [p.u.]
     * @param pole_pairs    Number of pole pairs
     * @param J_pu          Moment of inertia [p.u.]
     * @param B_pu          Viscous friction coefficient [p.u.]
     */
    void parametrize(double Rs_pu, double Rf_pu, double Ld_pu, double Lq_pu, double Lf_pu, double Lmd_pu,
                     double pole_pairs, double J_pu, double B_pu) {
        // Convert parameters from per-unit to SI units
        double Rs_si = Rs_pu * _base_values.impedance;
        double Rf_si = Rf_pu * _base_values.impedance;
        double Ld_si = Ld_pu * _base_values.impedance / (2.0 * pi * _base_values.frequency);
        double Lq_si = Lq_pu * _base_values.impedance / (2.0 * pi * _base_values.frequency);
        double Lf_si = Lf_pu * _base_values.impedance / (2.0 * pi * _base_values.frequency);
        double Lmd_si = Lmd_pu * _base_values.impedance / (2.0 * pi * _base_values.frequency);

        // For inertia: base inertia = base_torque / base_acceleration = base_torque / base_speed^2
        double base_inertia = _base_values.torque / (_base_values.speed * _base_values.speed);
        double J_si = J_pu * base_inertia;

        // For friction: base friction = base_torque / base_speed
        double base_friction = _base_values.torque / _base_values.speed;
        double B_si = B_pu * base_friction;

        // Set parameters on the underlying machine
        _machine.parametrize(Rs_si, Rf_si, Ld_si, Lq_si, Lf_si, Lmd_si, pole_pairs, J_si, B_si);
    }

    // Get the per-unit definition used for configuration
    const per_unit_definition& get_definition() const { return _definition; }

    // Get the calculated base values
    const per_unit_base_values& get_base_values() const { return _base_values; }

    // Update with per-unit voltage in alpha-beta frame, field voltage and torque
    void update(const alphabeta<double>& voltage_ab_pu, double field_voltage_pu, double load_torque_pu) {
        // Convert inputs from per-unit to SI
        alphabeta<double> voltage_ab_si = voltage_pu_to_si(voltage_ab_pu);
        double field_voltage_si = field_voltage_pu * _base_values.voltage;
        double load_torque_si = load_torque_pu * _base_values.torque;

        // Update the underlying machine
        _machine.update(voltage_ab_si, field_voltage_si, load_torque_si);
    }

    // Update with per-unit voltage in dq frame, field voltage and torque
    void update(const dq<double>& voltage_dq_pu, double field_voltage_pu, double load_torque_pu) {
        // Convert inputs from per-unit to SI
        dq<double> voltage_dq_si = voltage_dq_pu_to_si(voltage_dq_pu);
        double field_voltage_si = field_voltage_pu * _base_values.voltage;
        double load_torque_si = load_torque_pu * _base_values.torque;

        // Update the underlying machine
        _machine.update(voltage_dq_si, field_voltage_si, load_torque_si);
    }

    // Get stator currents in dq frame [p.u.]
    dq<double> get_stator_currents_pu() const {
        dq<double> current_si = _machine.get_stator_currents();
        return current_si_to_pu(current_si);
    }

    // Get stator currents in alpha-beta frame [p.u.]
    alphabeta<double> get_stator_currents_alphabeta_pu() const {
        alphabeta<double> current_si = _machine.get_stator_currents_alphabeta();
        return current_si_to_pu(current_si);
    }

    // Get field current [p.u.]
    double get_field_current_pu() const {
        double current_si = _machine.get_field_current();
        return current_si / _base_values.current;
    }

    // Get mechanical speed in per-unit (normalized to rated speed)
    double get_mechanical_speed_pu() const {
        double speed_si = _machine.get_mechanical_speed_rad_s();
        return speed_si / _base_values.speed;
    }

    // Get electrical speed in per-unit
    double get_electrical_speed_pu() const {
        double electrical_speed_si = _machine.get_electrical_speed_rad_s();
        return electrical_speed_si / (_base_values.speed * _machine.get_pole_pairs());
    }

    // Get rotor angle (pass-through, no conversion needed)
    angle<zero_to_two_pi> get_rotor_angle() const { return _machine.get_rotor_angle(); }

    // Get electromagnetic torque in per-unit
    double get_torque_pu() const {
        double torque_si = _machine.get_torque();
        return torque_si / _base_values.torque;
    }

    // Get field flux linkage in per-unit
    double get_field_flux_linkage_pu() const {
        double flux_si = _machine.get_field_flux_linkage();
        return flux_si / _base_values.flux;
    }

    // Get the underlying SI machine (for advanced access)
    const externally_excited_synchronous_machine& get_si_machine() const { return _machine; }

    /**
     * @brief Initialize the machine to a steady-state operating point.
     *
     * @param initial_speed_pu        Desired initial speed [p.u.]
     * @param initial_field_voltage_pu Field voltage at operating point [p.u.]
     * @param initial_load_torque_pu  Load torque at operating point [p.u.]
     * @return dq<double>             Required steady-state stator voltage [p.u.]
     */
    dq<double> initialize_steady_state_pu(double initial_speed_pu, double initial_field_voltage_pu,
                                          double initial_load_torque_pu) {
        // Convert inputs to SI
        double initial_speed_si = initial_speed_pu * _base_values.speed;
        double initial_field_voltage_si = initial_field_voltage_pu * _base_values.voltage;
        double initial_load_torque_si = initial_load_torque_pu * _base_values.torque;

        // Call SI machine initialization
        dq<double> voltage_dq_si =
            _machine.initialize_steady_state(initial_speed_si, initial_field_voltage_si, initial_load_torque_si);

        // Convert result back to per-unit
        return voltage_dq_si_to_pu(voltage_dq_si);
    }

private:
    // Store original definition and derived base values
    per_unit_definition _definition;
    per_unit_base_values _base_values;

    // The underlying SI-based machine
    externally_excited_synchronous_machine _machine;

    // Conversion methods for voltage (alpha-beta)
    alphabeta<double> voltage_pu_to_si(const alphabeta<double>& v_pu) const {
        return alphabeta<double>{v_pu.alpha() * _base_values.voltage, v_pu.beta() * _base_values.voltage};
    }

    alphabeta<double> voltage_si_to_pu(const alphabeta<double>& v_si) const {
        return alphabeta<double>{v_si.alpha() / _base_values.voltage, v_si.beta() / _base_values.voltage};
    }

    // Conversion methods for voltage (dq)
    dq<double> voltage_dq_pu_to_si(const dq<double>& v_pu) const {
        return dq<double>{v_pu.d() * _base_values.voltage, v_pu.q() * _base_values.voltage};
    }

    dq<double> voltage_dq_si_to_pu(const dq<double>& v_si) const {
        return dq<double>{v_si.d() / _base_values.voltage, v_si.q() / _base_values.voltage};
    }

    // Conversion methods for current (dq)
    dq<double> current_si_to_pu(const dq<double>& i_si) const {
        return dq<double>{i_si.d() / _base_values.current, i_si.q() / _base_values.current};
    }

    dq<double> current_pu_to_si(const dq<double>& i_pu) const {
        return dq<double>{i_pu.d() * _base_values.current, i_pu.q() * _base_values.current};
    }

    // Conversion methods for current (alpha-beta)
    alphabeta<double> current_si_to_pu(const alphabeta<double>& i_si) const {
        return alphabeta<double>{i_si.alpha() / _base_values.current, i_si.beta() / _base_values.current};
    }

    alphabeta<double> current_pu_to_si(const alphabeta<double>& i_pu) const {
        return alphabeta<double>{i_pu.alpha() * _base_values.current, i_pu.beta() * _base_values.current};
    }
};

}  // namespace yeet::control