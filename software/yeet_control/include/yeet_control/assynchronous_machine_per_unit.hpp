#pragma once

#include "angle.hpp"
#include "assynchronous_machine.hpp"
#include "constants.hpp"
#include "machine_per_unit_system.hpp"
#include "reference_frame.hpp"

namespace yeet::control {

class assynchronous_machine_per_unit {
public:
    // Constructor that takes a per_unit_definition struct
    explicit assynchronous_machine_per_unit(const per_unit_definition& definition, double dt = 1e-4)
        : _definition(definition), _base_values(definition), _machine(dt) {}

    // For backward compatibility - constructs using peak phase voltage directly
    explicit assynchronous_machine_per_unit(double peak_phase_voltage, double nominal_power, double nominal_frequency,
                                            double dt = 1e-4)
        : assynchronous_machine_per_unit(
              per_unit_definition::from_peak_phase_voltage(peak_phase_voltage, nominal_power, nominal_frequency), dt) {}

    // Set machine parameters in per-unit
    void parametrize(double Rs_pu, double Rr_pu, double Ls_pu, double Lr_pu, double Lm_pu, double pp, double J_pu,
                     double B_pu) {
        // Convert parameters from per-unit to SI units
        double Rs_si = Rs_pu * _base_values.impedance;
        double Rr_si = Rr_pu * _base_values.impedance;
        double Ls_si = Ls_pu * _base_values.impedance / (2.0 * pi<double> * _base_values.frequency);
        double Lr_si = Lr_pu * _base_values.impedance / (2.0 * pi<double> * _base_values.frequency);
        double Lm_si = Lm_pu * _base_values.impedance / (2.0 * pi<double> * _base_values.frequency);

        // For inertia: base inertia = base_torque / base_acceleration = base_torque / base_speed^2
        double base_inertia = _base_values.torque / (_base_values.speed * _base_values.speed);
        double J_si = J_pu * base_inertia;

        // For friction: base friction = base_torque / base_speed
        double base_friction = _base_values.torque / _base_values.speed;
        double B_si = B_pu * base_friction;

        // Set parameters on the underlying machine
        _machine.parametrize(Rs_si, Rr_si, Ls_si, Lr_si, Lm_si, pp, J_si, B_si);
    }

    // Get the per-unit definition used for configuration
    const per_unit_definition& get_definition() const { return _definition; }

    // Get the calculated base values
    const per_unit_base_values& get_base_values() const { return _base_values; }

    // Update with per-unit voltage and torque
    void update(const alphabeta<double>& voltage_ab_pu, double load_torque_pu) {
        // Convert inputs from per-unit to SI
        alphabeta<double> voltage_ab_si = voltage_pu_to_si(voltage_ab_pu);
        double load_torque_si = load_torque_pu * _base_values.torque;

        // Update the underlying machine
        _machine.update(voltage_ab_si, load_torque_si);
    }

    // Get methods that return per-unit values
    alphabeta<double> get_stator_current_pu() const {
        // Get currents in SI and convert to per-unit
        alphabeta<double> current_si = _machine.get_stator_current();
        return current_si_to_pu(current_si);
    }

    alphabeta<double> get_rotor_current_pu() const {
        alphabeta<double> current_si = _machine.get_rotor_current();
        return current_si_to_pu(current_si);
    }

    alphabeta<double> get_stator_flux_pu() const {
        alphabeta<double> flux_si = _machine.get_stator_flux();
        return flux_si_to_pu(flux_si);
    }

    alphabeta<double> get_rotor_flux_pu() const {
        alphabeta<double> flux_si = _machine.get_rotor_flux();
        return flux_si_to_pu(flux_si);
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

    // Get stator flux angle (pass-through, no conversion needed)
    angle<zero_to_two_pi> get_stator_angle() const { return _machine.get_stator_angle(); }

    // Get electromagnetic torque in per-unit
    double get_torque_pu() const {
        double torque_si = _machine.get_torque();
        return torque_si / _base_values.torque;
    }

    // Get the underlying SI machine (for advanced access)
    const assynchronous_machine& get_si_machine() const { return _machine; }

private:
    // Store original definition and derived base values
    per_unit_definition _definition;
    per_unit_base_values _base_values;

    // The underlying SI-based machine
    assynchronous_machine _machine;

    // Conversion methods for voltage
    alphabeta<double> voltage_pu_to_si(const alphabeta<double>& v_pu) const {
        return alphabeta<double>{v_pu.alpha() * _base_values.voltage, v_pu.beta() * _base_values.voltage};
    }

    alphabeta<double> voltage_si_to_pu(const alphabeta<double>& v_si) const {
        return alphabeta<double>{v_si.alpha() / _base_values.voltage, v_si.beta() / _base_values.voltage};
    }

    // Conversion methods for current
    alphabeta<double> current_si_to_pu(const alphabeta<double>& i_si) const {
        return alphabeta<double>{i_si.alpha() / _base_values.current, i_si.beta() / _base_values.current};
    }

    alphabeta<double> current_pu_to_si(const alphabeta<double>& i_pu) const {
        return alphabeta<double>{i_pu.alpha() * _base_values.current, i_pu.beta() * _base_values.current};
    }

    // Conversion methods for flux
    alphabeta<double> flux_si_to_pu(const alphabeta<double>& flux_si) const {
        return alphabeta<double>{flux_si.alpha() / _base_values.flux, flux_si.beta() / _base_values.flux};
    }

    alphabeta<double> flux_pu_to_si(const alphabeta<double>& flux_pu) const {
        return alphabeta<double>{flux_pu.alpha() * _base_values.flux, flux_pu.beta() * _base_values.flux};
    }
};

}  // namespace yeet::control