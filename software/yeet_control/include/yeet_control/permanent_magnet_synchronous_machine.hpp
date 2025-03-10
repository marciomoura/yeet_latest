#pragma once

#include <cmath>
#include <cstdint>
#include <functional>

#include "yeet_control/angle.hpp"
#include "yeet_control/range_kutta_derivative.hpp"
#include "yeet_control/reference_frame.hpp"

namespace yeet::control {

class permanent_magnet_synchronous_machine {
public:
    explicit permanent_magnet_synchronous_machine(double dt = 1e-4) : _dt(dt), _rotor_angle(0.0) {}

    /**
     * @brief Parametrize the machine in SI units.
     *
     * @param stator_resistance     Stator resistance [ohm]
     * @param inductance_d          d-axis inductance [H]
     * @param inductance_q          q-axis inductance [H]
     * @param pm_flux_linkage       Permanent magnet flux linkage [Wb]
     * @param pole_pairs            Number of pole pairs
     * @param moment_of_inertia     Inertia [kg·m²]
     * @param friction_coefficient  Viscous friction coefficient [N·m·s/rad]
     */
    void parametrize(double stator_resistance, double inductance_d, double inductance_q, double pm_flux_linkage,
                     double pole_pairs, double moment_of_inertia, double friction_coefficient) {
        _stator_resistance = stator_resistance;
        _inductance_d = inductance_d;
        _inductance_q = inductance_q;
        _pm_flux_linkage = pm_flux_linkage;
        _pole_pairs = static_cast<std::uint32_t>(pole_pairs);
        _moment_of_inertia = moment_of_inertia;
        _friction_coefficient = friction_coefficient;
    }

    /**
     * @brief Update the machine with voltage in alpha-beta frame.
     *
     * @param voltage_ab Alpha-beta voltage [V]
     * @param load_torque Load torque [N·m]
     */
    void update(const alphabeta<double>& voltage_ab, double load_torque) {
        // Convert alpha-beta voltage to dq using rotor angle
        dq<double> const voltage_dq = voltage_ab.to_dq(_rotor_angle);

        // Update with dq voltage
        update(voltage_dq, load_torque);
    }

    /**
     * @brief Update the machine with voltage in dq frame.
     *
     * @param voltage_dq d-q voltage [V]
     * @param load_torque Load torque [N·m]
     */
    void update(const dq<double>& voltage_dq, double load_torque) { update_state(voltage_dq, load_torque); }

    // Getters for machine state
    dq<double> get_currents() const { return _currents; }

    alphabeta<double> get_currents_alphabeta() const { return _currents.to_alphabeta(_rotor_angle); }

    double get_mechanical_speed_rad_s() const { return _mechanical_speed; }

    double get_electrical_speed_rad_s() const { return _mechanical_speed * _pole_pairs; }

    angle<zero_to_two_pi> get_rotor_angle() const { return _rotor_angle; }

    std::uint32_t get_pole_pairs() const { return _pole_pairs; }

    // Get electromagnetic torque [N·m]
    double get_torque() const {
        return (3.0 / 2.0) * _pole_pairs *
               (_pm_flux_linkage * _currents.q() + (_inductance_d - _inductance_q) * _currents.d() * _currents.q());
    }

    // Get machine parameters
    double get_inductance_d() const { return _inductance_d; }
    double get_inductance_q() const { return _inductance_q; }
    double get_pm_flux_linkage() const { return _pm_flux_linkage; }

    /**
     * @brief Initialize the machine to a steady-state operating point.
     *
     * @param initial_speed        Desired initial speed [rad/s]
     * @param initial_load_torque  Load torque at operating point [N·m]
     * @return dq<double>          Required steady-state voltage [V]
     */
    dq<double> initialize_steady_state(double initial_speed, double initial_load_torque) {
        // 1. Initialize mechanical state
        _mechanical_speed = initial_speed;
        double electrical_speed = initial_speed * _pole_pairs;

        // 2. Calculate required q-axis current for torque balance
        // For simplicity, using MTPA strategy with id = 0 for non-salient machines
        double required_torque = initial_load_torque + _friction_coefficient * initial_speed;
        double i_q = (2.0 * required_torque) / (3.0 * _pole_pairs * _pm_flux_linkage);

        // 3. Set d-axis current to zero (MTPA for non-salient)
        double i_d = 0.0;

        // 4. Calculate required voltages in steady state
        // v_d = R*i_d - ω*Lq*i_q
        // v_q = R*i_q + ω*Ld*i_d + ω*ψpm
        dq<double> steady_state_voltage;
        steady_state_voltage.d() = _stator_resistance * i_d - electrical_speed * _inductance_q * i_q;
        steady_state_voltage.q() = _stator_resistance * i_q + electrical_speed * _pm_flux_linkage;

        // 5. Set the initial states
        _currents.d() = i_d;
        _currents.q() = i_q;
        // Keep existing rotor angle (or set to zero if needed)

        return steady_state_voltage;
    }

private:
    // Structure to hold the machine state for RK4 integration
    struct MachineState {
        dq<double> currents;
        double mechanical_speed;
        double rotor_angle;

        // Required operators for RK4 implementation
        MachineState operator+(const MachineState& other) const {
            return {currents + other.currents, mechanical_speed + other.mechanical_speed,
                    rotor_angle + other.rotor_angle};
        }

        MachineState operator*(const double scalar) const {
            return {currents * scalar, mechanical_speed * scalar, rotor_angle * scalar};
        }
    };

    /**
     * @brief Update machine state using RK4 integration
     *
     * The dynamics are defined as:
     *   di_d/dt = (v_d - R*i_d + ω_e*L_q*i_q) / L_d
     *   di_q/dt = (v_q - R*i_q - ω_e*L_d*i_d - ω_e*ψ_pm) / L_q
     *   dω_m/dt = (T_e - T_load - B*ω_m) / J
     *   dθ_r/dt = ω_e = pole_pairs * ω_m
     */
    void update_state(const dq<double>& voltage_dq, double load_torque) {
        // Current machine state
        MachineState current_state = {_currents, _mechanical_speed, _rotor_angle.get_radians()};

        // Function to calculate state derivatives
        auto calculate_derivatives = [this, &voltage_dq, load_torque](const MachineState& state) -> MachineState {
            // Calculate electrical speed
            double electrical_speed = state.mechanical_speed * _pole_pairs;

            // Electrical dynamics
            dq<double> current_derivatives;
            current_derivatives.d() = (voltage_dq.d() - _stator_resistance * state.currents.d() +
                                       electrical_speed * _inductance_q * state.currents.q()) /
                                      _inductance_d;

            current_derivatives.q() =
                (voltage_dq.q() - _stator_resistance * state.currents.q() -
                 electrical_speed * _inductance_d * state.currents.d() - electrical_speed * _pm_flux_linkage) /
                _inductance_q;

            // Calculate torque
            double electrical_torque = (3.0 / 2.0) * _pole_pairs *
                                       (_pm_flux_linkage * state.currents.q() +
                                        (_inductance_d - _inductance_q) * state.currents.d() * state.currents.q());

            // Mechanical dynamics
            double speed_derivative =
                (electrical_torque - load_torque - _friction_coefficient * state.mechanical_speed) / _moment_of_inertia;

            // Rotor angle dynamics
            double angle_derivative = electrical_speed;

            // Return all derivatives
            return {current_derivatives, speed_derivative, angle_derivative};
        };

        // Perform RK4 integration
        MachineState next_state =
            range_kutta_derivative::calculate<MachineState>(current_state, calculate_derivatives, _dt);

        // Update machine state
        _currents = next_state.currents;
        _mechanical_speed = next_state.mechanical_speed;
        _rotor_angle.set_value(next_state.rotor_angle);
    }

    // Machine parameters (all in SI units)
    double _dt;                         // Time step [s]
    double _stator_resistance{0.0};     // Stator resistance [ohm]
    double _inductance_d{0.0};          // d-axis inductance [H]
    double _inductance_q{0.0};          // q-axis inductance [H]
    double _pm_flux_linkage{0.0};       // Permanent magnet flux linkage [Wb]
    std::uint32_t _pole_pairs{1};       // Number of pole pairs
    double _moment_of_inertia{0.0};     // Moment of inertia [kg·m²]
    double _friction_coefficient{0.0};  // Viscous friction coefficient [N·m·s/rad]

    // Machine state
    dq<double> _currents{0.0, 0.0};      // d-q axis currents [A]
    double _mechanical_speed{0.0};       // Mechanical speed [rad/s]
    angle<zero_to_two_pi> _rotor_angle;  // Electrical rotor angle [rad]
};

}  // namespace yeet::control