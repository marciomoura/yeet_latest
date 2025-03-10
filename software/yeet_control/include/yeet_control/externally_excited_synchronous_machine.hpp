#pragma once

#include <cmath>
#include <cstdint>
#include <functional>

#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/range_kutta_derivative.hpp"
#include "yeet_control/reference_frame.hpp"

namespace yeet::control {

class externally_excited_synchronous_machine {
public:
    explicit externally_excited_synchronous_machine(double dt = 1e-4) : _dt(dt), _rotor_angle(0.0) {}

    /**
     * @brief Parametrize the machine in SI units.
     *
     * @param stator_resistance      Stator resistance [ohm]
     * @param field_resistance       Field winding resistance [ohm]
     * @param inductance_d           d-axis inductance [H]
     * @param inductance_q           q-axis inductance [H]
     * @param field_inductance       Field winding self inductance [H]
     * @param mutual_inductance_md   Mutual inductance between d-axis and field winding [H]
     * @param pole_pairs             Number of pole pairs
     * @param moment_of_inertia      Inertia [kg·m²]
     * @param friction_coefficient   Viscous friction coefficient [N·m·s/rad]
     */
    void parametrize(double stator_resistance, double field_resistance, double inductance_d, double inductance_q,
                     double field_inductance, double mutual_inductance_md, double pole_pairs, double moment_of_inertia,
                     double friction_coefficient) {
        _stator_resistance = stator_resistance;
        _field_resistance = field_resistance;
        _inductance_d = inductance_d;
        _inductance_q = inductance_q;
        _field_inductance = field_inductance;
        _mutual_inductance_md = mutual_inductance_md;
        _pole_pairs = static_cast<std::uint32_t>(pole_pairs);
        _moment_of_inertia = moment_of_inertia;
        _friction_coefficient = friction_coefficient;
    }

    /**
     * @brief Update the machine with stator voltage in alpha-beta frame and field voltage
     *
     * @param voltage_ab     Alpha-beta stator voltage [V]
     * @param field_voltage  Field voltage [V]
     * @param load_torque    Load torque [N·m]
     */
    void update(const alphabeta<double>& voltage_ab, double field_voltage, double load_torque) {
        // Convert alpha-beta voltage to dq using rotor angle
        dq<double> const voltage_dq = voltage_ab.to_dq(_rotor_angle);

        // Update with dq voltage
        update(voltage_dq, field_voltage, load_torque);
    }

    /**
     * @brief Update the machine with stator voltage in dq frame and field voltage
     *
     * @param voltage_dq     d-q stator voltage [V]
     * @param field_voltage  Field voltage [V]
     * @param load_torque    Load torque [N·m]
     */
    void update(const dq<double>& voltage_dq, double field_voltage, double load_torque) {
        update_state(voltage_dq, field_voltage, load_torque);
    }

    // Getters for machine state
    dq<double> get_stator_currents() const { return _stator_currents; }

    alphabeta<double> get_stator_currents_alphabeta() const { return _stator_currents.to_alphabeta(_rotor_angle); }

    double get_field_current() const { return _field_current; }

    double get_mechanical_speed_rad_s() const { return _mechanical_speed; }

    double get_electrical_speed_rad_s() const { return _mechanical_speed * _pole_pairs; }

    angle<zero_to_two_pi> get_rotor_angle() const { return _rotor_angle; }

    std::uint32_t get_pole_pairs() const { return _pole_pairs; }

    // Get electromagnetic torque [N·m]
    double get_torque() const {
        // T_e = (3/2)*p*[(L_md*i_f + (L_d-L_q)*i_d)*i_q]
        return (3.0 / 2.0) * _pole_pairs *
               ((_mutual_inductance_md * _field_current + (_inductance_d - _inductance_q) * _stator_currents.d()) *
                _stator_currents.q());
    }

    // Get rotor field flux linkage [Wb]
    double get_field_flux_linkage() const { return _mutual_inductance_md * _field_current; }

    /**
     * @brief Initialize the machine to a steady-state operating point.
     *
     * @param initial_speed        Desired initial speed [rad/s]
     * @param initial_field_voltage Field voltage at operating point [V]
     * @param initial_load_torque  Load torque at operating point [N·m]
     * @return dq<double>          Required steady-state stator voltage [V]
     */
    dq<double> initialize_steady_state(double initial_speed, double initial_field_voltage, double initial_load_torque) {
        // 1. Initialize mechanical state
        _mechanical_speed = initial_speed;
        double electrical_speed = initial_speed * _pole_pairs;

        // 2. Calculate steady-state field current
        _field_current = initial_field_voltage / _field_resistance;

        // 3. Calculate required q-axis current for torque balance
        double required_torque = initial_load_torque + _friction_coefficient * initial_speed;
        double field_flux = _mutual_inductance_md * _field_current;
        double i_q = (2.0 * required_torque) / (3.0 * _pole_pairs * field_flux);

        // 4. Set d-axis current to zero (for simplicity)
        double i_d = 0.0;

        // 5. Calculate required voltages in steady state
        // v_d = R_s*i_d - w_e*L_q*i_q
        // v_q = R_s*i_q + w_e*L_d*i_d + w_e*L_md*i_f
        dq<double> steady_state_voltage;
        steady_state_voltage.d() = _stator_resistance * i_d - electrical_speed * _inductance_q * i_q;
        steady_state_voltage.q() = _stator_resistance * i_q + electrical_speed * _mutual_inductance_md * _field_current;

        // 6. Set the initial stator currents
        _stator_currents.d() = i_d;
        _stator_currents.q() = i_q;

        return steady_state_voltage;
    }

private:
    // Structure to hold the machine state for RK4 integration
    struct MachineState {
        dq<double> stator_currents;  // d-q axis stator currents [A]
        double field_current;        // field winding current [A]
        double mechanical_speed;     // mechanical speed [rad/s]
        double rotor_angle;          // electrical rotor angle [rad]

        // Required operators for RK4 implementation
        MachineState operator+(const MachineState& other) const {
            return {stator_currents + other.stator_currents, field_current + other.field_current,
                    mechanical_speed + other.mechanical_speed, rotor_angle + other.rotor_angle};
        }

        MachineState operator*(const double scalar) const {
            return {stator_currents * scalar, field_current * scalar, mechanical_speed * scalar, rotor_angle * scalar};
        }
    };

    /**
     * @brief Update machine state using RK4 integration
     *
     * The dynamics are defined as:
     *   di_d/dt = (v_d - R_s*i_d + ω_e*L_q*i_q) / L_d
     *   di_q/dt = (v_q - R_s*i_q - ω_e*L_d*i_d - ω_e*L_md*i_f) / L_q
     *   di_f/dt = (v_f - R_f*i_f) / L_f
     *   dω_m/dt = (T_e - T_load - B*ω_m) / J
     *   dθ_r/dt = ω_e = pole_pairs * ω_m
     */
    void update_state(const dq<double>& voltage_dq, double field_voltage, double load_torque) {
        // Current machine state
        MachineState current_state = {_stator_currents, _field_current, _mechanical_speed, _rotor_angle.get_radians()};

        // Function to calculate state derivatives
        auto calculate_derivatives = [this, &voltage_dq, field_voltage,
                                      load_torque](const MachineState& state) -> MachineState {
            // Calculate electrical speed
            double electrical_speed = state.mechanical_speed * _pole_pairs;

            // Electrical dynamics - stator currents
            dq<double> stator_current_derivatives;

            // d-axis current derivative
            stator_current_derivatives.d() = (voltage_dq.d() - _stator_resistance * state.stator_currents.d() +
                                              electrical_speed * _inductance_q * state.stator_currents.q()) /
                                             _inductance_d;

            // q-axis current derivative
            stator_current_derivatives.q() = (voltage_dq.q() - _stator_resistance * state.stator_currents.q() -
                                              electrical_speed * _inductance_d * state.stator_currents.d() -
                                              electrical_speed * _mutual_inductance_md * state.field_current) /
                                             _inductance_q;

            // Field current derivative
            double field_current_derivative =
                (field_voltage - _field_resistance * state.field_current) / _field_inductance;

            // Calculate electromagnetic torque
            double electrical_torque = (3.0 / 2.0) * _pole_pairs *
                                       ((_mutual_inductance_md * state.field_current +
                                         (_inductance_d - _inductance_q) * state.stator_currents.d()) *
                                        state.stator_currents.q());

            // Mechanical dynamics
            double speed_derivative =
                (electrical_torque - load_torque - _friction_coefficient * state.mechanical_speed) / _moment_of_inertia;

            // Rotor angle dynamics
            double angle_derivative = electrical_speed;

            // Return all derivatives
            return {stator_current_derivatives, field_current_derivative, speed_derivative, angle_derivative};
        };

        // Perform RK4 integration
        MachineState next_state =
            range_kutta_derivative::calculate<MachineState>(current_state, calculate_derivatives, _dt);

        // Update machine state
        _stator_currents = next_state.stator_currents;
        _field_current = next_state.field_current;
        _mechanical_speed = next_state.mechanical_speed;
        _rotor_angle.set_value(next_state.rotor_angle);
    }

    // Machine parameters (all in SI units)
    double _dt;                         // Time step [s]
    double _stator_resistance{0.0};     // Stator resistance [ohm]
    double _field_resistance{0.0};      // Field winding resistance [ohm]
    double _inductance_d{0.0};          // d-axis inductance [H]
    double _inductance_q{0.0};          // q-axis inductance [H]
    double _field_inductance{0.0};      // Field winding self inductance [H]
    double _mutual_inductance_md{0.0};  // Mutual inductance between d-axis and field [H]
    std::uint32_t _pole_pairs{1};       // Number of pole pairs
    double _moment_of_inertia{0.0};     // Moment of inertia [kg·m²]
    double _friction_coefficient{0.0};  // Viscous friction coefficient [N·m·s/rad]

    // Machine state
    dq<double> _stator_currents{0.0, 0.0};  // d-q axis stator currents [A]
    double _field_current{0.0};             // Field current [A]
    double _mechanical_speed{0.0};          // Mechanical speed [rad/s]
    angle<zero_to_two_pi> _rotor_angle;     // Electrical rotor angle [rad]
};

}  // namespace yeet::control