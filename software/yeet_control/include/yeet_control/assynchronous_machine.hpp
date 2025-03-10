#pragma once

#include <cmath>
#include <cstdint>
#include <functional>

#include "angle.hpp"
#include "range_kutta_derivative.hpp"
#include "reference_frame.hpp"

namespace yeet::control {

class assynchronous_machine {
public:
    explicit assynchronous_machine(double dt = 1e-4) : _dt(dt) {}

    // Set machine parameters in SI units
    // Rs, Rr: stator and rotor resistances [ohm]
    // Ls, Lr: stator and rotor self inductances [H]
    // Lm: mutual inductance [H]
    // pp: pole pairs
    // J: moment of inertia [kg·m²]
    // B: viscous friction coefficient [N·m·s/rad]
    void parametrize(double Rs, double Rr, double Ls, double Lr, double Lm, double pp, double J, double B) {
        _stator_resistance = Rs;
        _rotor_resistance = Rr;
        _stator_inductance = Ls;
        _rotor_inductance = Lr;
        _mutual_inductance = Lm;
        _pole_pairs = static_cast<std::uint32_t>(pp);
        _moment_of_inertia_j = J;
        _friction_b = B;

        // Calculate leakage factor (sigma)
        _sigma = 1.0 - (_mutual_inductance * _mutual_inductance) / (_stator_inductance * _rotor_inductance);

        // For better numerical stability, check sigma isn't too small
        if (_sigma < 0.05) {
            _sigma = 0.05;  // Minimum 5% leakage
            assert(_sigma >= 0.05 && "Leakage factor too small");
        }

        // Calculate currents from fluxes using flux-current relationship
        _stator_rotor_flux_dividend = 1.0 / (_stator_inductance * _rotor_inductance * _sigma);
    }

    // Update machine state with voltage in alpha-beta frame [V]
    // load_torque in [N·m]
    void update(const alphabeta<double>& voltage_ab, double load_torque) { update_state(voltage_ab, load_torque); }

    // Get stator currents in alpha-beta frame [A]
    alphabeta<double> get_stator_current() const {
        return calculate_stator_current(_stator_flux, _rotor_flux, _rotor_inductance, _mutual_inductance,
                                        _stator_rotor_flux_dividend);
    }

    // Get rotor currents in alpha-beta frame [A]
    alphabeta<double> get_rotor_current() const {
        return calculate_rotor_current(_stator_flux, _rotor_flux, _stator_inductance, _mutual_inductance,
                                       _stator_rotor_flux_dividend);
    }

    alphabeta<double> get_stator_flux() const { return _stator_flux; }

    alphabeta<double> get_rotor_flux() const { return _rotor_flux; }

    // Get mechanical speed directly in [rad/s]
    double get_mechanical_speed_rad_s() const { return _mechanical_speed; }

    // Get electrical speed [rad/s]
    double get_electrical_speed_rad_s() const { return _mechanical_speed * _pole_pairs; }

    // Get pole pairs
    std::uint32_t get_pole_pairs() const { return _pole_pairs; }

    // Get stator flux angle [rad]
    angle<zero_to_two_pi> get_stator_angle() const { return _stator_flux.phase(); }

    // Get electromagnetic torque [N·m]
    double get_torque() const {
        // Calculate torque using alpha-beta quantities
        alphabeta<double> const stator_currents = get_stator_current();

        return (3.0 / 2.0) * static_cast<double>(_pole_pairs) * cross_product(_rotor_flux, stator_currents);
    }

private:
    static alphabeta<double> calculate_stator_current(alphabeta<double> stator_flux, alphabeta<double> rotor_flux,
                                                      double rotor_inductance, double mutual_inductance,
                                                      double stator_rotor_flux_dividend) {
        // Calculate stator currents from fluxes
        alphabeta<double> const stator_current =
            (rotor_inductance * stator_flux - mutual_inductance * rotor_flux) * stator_rotor_flux_dividend;

        return stator_current;
    }

    static alphabeta<double> calculate_rotor_current(alphabeta<double> stator_flux, alphabeta<double> rotor_flux,
                                                     double stator_inductance, double mutual_inductance,
                                                     double stator_rotor_flux_dividend) {
        alphabeta<double> const rotor_current =
            (-mutual_inductance * stator_flux + stator_inductance * rotor_flux) * stator_rotor_flux_dividend;

        return rotor_current;
    }

    // Dynamic equations in stationary alpha-beta frame (SI units):
    // dψ_sα/dt = v_sα - Rs*i_sα
    // dψ_sβ/dt = v_sβ - Rs*i_sβ
    // dψ_rα/dt = -Rr/Lr*ψ_rα + Lm*Rr/(Lr*Ls)*(ψ_sα-ψ_rα) - ω_r*ψ_rβ
    // dψ_rβ/dt = -Rr/Lr*ψ_rβ + Lm*Rr/(Lr*Ls)*(ψ_sβ-ψ_rβ) + ω_r*ψ_rα
    // dω_m/dt = (T_e - T_load - B*ω_m)/J
    // where T_e = (3/2)*p*(ψ_rα*i_sβ - ψ_rβ*i_sα)
    void update_state(const alphabeta<double>& voltage_ab, double load_torque) {
        // Create a structure to hold the state
        struct MachineState {
            alphabeta<double> stator_flux{};
            alphabeta<double> rotor_flux{};
            double mechanical_speed{};

            // Required operators for RK4 implementation
            MachineState operator+(const MachineState& other) const {
                return {stator_flux + other.stator_flux, rotor_flux + other.rotor_flux,
                        mechanical_speed + other.mechanical_speed};
            }

            MachineState operator*(const double scalar) const {
                return {stator_flux * scalar, rotor_flux * scalar, mechanical_speed * scalar};
            }
        };

        // Current machine state
        MachineState current_state = {_stator_flux, _rotor_flux, _mechanical_speed};

        // Function to calculate state derivatives
        auto calculate_derivatives = [this, &voltage_ab, load_torque](const MachineState& state) -> MachineState {
            // Calculate electrical rotor speed [rad/s]
            double const omega_r = _pole_pairs * state.mechanical_speed;

            // Calculate stator currents from fluxes [A]
            alphabeta<double> const stator_current_ab =
                (_rotor_inductance * state.stator_flux - _mutual_inductance * state.rotor_flux) *
                _stator_rotor_flux_dividend;

            // Calculate electromagnetic torque [N·m]
            double const electrical_torque =
                (3.0 / 2.0) * _pole_pairs * cross_product(state.rotor_flux, stator_current_ab);

            // Calculate derivatives
            MachineState derivatives{};

            derivatives.stator_flux = voltage_ab - _stator_resistance * stator_current_ab;

            derivatives.rotor_flux.alpha() =
                -(_rotor_resistance / _rotor_inductance) * state.rotor_flux.alpha() +
                (_mutual_inductance * _rotor_resistance / (_rotor_inductance * _stator_inductance)) *
                    (state.stator_flux.alpha() - state.rotor_flux.alpha()) -
                omega_r * state.rotor_flux.beta();

            derivatives.rotor_flux.beta() =
                -(_rotor_resistance / _rotor_inductance) * state.rotor_flux.beta() +
                (_mutual_inductance * _rotor_resistance / (_rotor_inductance * _stator_inductance)) *
                    (state.stator_flux.beta() - state.rotor_flux.beta()) +
                omega_r * state.rotor_flux.alpha();

            derivatives.mechanical_speed =
                (electrical_torque - load_torque - _friction_b * state.mechanical_speed) / _moment_of_inertia_j;

            return derivatives;
        };

        MachineState state_kp1 =
            range_kutta_derivative::calculate<MachineState>(current_state, calculate_derivatives, _dt);

        // Update machine state
        _stator_flux = state_kp1.stator_flux;
        _rotor_flux = state_kp1.rotor_flux;
        _mechanical_speed = state_kp1.mechanical_speed;
    }

    // Parameters (all in SI units)
    double _dt;                     // Time step [s]
    double _stator_resistance{};    // Stator resistance [ohm]
    double _rotor_resistance{};     // Rotor resistance [ohm]
    double _stator_inductance{};    // Stator inductance [H]
    double _rotor_inductance{};     // Rotor inductance [H]
    double _mutual_inductance{};    // Mutual inductance [H]
    std::uint32_t _pole_pairs{1};   // Number of pole pairs
    double _moment_of_inertia_j{};  // Moment of inertia [kg·m²]
    double _friction_b{};           // Viscous friction coefficient [N·m·s/rad]
    double _sigma{1.0};             // Leakage factor: 1 - (L_m²)/(L_s*L_r)

    // State variables in alpha-beta frame
    alphabeta<double> _stator_flux{};  //  [Wb]
    alphabeta<double> _rotor_flux{};   //  [Wb]
    double _mechanical_speed{};        // Mechanical speed [rad/s]

    //
    double _stator_rotor_flux_dividend{};
};

}  // namespace yeet::control