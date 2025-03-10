#pragma once

#include <cmath>
#include <functional>

namespace yeet::control {

/**
 * @brief
 *
 * The Runge-Kutta 4th order method is a widely used numerical technique for solving
 * ordinary differential equations of the form: dy/dt = f(y, t)
 *
 * This implementation provides a generic way to integrate any system of differential
 * equations represented in state-space form, with accuracy of order O(h⁴).
 *
 * Benefits over simpler methods (like Euler):
 * - Higher accuracy (4th order vs 1st order)
 * - Better numerical stability
 * - Larger allowable time steps with acceptable error
 *
 * Requirements for the state structure to be used with range_kutta_derivative
 *
 * The state structure must provide:
 * 1. Addition operator: state + state -> state
 * 2. Scalar multiplication: state * double -> state
 *
 * Example implementation:
 *
 * struct MachineState {
 *     alphabeta<double> stator_flux{};
 *     alphabeta<double> rotor_flux{};
 *     double mechanical_speed{};
 *
 *     // Addition operator: adds corresponding components of two states
 *     MachineState operator+(const MachineState& other) const {
 *         return {stator_flux + other.stator_flux, rotor_flux + other.rotor_flux,
 *                 mechanical_speed + other.mechanical_speed};
 *     }
 *
 *     // Scalar multiplication: multiplies each component by a scalar
 *     MachineState operator*(const double scalar) const {
 *         return {stator_flux * scalar, rotor_flux * scalar, mechanical_speed * scalar};
 *     }
 * };
 */
class range_kutta_derivative {
public:
    /**
     * @brief Performs a single RK4 integration step on the given system state
     *
     * @tparam StatesT           Type representing the full state of the system
     * @param states             Current system state
     * @param calculate_derivatives Function that computes state derivatives (dy/dt) given a state
     * @param dt                 Integration time step
     * @return StatesT           Updated system state after time step dt
     *
     * Mathematical formulation (for scalar case, extends to vector case):
     * k₁ = f(y_n)
     * k₂ = f(y_n + dt/2 · k₁)
     * k₃ = f(y_n + dt/2 · k₂)
     * k₄ = f(y_n + dt · k₃)
     * y_{n+1} = y_n + dt/6 · (k₁ + 2k₂ + 2k₃ + k₄)
     */
    template <typename StatesT>
    static StatesT calculate(StatesT states, std::function<StatesT(StatesT const&)> calculate_derivatives, double dt) {
        // Store initial state at t_n
        StatesT states_k1 = states;

        // Stage 1: Evaluate derivatives at the initial state
        // k₁ = f(y_n)
        StatesT states_derivative_k1 = calculate_derivatives(states_k1);

        // Stage 2: Evaluate derivatives at the midpoint using k₁ slope
        // k₂ = f(y_n + dt/2 · k₁)
        StatesT states_k2 = states_k1 + states_derivative_k1 * (dt / 2.0);
        StatesT states_derivative_k2 = calculate_derivatives(states_k2);

        // Stage 3: Evaluate derivatives at the midpoint using k₂ slope
        // k₃ = f(y_n + dt/2 · k₂)
        StatesT states_k3 = states_k1 + states_derivative_k2 * (dt / 2.0);
        StatesT states_derivative_k3 = calculate_derivatives(states_k3);

        // Stage 4: Evaluate derivatives at the endpoint using k₃ slope
        // k₄ = f(y_n + dt · k₃)
        StatesT states_k4 = states_k1 + states_derivative_k3 * dt;
        StatesT states_derivative_k4 = calculate_derivatives(states_k4);

        // Combine all derivatives with appropriate weights to get final update
        // y_{n+1} = y_n + dt/6 · (k₁ + 2k₂ + 2k₃ + k₄)
        StatesT states_kp1 = states_k1 + (states_derivative_k1 + states_derivative_k2 * 2.0 +
                                          states_derivative_k3 * 2.0 + states_derivative_k4) *
                                             (dt / 6.0);

        return states_kp1;
    }
};

}  // namespace yeet::control
