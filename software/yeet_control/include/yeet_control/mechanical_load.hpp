#pragma once

#include "yeet_control/integrator.hpp"

namespace yeet::control {

/**
 * @brief Models a linear mechanical load using a per-unit (pu) system.
 *
 * This class implements a first-order linear model of a mechanical system,
 * commonly used for motor drive simulations. The model is described by the
 * per-unit equation of motion:
 *
 *   2H * d(omega_pu)/dt + D * omega_pu = T_e_pu - T_l_pu
 *
 * where:
 * - H is the inertia constant [s].
 * - D is the damping factor [pu].
 * - omega_pu is the mechanical angular speed [pu].
 * - T_e_pu is the electromagnetic torque from the motor [pu].
 * - T_l_pu is the external load torque [pu].
 *
 * The per-unit speed is calculated by integrating the per-unit angular acceleration.
 */
class mechanical_load {
public:
    /**
     * @brief Constructs the mechanical load model.
     * @param ts The simulation sampling time [s].
     */
    explicit mechanical_load(double ts);

    /**
     * @brief Configures the per-unit mechanical parameters of the load.
     * @param inertia_constant_seconds The inertia constant (H) [s]. Must be positive.
     * @param damping_factor_pu The per-unit damping factor (D) [pu]. Must be non-negative.
     */
    void configure(double inertia_constant_seconds, double damping_factor_pu);

    /**
     * @brief Executes one simulation step of the mechanical load.
     * @param electromagnetic_torque_pu The input electromagnetic torque [pu].
     * @param load_torque_pu The external load torque [pu] (optional, defaults to 0).
     * @return The updated mechanical speed [pu].
     */
    double update(double electromagnetic_torque_pu, double load_torque_pu = 0.0);

    /**
     * @brief Resets or the mechanical load with a specific speed.
     * @param mechanical_speed_pu The mechanical speed [pu]. Defaults to 0.0.
     */
    void reset(double mechanical_speed_pu = 0.0);

    /**
     * @brief Gets the current mechanical speed.
     * @return The mechanical speed [pu].
     */
    double get_speed_pu() const;

private:
    integrator<double> _speed_integrator;
    double _inertia_constant_seconds{1.0};
    double _damping_factor_pu{0.0};
};

}  // namespace yeet::control