#pragma once

#include "yeet_control/integrator.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Models a mechanical load using a per-unit (pu) system with proper units.
 *
 * This class implements a first-order model of a mechanical system with speed-dependent
 * load torque, commonly used for motor drive simulations. The model is described by the
 * per-unit equation of motion:
 *
 *   2H * d(omega_pu)/dt + D * omega_pu = T_e_pu - T_l_total_pu
 *
 * where the total load torque includes constant, linear, and quadratic components:
 *
 *   T_l_total_pu = T_l_external_pu + k1 * omega_pu + k2 * omega_pu^2
 *
 * Parameters:
 * - H is the inertia constant [s].
 * - D is the damping factor [pu] (velocity-dependent friction).
 * - omega_pu is the mechanical angular speed [pu].
 * - T_e_pu is the electromagnetic torque from the motor [pu].
 * - T_l_external_pu is the external constant load torque [pu].
 * - k1 is the linear load coefficient [pu/pu] (viscous losses, windage).
 * - k2 is the quadratic load coefficient [pu/pu^2] (aerodynamic drag, fan/pump loads).
 *
 * Typical applications:
 * - k1 > 0, k2 = 0: Fan/blower at low speed (linear airflow resistance)
 * - k1 = 0, k2 > 0: Centrifugal pump/compressor (torque ∝ speed²)
 * - k1 > 0, k2 > 0: Gas turbine compressor (combined viscous + aerodynamic)
 *
 * The per-unit speed is calculated by integrating the per-unit angular acceleration.
 */
class mechanical_load {
public:
    /**
     * @brief Constructs the mechanical load model.
     * @param sampling_time The simulation sampling time [s].
     */
    explicit mechanical_load(double sampling_time);

    /**
     * @brief Configures the per-unit mechanical parameters of the load.
     * @param inertia_constant The inertia constant (H). Must be positive.
     * @param damping_factor The per-unit damping factor (D). Must be non-negative.
     * @param linear_load_coeff Linear load coefficient k1 [pu/pu]. Defaults to 0.0.
     * @param quadratic_load_coeff Quadratic load coefficient k2 [pu/pu^2]. Defaults to 0.0.
     */
    void configure(inertia_constant_t inertia_constant_seconds,
        damping_coefficient_pu_t damping_factor_pu,
        real_t linear_load_coeff = 0.0,
        real_t quadratic_load_coeff = 0.0);

    /**
     * @brief Executes one simulation step of the mechanical load.
     * @param electromagnetic_torque The input electromagnetic torque.
     * @param load_torque The external load torque (optional, defaults to 0).
     * @return The updated mechanical speed.
     */
    speed_pu_t update(torque_pu_t electromagnetic_torque_pu, torque_pu_t load_torque = torque_pu_t{0.0});

    /**
     * @brief Resets the mechanical load with a specific speed.
     * @param mechanical_speed_pu The mechanical speed [pu]. Defaults to 0.0.
     */
    void reset(speed_pu_t mechanical_speed_pu = speed_pu_t{0.0});

    /**
     * @brief Gets the current mechanical speed.
     * @return The mechanical speed.
     */
    speed_pu_t get_speed_pu() const noexcept;

private:
    integrator<real_t> _speed_integrator;
    inertia_constant_t _inertia_constant_seconds{1.0};
    damping_coefficient_pu_t _damping_factor_pu{0.0};
    real_t _linear_load_coefficient{0.0};     ///< k1: Linear load term [pu/pu]
    real_t _quadratic_load_coefficient{0.0};  ///< k2: Quadratic load term [pu/pu^2]
};

}  // namespace yeet::control