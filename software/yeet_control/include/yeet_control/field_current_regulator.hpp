#pragma once

#include "pi_controller.hpp"
#include "types.hpp"
#include "units.hpp"

namespace yeet::control {

/**
 * @brief Field current regulator for synchronous machine excitation systems.
 *
 * This component models a closed-loop field current controller that regulates the actual
 * field current to match a commanded reference. It accounts for:
 * - Field winding resistance and inductance (L/R time constant)
 * - PI control to reject disturbances and eliminate steady-state error
 * - Voltage limits representing exciter capability
 *
 * The field circuit is modeled as: τ_f × di_f/dt + R_f × i_f = v_f
 * Where:
 *   τ_f = L_f / R_f  (field time constant, typically 0.5-5 seconds for LCI machines)
 *   R_f = field winding resistance
 *   L_f = field winding inductance
 *   v_f = exciter output voltage
 *   i_f = field current
 *
 * The PI controller generates v_f to drive i_f toward the reference.
 */
class field_current_regulator {
public:
    /**
     * @brief Constructs the field current regulator.
     * @param sampling_time The control loop sampling time [s].
     */
    explicit field_current_regulator(duration_t sampling_time);

    /**
     * @brief Configures the field winding electrical parameters.
     *
     * @param field_resistance Field winding resistance in per-unit [pu].
     * @param field_time_constant Field winding L/R time constant [s].
     *        Typical values: 0.5s (small machines) to 5s (large turbogenerators).
     */
    void configure_field_circuit(resistance_pu_t field_resistance, duration_t field_time_constant);

    /**
     * @brief Configures the PI controller gains.
     *
     * @param kp Proportional gain [pu/pu].
     * @param ki Integral gain [pu/(pu*s)].
     */
    void configure_pi_gains(real_t kp, real_t ti);

    /**
     * @brief Configures the exciter voltage limits.
     *
     * @param min_voltage_pu Minimum exciter output voltage [pu]. Typically 0.0.
     * @param max_voltage_pu Maximum exciter output voltage [pu]. Typically 1.5-2.0.
     */
    void configure_voltage_limits(voltage_pu_t min_voltage_pu, voltage_pu_t max_voltage_pu);

    /**
     * @brief Updates the field current regulator for one cycle.
     *
     * This method:
     * 1. Runs the PI controller to compute exciter voltage command
     * 2. Updates the field circuit dynamics to compute actual field current
     * 3. Returns the calculated field current for use in flux/voltage calculations
     *
     * @param field_current_reference The commanded field current [pu].
     * @return The actual field current after dynamics [pu].
     */
    current_pu_t update(current_pu_t field_current_reference) noexcept;

    /**
     * @brief Resets the regulator to initial conditions.
     * @param initial_field_current Initial field current [pu]. Defaults to 0.0.
     */
    void reset(current_pu_t initial_field_current = current_pu_t{0.0});

    /**
     * @brief Gets the current field current output.
     * @return Field current [pu].
     */
    current_pu_t get_field_current() const noexcept;

    /**
     * @brief Gets the current exciter voltage command.
     * @return Exciter voltage output [pu].
     */
    voltage_pu_t get_exciter_voltage() const noexcept;

private:
    const duration_t _sampling_time;

    // Field circuit parameters
    resistance_pu_t _field_resistance{0.01};  // Typical field resistance
    duration_t _field_time_constant{2.5};     // Typical L/R time constant

    // PI controller for field current regulation
    pi_controller<real_t> _pi_controller;

    // Voltage limits
    voltage_pu_t _min_voltage{0.0};
    voltage_pu_t _max_voltage{2.0};

    // State variables
    current_pu_t _field_current{0.0};    // Actual field current
    voltage_pu_t _exciter_voltage{0.0};  // Exciter output voltage

    /**
     * @brief Updates the field circuit dynamics given exciter voltage.
     * @param exciter_voltage Applied exciter voltage [pu].
     * @return New field current [pu].
     */
    current_pu_t update_field_circuit_dynamics(voltage_pu_t exciter_voltage) noexcept;
};

}  // namespace yeet::control
