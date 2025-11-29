#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>

#include "yeet_control/range_kutta_derivative.hpp"
#include "yeet_control/synchronous_machine.hpp"
#include "yeet_control/thyristor_back_to_back.hpp"
#include "yeet_control/types.hpp"

namespace yeet::control {

/**
 * @brief Complete LCI (Load Commutated Inverter) Drive Topology.
 *
 * This class models the full system:
 * Grid -> Thyristor Rectifier -> DC Link -> Thyristor Inverter -> Synchronous Machine
 *
 * Architecture:
 * - Uses thyristor_back_to_back for the electrical converter model
 * - Machine flux/mechanical dynamics integrated separately based on phase currents
 */
class thyristor_back_to_back_synchronous_machine {
public:
    /**
     * @brief Constructs the full drive system.
     *
     * @param grid_resistance   Grid equivalent series resistance per phase [Ohms]
     * @param grid_inductance   Grid equivalent series inductance per phase [Henry]
     * @param dc_resistance     DC link resistance [Ohms]
     * @param dc_inductance     DC link inductance [Henry]
     * @param machine           Configured synchronous_machine instance (motor side)
     * @param ts                Sampling time [s]
     */
    thyristor_back_to_back_synchronous_machine(real_t grid_resistance,
        real_t grid_inductance,
        real_t dc_resistance,
        real_t dc_inductance,
        const synchronous_machine& machine,
        real_t ts);

    /**
     * @brief Initializes the system states.
     * @param initial_speed Initial rotor speed [rad/s]
     * @param initial_angle Initial rotor angle [rad]
     * @param initial_field_flux Initial field winding flux linkage [Wb] (default: 0)
     */
    void initialize(real_t initial_speed, real_t initial_angle, real_t initial_field_flux = 0.0);

    /**
     * @brief Executes one simulation step of the full system.
     *
     * @param v_grid_abc       Grid source voltages [V]
     * @param rect_pulses      Gate pulses for Rectifier (Grid side)
     * @param inv_pulses       Gate pulses for Inverter (Motor side)
     * @param v_field          Machine field excitation voltage [V]
     * @param torque_load      Mechanical load torque [Nm]
     */
    void update(const abc<voltage_t>& v_grid_abc,
        const std::array<bool, 6>& rect_pulses,
        const std::array<bool, 6>& inv_pulses,
        voltage_t v_field,
        torque_t torque_load);

    // --- Getters for Telemetry ---

    // Grid Side
    abc<current_t> get_grid_currents() const;
    current_t get_dc_link_current() const;
    voltage_t get_rectifier_dc_voltage() const;

    // Inverter/Motor Side
    voltage_t get_inverter_dc_voltage() const;
    abc<current_t> get_motor_currents() const;
    abc<voltage_t> get_motor_voltages() const;
    abc<voltage_t> get_motor_back_emf() const;

    const synchronous_machine_state& get_machine_state() const;
    const std::array<bool, 6>& get_inverter_states() const;
    const std::array<bool, 6>& get_rectifier_states() const;

    // AC Terminal Voltages (for voltage notch measurement)
    abc<voltage_t> get_grid_line_voltages() const;
    abc<voltage_t> get_machine_line_voltages() const;

private:
    // --- Circuit Parameters ---
    synchronous_machine _machine;  ///< Handles Motor flux/mechanical dynamics
    real_t _ts;

    // --- Sub-Components ---
    thyristor_back_to_back _converter;  ///< Handles Electrical Converter Physics

    // --- States ---
    synchronous_machine_state _machine_state;  ///< Motor state (fluxes, speed, angle)

    // --- Cached Signals ---
    abc<voltage_t> _v_back_emf{voltage_t(0.0), voltage_t(0.0), voltage_t(0.0)};
};

}  // namespace yeet::control
