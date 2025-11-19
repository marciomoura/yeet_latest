#pragma once

#include <array>
#include <functional>

#include "yeet_control/range_kutta_derivative.hpp"
#include "yeet_control/synchronous_machine.hpp"
#include "yeet_control/thyristor_bridge.hpp"
#include "yeet_control/types.hpp"

namespace yeet::control {

/**
 * @brief Complete LCI (Load Commutated Inverter) Drive Topology.
 *
 * This class models the full system:
 * Grid -> Thyristor Rectifier -> DC Link -> Thyristor Inverter -> Synchronous Machine
 *
 * Architecture:
 * - Unified DC link equation couples grid-side and machine-side dynamics
 * - Grid inductance and machine inductance are BOTH part of the DC link total inductance
 * - Commutation on grid-side uses grid voltage differences
 * - Commutation on machine-side uses machine back-EMF voltage differences
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
    current_t get_dc_link_current() const { return current_t(_i_dc); }
    voltage_t get_rectifier_dc_voltage() const { return voltage_t(_v_rect_dc); }

    // Inverter/Motor Side
    voltage_t get_inverter_dc_voltage() const { return voltage_t(_v_inv_dc); }
    abc<current_t> get_motor_currents() const;
    abc<voltage_t> get_motor_voltages() const { return _v_motor_abc; }
    abc<voltage_t> get_motor_back_emf() const;  // Pure back-EMF without resistive drop
    const synchronous_machine_state& get_machine_state() const { return _machine_state; }
    const std::array<bool, 6>& get_inverter_states() const { return _inverter_bridge.get_conduction_states(); }
    const std::array<bool, 6>& get_rectifier_states() const { return _rectifier_bridge.get_conduction_states(); }

    // AC Terminal Voltages (for voltage notch measurement)
    const std::array<double, 3>& get_grid_line_voltages() const { return _rectifier_bridge.get_ac_line_voltages(); }
    const std::array<double, 3>& get_machine_line_voltages() const { return _inverter_bridge.get_ac_line_voltages(); }

private:
    // --- Circuit Parameters ---
    real_t _grid_resistance;
    real_t _grid_inductance;
    real_t _dc_resistance;
    real_t _dc_inductance;
    real_t _machine_resistance;  // Stator resistance from machine model
    real_t _machine_inductance;  // Average machine inductance for DC link coupling
    real_t _ts;

    // --- Sub-Components ---
    thyristor_bridge _rectifier_bridge;  ///< Handles Rectifier switching logic
    thyristor_bridge _inverter_bridge;   ///< Handles Inverter switching logic
    synchronous_machine _machine;        ///< Handles Motor flux/mechanical dynamics

    // --- States ---
    synchronous_machine_state _machine_state;  ///< Motor state (fluxes, speed, angle)
    real_t _i_dc{0.0};                         ///< DC link current [A]
    real_t _i_comm_rect{0.0};                  ///< Rectifier commutation current [A]
    real_t _i_comm_inv{0.0};                   ///< Inverter commutation current [A]

    // --- Rectifier Commutation Tracking ---
    int _rect_incoming_thy{-1};               ///< Index of incoming rectifier thyristor (0-5)
    int _rect_outgoing_thy{-1};               ///< Index of outgoing rectifier thyristor (0-5)
    std::array<bool, 6> _prev_rect_states{};  ///< Previous rectifier states for edge detection

    // --- Inverter Commutation Tracking ---
    int _inv_incoming_thy{-1};               ///< Index of incoming inverter thyristor (0-5)
    int _inv_outgoing_thy{-1};               ///< Index of outgoing inverter thyristor (0-5)
    std::array<bool, 6> _prev_inv_states{};  ///< Previous inverter states for edge detection

    // --- Cached Signals ---
    abc<voltage_t> _v_motor_abc{voltage_t(0.0), voltage_t(0.0), voltage_t(0.0)};
    mutable real_t _v_rect_dc{0.0};
    mutable real_t _v_inv_dc{0.0};

    // --- Thyristor Current Arrays ---
    std::array<real_t, 6> _i_thy_rect{};
    std::array<real_t, 6> _i_thy_inv{};

    // --- Internal State for RK4 Integration ---
    struct CurrentState {
        real_t i_dc;
        real_t i_comm_rect;
        real_t i_comm_inv;

        CurrentState operator+(const CurrentState& other) const
        {
            return {i_dc + other.i_dc, i_comm_rect + other.i_comm_rect, i_comm_inv + other.i_comm_inv};
        }

        CurrentState operator*(real_t scalar) const
        {
            return {i_dc * scalar, i_comm_rect * scalar, i_comm_inv * scalar};
        }
    };

    // --- Helper Methods ---

    /**
     * @brief Calculates derivatives for DC link and commutation currents (RK4 integration).
     */
    CurrentState calculate_current_derivatives(
        const CurrentState& state, const abc<voltage_t>& v_grid, const abc<voltage_t>& v_machine) const;

    /**
     * @brief Updates rectifier bridge conduction based on grid voltages and pulses.
     */
    void update_rectifier_conduction(const abc<voltage_t>& v_grid, const std::array<bool, 6>& pulses);

    /**
     * @brief Updates inverter bridge conduction based on machine voltages and pulses.
     */
    void update_inverter_conduction(const abc<voltage_t>& v_machine, const std::array<bool, 6>& pulses);

    /**
     * @brief Detects if a new rectifier commutation event has started.
     */
    void detect_rectifier_commutation();

    /**
     * @brief Detects if a new inverter commutation event has started.
     */
    void detect_inverter_commutation();

    /**
     * @brief Updates thyristor current arrays based on DC current and commutation states.
     */
    void update_thyristor_currents();
};

}  // namespace yeet::control