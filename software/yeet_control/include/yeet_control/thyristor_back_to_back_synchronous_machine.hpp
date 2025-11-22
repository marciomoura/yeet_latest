#pragma once

#include <algorithm>
#include <array>
#include <cmath>
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
        real_t ts)
        : _grid_resistance(grid_resistance),
          _grid_inductance(grid_inductance),
          _dc_resistance(dc_resistance),
          _dc_inductance(dc_inductance),
          _machine(machine),
          _ts(ts)
    {
        // Extract machine parameters for DC link coupling
        _machine_resistance = _machine.get_stator_resistance().value();

        // Use subtransient inductance because commutation sees the locked-rotor inductance
        const auto subtransient = _machine.get_subtransient_inductances();
        _machine_inductance = 0.5 * (subtransient.d_axis.value() + subtransient.q_axis.value());
    }

    /**
     * @brief Initializes the system states.
     * @param initial_speed Initial rotor speed [rad/s]
     * @param initial_angle Initial rotor angle [rad]
     * @param initial_field_flux Initial field winding flux linkage [Wb] (default: 0)
     */
    void initialize(real_t initial_speed, real_t initial_angle, real_t initial_field_flux = 0.0)
    {
        // Reset bridges
        _rectifier_bridge.reset();
        _inverter_bridge.reset();

        // Reset DC link
        _i_dc = 0.0;

        // Initialize machine state
        _machine_state = synchronous_machine_state{};
        _machine_state.omega_r = initial_speed;
        _machine_state.theta_r = initial_angle;
        _machine_state.psi_fD = initial_field_flux;

        // Initialize damper fluxes to steady-state for zero damper currents
        real_t L_ad = _machine.get_mutual_inductance_d().value();
        real_t L_fD = _machine.get_field_inductance().value();

        if (initial_field_flux != 0.0) {
            // Assuming steady state: i_kD = 0, i_d = 0
            // psi_fD = L_fD * i_f
            // psi_kD = L_ad * i_f = L_ad * (psi_fD / L_fD)
            _machine_state.psi_kD = (L_ad / L_fD) * initial_field_flux;
        }
        _machine_state.psi_kQ = 0.0;
    }

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
        torque_t torque_load)
    {
        // 1. Get currents from previous step
        abc<current_t> i_grid = get_grid_currents();
        abc<current_t> i_motor = get_motor_currents();

        // 2. Calculate terminal voltages
        // Grid: V_term = V_source - R * I
        voltage_t v_rect_a = v_grid_abc.a() - resistance_t(_grid_resistance) * i_grid.a();
        voltage_t v_rect_b = v_grid_abc.b() - resistance_t(_grid_resistance) * i_grid.b();
        voltage_t v_rect_c = v_grid_abc.c() - resistance_t(_grid_resistance) * i_grid.c();
        _v_term_rect = abc<voltage_t>(v_rect_a, v_rect_b, v_rect_c);

        // Machine: V_term = V_back_emf + R * I_motor
        // Note: i_motor is current INTO machine.
        // We need Back-EMF.
        abc<current_t> zero_current(current_t(0.0), current_t(0.0), current_t(0.0));
        _v_back_emf = _machine.get_terminal_voltages_abc(_machine_state, zero_current);

        voltage_t v_inv_a = _v_back_emf.a() + resistance_t(_machine_resistance) * i_motor.a();
        voltage_t v_inv_b = _v_back_emf.b() + resistance_t(_machine_resistance) * i_motor.b();
        voltage_t v_inv_c = _v_back_emf.c() + resistance_t(_machine_resistance) * i_motor.c();
        _v_term_inv = abc<voltage_t>(v_inv_a, v_inv_b, v_inv_c);

        // Update cached motor voltages for telemetry
        _v_motor_abc = _v_term_inv;

        // 3. Estimate DC rails
        voltage_t v_p_rect{0.0}, v_n_rect{0.0};
        if (_i_dc > 1e-6) {
            v_p_rect = voltage_t(_v_rect_dc * 0.5);
            v_n_rect = voltage_t(_v_rect_dc * -0.5);
        }

        voltage_t v_p_inv{0.0}, v_n_inv{0.0};
        if (_i_dc > 1e-6) {
            v_p_inv = voltage_t(_v_inv_dc * 0.5);
            v_n_inv = voltage_t(_v_inv_dc * -0.5);
        }

        // 4. Update Conduction State
        thyristor_pulses p_rect;
        for (int i = 0; i < 6; ++i) p_rect[i] = rect_pulses[i];
        _rectifier_bridge.update_conduction_state(_v_term_rect, v_p_rect, v_n_rect, p_rect);

        thyristor_pulses p_inv;
        for (int i = 0; i < 6; ++i) p_inv[i] = inv_pulses[i];
        _inverter_bridge.update_conduction_state(_v_term_inv, v_p_inv, v_n_inv, p_inv);

        // 5. Update Commutation
        inductance_t l_comm_rect = inductance_t(_grid_inductance * 2.0);
        inductance_t l_comm_inv = inductance_t(_machine_inductance * 2.0);

        _rectifier_bridge.update_commutation_state(current_t(_i_dc), l_comm_rect, duration_t(_ts), _v_term_rect);
        _inverter_bridge.update_commutation_state(current_t(_i_dc), l_comm_inv, duration_t(_ts), _v_term_inv);

        // 6. Update Thyristor Currents
        _rectifier_bridge.update_thyristor_currents(current_t(_i_dc));
        _inverter_bridge.update_thyristor_currents(current_t(_i_dc));

        // 7. Force Natural Turn-Off
        i_grid = get_grid_currents();
        i_motor = get_motor_currents();

        _rectifier_bridge.force_natural_turn_off(i_grid, current_t(_i_dc));

        // Inverter bridge expects current INTO bridge. i_motor is OUT of bridge.
        abc<current_t> i_inv_into_bridge(
            current_t(-i_motor.a().value()), current_t(-i_motor.b().value()), current_t(-i_motor.c().value()));
        _inverter_bridge.force_natural_turn_off(i_inv_into_bridge, current_t(_i_dc));

        // 8. Calculate DC Voltages
        _v_rect_dc = _rectifier_bridge.get_instantaneous_dc_voltage(_v_term_rect).value();
        _v_inv_dc = _inverter_bridge.get_instantaneous_dc_voltage(_v_term_inv).value();

        // 9. Integrate DC Link
        real_t r_total = _grid_resistance * 2.0 + _dc_resistance + _machine_resistance * 2.0;
        real_t l_total = _grid_inductance * 2.0 + _dc_inductance + _machine_inductance * 2.0;

        real_t v_net = _v_rect_dc + _v_inv_dc - (r_total * _i_dc);  // Note: + _v_inv_dc because of flipped topology

        real_t di_dc = (v_net / l_total) * _ts;
        _i_dc += di_dc;
        if (_i_dc < 0.0) _i_dc = 0.0;

        // 10. Integrate Machine
        // Use updated currents
        abc<current_t> i_motor_next = get_motor_currents();

        auto machine_deriv_func = [&](const synchronous_machine_state& state) {
            return _machine.calculate_derivatives_current_fed(state, i_motor_next, v_field, torque_load);
        };
        _machine_state = range_kutta_derivative::calculate(_machine_state, machine_deriv_func, _ts);
    }

    // --- Getters for Telemetry ---

    // Grid Side
    abc<current_t> get_grid_currents() const
    {
        const auto& i_th = _rectifier_bridge.get_thyristor_currents();
        // Phase A: T1 - T4 (Into Bridge)
        return abc<current_t>{i_th[0] - i_th[3], i_th[2] - i_th[5], i_th[4] - i_th[1]};
    }

    current_t get_dc_link_current() const { return current_t(_i_dc); }
    voltage_t get_rectifier_dc_voltage() const { return voltage_t(_v_rect_dc); }

    // Inverter/Motor Side
    voltage_t get_inverter_dc_voltage() const { return voltage_t(_v_inv_dc); }

    abc<current_t> get_motor_currents() const
    {
        const auto& i_th = _inverter_bridge.get_thyristor_currents();
        // Phase A: T1 - T4 is Into Bridge.
        // Machine Current is Out of Bridge (Into Machine).
        // Current flows from DC+ (Top Thyristors) -> Machine -> DC- (Bottom Thyristors)
        // So I_machine = I_top - I_bottom
        // T1, T3, T5 are Top (Cathode to DC+? No, Anode to DC+ in Rectifier, but this is Inverter)
        // In LCI Inverter:
        // DC Current enters bridge.
        // If we assume standard bridge topology:
        // Top (1,3,5) connected to DC+. Bottom (4,6,2) connected to DC-.
        // Current into Phase A = i_T1 - i_T4.
        return abc<current_t>{i_th[0] - i_th[3], i_th[2] - i_th[5], i_th[4] - i_th[1]};
    }

    abc<voltage_t> get_motor_voltages() const { return _v_motor_abc; }

    abc<voltage_t> get_motor_back_emf() const { return _v_back_emf; }

    const synchronous_machine_state& get_machine_state() const { return _machine_state; }
    const std::array<bool, 6>& get_inverter_states() const { return _inverter_bridge.get_conduction_states(); }
    const std::array<bool, 6>& get_rectifier_states() const { return _rectifier_bridge.get_conduction_states(); }

    // AC Terminal Voltages (for voltage notch measurement)
    abc<voltage_t> get_grid_line_voltages() const { return _rectifier_bridge.get_ac_line_voltages(); }
    abc<voltage_t> get_machine_line_voltages() const { return _inverter_bridge.get_ac_line_voltages(); }

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

    // --- Cached Signals ---
    abc<voltage_t> _v_motor_abc{voltage_t(0.0), voltage_t(0.0), voltage_t(0.0)};
    abc<voltage_t> _v_back_emf{voltage_t(0.0), voltage_t(0.0), voltage_t(0.0)};
    mutable real_t _v_rect_dc{0.0};
    mutable real_t _v_inv_dc{0.0};
    abc<voltage_t> _v_term_rect{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
    abc<voltage_t> _v_term_inv{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
};

}  // namespace yeet::control
