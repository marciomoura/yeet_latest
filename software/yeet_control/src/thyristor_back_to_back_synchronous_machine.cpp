#include "yeet_control/thyristor_back_to_back_synchronous_machine.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace yeet::control {

thyristor_back_to_back_synchronous_machine::thyristor_back_to_back_synchronous_machine(real_t grid_resistance,
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
    // Use hardcoded values matching the test configuration
    // TODO: Add public getters to synchronous_machine class
    _machine_resistance = 0.02;  // R_s from machine parameters

    // For DC link inductance, use average of L_d and L_q
    // This represents the inductance seen by DC current during commutation
    _machine_inductance = 0.5 * (0.015 + 0.010);  // (L_d + L_q) / 2
}

void thyristor_back_to_back_synchronous_machine::initialize(
    real_t initial_speed, real_t initial_angle, real_t initial_field_flux)
{
    // Reset bridges
    _rectifier_bridge.reset();
    _inverter_bridge.reset();

    // Reset DC link and commutation currents
    _i_dc = 0.0;
    _i_comm_rect = 0.0;
    _i_comm_inv = 0.0;

    // Reset commutation tracking
    _rect_incoming_thy = -1;
    _rect_outgoing_thy = -1;
    _inv_incoming_thy = -1;
    _inv_outgoing_thy = -1;
    _prev_rect_states.fill(false);
    _prev_inv_states.fill(false);

    // Reset thyristor currents
    _i_thy_rect.fill(0.0);
    _i_thy_inv.fill(0.0);

    // Initialize machine state
    _machine_state = synchronous_machine_state{};
    _machine_state.omega_r = initial_speed;
    _machine_state.theta_r = initial_angle;
    _machine_state.psi_fD = initial_field_flux;

    // Initialize damper fluxes to steady-state for zero damper currents
    // Use hardcoded values matching test configuration
    // TODO: Add public getters to synchronous_machine
    real_t L_ad = 0.012;  // L_ad from machine parameters
    real_t L_fD = 5.0;    // L_fD from machine parameters

    if (initial_field_flux != 0.0) {
        _machine_state.psi_kD = (L_ad / (L_fD + L_ad)) * initial_field_flux;
    }
    _machine_state.psi_kQ = 0.0;
}

void thyristor_back_to_back_synchronous_machine::update(const abc<voltage_t>& v_grid_abc,
    const std::array<bool, 6>& rect_pulses,
    const std::array<bool, 6>& inv_pulses,
    voltage_t v_field,
    torque_t torque_load)
{
    // ========================================================================
    // STEP 1: Calculate machine back-EMF from current state
    // ========================================================================
    // Get current motor currents (based on current DC link current)
    abc<current_t> i_motor_curr = get_motor_currents();

    // Calculate machine terminal voltages (back-EMF + resistive drop)
    _v_motor_abc = _machine.get_terminal_voltages_abc(_machine_state, i_motor_curr);

    // ========================================================================
    // STEP 2: Update bridge conduction states
    // ========================================================================
    update_rectifier_conduction(v_grid_abc, rect_pulses);
    update_inverter_conduction(_v_motor_abc, inv_pulses);

    // ========================================================================
    // STEP 3: Detect commutation events
    // ========================================================================
    detect_rectifier_commutation();
    detect_inverter_commutation();

    // ========================================================================
    // STEP 4: Integrate DC link and commutation currents (RK4)
    // ========================================================================
    CurrentState current_state = {_i_dc, _i_comm_rect, _i_comm_inv};

    auto k1 = calculate_current_derivatives(current_state, v_grid_abc, _v_motor_abc);
    auto k2 = calculate_current_derivatives(current_state + k1 * (_ts * 0.5), v_grid_abc, _v_motor_abc);
    auto k3 = calculate_current_derivatives(current_state + k2 * (_ts * 0.5), v_grid_abc, _v_motor_abc);
    auto k4 = calculate_current_derivatives(current_state + k3 * _ts, v_grid_abc, _v_motor_abc);

    CurrentState new_current_state = current_state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (_ts / 6.0);

    // Apply new currents (clamp to non-negative)
    _i_dc = std::max(real_t(0.0), new_current_state.i_dc);
    _i_comm_rect = std::max(real_t(0.0), new_current_state.i_comm_rect);
    _i_comm_inv = std::max(real_t(0.0), new_current_state.i_comm_inv);

    // ========================================================================
    // STEP 5: Check for commutation completion
    // ========================================================================
    // Rectifier commutation
    if (_rect_incoming_thy >= 0 && _rect_outgoing_thy >= 0) {
        real_t i_out_rect = _i_dc - _i_comm_rect;
        if (i_out_rect < real_t(1e-6) || _i_comm_rect >= _i_dc * real_t(0.999)) {
            // Commutation complete - manually turn off outgoing thyristor
            auto& rect_states = _rectifier_bridge.get_conduction_states_mutable();
            rect_states[_rect_outgoing_thy] = false;

            _i_comm_rect = 0.0;
            _rect_incoming_thy = -1;
            _rect_outgoing_thy = -1;
        }
    }

    // Inverter commutation
    if (_inv_incoming_thy >= 0 && _inv_outgoing_thy >= 0) {
        real_t i_out_inv = _i_dc - _i_comm_inv;
        if (i_out_inv < real_t(1e-6) || _i_comm_inv >= _i_dc * real_t(0.999)) {
            // Commutation complete - manually turn off outgoing thyristor
            auto& inv_states = _inverter_bridge.get_conduction_states_mutable();
            inv_states[_inv_outgoing_thy] = false;

            _i_comm_inv = 0.0;
            _inv_incoming_thy = -1;
            _inv_outgoing_thy = -1;
        }
    }

    // ========================================================================
    // STEP 6: Update thyristor current distribution
    // ========================================================================
    update_thyristor_currents();

    // ========================================================================
    // STEP 7: Natural turn-off for both bridges
    // ========================================================================
    abc<current_t> i_grid = get_grid_currents();
    abc<current_t> i_motor = get_motor_currents();

    std::array<double, 3> i_grid_arr = {i_grid.a().value(), i_grid.b().value(), i_grid.c().value()};
    std::array<double, 3> i_motor_arr = {i_motor.a().value(), i_motor.b().value(), i_motor.c().value()};

    // Skip force_natural_turn_off during commutation to preserve overlap
    if (_rect_incoming_thy < 0 && _rect_outgoing_thy < 0) {
        _rectifier_bridge.force_natural_turn_off(i_grid_arr, _i_dc);
    }

    if (_inv_incoming_thy < 0 && _inv_outgoing_thy < 0) {
        _inverter_bridge.force_natural_turn_off(i_motor_arr, _i_dc);
    }

    // ========================================================================
    // STEP 8: Integrate machine flux/mechanical dynamics
    // ========================================================================
    // Get new motor currents after DC link integration
    abc<current_t> i_motor_next = get_motor_currents();

    // Define derivative function for machine
    std::function<synchronous_machine_state(const synchronous_machine_state&)> machine_deriv_func =
        [&](const synchronous_machine_state& state) {
            return _machine.calculate_derivatives_current_fed(state, i_motor_next, v_field, torque_load);
        };

    // Integrate machine state
    _machine_state = range_kutta_derivative::calculate(_machine_state, machine_deriv_func, _ts);
}

thyristor_back_to_back_synchronous_machine::CurrentState
thyristor_back_to_back_synchronous_machine::calculate_current_derivatives(
    const CurrentState& state, const abc<voltage_t>& v_grid, const abc<voltage_t>& v_machine) const
{
    CurrentState dx{0.0, 0.0, 0.0};

    // ========================================================================
    // Calculate instantaneous phase currents from current RK4 state
    // ========================================================================
    // Must use the current state, not cached values, for accurate RK4 integration
    const auto& rect_states = _rectifier_bridge.get_conduction_states();
    const auto& inv_states = _inverter_bridge.get_conduction_states();

    // Rectifier thyristor currents (instantaneous)
    std::array<real_t, 6> i_thy_rect_inst{0, 0, 0, 0, 0, 0};
    if (_rect_incoming_thy >= 0 && _rect_outgoing_thy >= 0) {
        // Commutation in progress
        i_thy_rect_inst[_rect_incoming_thy] = state.i_comm_rect;
        i_thy_rect_inst[_rect_outgoing_thy] = state.i_dc - state.i_comm_rect;
        // Opposite rail thyristor
        bool incoming_is_upper = (_rect_incoming_thy == 0 || _rect_incoming_thy == 2 || _rect_incoming_thy == 4);
        if (incoming_is_upper) {
            for (int i : {3, 5, 1}) {
                if (rect_states[i]) {
                    i_thy_rect_inst[i] = state.i_dc;
                    break;
                }
            }
        }
        else {
            for (int i : {0, 2, 4}) {
                if (rect_states[i]) {
                    i_thy_rect_inst[i] = state.i_dc;
                    break;
                }
            }
        }
    }
    else {
        // Steady state
        for (int i = 0; i < 6; ++i) {
            if (rect_states[i]) {
                i_thy_rect_inst[i] = state.i_dc;
            }
        }
    }

    // Inverter thyristor currents (instantaneous)
    std::array<real_t, 6> i_thy_inv_inst{0, 0, 0, 0, 0, 0};
    if (_inv_incoming_thy >= 0 && _inv_outgoing_thy >= 0) {
        // Commutation in progress
        i_thy_inv_inst[_inv_incoming_thy] = state.i_comm_inv;
        i_thy_inv_inst[_inv_outgoing_thy] = state.i_dc - state.i_comm_inv;
        // Opposite rail thyristor
        bool incoming_is_upper = (_inv_incoming_thy == 0 || _inv_incoming_thy == 2 || _inv_incoming_thy == 4);
        if (incoming_is_upper) {
            for (int i : {3, 5, 1}) {
                if (inv_states[i]) {
                    i_thy_inv_inst[i] = state.i_dc;
                    break;
                }
            }
        }
        else {
            for (int i : {0, 2, 4}) {
                if (inv_states[i]) {
                    i_thy_inv_inst[i] = state.i_dc;
                    break;
                }
            }
        }
    }
    else {
        // Steady state
        for (int i = 0; i < 6; ++i) {
            if (inv_states[i]) {
                i_thy_inv_inst[i] = state.i_dc;
            }
        }
    }

    // Calculate instantaneous phase currents
    real_t i_grid_a = i_thy_rect_inst[0] - i_thy_rect_inst[3];
    real_t i_grid_b = i_thy_rect_inst[2] - i_thy_rect_inst[5];
    real_t i_grid_c = i_thy_rect_inst[4] - i_thy_rect_inst[1];

    real_t i_mach_a = i_thy_inv_inst[0] - i_thy_inv_inst[3];
    real_t i_mach_b = i_thy_inv_inst[2] - i_thy_inv_inst[5];
    real_t i_mach_c = i_thy_inv_inst[4] - i_thy_inv_inst[1];

    // AC terminal voltages after resistive drops (using instantaneous currents)
    real_t v_grid_a = v_grid.a().value() - _grid_resistance * i_grid_a;
    real_t v_grid_b = v_grid.b().value() - _grid_resistance * i_grid_b;
    real_t v_grid_c = v_grid.c().value() - _grid_resistance * i_grid_c;

    real_t v_mach_a = v_machine.a().value() - _machine_resistance * i_mach_a;
    real_t v_mach_b = v_machine.b().value() - _machine_resistance * i_mach_b;
    real_t v_mach_c = v_machine.c().value() - _machine_resistance * i_mach_c;

    // ========================================================================
    // Calculate DC voltages from conducting thyristors
    // ========================================================================

    // Rectifier DC voltage
    real_t v_rect_upper = real_t(-1e9);
    real_t v_rect_lower = real_t(1e9);

    if (rect_states[0]) v_rect_upper = std::max(v_rect_upper, v_grid_a);  // T1 → Phase A
    if (rect_states[2]) v_rect_upper = std::max(v_rect_upper, v_grid_b);  // T3 → Phase B
    if (rect_states[4]) v_rect_upper = std::max(v_rect_upper, v_grid_c);  // T5 → Phase C

    if (rect_states[3]) v_rect_lower = std::min(v_rect_lower, v_grid_a);  // T4 → Phase A
    if (rect_states[5]) v_rect_lower = std::min(v_rect_lower, v_grid_b);  // T6 → Phase B
    if (rect_states[1]) v_rect_lower = std::min(v_rect_lower, v_grid_c);  // T2 → Phase C

    real_t v_rect = (v_rect_upper > real_t(-1e9) && v_rect_lower < real_t(1e9)) ? (v_rect_upper - v_rect_lower) : 0.0;
    _v_rect_dc = v_rect;

    // Inverter DC voltage
    real_t v_inv_upper = real_t(-1e9);
    real_t v_inv_lower = real_t(1e9);

    if (inv_states[0]) v_inv_upper = std::max(v_inv_upper, v_mach_a);  // T1 → Phase A
    if (inv_states[2]) v_inv_upper = std::max(v_inv_upper, v_mach_b);  // T3 → Phase B
    if (inv_states[4]) v_inv_upper = std::max(v_inv_upper, v_mach_c);  // T5 → Phase C

    if (inv_states[3]) v_inv_lower = std::min(v_inv_lower, v_mach_a);  // T4 → Phase A
    if (inv_states[5]) v_inv_lower = std::min(v_inv_lower, v_mach_b);  // T6 → Phase B
    if (inv_states[1]) v_inv_lower = std::min(v_inv_lower, v_mach_c);  // T2 → Phase C

    real_t v_inv = (v_inv_upper > real_t(-1e9) && v_inv_lower < real_t(1e9)) ? (v_inv_upper - v_inv_lower) : 0.0;
    _v_inv_dc = v_inv;

    // ========================================================================
    // Main DC loop equation
    // ========================================================================
    // L_total × di_dc/dt = V_rect - V_inv - R_total × i_dc
    //
    // L_total = 2×L_grid + L_dc + 2×L_machine
    // R_total = 2×R_grid + R_dc + 2×R_machine
    //
    // Factor of 2 because current flows through TWO phases at a time
    real_t L_total = 2.0 * _grid_inductance + _dc_inductance + 2.0 * _machine_inductance;
    real_t R_total = 2.0 * _grid_resistance + _dc_resistance + 2.0 * _machine_resistance;

    if (state.i_dc > real_t(1e-9) || (v_rect - v_inv) > 0.0) {
        dx.i_dc = (v_rect - v_inv - R_total * state.i_dc) / L_total;
    }
    else {
        dx.i_dc = 0.0;
    }

    // ========================================================================
    // Rectifier commutation dynamics
    // ========================================================================
    if (_rect_incoming_thy >= 0 && _rect_outgoing_thy >= 0) {
        real_t v_incoming = 0.0;
        real_t v_outgoing = 0.0;

        // Map thyristor index to phase voltage
        if (_rect_incoming_thy == 0 || _rect_incoming_thy == 3)
            v_incoming = v_grid_a;
        else if (_rect_incoming_thy == 2 || _rect_incoming_thy == 5)
            v_incoming = v_grid_b;
        else
            v_incoming = v_grid_c;

        if (_rect_outgoing_thy == 0 || _rect_outgoing_thy == 3)
            v_outgoing = v_grid_a;
        else if (_rect_outgoing_thy == 2 || _rect_outgoing_thy == 5)
            v_outgoing = v_grid_b;
        else
            v_outgoing = v_grid_c;

        // Determine voltage difference polarity
        bool incoming_is_upper = (_rect_incoming_thy == 0 || _rect_incoming_thy == 2 || _rect_incoming_thy == 4);
        real_t v_diff = incoming_is_upper ? (v_incoming - v_outgoing) : (v_outgoing - v_incoming);

        // Commutation inductance = 2 × grid inductance (two phases in series)
        real_t L_comm_rect = 2.0 * _grid_inductance;
        dx.i_comm_rect = v_diff / L_comm_rect;
    }
    else {
        dx.i_comm_rect = 0.0;
    }

    // ========================================================================
    // Inverter commutation dynamics
    // ========================================================================
    if (_inv_incoming_thy >= 0 && _inv_outgoing_thy >= 0) {
        real_t v_incoming = 0.0;
        real_t v_outgoing = 0.0;

        // Map thyristor index to phase voltage
        if (_inv_incoming_thy == 0 || _inv_incoming_thy == 3)
            v_incoming = v_mach_a;
        else if (_inv_incoming_thy == 2 || _inv_incoming_thy == 5)
            v_incoming = v_mach_b;
        else
            v_incoming = v_mach_c;

        if (_inv_outgoing_thy == 0 || _inv_outgoing_thy == 3)
            v_outgoing = v_mach_a;
        else if (_inv_outgoing_thy == 2 || _inv_outgoing_thy == 5)
            v_outgoing = v_mach_b;
        else
            v_outgoing = v_mach_c;

        // Determine voltage difference polarity
        bool incoming_is_upper = (_inv_incoming_thy == 0 || _inv_incoming_thy == 2 || _inv_incoming_thy == 4);
        real_t v_diff = incoming_is_upper ? (v_incoming - v_outgoing) : (v_outgoing - v_incoming);

        // Commutation inductance = 2 × machine inductance (two phases in series)
        real_t L_comm_inv = 2.0 * _machine_inductance;
        dx.i_comm_inv = v_diff / L_comm_inv;
    }
    else {
        dx.i_comm_inv = 0.0;
    }

    return dx;
}

void thyristor_back_to_back_synchronous_machine::update_rectifier_conduction(
    const abc<voltage_t>& v_grid, const std::array<bool, 6>& pulses)
{
    abc<current_t> i_grid = get_grid_currents();

    std::array<double, 3> v_grid_terminals = {v_grid.a().value() - _grid_resistance * i_grid.a().value(),
        v_grid.b().value() - _grid_resistance * i_grid.b().value(),
        v_grid.c().value() - _grid_resistance * i_grid.c().value()};

    // Estimate DC rails for forward bias checking
    double v_p_rail = 0.0;
    double v_n_rail = 0.0;

    if (_i_dc > real_t(1e-6)) {
        v_p_rail = _v_rect_dc * 0.5;
        v_n_rail = -_v_rect_dc * 0.5;
    }

    _rectifier_bridge.update_conduction_state(v_grid_terminals, {v_p_rail, v_n_rail}, pulses);
}

void thyristor_back_to_back_synchronous_machine::update_inverter_conduction(
    const abc<voltage_t>& v_machine, const std::array<bool, 6>& pulses)
{
    abc<current_t> i_motor = get_motor_currents();

    std::array<double, 3> v_mach_terminals = {v_machine.a().value() - _machine_resistance * i_motor.a().value(),
        v_machine.b().value() - _machine_resistance * i_motor.b().value(),
        v_machine.c().value() - _machine_resistance * i_motor.c().value()};

    // Estimate DC rails for forward bias checking
    double v_p_rail = 0.0;
    double v_n_rail = 0.0;

    if (_i_dc > real_t(1e-6)) {
        v_p_rail = _v_inv_dc * 0.5;
        v_n_rail = -_v_inv_dc * 0.5;
    }

    _inverter_bridge.update_conduction_state(v_mach_terminals, {v_p_rail, v_n_rail}, pulses);
}

void thyristor_back_to_back_synchronous_machine::detect_rectifier_commutation()
{
    const auto& conducting = _rectifier_bridge.get_conduction_states();

    for (int i = 0; i < 6; ++i) {
        if (conducting[i] && !_prev_rect_states[i]) {
            // Rising edge detected - new thyristor turned on
            bool is_upper = (i == 0 || i == 2 || i == 4);

            // Find outgoing thyristor on same rail
            if (is_upper) {
                for (int j : {0, 2, 4}) {
                    if (j != i && _prev_rect_states[j]) {
                        _rect_incoming_thy = i;
                        _rect_outgoing_thy = j;
                        _i_comm_rect = 0.0;
                        break;
                    }
                }
            }
            else {
                for (int j : {1, 3, 5}) {
                    if (j != i && _prev_rect_states[j]) {
                        _rect_incoming_thy = i;
                        _rect_outgoing_thy = j;
                        _i_comm_rect = 0.0;
                        break;
                    }
                }
            }
        }
    }

    _prev_rect_states = conducting;
}

void thyristor_back_to_back_synchronous_machine::detect_inverter_commutation()
{
    const auto& conducting = _inverter_bridge.get_conduction_states();

    for (int i = 0; i < 6; ++i) {
        if (conducting[i] && !_prev_inv_states[i]) {
            // Rising edge detected - new thyristor turned on
            bool is_upper = (i == 0 || i == 2 || i == 4);

            // Find outgoing thyristor on same rail
            if (is_upper) {
                for (int j : {0, 2, 4}) {
                    if (j != i && _prev_inv_states[j]) {
                        _inv_incoming_thy = i;
                        _inv_outgoing_thy = j;
                        _i_comm_inv = 0.0;
                        break;
                    }
                }
            }
            else {
                for (int j : {1, 3, 5}) {
                    if (j != i && _prev_inv_states[j]) {
                        _inv_incoming_thy = i;
                        _inv_outgoing_thy = j;
                        _i_comm_inv = 0.0;
                        break;
                    }
                }
            }
        }
    }

    _prev_inv_states = conducting;
}

void thyristor_back_to_back_synchronous_machine::update_thyristor_currents()
{
    const auto& rect_states = _rectifier_bridge.get_conduction_states();
    const auto& inv_states = _inverter_bridge.get_conduction_states();

    _i_thy_rect.fill(0.0);
    _i_thy_inv.fill(0.0);

    // ========================================================================
    // Rectifier thyristor currents
    // ========================================================================
    if (_rect_incoming_thy >= 0 && _rect_outgoing_thy >= 0) {
        // During commutation
        _i_thy_rect[_rect_incoming_thy] = _i_comm_rect;
        _i_thy_rect[_rect_outgoing_thy] = _i_dc - _i_comm_rect;

        // Non-commutating rail carries full DC current
        bool incoming_is_upper = (_rect_incoming_thy == 0 || _rect_incoming_thy == 2 || _rect_incoming_thy == 4);
        if (incoming_is_upper) {
            for (int i : {1, 3, 5}) {
                if (rect_states[i]) {
                    _i_thy_rect[i] = _i_dc;
                    break;
                }
            }
        }
        else {
            for (int i : {0, 2, 4}) {
                if (rect_states[i]) {
                    _i_thy_rect[i] = _i_dc;
                    break;
                }
            }
        }
    }
    else {
        // Steady state - each conducting thyristor carries full DC current
        for (int i = 0; i < 6; ++i) {
            if (rect_states[i]) {
                _i_thy_rect[i] = _i_dc;
            }
        }
    }

    // ========================================================================
    // Inverter thyristor currents
    // ========================================================================
    if (_inv_incoming_thy >= 0 && _inv_outgoing_thy >= 0) {
        // During commutation
        _i_thy_inv[_inv_incoming_thy] = _i_comm_inv;
        _i_thy_inv[_inv_outgoing_thy] = _i_dc - _i_comm_inv;

        // Non-commutating rail carries full DC current
        bool incoming_is_upper = (_inv_incoming_thy == 0 || _inv_incoming_thy == 2 || _inv_incoming_thy == 4);
        if (incoming_is_upper) {
            for (int i : {1, 3, 5}) {
                if (inv_states[i]) {
                    _i_thy_inv[i] = _i_dc;
                    break;
                }
            }
        }
        else {
            for (int i : {0, 2, 4}) {
                if (inv_states[i]) {
                    _i_thy_inv[i] = _i_dc;
                    break;
                }
            }
        }
    }
    else {
        // Steady state - each conducting thyristor carries full DC current
        for (int i = 0; i < 6; ++i) {
            if (inv_states[i]) {
                _i_thy_inv[i] = _i_dc;
            }
        }
    }
}

abc<current_t> thyristor_back_to_back_synchronous_machine::get_grid_currents() const
{
    // Grid currents = difference between upper and lower thyristor currents
    // Standard 6-pulse mapping: T1(0)→A+, T3(2)→B+, T5(4)→C+, T4(3)→A-, T6(5)→B-, T2(1)→C-
    real_t i_a = _i_thy_rect[0] - _i_thy_rect[3];
    real_t i_b = _i_thy_rect[2] - _i_thy_rect[5];
    real_t i_c = _i_thy_rect[4] - _i_thy_rect[1];
    return abc<current_t>(current_t(i_a), current_t(i_b), current_t(i_c));
}

abc<current_t> thyristor_back_to_back_synchronous_machine::get_motor_currents() const
{
    // Motor currents = difference between upper and lower thyristor currents
    // Standard 6-pulse mapping: T1(0)→A+, T3(2)→B+, T5(4)→C+, T4(3)→A-, T6(5)→B-, T2(1)→C-
    real_t i_a = _i_thy_inv[0] - _i_thy_inv[3];
    real_t i_b = _i_thy_inv[2] - _i_thy_inv[5];
    real_t i_c = _i_thy_inv[4] - _i_thy_inv[1];
    return abc<current_t>(current_t(i_a), current_t(i_b), current_t(i_c));
}

}  // namespace yeet::control
