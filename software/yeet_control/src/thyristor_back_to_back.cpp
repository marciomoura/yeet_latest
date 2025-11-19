#include "yeet_control/thyristor_back_to_back.hpp"

#include <algorithm>
#include <cmath>

namespace yeet::control {

thyristor_back_to_back::thyristor_back_to_back(
    double grid_res, double grid_ind, double dc_res, double dc_ind, double mach_res, double mach_ind, double ts)
    : _grid_resistance(grid_res),
      _grid_inductance(grid_ind),
      _dc_resistance(dc_res),
      _dc_inductance(dc_ind),
      _machine_resistance(mach_res),
      _machine_inductance(mach_ind),
      _sampling_time(ts)
{
}

void thyristor_back_to_back::reset()
{
    _i_dc = 0.0;
    _i_comm_rect = 0.0;
    _i_comm_inv = 0.0;
    _outgoing_rect = -1;
    _incoming_rect = -1;
    _outgoing_inv = -1;
    _incoming_inv = -1;
    _i_thy_rect.fill(0.0);
    _i_thy_inv.fill(0.0);
    _v_rect_dc = 0.0;
    _v_inv_dc = 0.0;
    _rect_bridge.reset();
    _inv_bridge.reset();
}

void thyristor_back_to_back::update(const abc<double>& v_grid,
    const abc<double>& v_machine,
    const std::array<bool, 6>& p_rect,
    const std::array<bool, 6>& p_inv)
{
    // Step 1: Update rectifier bridge conduction
    update_rectifier_conduction(v_grid, p_rect);

    // Step 2: Update inverter bridge conduction
    update_inverter_conduction(v_machine, p_inv);

    // Step 3: Detect commutation events
    detect_rectifier_commutation();
    detect_inverter_commutation();

    // Step 4: RK4 integration of currents
    State current_state = {_i_dc, _i_comm_rect, _i_comm_inv};

    auto k1 = calculate_derivatives(current_state, v_grid, v_machine);
    auto k2 = calculate_derivatives(current_state + k1 * (_sampling_time * 0.5), v_grid, v_machine);
    auto k3 = calculate_derivatives(current_state + k2 * (_sampling_time * 0.5), v_grid, v_machine);
    auto k4 = calculate_derivatives(current_state + k3 * _sampling_time, v_grid, v_machine);

    State new_state = current_state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (_sampling_time / 6.0);

    // Apply new state - clamp to non-negative
    _i_dc = std::max(0.0, new_state.i_dc);
    _i_comm_rect = std::max(0.0, new_state.i_comm_rect);
    _i_comm_inv = std::max(0.0, new_state.i_comm_inv);

    // Step 5: Check for commutation completion - Rectifier
    if (_outgoing_rect >= 0 && _incoming_rect >= 0) {
        double i_out_rect = _i_dc - _i_comm_rect;
        if (i_out_rect < 1e-6 || _i_comm_rect >= _i_dc * 0.999) {
            _i_comm_rect = 0.0;
            _outgoing_rect = -1;
            _incoming_rect = -1;
        }
    }

    // Step 6: Check for commutation completion - Inverter
    if (_outgoing_inv >= 0 && _incoming_inv >= 0) {
        double i_out_inv = _i_dc - _i_comm_inv;
        if (i_out_inv < 1e-6 || _i_comm_inv >= _i_dc * 0.999) {
            _i_comm_inv = 0.0;
            _outgoing_inv = -1;
            _incoming_inv = -1;
        }
    }

    // Step 7: Update thyristor currents
    update_thyristor_currents();

    // Step 8: Natural turn-off for both bridges
    abc<double> i_grid = get_grid_currents();
    abc<double> i_mach = get_machine_currents();

    std::array<double, 3> i_grid_arr = {i_grid.a(), i_grid.b(), i_grid.c()};
    std::array<double, 3> i_mach_arr = {i_mach.a(), i_mach.b(), i_mach.c()};

    _rect_bridge.force_natural_turn_off(i_grid_arr, _i_dc);
    _inv_bridge.force_natural_turn_off(i_mach_arr, _i_dc);
}

abc<double> thyristor_back_to_back::get_grid_currents() const
{
    double i_a = _i_thy_rect[0] - _i_thy_rect[3];
    double i_b = _i_thy_rect[2] - _i_thy_rect[5];
    double i_c = _i_thy_rect[4] - _i_thy_rect[1];
    return abc<double>(i_a, i_b, i_c);
}

abc<double> thyristor_back_to_back::get_machine_currents() const
{
    double i_a = _i_thy_inv[0] - _i_thy_inv[3];
    double i_b = _i_thy_inv[2] - _i_thy_inv[5];
    double i_c = _i_thy_inv[4] - _i_thy_inv[1];
    return abc<double>(i_a, i_b, i_c);
}

thyristor_back_to_back::State thyristor_back_to_back::calculate_derivatives(
    const State& state, const abc<double>& vg, const abc<double>& vm) const
{
    State dx{0.0, 0.0, 0.0};

    // Calculate phase currents for voltage drop estimation
    abc<double> i_grid = get_grid_currents();
    abc<double> i_mach = get_machine_currents();

    // AC terminal voltages after impedance drops
    double v_grid_a = vg.a() - _grid_resistance * i_grid.a();
    double v_grid_b = vg.b() - _grid_resistance * i_grid.b();
    double v_grid_c = vg.c() - _grid_resistance * i_grid.c();

    double v_mach_a = vm.a() - _machine_resistance * i_mach.a();
    double v_mach_b = vm.b() - _machine_resistance * i_mach.b();
    double v_mach_c = vm.c() - _machine_resistance * i_mach.c();

    // Find DC voltages from conducting thyristors
    const auto& rect_states = _rect_bridge.get_conduction_states();
    const auto& inv_states = _inv_bridge.get_conduction_states();

    // Rectifier DC voltage
    double v_rect_upper = -1e9;
    double v_rect_lower = 1e9;

    if (rect_states[0]) v_rect_upper = v_grid_a;  // T1
    if (rect_states[2]) v_rect_upper = v_grid_b;  // T3
    if (rect_states[4]) v_rect_upper = v_grid_c;  // T5

    if (rect_states[3]) v_rect_lower = v_grid_a;  // T4
    if (rect_states[5]) v_rect_lower = v_grid_b;  // T6
    if (rect_states[1]) v_rect_lower = v_grid_c;  // T2

    double v_rect = (v_rect_upper > -1e9 && v_rect_lower < 1e9) ? (v_rect_upper - v_rect_lower) : 0.0;
    _v_rect_dc = v_rect;

    // Inverter DC voltage
    double v_inv_upper = -1e9;
    double v_inv_lower = 1e9;

    if (inv_states[0]) v_inv_upper = v_mach_a;  // T1
    if (inv_states[2]) v_inv_upper = v_mach_b;  // T3
    if (inv_states[4]) v_inv_upper = v_mach_c;  // T5

    if (inv_states[3]) v_inv_lower = v_mach_a;  // T4
    if (inv_states[5]) v_inv_lower = v_mach_b;  // T6
    if (inv_states[1]) v_inv_lower = v_mach_c;  // T2

    double v_inv = (v_inv_upper > -1e9 && v_inv_lower < 1e9) ? (v_inv_upper - v_inv_lower) : 0.0;
    _v_inv_dc = v_inv;

    // Main DC loop equation
    // L_total * di_dc/dt = V_rect - V_inv - R_total * i_dc
    double L_total = 2.0 * _grid_inductance + _dc_inductance + 2.0 * _machine_inductance;
    double R_total = 2.0 * _grid_resistance + _dc_resistance + 2.0 * _machine_resistance;

    if (state.i_dc > 1e-9 || (v_rect - v_inv) > 0.0) {
        dx.i_dc = (v_rect - v_inv - R_total * state.i_dc) / L_total;
    }
    else {
        dx.i_dc = 0.0;
    }

    // Rectifier commutation dynamics
    if (_incoming_rect >= 0 && _outgoing_rect >= 0) {
        double v_incoming = 0.0;
        double v_outgoing = 0.0;

        // Map thyristor to phase voltage
        if (_incoming_rect == 0 || _incoming_rect == 3)
            v_incoming = v_grid_a;
        else if (_incoming_rect == 2 || _incoming_rect == 5)
            v_incoming = v_grid_b;
        else
            v_incoming = v_grid_c;

        if (_outgoing_rect == 0 || _outgoing_rect == 3)
            v_outgoing = v_grid_a;
        else if (_outgoing_rect == 2 || _outgoing_rect == 5)
            v_outgoing = v_grid_b;
        else
            v_outgoing = v_grid_c;

        bool incoming_is_upper = (_incoming_rect == 0 || _incoming_rect == 2 || _incoming_rect == 4);
        double v_diff = incoming_is_upper ? (v_incoming - v_outgoing) : (v_outgoing - v_incoming);

        double L_comm = 2.0 * _grid_inductance;
        dx.i_comm_rect = v_diff / L_comm;
    }
    else {
        dx.i_comm_rect = 0.0;
    }

    // Inverter commutation dynamics
    if (_incoming_inv >= 0 && _outgoing_inv >= 0) {
        double v_incoming = 0.0;
        double v_outgoing = 0.0;

        // Map thyristor to phase voltage
        if (_incoming_inv == 0 || _incoming_inv == 3)
            v_incoming = v_mach_a;
        else if (_incoming_inv == 2 || _incoming_inv == 5)
            v_incoming = v_mach_b;
        else
            v_incoming = v_mach_c;

        if (_outgoing_inv == 0 || _outgoing_inv == 3)
            v_outgoing = v_mach_a;
        else if (_outgoing_inv == 2 || _outgoing_inv == 5)
            v_outgoing = v_mach_b;
        else
            v_outgoing = v_mach_c;

        bool incoming_is_upper = (_incoming_inv == 0 || _incoming_inv == 2 || _incoming_inv == 4);
        double v_diff = incoming_is_upper ? (v_incoming - v_outgoing) : (v_outgoing - v_incoming);

        double L_comm = 2.0 * _machine_inductance;
        dx.i_comm_inv = v_diff / L_comm;
    }
    else {
        dx.i_comm_inv = 0.0;
    }

    return dx;
}

void thyristor_back_to_back::update_rectifier_conduction(const abc<double>& v_grid, const std::array<bool, 6>& pulses)
{
    abc<double> i_grid = get_grid_currents();

    std::array<double, 3> v_grid_terminals = {v_grid.a() - _grid_resistance * i_grid.a(),
        v_grid.b() - _grid_resistance * i_grid.b(), v_grid.c() - _grid_resistance * i_grid.c()};

    double v_p_rail = 0.0;
    double v_n_rail = 0.0;

    if (_i_dc > 1e-6) {
        v_p_rail = _v_rect_dc * 0.5;
        v_n_rail = -_v_rect_dc * 0.5;
    }

    _rect_bridge.update_conduction_state(v_grid_terminals, {v_p_rail, v_n_rail}, pulses);
}

void thyristor_back_to_back::update_inverter_conduction(const abc<double>& v_machine, const std::array<bool, 6>& pulses)
{
    abc<double> i_mach = get_machine_currents();

    std::array<double, 3> v_mach_terminals = {v_machine.a() - _machine_resistance * i_mach.a(),
        v_machine.b() - _machine_resistance * i_mach.b(), v_machine.c() - _machine_resistance * i_mach.c()};

    double v_p_rail = 0.0;
    double v_n_rail = 0.0;

    if (_i_dc > 1e-6) {
        v_p_rail = _v_inv_dc * 0.5;
        v_n_rail = -_v_inv_dc * 0.5;
    }

    _inv_bridge.update_conduction_state(v_mach_terminals, {v_p_rail, v_n_rail}, pulses);
}

void thyristor_back_to_back::detect_rectifier_commutation()
{
    const auto& conducting = _rect_bridge.get_conduction_states();
    static std::array<bool, 6> prev_rect = {false, false, false, false, false, false};

    for (int i = 0; i < 6; ++i) {
        if (conducting[i] && !prev_rect[i]) {
            bool is_upper = (i == 0 || i == 2 || i == 4);

            if (is_upper) {
                for (int j : {0, 2, 4}) {
                    if (j != i && prev_rect[j]) {
                        _incoming_rect = i;
                        _outgoing_rect = j;
                        _i_comm_rect = 0.0;
                        break;
                    }
                }
            }
            else {
                for (int j : {1, 3, 5}) {
                    if (j != i && prev_rect[j]) {
                        _incoming_rect = i;
                        _outgoing_rect = j;
                        _i_comm_rect = 0.0;
                        break;
                    }
                }
            }
        }
    }

    prev_rect = conducting;
}

void thyristor_back_to_back::detect_inverter_commutation()
{
    const auto& conducting = _inv_bridge.get_conduction_states();
    static std::array<bool, 6> prev_inv = {false, false, false, false, false, false};

    for (int i = 0; i < 6; ++i) {
        if (conducting[i] && !prev_inv[i]) {
            bool is_upper = (i == 0 || i == 2 || i == 4);

            if (is_upper) {
                for (int j : {0, 2, 4}) {
                    if (j != i && prev_inv[j]) {
                        _incoming_inv = i;
                        _outgoing_inv = j;
                        _i_comm_inv = 0.0;
                        break;
                    }
                }
            }
            else {
                for (int j : {1, 3, 5}) {
                    if (j != i && prev_inv[j]) {
                        _incoming_inv = i;
                        _outgoing_inv = j;
                        _i_comm_inv = 0.0;
                        break;
                    }
                }
            }
        }
    }

    prev_inv = conducting;
}

void thyristor_back_to_back::update_thyristor_currents()
{
    const auto& rect_states = _rect_bridge.get_conduction_states();
    const auto& inv_states = _inv_bridge.get_conduction_states();

    _i_thy_rect.fill(0.0);
    _i_thy_inv.fill(0.0);

    // Rectifier thyristor currents
    if (_outgoing_rect >= 0 && _incoming_rect >= 0) {
        _i_thy_rect[_incoming_rect] = _i_comm_rect;
        _i_thy_rect[_outgoing_rect] = _i_dc - _i_comm_rect;

        bool incoming_is_upper = (_incoming_rect == 0 || _incoming_rect == 2 || _incoming_rect == 4);
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
        for (int i = 0; i < 6; ++i) {
            if (rect_states[i]) {
                _i_thy_rect[i] = _i_dc;
            }
        }
    }

    // Inverter thyristor currents
    if (_outgoing_inv >= 0 && _incoming_inv >= 0) {
        _i_thy_inv[_incoming_inv] = _i_comm_inv;
        _i_thy_inv[_outgoing_inv] = _i_dc - _i_comm_inv;

        bool incoming_is_upper = (_incoming_inv == 0 || _incoming_inv == 2 || _incoming_inv == 4);
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
        for (int i = 0; i < 6; ++i) {
            if (inv_states[i]) {
                _i_thy_inv[i] = _i_dc;
            }
        }
    }
}

}  // namespace yeet::control
