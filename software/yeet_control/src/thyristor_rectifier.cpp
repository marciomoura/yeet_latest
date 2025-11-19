#include "yeet_control/thyristor_rectifier.hpp"

#include <algorithm>

namespace yeet::control {

thyristor_rectifier::thyristor_rectifier(double grid_resistance,
    double grid_inductance,
    double load_resistance,
    double load_inductance,
    double sampling_time)
    : _grid_resistance(grid_resistance),
      _grid_inductance(grid_inductance),
      _load_resistance(load_resistance),
      _load_inductance(load_inductance),
      _sampling_time(sampling_time)
{
}

void thyristor_rectifier::update(
    const abc<double>& v_ac_source, const std::array<bool, 6>& pulses, double v_dc_back_emf)
{
    // Step 1: Update thyristor turn-on based on gate pulses and forward bias
    // This must happen before integration to determine which thyristors conduct this time step
    update_bridge_conduction(v_ac_source, pulses);

    // Step 2: RK4 integration (uses current bridge state, does not modify it)
    State current_state = {_i_dc};
    auto k1 = calculate_derivatives(current_state, v_ac_source, v_dc_back_emf);
    auto k2 = calculate_derivatives(current_state + k1 * (_sampling_time * 0.5), v_ac_source, v_dc_back_emf);
    auto k3 = calculate_derivatives(current_state + k2 * (_sampling_time * 0.5), v_ac_source, v_dc_back_emf);
    auto k4 = calculate_derivatives(current_state + k3 * _sampling_time, v_ac_source, v_dc_back_emf);

    // Update state: y_new = y_old + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
    State new_state = current_state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (_sampling_time / 6.0);

    // Apply state (discontinuous mode: prevent negative current)
    _i_dc = std::max(0.0, new_state.i_dc);

    // Step 3: Check for natural commutation (thyristor turn-off) based on NEW current
    // Must happen after integration so we use the updated current for commutation detection
    abc<double> i_grid_currents = get_grid_currents();
    std::array<double, 3> i_ac_array = {i_grid_currents.a(), i_grid_currents.b(), i_grid_currents.c()};
    _bridge.force_natural_turn_off(i_ac_array, _i_dc);
}

void thyristor_rectifier::reset()
{
    _i_dc = 0.0;
    _v_dc_output = 0.0;
    _bridge.reset();
}

abc<double> thyristor_rectifier::get_grid_currents() const noexcept
{
    // AC currents are determined by DC current and thyristor conduction states
    std::array<double, 3> i_ac = _bridge.calculate_ac_currents(_i_dc);
    return abc<double>(i_ac[0], i_ac[1], i_ac[2]);
}

thyristor_rectifier::State thyristor_rectifier::calculate_derivatives(
    const State& state, const abc<double>& v_ac_source, double v_dc_back_emf)
{
    // Query current bridge conduction state (read-only, does not modify)
    const auto& conduction_states = _bridge.get_conduction_states();

    // Get AC currents based on DC current and thyristor states
    std::array<double, 3> i_ac = _bridge.calculate_ac_currents(state.i_dc);

    // Calculate voltage drops across grid impedances
    abc<double> v_grid_drop(_grid_resistance * i_ac[0], _grid_resistance * i_ac[1], _grid_resistance * i_ac[2]);

    // AC terminal voltages after grid impedance
    abc<double> v_ac_terminals(
        v_ac_source.a() - v_grid_drop.a(), v_ac_source.b() - v_grid_drop.b(), v_ac_source.c() - v_grid_drop.c());

    // Determine DC rail voltages based on thyristor conduction
    double v_p_rail = -1e9;  // Positive DC rail
    double v_n_rail = 1e9;   // Negative DC rail

    if (conduction_states[0]) v_p_rail = std::max(v_p_rail, v_ac_terminals.a());  // T1
    if (conduction_states[2]) v_p_rail = std::max(v_p_rail, v_ac_terminals.b());  // T3
    if (conduction_states[4]) v_p_rail = std::max(v_p_rail, v_ac_terminals.c());  // T5
    if (conduction_states[3]) v_n_rail = std::min(v_n_rail, v_ac_terminals.a());  // T4
    if (conduction_states[5]) v_n_rail = std::min(v_n_rail, v_ac_terminals.b());  // T6
    if (conduction_states[1]) v_n_rail = std::min(v_n_rail, v_ac_terminals.c());  // T2

    // DC voltage output (store for getter) - this is AFTER grid impedance
    _v_dc_output = (v_p_rail < -1e8 || v_n_rail > 1e8) ? 0.0 : v_p_rail - v_n_rail;

    // Total inductance in series path (2 grid phases + DC load)
    // Total L = 2*L_grid + L_load
    // Note: Grid resistance drop already accounted for in v_ac_terminals/v_dc_output
    double total_inductance = 2.0 * _grid_inductance + _load_inductance;

    // DC current derivative: L_total * di_dc/dt = v_bridge_dc - R_load*i_dc - v_dc_back_emf
    State derivatives{};
    if (state.i_dc <= 1e-9 && (_v_dc_output - v_dc_back_emf) < 0.0) {
        // Discontinuous mode: clamp derivative to zero
        derivatives.i_dc = 0.0;
    }
    else {
        derivatives.i_dc = (_v_dc_output - _load_resistance * state.i_dc - v_dc_back_emf) / total_inductance;
    }

    return derivatives;
}

void thyristor_rectifier::update_bridge_conduction(const abc<double>& v_ac_source, const std::array<bool, 6>& pulses)
{
    // Get AC currents based on current DC current and thyristor states
    std::array<double, 3> i_ac = _bridge.calculate_ac_currents(_i_dc);

    // Estimate AC terminal voltages after grid impedance
    std::array<double, 3> v_ac_terminals = {v_ac_source.a() - _grid_resistance * i_ac[0],
        v_ac_source.b() - _grid_resistance * i_ac[1], v_ac_source.c() - _grid_resistance * i_ac[2]};

    // Estimate DC rail voltages
    double v_p_rail = -1e9;
    double v_n_rail = 1e9;
    const auto& states = _bridge.get_conduction_states();

    if (states[0]) v_p_rail = std::max(v_p_rail, v_ac_terminals[0]);
    if (states[2]) v_p_rail = std::max(v_p_rail, v_ac_terminals[1]);
    if (states[4]) v_p_rail = std::max(v_p_rail, v_ac_terminals[2]);
    if (states[3]) v_n_rail = std::min(v_n_rail, v_ac_terminals[0]);
    if (states[5]) v_n_rail = std::min(v_n_rail, v_ac_terminals[1]);
    if (states[1]) v_n_rail = std::min(v_n_rail, v_ac_terminals[2]);

    // Update thyristor conduction
    _bridge.update_conduction_state(v_ac_terminals, {v_p_rail, v_n_rail}, pulses);
}

}  // namespace yeet::control
