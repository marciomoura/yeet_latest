#include "yeet_control/thyristor_back_to_back.hpp"

#include <algorithm>
#include <iostream>

namespace {
constexpr double kMinConductionCurrent = 1e-3;
constexpr double kMinBackEmfForNaturalCommutation = 0.05;
constexpr double kOpenCircuitVoltage = 1e9;
}  // namespace

namespace yeet::control {

thyristor_back_to_back::thyristor_back_to_back(resistance_t grid_res,
    inductance_t grid_ind,
    resistance_t dc_res,
    inductance_t dc_ind,
    resistance_t mach_res,
    inductance_t mach_ind,
    duration_t ts)
    : _grid_resistance(grid_res),
      _grid_inductance(grid_ind),
      _dc_resistance(dc_res),
      _dc_inductance(dc_ind),
      _machine_resistance(mach_res),
      _machine_inductance(mach_ind),
      _sampling_time(ts)
{
}

void thyristor_back_to_back::update(const abc<voltage_t>& v_grid,
    const abc<voltage_t>& v_machine,
    const thyristor_pulses& p_rect,
    const thyristor_pulses& p_inv)
{
    // 1. Calculate AC currents from previous step's thyristor currents
    abc<current_t> i_grid = get_grid_currents();
    abc<current_t> i_mach = get_machine_currents();

    // 2. Calculate AC terminal voltages
    _v_term_rect = v_grid - _grid_resistance * i_grid;
    _v_term_inv = v_machine + _machine_resistance * i_mach;

    // 3. Estimate DC rail voltages for forward bias check (Rectifier)
    voltage_t v_p_rect{0.0};
    voltage_t v_n_rect{0.0};

    bool rect_upper_conducting = false;
    bool rect_lower_conducting = false;
    const auto& rect_states = _rect_bridge.get_conduction_states();
    if (rect_states[0] || rect_states[2] || rect_states[4]) rect_upper_conducting = true;
    if (rect_states[3] || rect_states[5] || rect_states[1]) rect_lower_conducting = true;

    if (_i_dc.value() > kMinConductionCurrent) {
        if (rect_upper_conducting) {
            v_p_rect = _rect_bridge.get_positive_rail_voltage(_v_term_rect);
        }
        else {
            v_p_rect = voltage_t{-kOpenCircuitVoltage};
        }

        if (rect_lower_conducting) {
            v_n_rect = _rect_bridge.get_negative_rail_voltage(_v_term_rect);
        }
        else {
            v_n_rect = voltage_t{kOpenCircuitVoltage};
        }
    }
    else {
        v_p_rect = voltage_t{-kOpenCircuitVoltage};
        v_n_rect = voltage_t{kOpenCircuitVoltage};
    }

    // Update Rectifier Conduction State
    _rect_bridge.update_conduction_state(_v_term_rect, v_p_rect, v_n_rect, p_rect);

    // Get Rectifier Potentials for Inverter Bias
    // If rectifier is conducting, these are the potentials applied to the DC link.
    // If not conducting, we assume the potentials are 0 (or open), but usually
    // the rectifier turns on first.
    voltage_t rect_pos_pot = _rect_bridge.get_positive_rail_voltage(_v_term_rect);
    voltage_t rect_neg_pot = _rect_bridge.get_negative_rail_voltage(_v_term_rect);

    // 4. Estimate DC rail voltages for forward bias check (Inverter)
    // Inverter is connected in "reverse" topology to allow DC -> AC power flow.
    // DC Link (+) connects to Inverter Anode Rail (Lower Thyristors).
    // DC Link (-) connects to Inverter Cathode Rail (Upper Thyristors).
    voltage_t v_p_inv{0.0};  // DC Link (+) potential at Inverter input
    voltage_t v_n_inv{0.0};  // DC Link (-) potential (Ground/Return)

    bool inv_upper_conducting = false;  // Connected to DC- (Cathodes)
    bool inv_lower_conducting = false;  // Connected to DC+ (Anodes)
    const auto& inv_states = _inv_bridge.get_conduction_states();
    if (inv_states[0] || inv_states[2] || inv_states[4]) inv_upper_conducting = true;
    if (inv_states[3] || inv_states[5] || inv_states[1]) inv_lower_conducting = true;

    // Determine potential at Inverter DC+ terminal (connected to Anode Rail)
    if (inv_lower_conducting) {
        // Clamped to AC phase via conducting lower thyristor
        v_p_inv = _inv_bridge.get_negative_rail_voltage(_v_term_inv);
    }
    else {
        // Not conducting: sees Rectifier DC+ potential (ignoring R/L drops for bias check)
        v_p_inv = rect_pos_pot;
    }

    // Determine potential at Inverter DC- terminal (connected to Cathode Rail)
    if (inv_upper_conducting) {
        // Clamped to AC phase via conducting upper thyristor
        v_n_inv = _inv_bridge.get_positive_rail_voltage(_v_term_inv);
    }
    else {
        // Not conducting: sees Rectifier DC- potential
        v_n_inv = rect_neg_pot;
    }

    // Update Inverter Conduction State
    // SWAPPED ARGUMENTS: v_dc_pos param gets v_n_inv (Top Rail), v_dc_neg param gets v_p_inv (Bottom Rail).
    // This maps DC+ to Anodes and DC- to Cathodes.
    _inv_bridge.update_conduction_state(_v_term_inv, v_n_inv, v_p_inv, p_inv);

    // 5. Update Commutation
    inductance_t l_comm_rect = _grid_inductance * 2.0;
    inductance_t l_comm_inv = _machine_inductance * 2.0;

    _rect_bridge.update_commutation_state(_i_dc, l_comm_rect, _sampling_time, _v_term_rect);
    _inv_bridge.update_commutation_state(_i_dc, l_comm_inv, _sampling_time, _v_term_inv);

    // 6. Update Thyristor Currents
    _rect_bridge.update_thyristor_currents(_i_dc);
    _inv_bridge.update_thyristor_currents(_i_dc);

    // Enforce physical constraint: If either bridge loses its conducting path, current is zero.
    if (!_rect_bridge.has_conducting_path() || !_inv_bridge.has_conducting_path()) {
        _i_dc = current_t{0.0};
        _rect_bridge.update_thyristor_currents(_i_dc);
        _inv_bridge.update_thyristor_currents(_i_dc);
    }

    // 7. Force Natural Turn-Off
    i_grid = get_grid_currents();
    // For Inverter, we need "Current Into Bridge" for the check logic.
    // get_machine_currents returns Out of Bridge.
    abc<current_t> i_inv_into_bridge = abc<current_t>{-i_mach.a(), -i_mach.b(), -i_mach.c()};

    _rect_bridge.force_natural_turn_off(i_grid, _i_dc);
    _inv_bridge.force_natural_turn_off(i_inv_into_bridge, _i_dc);

    // 8. Calculate DC Voltages
    _v_rect_dc = _rect_bridge.get_instantaneous_dc_voltage(_v_term_rect);

    // Inverter DC voltage:
    // With swapped inputs (Cathode=0, Anode=V_p), get_instantaneous_dc_voltage returns (0 - V_p).
    // So it returns a negative value representing -(Voltage Drop).
    _v_inv_dc = _inv_bridge.get_instantaneous_dc_voltage(_v_term_inv);

    // 9. Integrate DC Link Current
    // Loop: V_rect - V_drop_inv - I*R = L di/dt
    // V_drop_inv = -_v_inv_dc
    // So: V_rect + _v_inv_dc - I*R = L di/dt
    if (_rect_bridge.has_conducting_path() && _inv_bridge.has_conducting_path()) {
        resistance_t r_total = _dc_resistance;
        inductance_t l_total = _grid_inductance * 2.0 + _dc_inductance + _machine_inductance * 2.0;

        voltage_t v_net = _v_rect_dc + _v_inv_dc - (r_total * _i_dc);

        // Forward Euler
        current_t di_dc = (v_net / l_total) * _sampling_time;
        _i_dc = _i_dc + di_dc;

        if (_i_dc.value() < 0.0) {
            _i_dc = current_t{0.0};
        }
    }
    else {
        _i_dc = current_t{0.0};
    }

    // 10. Final thyristor current update
    _rect_bridge.update_thyristor_currents(_i_dc);
    _inv_bridge.update_thyristor_currents(_i_dc);
}

void thyristor_back_to_back::reset()
{
    _i_dc = current_t{0.0};
    _v_rect_dc = voltage_t{0.0};
    _v_inv_dc = voltage_t{0.0};
    _rect_bridge.reset();
    _inv_bridge.reset();
}

abc<current_t> thyristor_back_to_back::get_grid_currents() const
{
    const auto& i_th = _rect_bridge.get_thyristor_currents();
    // Phase A: T1 - T4 (Into Bridge)
    return abc<current_t>{i_th[0] - i_th[3], i_th[2] - i_th[5], i_th[4] - i_th[1]};
}

abc<current_t> thyristor_back_to_back::get_machine_currents() const
{
    const auto& i_th = _inv_bridge.get_thyristor_currents();
    // Phase A: T1 - T4 is Into Bridge.
    // Machine Current is Out of Bridge.
    // So -(T1 - T4) = T4 - T1.
    return abc<current_t>{i_th[3] - i_th[0], i_th[5] - i_th[2], i_th[1] - i_th[4]};
}

}  // namespace yeet::control
