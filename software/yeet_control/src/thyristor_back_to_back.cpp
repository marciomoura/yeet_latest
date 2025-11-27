#include "yeet_control/thyristor_back_to_back.hpp"

#include <algorithm>
#include <iostream>

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
    // Rectifier: Current flows Grid -> Bridge (Into Bridge)
    // Inverter: Current flows Bridge -> Machine (Out of Bridge)
    // Note: thyristor_bridge::calculate_ac_currents returns "Net Current Into Bridge"
    // (Upper - Lower).
    // For Rectifier: This is correct (Grid -> Bridge).
    // For Inverter: This is correct (Bridge -> Machine means Out of Bridge, so Into Bridge is negative).

    // We use the bridge's internal state to get currents
    abc<current_t> i_grid = get_grid_currents();
    abc<current_t> i_mach = get_machine_currents();  // This is current INTO machine (Out of Bridge)

    // 2. Calculate AC terminal voltages
    // Rectifier: V_term = V_grid - Z_grid * I_into_bridge
    //
    // Mathematical Note:
    // We only subtract the resistive drop here. The inductive voltage drop (L * di/dt)
    // is NOT subtracted at this stage because it is a dynamic term that results from
    // the differential equation we solve in Step 9.
    // Effectively, we are calculating the "internal" voltage available to drive
    // the change in current through the total system inductance.
    _v_term_rect = v_grid - _grid_resistance * i_grid;

    // Inverter: V_term = V_mach + Z_mach * I_into_machine
    // I_into_machine = i_mach.
    // So V_term = V_mach + R * i_mach.
    _v_term_inv = v_machine + _machine_resistance * i_mach;

    // 3. Estimate DC rail voltages for forward bias check
    // Rectifier
    voltage_t v_p_rect{0.0};
    voltage_t v_n_rect{0.0};

    bool rect_upper_conducting = false;
    bool rect_lower_conducting = false;
    const auto& rect_states = _rect_bridge.get_conduction_states();
    if (rect_states[0] || rect_states[2] || rect_states[4]) rect_upper_conducting = true;
    if (rect_states[3] || rect_states[5] || rect_states[1]) rect_lower_conducting = true;

    if (_i_dc.value() > 1e-6) {
        if (rect_upper_conducting) {
            v_p_rect = _rect_bridge.get_positive_rail_voltage(_v_term_rect);
        }
        else {
            v_p_rect = voltage_t{-1e9};
        }

        if (rect_lower_conducting) {
            v_n_rect = _rect_bridge.get_negative_rail_voltage(_v_term_rect);
        }
        else {
            v_n_rect = voltage_t{1e9};
        }
    }

    // Inverter
    // Inverter produces negative voltage.
    // V_inv = V_pos - V_neg.
    voltage_t v_p_inv{0.0};
    voltage_t v_n_inv{0.0};

    bool inv_upper_conducting = false;
    bool inv_lower_conducting = false;
    const auto& inv_states = _inv_bridge.get_conduction_states();
    if (inv_states[0] || inv_states[2] || inv_states[4]) inv_upper_conducting = true;
    if (inv_states[3] || inv_states[5] || inv_states[1]) inv_lower_conducting = true;

    if (_i_dc.value() > 1e-6) {
        if (inv_upper_conducting) {
            v_p_inv = _inv_bridge.get_positive_rail_voltage(_v_term_inv);
        }
        else {
            v_p_inv = voltage_t{-1e9};
        }

        if (inv_lower_conducting) {
            v_n_inv = _inv_bridge.get_negative_rail_voltage(_v_term_inv);
        }
        else {
            v_n_inv = voltage_t{1e9};
        }
    }

    // 4. Update Bridge Conduction State
    _rect_bridge.update_conduction_state(_v_term_rect, v_p_rect, v_n_rect, p_rect);
    _inv_bridge.update_conduction_state(_v_term_inv, v_p_inv, v_n_inv, p_inv);

    // 5. Update Commutation
    inductance_t l_comm_rect = _grid_inductance * 2.0;
    inductance_t l_comm_inv = _machine_inductance * 2.0;

    _rect_bridge.update_commutation_state(_i_dc, l_comm_rect, _sampling_time, _v_term_rect);
    _inv_bridge.update_commutation_state(_i_dc, l_comm_inv, _sampling_time, _v_term_inv);

    // 6. Update Thyristor Currents
    _rect_bridge.update_thyristor_currents(_i_dc);
    _inv_bridge.update_thyristor_currents(_i_dc);

    // Enforce physical constraint: If either bridge loses its conducting path (e.g. discontinuous mode),
    // the DC link current must be zero. The inductor cannot sustain current through an open switch.
    if (!_rect_bridge.has_conducting_path() || !_inv_bridge.has_conducting_path()) {
        _i_dc = current_t{0.0};
        _rect_bridge.update_thyristor_currents(_i_dc);
        _inv_bridge.update_thyristor_currents(_i_dc);
    }

    // 7. Force Natural Turn-Off
    // Re-calculate currents for turn-off check
    i_grid = get_grid_currents();
    // For Inverter, we need "Current Into Bridge" for the check logic inside thyristor_bridge
    // get_machine_currents returns Out of Bridge.
    // So we pass -i_mach.
    abc<current_t> i_inv_into_bridge = abc<current_t>{-i_mach.a(), -i_mach.b(), -i_mach.c()};

    _rect_bridge.force_natural_turn_off(i_grid, _i_dc);
    _inv_bridge.force_natural_turn_off(i_inv_into_bridge, _i_dc);

    // 8. Calculate DC Voltages
    _v_rect_dc = _rect_bridge.get_instantaneous_dc_voltage(_v_term_rect);
    // The inverter bridge produces a negative voltage (V_dc < 0) when operating
    // in inverter mode (alpha > 90). This negative voltage opposes the rectifier's
    // positive voltage in the loop.
    _v_inv_dc = _inv_bridge.get_instantaneous_dc_voltage(_v_term_inv);

    // 9. Integrate DC Link Current
    // Loop: V_rect + V_inv - I*R = L di/dt
    // Note: V_inv is negative in inverter mode, so adding it correctly subtracts
    // the counter-EMF magnitude from the rectifier voltage.
    // Example: V_rect = 5000V, V_inv = -4800V. Net = 5000 + (-4800) = 200V.

    // Mathematical Note:
    // The total inductance (l_total) includes the grid and machine inductances
    // because they are in series with the DC link during conduction.
    // When two thyristors conduct (e.g., Phase A and B), the current flows through
    // two line inductances. Hence the factor of 2.0.
    // The equation is:
    // (V_grid_source - I*R_grid) - (V_mach_source + I*R_mach) - I*R_dc = L_total * di/dt
    //
    // Correction: The resistive drops for grid and machine are already subtracted
    // in Step 2 (Calculate AC terminal voltages) to get _v_term_rect and _v_term_inv.
    // Therefore, _v_rect_dc and _v_inv_dc already account for the AC-side resistive drops.
    // We must NOT subtract them again here. Only the DC link resistance remains.
    resistance_t r_total = _dc_resistance;
    inductance_t l_total = _grid_inductance * 2.0 + _dc_inductance + _machine_inductance * 2.0;

    voltage_t v_net = _v_rect_dc + _v_inv_dc - (r_total * _i_dc);

    // Forward Euler
    current_t di_dc = (v_net / l_total) * _sampling_time;
    _i_dc = _i_dc + di_dc;

    // Clamp to zero
    if (_i_dc.value() < 0.0) {
        _i_dc = current_t{0.0};
    }

    // 10. Final thyristor current update with integrated DC current
    // This ensures get_grid_currents() and get_machine_currents() return values
    // consistent with the new _i_dc value.
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
