#include "yeet_control/thyristor_back_to_back.hpp"

#include <algorithm>

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
    _v_term_rect.a() = v_grid.a() - _grid_resistance * i_grid.a();
    _v_term_rect.b() = v_grid.b() - _grid_resistance * i_grid.b();
    _v_term_rect.c() = v_grid.c() - _grid_resistance * i_grid.c();

    // Inverter: V_term = V_mach + Z_mach * I_into_machine
    // I_into_machine = i_mach.
    // So V_term = V_mach + R * i_mach.
    _v_term_inv.a() = v_machine.a() + _machine_resistance * i_mach.a();
    _v_term_inv.b() = v_machine.b() + _machine_resistance * i_mach.b();
    _v_term_inv.c() = v_machine.c() + _machine_resistance * i_mach.c();

    // 3. Estimate DC rail voltages for forward bias check
    // Rectifier
    voltage_t v_p_rect{0.0};
    voltage_t v_n_rect{0.0};
    if (_i_dc.value() > 1e-6) {
        v_p_rect = _v_rect_dc * 0.5;
        v_n_rect = _v_rect_dc * -0.5;
    }

    // Inverter
    // Inverter produces negative voltage.
    // V_inv = V_pos - V_neg.
    voltage_t v_p_inv{0.0};
    voltage_t v_n_inv{0.0};
    if (_i_dc.value() > 1e-6) {
        v_p_inv = _v_inv_dc * 0.5;
        v_n_inv = _v_inv_dc * -0.5;
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
    // Invert the inverter voltage so that it represents a counter-voltage (load)
    // when positive. This allows the loop equation (V_rect - V_inv) to correctly
    // represent the voltage difference driving the current.
    _v_inv_dc = -_inv_bridge.get_instantaneous_dc_voltage(_v_term_inv);

    // 9. Integrate DC Link Current
    // Loop: V_rect - V_inv - I*R = L di/dt
    // (V_inv opposes V_rect in standard back-to-back connection)

    // Mathematical Note:
    // The total inductance (l_total) includes the grid and machine inductances
    // because they are in series with the DC link during conduction.
    // When two thyristors conduct (e.g., Phase A and B), the current flows through
    // two line inductances. Hence the factor of 2.0.
    // The equation is:
    // (V_grid_source - I*R_grid) - (V_mach_source + I*R_mach) - I*R_dc = L_total * di/dt
    resistance_t r_total = _grid_resistance * 2.0 + _dc_resistance + _machine_resistance * 2.0;
    inductance_t l_total = _grid_inductance * 2.0 + _dc_inductance + _machine_inductance * 2.0;

    voltage_t v_net = _v_rect_dc - _v_inv_dc - (r_total * _i_dc);

    // Forward Euler
    current_t di_dc = (v_net / l_total) * _sampling_time;
    _i_dc = _i_dc + di_dc;

    // Clamp to zero
    if (_i_dc.value() < 0.0) {
        _i_dc = current_t{0.0};
    }
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
