#include "yeet_control/thyristor_inverter.hpp"

#include <algorithm>

namespace yeet::control {

thyristor_inverter::thyristor_inverter(resistance_t dc_link_resistance,
    inductance_t dc_link_inductance,
    resistance_t ac_resistance,
    inductance_t ac_inductance,
    duration_t sampling_time)
    : _dc_link_resistance(dc_link_resistance),
      _dc_link_inductance(dc_link_inductance),
      _ac_resistance(ac_resistance),
      _ac_inductance(ac_inductance),
      _sampling_time(sampling_time)
{
}

void thyristor_inverter::update(voltage_t v_dc_source, const thyristor_pulses& pulses, const abc<voltage_t>& v_ac_emf)
{
    // 1. Calculate AC currents from previous step's thyristor currents
    // Current flows OUT of the bridge into the AC network
    abc<current_t> i_ac = get_ac_currents();

    // 2. Calculate AC terminal voltages
    // V_term = V_emf + R_ac * I_ac (Inductive drop ignored for conduction check approximation)
    abc<voltage_t> v_term_abc;
    v_term_abc.a() = v_ac_emf.a() + _ac_resistance * i_ac.a();
    v_term_abc.b() = v_ac_emf.b() + _ac_resistance * i_ac.b();
    v_term_abc.c() = v_ac_emf.c() + _ac_resistance * i_ac.c();

    // 3. Estimate DC rail voltages for forward bias check
    voltage_t v_p_rail{0.0};
    voltage_t v_n_rail{0.0};

    if (_i_dc.value() > 1e-6) {
        v_p_rail = _v_bridge_dc * 0.5;
        v_n_rail = _v_bridge_dc * -0.5;
    }
    else {
        // If not conducting, the DC source voltage appears across the bridge
        // Assume symmetric distribution for bias check
        v_p_rail = v_dc_source * 0.5;
        v_n_rail = v_dc_source * -0.5;
    }

    // 4. Update Bridge Conduction State
    _bridge.update_conduction_state(v_term_abc, v_p_rail, v_n_rail, pulses);

    // 5. Update Commutation
    // Commutation inductance is 2 * L_ac (line-to-line)
    inductance_t l_comm = _ac_inductance * 2.0;
    _bridge.update_commutation_state(_i_dc, l_comm, _sampling_time, v_term_abc);

    // 6. Update Thyristor Currents
    _bridge.update_thyristor_currents(_i_dc);

    // 7. Force Natural Turn-Off
    i_ac = get_ac_currents();
    _bridge.force_natural_turn_off(i_ac, _i_dc);

    // 8. Calculate Bridge DC Voltage
    _v_bridge_dc = _bridge.get_instantaneous_dc_voltage(v_term_abc);

    // 9. Integrate DC Link Current
    // Loop: V_source + V_bridge - I*R = L di/dt
    // Inverter Mode: V_bridge is negative (Counter-EMF).
    // Standard LCI Topology: Inverter opposes Rectifier (Source).
    // V_source (Positive) + V_bridge (Negative) = Net Driving Voltage
    voltage_t v_net = v_dc_source + _v_bridge_dc - (_dc_link_resistance * _i_dc);

    // Forward Euler
    current_t di_dc = (v_net / _dc_link_inductance) * _sampling_time;
    _i_dc = _i_dc + di_dc;

    // Clamp to zero (unidirectional current)
    if (_i_dc.value() < 0.0) {
        _i_dc = current_t{0.0};
    }
}

void thyristor_inverter::reset()
{
    _i_dc = current_t{0.0};
    _v_bridge_dc = voltage_t{0.0};
    _bridge.reset();
}

abc<current_t> thyristor_inverter::get_ac_currents() const noexcept
{
    const auto& i_th = _bridge.get_thyristor_currents();
    // Phase A: T1 (upper) - T4 (lower)
    // Positive current flows OUT of the bridge
    return abc<current_t>{i_th[0] - i_th[3], i_th[2] - i_th[5], i_th[4] - i_th[1]};
}

current_t thyristor_inverter::get_dc_current() const noexcept { return _i_dc; }

}  // namespace yeet::control
