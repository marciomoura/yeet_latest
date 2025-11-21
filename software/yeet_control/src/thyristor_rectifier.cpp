#include "yeet_control/thyristor_rectifier.hpp"

#include <algorithm>

namespace yeet::control {

thyristor_rectifier::thyristor_rectifier(resistance_t grid_resistance,
    inductance_t grid_inductance,
    resistance_t load_resistance,
    inductance_t load_inductance,
    duration_t sampling_time)
    : _grid_resistance(grid_resistance),
      _grid_inductance(grid_inductance),
      _load_resistance(load_resistance),
      _load_inductance(load_inductance),
      _sampling_time(sampling_time)
{
}

void thyristor_rectifier::update(
    const abc<voltage_t>& v_ac_source, const thyristor_pulses& pulses, voltage_t v_dc_back_emf)
{
    // 1. Calculate grid currents from previous step's thyristor currents
    abc<current_t> i_grid = get_grid_currents();

    // 2. Calculate AC terminal voltages (subtracting grid impedance drop)
    abc<voltage_t> v_term_abc;
    v_term_abc.a() = v_ac_source.a() - _grid_resistance * i_grid.a();
    v_term_abc.b() = v_ac_source.b() - _grid_resistance * i_grid.b();
    v_term_abc.c() = v_ac_source.c() - _grid_resistance * i_grid.c();

    // 3. Estimate DC rail voltages for forward bias check
    voltage_t v_p_rail{0.0};
    voltage_t v_n_rail{0.0};

    if (_i_dc.value() > 1e-6) {
        v_p_rail = _v_dc_output * 0.5;
        v_n_rail = _v_dc_output * -0.5;
    }

    // 4. Update Bridge Conduction State (Firing)
    _bridge.update_conduction_state(v_term_abc, v_p_rail, v_n_rail, pulses);

    // 5. Update Commutation (Current Transfer)
    // Commutation inductance is 2 * L_grid (line-to-line)
    inductance_t l_comm = _grid_inductance * 2.0;
    _bridge.update_commutation_state(_i_dc, l_comm, _sampling_time, v_term_abc);

    // 6. Update Thyristor Currents
    _bridge.update_thyristor_currents(_i_dc);

    // 7. Force Natural Turn-Off (if current drops to zero)
    // Re-calculate grid currents with new thyristor currents
    i_grid = get_grid_currents();
    _bridge.force_natural_turn_off(i_grid, _i_dc);

    // 8. Calculate DC Output Voltage
    _v_dc_output = _bridge.get_instantaneous_dc_voltage(v_term_abc);

    // 9. Integrate DC Link Current
    // L_total * di/dt = V_dc - R_total * i - V_back_emf
    resistance_t r_total = _grid_resistance * 2.0 + _load_resistance;
    inductance_t l_total = _grid_inductance * 2.0 + _load_inductance;

    voltage_t v_net = _v_dc_output - (r_total * _i_dc) - v_dc_back_emf;

    // Forward Euler
    current_t di_dc = (v_net / l_total) * _sampling_time;
    _i_dc = _i_dc + di_dc;

    // Clamp to zero (unidirectional current)
    if (_i_dc.value() < 0.0) {
        _i_dc = current_t{0.0};
    }
}

void thyristor_rectifier::reset()
{
    _i_dc = current_t{0.0};
    _v_dc_output = voltage_t{0.0};
    _bridge.reset();
}

abc<current_t> thyristor_rectifier::get_grid_currents() const noexcept
{
    const auto& i_th = _bridge.get_thyristor_currents();
    // Phase A: T1 - T4
    // Phase B: T3 - T6
    // Phase C: T5 - T2
    return abc<current_t>{i_th[0] - i_th[3], i_th[2] - i_th[5], i_th[4] - i_th[1]};
}

current_t thyristor_rectifier::get_dc_current() const noexcept { return _i_dc; }

}  // namespace yeet::control