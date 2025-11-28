#include "yeet_control/thyristor_back_to_back_synchronous_machine.hpp"

#include "yeet_control/range_kutta_derivative.hpp"

namespace yeet::control {

thyristor_back_to_back_synchronous_machine::thyristor_back_to_back_synchronous_machine(real_t grid_resistance,
    real_t grid_inductance,
    real_t dc_resistance,
    real_t dc_inductance,
    const synchronous_machine& machine,
    real_t ts)
    : _machine(machine),
      _ts(ts),
      _converter(resistance_t(grid_resistance),
          inductance_t(grid_inductance),
          resistance_t(dc_resistance),
          inductance_t(dc_inductance),
          resistance_t(machine.get_stator_resistance().value()),
          inductance_t(0.5 * (machine.get_subtransient_inductances().d_axis.value() +
                                 machine.get_subtransient_inductances().q_axis.value())),
          duration_t(ts))
{
}

void thyristor_back_to_back_synchronous_machine::initialize(
    real_t initial_speed, real_t initial_angle, real_t initial_field_flux)
{
    // Reset converter
    _converter.reset();

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

void thyristor_back_to_back_synchronous_machine::update(const abc<voltage_t>& v_grid_abc,
    const std::array<bool, 6>& rect_pulses,
    const std::array<bool, 6>& inv_pulses,
    voltage_t v_field,
    torque_t torque_load)
{
    // 1. Calculate Back-EMF from current machine state
    // Note: i_motor is current INTO machine.
    // We need Back-EMF assuming zero current to get the source voltage behind impedance
    abc<current_t> zero_current(current_t(0.0), current_t(0.0), current_t(0.0));
    _v_back_emf = _machine.get_terminal_voltages_abc(_machine_state, zero_current);

    // 2. Update Converter Model
    thyristor_pulses p_rect_struct;
    for (int i = 0; i < 6; ++i) p_rect_struct[i] = rect_pulses[i];

    thyristor_pulses p_inv_struct;
    for (int i = 0; i < 6; ++i) p_inv_struct[i] = inv_pulses[i];

    _converter.update(v_grid_abc, _v_back_emf, p_rect_struct, p_inv_struct);

    // 3. Integrate Machine
    // Use updated currents from the converter
    // Note: get_machine_currents() returns current OUT of the converter bridge.
    // The synchronous machine model expects current INTO the machine (positive for motoring).
    // Since the inverter bridge has a "swapped" topology where DC+ connects to lower thyristors,
    // the current direction needs to be negated to match the machine convention.
    abc<current_t> i_motor_raw = _converter.get_machine_currents();
    abc<current_t> i_motor_next = abc<current_t>(
        current_t(-i_motor_raw.a().value()), current_t(-i_motor_raw.b().value()), current_t(-i_motor_raw.c().value()));

    auto machine_deriv_func = [&](const synchronous_machine_state& state) {
        return _machine.calculate_derivatives_current_fed(state, i_motor_next, v_field, torque_load);
    };
    _machine_state = range_kutta_derivative::calculate(_machine_state, machine_deriv_func, _ts);
}

abc<current_t> thyristor_back_to_back_synchronous_machine::get_grid_currents() const
{
    return _converter.get_grid_currents();
}

current_t thyristor_back_to_back_synchronous_machine::get_dc_link_current() const
{
    return _converter.get_dc_current();
}

voltage_t thyristor_back_to_back_synchronous_machine::get_rectifier_dc_voltage() const
{
    return _converter.get_rectifier_dc_voltage();
}

voltage_t thyristor_back_to_back_synchronous_machine::get_inverter_dc_voltage() const
{
    return _converter.get_inverter_dc_voltage();
}

abc<current_t> thyristor_back_to_back_synchronous_machine::get_motor_currents() const
{
    return _converter.get_machine_currents();
}

abc<voltage_t> thyristor_back_to_back_synchronous_machine::get_motor_voltages() const
{
    return _converter.get_inverter_terminal_voltages();
}

abc<voltage_t> thyristor_back_to_back_synchronous_machine::get_motor_back_emf() const { return _v_back_emf; }

const synchronous_machine_state& thyristor_back_to_back_synchronous_machine::get_machine_state() const
{
    return _machine_state;
}

const std::array<bool, 6>& thyristor_back_to_back_synchronous_machine::get_inverter_states() const
{
    return _converter.get_inverter_bridge().get_conduction_states();
}

const std::array<bool, 6>& thyristor_back_to_back_synchronous_machine::get_rectifier_states() const
{
    return _converter.get_rectifier_bridge().get_conduction_states();
}

abc<voltage_t> thyristor_back_to_back_synchronous_machine::get_grid_line_voltages() const
{
    return _converter.get_rectifier_bridge().get_ac_line_voltages();
}

abc<voltage_t> thyristor_back_to_back_synchronous_machine::get_machine_line_voltages() const
{
    return _converter.get_inverter_bridge().get_ac_line_voltages();
}

}  // namespace yeet::control
