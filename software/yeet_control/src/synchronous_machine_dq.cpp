#include "yeet_control/synchronous_machine_dq.hpp"

#include <cmath>

#include "yeet_control/units.hpp"

namespace yeet::control {

synchronous_machine_dq::synchronous_machine_dq(real_t sampling_time)
    : _sampling_time(sampling_time), _psi_d_deriv(sampling_time), _psi_q_deriv(sampling_time)
{
    assert(sampling_time > 0.0f && "Sampling time must be positive.");
    recompute_reduced_d_axis_inductance();
}

void synchronous_machine_dq::configure_stator_inductances(
    inductance_pu_t d_axis_inductance_pu, inductance_pu_t q_axis_inductance_pu)
{
    assert(d_axis_inductance_pu.value() > 0.0f && "D-axis inductance must be positive.");
    assert(q_axis_inductance_pu.value() > 0.0f && "Q-axis inductance must be positive.");
    _l_dq = dq<inductance_pu_t>{d_axis_inductance_pu, q_axis_inductance_pu};
    recompute_reduced_d_axis_inductance();
}

void synchronous_machine_dq::configure_field_coupling(
    inductance_pu_t field_inductance_pu, inductance_pu_t mutual_inductance_pu, resistance_pu_t field_resistance_pu)
{
    assert(field_inductance_pu.value() > 0.0f && "Field inductance must be positive.");
    assert(mutual_inductance_pu.value() > 0.0f && "Mutual inductance must be positive.");
    assert(field_resistance_pu.value() >= 0.0f && "Field resistance must be non-negative.");
    _lf = field_inductance_pu;
    _lmd = mutual_inductance_pu;
    _rf = field_resistance_pu;
    recompute_reduced_d_axis_inductance();
}

void synchronous_machine_dq::configure_pole_pairs(unsigned pole_pairs)
{
    assert(pole_pairs > 0 && "Pole pairs must be positive.");
    _pole_pairs = pole_pairs;
    _torque_scale = 1.5f * static_cast<real_t>(_pole_pairs);
}

void synchronous_machine_dq::configure_base_electrical_frequency_hz(real_t base_frequency_hz)
{
    assert(base_frequency_hz > 0.0f && "Base frequency must be positive.");
    constexpr real_t two_pi = static_cast<real_t>(6.28318530717958647692);  // avoid M_PI dependency
    _base_electrical_angular_frequency_rad_per_second = two_pi * base_frequency_hz;
}

void synchronous_machine_dq::configure_stator_resistance(resistance_pu_t stator_resistance_pu)
{
    assert(stator_resistance_pu.value() >= 0.0f && "Stator resistance must be non-negative.");
    _rs = stator_resistance_pu;
}

void synchronous_machine_dq::reset(magnetic_flux_pu_t initial_field_flux_linkage_pu,
    current_pu_t initial_d_axis_current_pu,
    current_pu_t initial_q_axis_current_pu)
{
    _psi_f = initial_field_flux_linkage_pu;

    // compute consistent initial d and q flux linkages to reset derivatives
    const dq<current_pu_t> initial_currents_dq{initial_d_axis_current_pu, initial_q_axis_current_pu};
    const dq<magnetic_flux_pu_t> initial_flux_linkages = calculate_flux_linkages(initial_currents_dq);

    _psi_d_deriv.reset(initial_flux_linkages.d().value());
    _psi_q_deriv.reset(initial_flux_linkages.q().value());

    _e_dq = dq<voltage_pu_t>{};
    _te = torque_pu_t{0.0f};
    _i_f = current_pu_t{0.0f};
    _e_abc = abc<voltage_pu_t>{voltage_pu_t{0.0f}, voltage_pu_t{0.0f}, voltage_pu_t{0.0f}};
}

torque_pu_t synchronous_machine_dq::update(const abc<current_pu_t>& phase_currents_abc_pu,
    voltage_pu_t field_voltage_pu,
    speed_pu_t electrical_speed_pu,
    const angle_wrapped& electrical_angle) noexcept
{
    // 1) park transform currents using the provided angle
    const dq<current_pu_t> currents_dq = phase_currents_abc_pu.to_dq(electrical_angle);

    // 2) field current
    _i_f = (_psi_f - _lmd * currents_dq.d()) / _lf;

    // 3) field flux integration
    const voltage_pu_t d_psi_f_dt = field_voltage_pu - _rf * _i_f;
    _psi_f = _psi_f + magnetic_flux_pu_t{_sampling_time * d_psi_f_dt};

    // 4) flux linkages
    const dq<magnetic_flux_pu_t> flux_linkages_dq = calculate_flux_linkages(currents_dq);

    // 5) derivatives using the derivative component
    _psi_d_deriv.update(flux_linkages_dq.d());
    _psi_q_deriv.update(flux_linkages_dq.q());

    const dq<voltage_pu_t> stator_flux_derivatives_dq{
        voltage_pu_t{_psi_d_deriv.get_output()}, voltage_pu_t{_psi_q_deriv.get_output()}};

    // 6) induced dq voltages (back emf)
    const angular_frequency_pu_t omega_e_rad_per_second =
        angular_frequency_pu_t{electrical_speed_pu * _base_electrical_angular_frequency_rad_per_second};

    _e_dq = dq<voltage_pu_t>{stator_flux_derivatives_dq.d() - omega_e_rad_per_second * flux_linkages_dq.q(),
        stator_flux_derivatives_dq.q() + omega_e_rad_per_second * flux_linkages_dq.d()};

    // 7) electromagnetic torque
    _te = torque_pu_t{_torque_scale * (flux_linkages_dq.d().value() * currents_dq.q().value() -
                                          flux_linkages_dq.q().value() * currents_dq.d().value())};

    // 8) inverse park to abc using the provided angle
    _e_abc = _e_dq.to_abc(electrical_angle);

    // Step 9 (update stored fluxes) is now handled internally by the derivative components.

    return _te;
}

void synchronous_machine_dq::recompute_reduced_d_axis_inductance()
{
    // reduced_d = Ld - Lmd^2 / Lf
    const real_t ld = _l_dq.d().value();
    const real_t lmd = _lmd.value();
    const real_t lf = _lf.value();
    const real_t reduction = (lmd * lmd) / lf;
    assert(ld > reduction &&
           "Stator d-axis inductance must exceed (mutual^2 / field) to avoid negative reduced inductance.");
    _ld_prime = inductance_pu_t{ld - reduction};
    _coupling_gain = lmd / lf;
}

dq<magnetic_flux_pu_t> synchronous_machine_dq::calculate_flux_linkages(const dq<current_pu_t>& currents_dq) const
{
    const magnetic_flux_pu_t flux_d = _ld_prime * currents_dq.d() + _coupling_gain * _psi_f;
    const magnetic_flux_pu_t flux_q = _l_dq.q() * currents_dq.q();
    return {flux_d, flux_q};
}

abc<voltage_pu_t> synchronous_machine_dq::get_back_emf_abc() const noexcept { return _e_abc; }

dq<voltage_pu_t> synchronous_machine_dq::get_induced_voltage_dq() const noexcept { return _e_dq; }

torque_pu_t synchronous_machine_dq::get_electromagnetic_torque_pu() const noexcept { return _te; }

current_pu_t synchronous_machine_dq::get_field_current_pu() const noexcept { return _i_f; }

magnetic_flux_pu_t synchronous_machine_dq::get_field_flux_linkage_pu() const noexcept { return _psi_f; }

}  // namespace yeet::control
