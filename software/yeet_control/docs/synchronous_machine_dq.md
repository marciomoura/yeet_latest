# synchronous_machine_dq Electrical Model

## Purpose
Provides the electrical dq (3rd order) model of a salient synchronous machine with one field winding. Outputs:
- Induced phase back-EMF (abc) including transformer + speed components
- Electromagnetic torque (per-unit)
Mechanical dynamics are handled externally by mechanical_load.

## Inputs (update)
1. Stator phase currents i_abc (pu)
2. Field (excitation) voltage v_f (pu)
3. Electrical speed ω_e_pu (per-unit of configured base electrical frequency)

## Outputs (getters)
- get_induced_voltage_abc()
- get_torque_pu()
- get_field_flux_pu()
- get_angle_rad()
- get_e_d_pu(), get_e_q_pu()

## Core Equations
ψ_f = L_md i_d + L_f i_f
i_f = (ψ_f − L_md i_d)/L_f
ψ_d = (L_d − L_md^2/L_f) i_d + (L_md/L_f) ψ_f = L_d' i_d + (L_md/L_f) ψ_f
ψ_q = L_q i_q
dψ_f/dt = v_f − R_f i_f
e_d = dψ_d/dt − ω_e ψ_q
e_q = dψ_q/dt + ω_e ψ_d
T_e_pu = 1.5 p (ψ_d i_q − ψ_q i_d)
dθ_e/dt = ω_e

Forward Euler for ψ_f; finite-difference for flux derivatives.

## Usage Sequence
1. Configure inductances and field params:
   configure_stator_inductances(Ld, Lq)
   configure_field_coupling(Lf, L_md, R_f)
2. Configure pole pairs & base electrical frequency.
3. reset(...)
4. Each cycle:
   machine.update(i_abc, v_f, omega_e_pu)
   torque = machine.get_torque_pu()
   E_abc  = machine.get_induced_voltage_abc()
   mechanical_load.update(torque, load_torque)

## Network Interface
Terminal phase voltage (if modeling stator resistance externally):
v_phase = R_s * i_phase + E_phase
Do not place stator inductance externally; saliency and L di/dt are embedded in generated EMF.

## Notes
- Provide electrical speed per-unit (mechanical_speed_pu * pole_pairs if base mechanical equals base synchronous).
- L_d must exceed L_md^2 / L_f to keep L_d' positive.
- Zero sequence ignored (balanced system assumption).

## Extensions
- Add damper windings: additional flux states ψ_kd, ψ_kq.
- Saturation: make L_d', L_q functions of |ψ| or i.
- AVR: close a regulator on terminal voltage magnitude to set v_f.