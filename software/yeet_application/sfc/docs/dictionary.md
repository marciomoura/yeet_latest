# Dictionary of LCI System Terms

[← Back to Documentation Index](index.md)

This comprehensive dictionary establishes standardized terminology and variable naming conventions for Load-Commutated Inverter (LCI) systems, including vector components essential for modern digital control and signal processing.

## Naming Convention

The standard naming convention is **`location_quantity_unit`** or **`location_component_attribute`**. For vector components, the coordinate system is appended (e.g., `machine_current_dq`, `grid_voltage_alphabeta`).

---

## **1. Grid-Side (Source-Side) Components and Variables**

| Term/Component                   | Programming Name (Variable)   | Symbol (LaTeX)      | Description and Role                                                                                                    |
| :------------------------------- | :---------------------------- | :------------------ | :---------------------------------------------------------------------------------------------------------------------- |
| **Grid-Side Converter**          | `grid_converter`              | -                   | The 6-pulse thyristor bridge that converts AC power from the grid into DC power. It operates as a controlled rectifier. |
| **Grid Voltage (ABC Line)**      | `grid_voltage_abc_line`       | $V_{LL,S,abc}$      | The instantaneous three-phase line-to-line voltages (A-B, B-C, C-A) from the utility grid.                              |
| **Grid Voltage (ABC Phase)**     | `grid_voltage_abc_phase`      | $V_{S,abc}$         | The instantaneous three-phase line-to-neutral voltages (A, B, C) from the utility grid.                                 |
| **Grid Voltage (L-L RMS)**       | `grid_voltage_line_rms`       | $V_{LL,S}$          | The RMS line-to-line voltage supplied by the utility grid.                                                              |
| **Grid Voltage (Phase RMS)**     | `grid_voltage_phase_rms`      | $V_{S}$             | The RMS line-to-neutral voltage supplied by the utility grid.                                                           |
| **Grid Voltage (αβ Components)** | `grid_voltage_alphabeta`      | $v_{S,\alpha\beta}$ | The stationary-frame voltage vector components obtained via Clarke transform: [$v_{S,\alpha}$, $v_{S,\beta}$].          |
| **Grid Voltage (dq Components)** | `grid_voltage_dq`             | $v_{S,dq}$          | The rotating-frame voltage vector components: [$v_{S,d}$, $v_{S,q}$].                                                   |
| **Grid Current (ABC)**           | `grid_current_abc`            | $I_{S,abc}$         | The instantaneous three-phase line currents (A, B, C) drawn from the grid.                                              |
| **Grid Current (RMS)**           | `grid_current_rms`            | $I_{S}$             | The RMS AC line current drawn from the grid.                                                                            |
| **Grid Current (αβ Components)** | `grid_current_alphabeta`      | $i_{S,\alpha\beta}$ | The stationary-frame current vector components: [$i_{S,\alpha}$, $i_{S,\beta}$].                                        |
| **Grid Current (dq Components)** | `grid_current_dq`             | $i_{S,dq}$          | The rotating-frame current vector components: [$i_{S,d}$, $i_{S,q}$].                                                   |
| **Grid Frequency**               | `grid_frequency`              | $f_S$ or $\omega_S$ | The electrical frequency of the utility grid, typically fixed at 50 or 60 Hz.                                           |
| **Grid Transformation Angle**    | `grid_transform_angle`        | $\theta_S$          | The angle of the rotating dq reference frame, synchronized with grid voltage. $\theta_S = \int \omega_S dt$.            |
| **Grid-Side Firing Angle**       | `grid_firing_angle`           | $\alpha_R$          | The angle that controls the converter's output DC voltage ($V_d$).                                                      |
| **Grid Commutating Inductance**  | `grid_commutating_inductance` | $L_s$               | The per-phase inductance on the grid side that governs commutation.                                                     |
| **Grid Real Power**              | `grid_real_power`             | $P_S$               | The active power drawn from the grid.                                                                                   |
| **Grid Reactive Power**          | `grid_reactive_power`         | $Q_S$               | The reactive power **consumed** by the grid-side converter.                                                             |

## **2. DC Link Components and Variables**

| Term/Component                  | Programming Name (Variable) | Symbol (LaTeX) | Description and Role                                                             |
| :------------------------------ | :-------------------------- | :------------- | :------------------------------------------------------------------------------- |
| **DC Link Inductor**            | `dc_link_inductor`          | $L_d$          | A large inductor to smooth the DC current.                                       |
| **DC Link Current**             | `dc_link_current`           | $I_d$          | The average DC current flowing through the link, proportional to machine torque. |
| **DC Link Current Reference**   | `dc_link_current_ref`       | $I_{d,ref}$    | The setpoint for the DC link current.                                            |
| **DC Link Voltage (Rectifier)** | `dc_link_voltage_rect`      | $V_d$          | The average DC voltage at the output of the grid-side converter.                 |
| **DC Link Voltage (Inverter)**  | `dc_link_voltage_inv`       | $V_{di}$       | The average DC back-voltage at the input of the machine-side converter.          |

## **3. Machine-Side (Load-Side) Components and Variables**

| Term/Component                      | Programming Name (Variable)      | Symbol (LaTeX)      | Description and Role                                                                                       |
| :---------------------------------- | :------------------------------- | :------------------ | :--------------------------------------------------------------------------------------------------------- |
| **Machine-Side Converter**          | `machine_converter`              | -                   | The 6-pulse thyristor bridge that inverts DC power to variable-frequency AC power.                         |
| **Machine Voltage (ABC Line)**      | `machine_voltage_abc_line`       | $V_{LL,M,abc}$      | The instantaneous three-phase line-to-line voltages (A-B, B-C, C-A) at machine terminals.                  |
| **Machine Voltage (ABC Phase)**     | `machine_voltage_abc_phase`      | $V_{M,abc}$         | The instantaneous three-phase line-to-neutral voltages (A, B, C) at machine terminals.                     |
| **Machine Voltage (L-L RMS)**       | `machine_voltage_line_rms`       | $V_{LL,M}$          | The RMS line-to-line voltage at the terminals of the synchronous machine.                                  |
| **Machine Voltage (Phase RMS)**     | `machine_voltage_phase_rms`      | $V_{M}$             | The RMS line-to-neutral voltage at the terminals of the synchronous machine.                               |
| **Machine Voltage (αβ Components)** | `machine_voltage_alphabeta`      | $v_{M,\alpha\beta}$ | The stationary-frame voltage vector components: [$v_{M,\alpha}$, $v_{M,\beta}$].                           |
| **Machine Voltage (dq Components)** | `machine_voltage_dq`             | $v_{M,dq}$          | The rotating-frame voltage vector components: [$v_{M,d}$, $v_{M,q}$].                                      |
| **Machine Current (ABC)**           | `machine_current_abc`            | $I_{M,abc}$         | The instantaneous three-phase line currents (A, B, C) flowing into the machine.                            |
| **Machine Current (RMS)**           | `machine_current_rms`            | $I_{M}$             | The RMS AC line current flowing into the machine.                                                          |
| **Machine Current (αβ Components)** | `machine_current_alphabeta`      | $i_{M,\alpha\beta}$ | The stationary-frame current vector components: [$i_{M,\alpha}$, $i_{M,\beta}$].                           |
| **Machine Current (dq Components)** | `machine_current_dq`             | $i_{M,dq}$          | The rotating-frame current vector components: [$i_{M,d}$, $i_{M,q}$].                                      |
| **Machine Frequency**               | `machine_frequency`              | $f_M$ or $\omega_M$ | The electrical frequency supplied to the machine, proportional to speed.                                   |
| **Machine Transformation Angle**    | `machine_transform_angle`        | $\theta_M$          | The angle of the rotating dq reference frame, synchronized with rotor flux. $\theta_M = \int \omega_M dt$. |
| **Machine-Side Advance Angle**      | `machine_advance_angle`          | $\beta$             | The inverter advance angle ($\beta = 180^\circ - \alpha_I$) that controls the machine's power factor.      |
| **Machine Extinction Angle**        | `machine_extinction_angle`       | $\gamma$            | The critical angle for safe thyristor turn-off. Maintained above a minimum safe value.                     |
| **Machine Commutating Inductance**  | `machine_commutating_inductance` | $L_m$               | The per-phase inductance of the machine that governs commutation.                                          |
| **Machine Flux (dq Components)**    | `machine_flux_dq`                | $\psi_{M,dq}$       | The magnetic flux linkage vector components: [$\psi_{M,d}$, $\psi_{M,q}$].                                 |
| **Machine Flux Vector Magnitude**   | `machine_flux_magnitude`         | $\psi_M$            | The magnitude of the machine's resultant flux vector: $\sqrt{\psi_{M,d}^2 + \psi_{M,q}^2}$.                |

## **4. Synchronous Machine and Mechanical System**

| Term/Component                | Programming Name (Variable) | Symbol (LaTeX) | Description and Role                                                                                |
| :---------------------------- | :-------------------------- | :------------- | :-------------------------------------------------------------------------------------------------- |
| **Synchronous Machine**       | `synchronous_machine`       | -              | The load being driven by the LCI.                                                                   |
| **Machine Speed**             | `machine_speed_rpm`         | $n$            | The mechanical rotational speed of the machine shaft.                                               |
| **Machine Speed Reference**   | `machine_speed_ref`         | $n_{ref}$      | The desired speed setpoint for the machine.                                                         |
| **Machine Electrical Torque** | `machine_electrical_torque` | $T_e$          | The torque produced by the machine, proportional to `dc_link_current`.                              |
| **Machine Field Current**     | `machine_field_current`     | $I_f$          | The DC current in the rotor, controlling the machine's magnetic flux and reactive power capability. |

---

## **Additional Notes on Vector Components**

**Clarke Transform (αβ):** Converts three-phase ABC quantities to stationary two-phase αβ components:
- `_alphabeta` variables contain both α and β components as a vector [α, β]
- Used for the stationary reference frame algorithms

**Park Transform (dq):** Converts αβ quantities to rotating two-phase dq components:
- `_dq` variables contain both d and q components as a vector [d, q]  
- d-axis: aligned with flux (for flux control)
- q-axis: aligned with torque (for torque control)
- Used for advanced control strategies and steady-state analysis

**Transformation Angles:**
- `grid_transform_angle` (θₛ): Grid-side dq frame angle
- `machine_transform_angle` (θₘ): Machine-side dq frame angle, typically rotor flux aligned

---

[← Back to Documentation Index](index.md)