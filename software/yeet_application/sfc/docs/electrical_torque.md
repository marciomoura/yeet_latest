# LCI Electrical Torque Model Documentation

## 1. Introduction

This document provides a comprehensive explanation of how electromagnetic torque is produced in a Load-Commutated Inverter (LCI) fed synchronous motor drive. It begins with the fundamental physics and derives the complete torque equation. It then shows how this equation is simplified to a linear, per-unit model suitable for designing and simulating high-level control systems, such as a speed controller.

The final section serves as a practical guide, showing how to use this simplified electrical model in conjunction with the `mechanical_load` class to create a complete plant model for closed-loop simulations.

## 2. Derivation of the LCI Torque Equation

The key to understanding LCI torque is the principle of power balance. Assuming a lossless system, the power drawn from the DC link must equal the electrical power delivered to the motor, which is then converted to mechanical power.

$$ P_{dc} = P_{ac, motor} = P_{air\_gap} $$

The air gap power ($P_{air\_gap}$) is related to the electromagnetic torque ($T_e$) and the mechanical speed ($\omega_m$) by the fundamental relationship:

$$ P_{air\_gap} = T_e \cdot \omega_m $$

By finding an expression for the DC power, we can solve for the torque.

### 2.1. DC Link Power and Inverter Voltage

The power in the DC link is given by:

$$ P_{dc} = V_d \cdot I_d $$

Where $V_d$ is the average DC voltage at the *inverter's* input terminals, and $I_d$ is the DC link current.

The inverter's DC voltage is determined by the motor's back-EMF ($E_{LL}$) and the inverter firing angle ($\beta$). For a 6-pulse bridge, this relationship is:

$$ V_d = \frac{3\sqrt{2}}{\pi} E_{LL} \cos(\beta) $$

The motor's back-EMF is, in turn, proportional to the rotor magnetic flux (set by the field current, $I_f$) and the rotational speed ($\omega_m$):

$$ E_{LL} = K_{\phi} \cdot I_f \cdot \omega_m $$

where $K_{\phi}$ is the motor's flux constant.

### 2.2. Assembling the Full Torque Equation

By substituting these expressions into the power balance equation, we can solve for torque:

1.  Start with power balance: $T_e \cdot \omega_m = V_d \cdot I_d$
2.  Substitute for $V_d$: $T_e \cdot \omega_m = \left( \frac{3\sqrt{2}}{\pi} E_{LL} \cos(\beta) \right) I_d$
3.  Substitute for $E_{LL}$: $T_e \cdot \omega_m = \left( \frac{3\sqrt{2}}{\pi} (K_{\phi} \cdot I_f \cdot \omega_m) \cos(\beta) \right) I_d$

The mechanical speed $\omega_m$ appears on both sides and can be cancelled, yielding the final torque equation:

$$ T_e = \left( \frac{3\sqrt{2}}{\pi} K_{\phi} \cos(\beta) \right) \cdot I_f \cdot I_d $$

This fundamental result shows that **torque is directly proportional to the product of the DC link current ($I_d$) and the motor field current ($I_f$)**.

## 3. Simplified Torque Model for Control Simulation

For designing high-level controllers (e.g., a speed loop), the full equation is often simplified.

### 3.1. The Constant Flux Assumption

In the normal operating range (below base speed), an LCI drive is controlled by holding the **motor field current ($I_f$) constant** at its nominal value to produce maximum magnetic flux. The **inverter firing angle ($\beta$) is also held nearly constant** to ensure safe commutation.

Under these conditions, all terms except $I_d$ can be grouped into a single motor torque constant, $K_m$:

$$ K_m = \left( \frac{3\sqrt{2}}{\pi} K_{\phi} \cos(\beta) \right) I_{f,nominal} \quad (\text{Units: Nm/A}) $$

The torque equation simplifies to a linear relationship:

$$ T_e = K_m \cdot I_d $$

### 3.2. The Per-Unit Torque Model

This relationship is most powerful in a per-unit (pu) system. By selecting the base DC current ($I_{d,base}$) as the current that produces base torque ($T_{base}$) at nominal flux, the equation becomes trivial.

Given $T_{base} = K_m \cdot I_{d,base}$, the per-unit torque is:

$$ T_{e,pu} = \frac{T_e}{T_{base}} = \frac{K_m \cdot I_d}{K_m \cdot I_{d,base}} = \frac{I_d}{I_{d,base}} = I_{d,pu} $$

$$ \implies T_{e,pu} = I_{d,pu} $$

This is the ultimate simplified model for control simulation: **the per-unit electromagnetic torque is equal to the per-unit DC link current.** A speed controller's output can be treated as a per-unit torque command ($T_{e,ref,pu}$), which is then used as the reference for the inner DC current control loop.

## 4. Guide: Creating a Closed-Loop Plant Model

This section explains how to use the simplified electrical model with the `mechanical_load` class to build a complete plant model for simulation.

### 4.1. Assembling the Simulation Flow

A typical speed control loop simulation is structured as follows:

```
                             +-----------+      +--------------------------+       +--------------------+
Speed Reference (+)---> O --->|  Speed    |----->| Simplified Electrical    |----->|                    |
   omega_ref_pu   ^     -|    | Controller|      |      Model               |      |  mechanical_load   |   T_l_pu
                  |      |    | (PI)      |      | T_e,pu = I_d,pu = T_e,ref_pu |  |       (Plant)      |------>O
                  |      |    +-----------+      +--------------------------+      |                    |
                  |      |       T_e,ref_pu                 T_e,pu                 +---------+----------+
                  |      |                                                                    | omega_pu
                  |      +--------------------------------------------------------------------+
                  |                                     Feedback                              |
                  +---------------------------------------------------------------------------+
```

1.  **Controller Output:** A speed controller generates a torque reference, $T_{e,ref,pu}$.
2.  **Electrical Model:** We assume the inner current loop is fast and ideal, so the actual torque equals the reference: $T_{e,pu} = T_{e,ref,pu}$.
3.  **Mechanical Model:** This $T_{e,pu}$ is the input to the `mechanical_load::update()` method.
4.  **Plant Output:** The `mechanical_load` class calculates the new per-unit speed, $\omega_{pu}$.
5.  **Feedback:** The new speed is fed back to the speed controller, closing the loop.

### 4.2. Parametrizing the `mechanical_load` Class

To use the `mechanical_load` class, you must provide the inertia constant ($H$) and damping factor ($D$) from motor and load datasheets.

#### Calculating the Inertia Constant ($H$)

$H$ (in seconds) is the kinetic energy at base speed divided by the base power.

$$ H = \frac{\text{Kinetic Energy}_{base}}{\text{Power}_{base}} = \frac{\frac{1}{2} J_{total} \omega_{m,base}^2}{S_{base}} $$

**Procedure:**
1.  **Find Total Inertia ($J_{total}$):** Sum the inertia of the motor and the connected load, referred to the motor shaft (in **kg·m²**).
2.  **Find Base Power ($S_{base}$):** Use the motor's nominal power rating (in **Watts**).
3.  **Find Base Speed ($\omega_{m,base}$):** Convert the motor's nominal RPM to **rad/s** ($\text{RPM} \times 2\pi/60$).

#### Estimating the Damping Factor ($D$)

$D$ (in per-unit) is the frictional torque at base speed, divided by the base torque.

$$ D = \frac{T_{d,base}}{T_{base}} = \frac{P_{friction,base} / \omega_{m,base}}{S_{base} / \omega_{m,base}} = \frac{P_{friction,base}}{S_{base}} $$

**Procedure:**
1.  **Find Frictional Losses ($P_{friction,base}$):** Find the "friction and windage losses" at rated speed from the datasheet (in **Watts**).
2.  **Find Base Power ($S_{base}$):** Use the motor's nominal power rating (in **Watts**).
3.  **Calculate D:** Divide the frictional losses by the base power. If no data is available, a small value (e.g., 0.01) is a reasonable estimate for large machines.

### 4.3. Factors Influencing the Inertia Constant (H)

The inertia constant `H` is a measure of how much kinetic energy is stored in the rotating mass at rated speed, relative to the machine's power rating.
$$ H = \frac{\text{Kinetic Energy at Rated Speed}}{\text{Rated Apparent Power}} = \frac{\frac{1}{2} J \omega_{m,base}^2}{S_{base}} $$
This formula reveals the two key physical factors:
*   **Moment of Inertia ($J$):** The physical mass and its distribution around the axis of rotation. Heavier, larger-diameter rotors have more inertia.
*   **Rated Speed ($\omega_{m,base}$):** Kinetic energy increases with the square of the speed.

These factors are determined by the machine's design, which is driven by its application.

### Typical `H` Values for a 2 MVA Machine by Type

Here are the common categories of synchronous machines and their likely `H` values.

#### 1. High-Speed Machines (Turbogenerators)
These machines are driven by steam or gas turbines and have a high speed of rotation. They typically have 2 or 4 poles.

*   **2-Pole Machine:** 3000 RPM (50 Hz) or 3600 RPM (60 Hz)
*   **Characteristics:** Long, narrow rotor to withstand high centrifugal forces. For their power rating, their physical inertia ($J$) is relatively low.
*   **Typical `H` Range:** **1.5 to 3.0 s**

These machines have the lowest `H` values because of their high speed and optimized rotor design.

#### 2. Medium-Speed Machines (Standard Synchronous Motors, Diesel Generators)
This is a very common category for industrial applications like large pumps, compressors, and fans. They have more poles than turbogenerators.

*   **4, 6, or 8-Pole Machine:** 1500/1800 RPM, 1000/1200 RPM, 750/900 RPM.
*   **Characteristics:** Rotors have a larger diameter and are shorter than turbogenerators. This is the most "standard" configuration.
*   **Typical `H` Range:** **2.0 to 4.5 s**

If you have no other information about your 2 MVA machine, assuming it's a 4-pole or 6-pole motor and using a value of **H ≈ 3.0 s** is a very reasonable starting point for a simulation.

#### 3. Low-Speed Machines (Hydro-generators, Low-Speed Motors)
These machines are designed for low-speed applications, such as hydro-turbines or large, slow-moving industrial loads like ore-grinding mills.

*   **Multi-Pole Machine (10+ poles):** Speed is typically below 750 RPM.
*   **Characteristics:** Very large diameter rotors ("salient-pole" design) to accommodate the large number of magnetic poles. The massive physical size gives them an extremely high moment of inertia ($J$).
*   **Typical `H` Range:** **3.5 to 6.0 s**, and can be even higher for very large hydro units.

#### 4. Special Case: Synchronous Condensers or High-Inertia Motors
Sometimes, a synchronous machine is designed specifically to provide high inertia to stabilize a weak grid or to handle pulsating loads. This is often achieved by adding a **flywheel**.

*   **Characteristics:** Mechanically coupled to a large, heavy flywheel.
*   **Typical `H` Range:** **5.0 s and above.** The value is determined by the system's stability requirements, not the machine's intrinsic properties.

### Summary Table

| Machine Type / Application | Typical Speed (60 Hz) | Typical `H` Constant [s] for 2 MVA | Key Characteristics |
| :--- | :--- | :--- | :--- |
| Turbogenerator (Steam/Gas) | 3600 RPM (2-Pole) | **1.5 – 3.0** | High speed, low inertia ($J$) |
| Standard Motor/Generator | 1800/1200 RPM (4/6-Pole) | **2.0 – 4.5** | Most common industrial configuration |
| Low-Speed Motor/Hydro-gen. | < 900 RPM (8+ Poles) | **3.5 – 6.0** | Low speed, very high inertia ($J$) |
| High-Inertia Design | Varies | **> 5.0** | Flywheel added for grid stability |

### How to Find the Exact Value

For an accurate simulation, always refer to the **manufacturer's datasheet**. Look for one of these values:
1.  **Inertia Constant (H):** The most direct value, given in seconds.
2.  **Moment of Inertia (J):** Given in **kg·m²**. If you have this, you can calculate `H` yourself.
3.  **WR² or GD²:** An older but still common metric, often called "flywheel effect." It's given in units like lb·ft² or kg·m². Be very careful with the units and the definition (some include gravity, some don't) when converting `WR²` to `J`.