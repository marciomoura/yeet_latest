## Mathematical Documentation for the `mechanical_load` Model

### 1. Introduction

This document details the mathematical foundation of the `mechanical_load` class. The class models the rotational dynamics of a linear mechanical system, such as the one coupled to an electric motor. The model is presented in a continuous-time framework and uses a **per-unit (pu)** system for normalization, which is standard practice in power systems and motor drive analysis.

### 2. The Fundamental Equation of Motion in SI Units

The model is based on Newton's second law for rotation, which states that the net torque applied to a rigid body is equal to its moment of inertia multiplied by its angular acceleration.

$$
\sum \tau = J \alpha
$$

For a typical motor-load system, the net torque is the sum of the electromagnetic torque produced by the motor ($T_e$), the external load torque ($T_l$), and the internal damping torque ($T_d$).

The governing equation in SI units is:
$$
J \frac{d\omega_m}{dt} = T_e - T_l - T_d
$$

The damping torque is modeled as being linearly proportional to the mechanical angular speed, representing viscous friction:
$$
T_d = K_d \omega_m
$$

Substituting this into the main equation gives the complete first-order ordinary differential equation (ODE) in SI units:
$$
J \frac{d\omega_m}{dt} + K_d \omega_m = T_e - T_l
$$

#### 2.1. SI Unit Definitions

| Symbol | Description | SI Unit |
| :--- | :--- | :--- |
| $\omega_m$ | Mechanical angular speed | radians per second (rad/s) |
| $t$ | Time | seconds (s) |
| $J$ | Total moment of inertia of the rotor and connected load | kilogram-meter squared (kg·m²) |
| $K_d$ | Viscous damping coefficient | Newton-meter-seconds per radian (N·m·s/rad) |
| $T_e$ | Electromagnetic torque | Newton-meters (N·m) |
| $T_l$ | External load torque | Newton-meters (N·m) |

### 3. The Per-Unit (pu) System

The per-unit system normalizes all physical quantities against a set of defined **base values**. This simplifies analysis by creating dimensionless parameters and making models scalable across machines of different power ratings.

A quantity in per-unit is defined as:
$$
\text{Quantity}_{pu} = \frac{\text{Actual Quantity}}{\text{Base Quantity}}
$$

#### 3.1. Base Values

To normalize the mechanical equation, we define the following base values:
*   **Base Power ($S_{base}$):** The nominal apparent power rating of the machine (in Volt-Amperes, VA) or its real power (in Watts, W).
*   **Base Mechanical Speed ($\omega_{m,base}$):** The nominal or synchronous mechanical speed of the machine (in rad/s).

From these, we can derive the **Base Torque ($T_{base}$)**:
$$
T_{base} = \frac{S_{base}}{\omega_{m,base}} \quad (\text{Units: } \frac{W}{rad/s} = N \cdot m)
$$

### 4. Derivation of the Per-Unit Equation of Motion

The goal is to transform the SI equation into the per-unit form used by the class. We start with the SI equation:
$$
J \frac{d\omega_m}{dt} + K_d \omega_m = T_e - T_l
$$

First, we express the SI variables in terms of their per-unit counterparts:
*   $\omega_m = \omega_{pu} \cdot \omega_{m,base}$
*   $T_e = T_{e,pu} \cdot T_{base}$
*   $T_l = T_{l,pu} \cdot T_{base}$

For the time derivative:
$$
\frac{d\omega_m}{dt} = \frac{d}{dt}(\omega_{pu} \cdot \omega_{m,base}) = \omega_{m,base} \frac{d\omega_{pu}}{dt}
$$

Now, substitute these expressions back into the SI equation:
$$
J \left( \omega_{m,base} \frac{d\omega_{pu}}{dt} \right) + K_d \left( \omega_{pu} \omega_{m,base} \right) = (T_{e,pu} \cdot T_{base}) - (T_{l,pu} \cdot T_{base})
$$

To complete the normalization, we divide the entire equation by $T_{base}$:
$$
\left( \frac{J \omega_{m,base}}{T_{base}} \right) \frac{d\omega_{pu}}{dt} + \left( \frac{K_d \omega_{m,base}}{T_{base}} \right) \omega_{pu} = T_{e,pu} - T_{l,pu}
$$

The two coefficients in parentheses are now defined as the per-unit parameters $H$ and $D$.

#### 4.1. The Inertia Constant (H)

The inertia constant, $H$, is defined as the kinetic energy of the rotating mass at base speed, divided by the base power.
$$
H = \frac{E_{k,base}}{S_{base}} = \frac{\frac{1}{2} J \omega_{m,base}^2}{S_{base}}
$$
Its unit is **seconds (s)**. It physically represents the time required for the machine, under a constant net torque equal to $T_{base}$, to accelerate from rest to $\omega_{m,base}$.

By substituting $T_{base} = S_{base} / \omega_{m,base}$ into the first coefficient of our derived equation, we can relate it to $H$:
$$
\frac{J \omega_{m,base}}{T_{base}} = \frac{J \omega_{m,base}}{S_{base} / \omega_{m,base}} = \frac{J \omega_{m,base}^2}{S_{base}} = 2H
$$

#### 4.2. The Damping Factor (D)

The per-unit damping factor, $D$, is defined as the damping torque at base speed, normalized by the base torque.
$$
D = \frac{T_{d,base}}{T_{base}} = \frac{K_d \omega_{m,base}}{T_{base}}
$$
It is a dimensionless quantity, expressed in **per-unit (pu)**. It corresponds directly to the second coefficient in our derived equation.

### 5. Final Governing Equation

By substituting the definitions of $2H$ and $D$ into our normalized equation, we arrive at the final continuous-time model implemented by the `mechanical_load` class:

$$
2H \frac{d\omega_{pu}}{dt} + D \omega_{pu} = T_{e,pu} - T_{l,pu}
$$

#### 5.1. Per-Unit Variable and Parameter Definitions

| Symbol | C++ Name | Description | Units |
| :--- | :--- | :--- | :--- |
| $\omega_{pu}$ | `_speed_integrator.get_state()` | Per-unit mechanical angular speed | pu |
| $H$ | `inertia_constant_seconds` | Inertia constant | s |
| $D$ | `damping_factor_pu` | Per-unit damping factor | pu |
| $T_{e,pu}$ | `electromagnetic_torque_pu` | Per-unit electromagnetic torque | pu |
| $T_{l,pu}$ | `load_torque_pu` | Per-unit external load torque | pu |

### 6. Steady-State Analysis

The steady-state condition of the system is reached when the speed is no longer changing, meaning the angular acceleration is zero. Mathematically, this is expressed as:
$$
\frac{d\omega_{pu}}{dt} = 0
$$

By setting the derivative term in the governing equation to zero, we can solve for the steady-state mechanical speed, $\omega_{pu,ss}$:
$$
2H \left(0\right) + D \omega_{pu,ss} = T_{e,pu} - T_{l,pu}
$$

This simplifies to an algebraic relationship:
$$
D \omega_{pu,ss} = T_{e,pu} - T_{l,pu}
$$

Solving for the steady-state speed gives:
$$
\omega_{pu,ss} = \frac{T_{e,pu} - T_{l,pu}}{D}
$$

This equation shows that the final speed of the load is directly proportional to the net torque (electromagnetic torque minus load torque) and inversely proportional to the damping factor. If there is no damping ($D=0$), a steady state is only possible if the net torque is also zero. Otherwise, the load will accelerate or decelerate indefinitely.