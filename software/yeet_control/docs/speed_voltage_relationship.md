## Mathematical Documentation for the `speed_voltage_relationship` Model

### 1. Introduction

This document details the mathematical foundation of the `speed_voltage_relationship` class. This model captures the fundamental linear relationship between terminal voltage magnitude and mechanical speed in electric machines. The model operates entirely in per-unit and provides a simplified interface for voltage magnitude control based on speed.

### 2. Simplified Linear Speed-Voltage Model

For control applications where the plant model already handles electrical reactance and complex impedance effects, the speed-voltage relationship can be simplified to a direct linear relationship between voltage magnitude and speed:

$$
|V_{pu}| = k_{vs} \omega_{pu} + V_{offset,pu}
$$

Where:
- $|V_{pu}|$ is the per-unit voltage magnitude
- $k_{vs}$ is the voltage-speed coefficient (dimensionless)
- $\omega_{pu}$ is the per-unit mechanical speed
- $V_{offset,pu}$ is the per-unit voltage offset

### 3. Dynamic Effects (Optional)

For systems with electrical time constants, a first-order dynamic relationship can be modeled:

$$
\tau_{sv} \frac{d|V_{pu}|}{dt} + |V_{pu}| = k_{vs} \omega_{pu} + V_{offset,pu}
$$

When $\tau_{sv} = 0$, the model becomes purely algebraic (instantaneous).

### 4. Implementation Parameters

| Symbol          | C++ Name                    | Description               | Typical Range   |
| :-------------- | :-------------------------- | :------------------------ | :-------------- |
| $k_{vs}$        | `voltage_speed_coefficient` | Voltage-speed coefficient | 0.8 - 1.2 pu/pu |
| $V_{offset,pu}$ | `voltage_offset_pu`         | Per-unit voltage offset   | 0.0 - 0.1 pu    |
| $\tau_{sv}$     | `time_constant_seconds`     | Time constant             | 0.0 - 0.05 s    |

### 5. Applications

- **Voltage Reference Generation**: Creating voltage commands based on speed references
- **Feed-forward Control**: Improving dynamic response in motor drive systems
- **System Simulation**: Coupling mechanical and electrical subsystems
- **Protection**: Overvoltage detection during high-speed operation