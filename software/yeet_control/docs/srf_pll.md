# SRF-PLL for Position and Speed Estimation: A Mathematical Deep Dive

## Introduction

In modern motor control systems, particularly for permanent magnet synchronous motors (PMSMs) and AC induction motors, sensorless control techniques are increasingly favored. These methods eliminate the need for physical position sensors like encoders or resolvers, reducing cost, size, and potential points of failure. A cornerstone of many sensorless control strategies is the Synchronous Reference Frame Phase-Locked Loop (SRF-PLL).

This document provides a detailed exploration of the SRF-PLL, focusing on the fundamental mathematics that underpin its ability to accurately estimate rotor position and speed.

## The Synchronous Reference Frame (SRF)

The core principle of Field-Oriented Control (FOC) and, by extension, the SRF-PLL, is the transformation of stator currents from a three-phase stationary reference frame (a, b, c) into a two-phase rotating reference frame (d, q). This transformation simplifies the control problem by converting the sinusoidal AC signals of the stator into DC quantities, which are much easier to regulate. This is accomplished through two key mathematical transformations: the Clarke and Park transforms.

### Clarke Transformation

The Clarke transform converts the three-phase stationary system (ia, ib, ic) into a two-phase stationary orthogonal system (iα, iβ). This reduces the complexity from three variables to two. The transformation is as follows:

```
iα = ia
iβ = (ia + 2*ib) / sqrt(3)
```

For a balanced three-phase system where `ia + ib + ic = 0`, the equations can be simplified.

### Park Transformation

Following the Clarke transform, the Park transform converts the two-phase stationary reference frame (α, β) into a two-phase rotating reference frame (d, q). This frame rotates in synchrony with the rotor's magnetic flux. The transformation requires the rotor angle (θ) and is defined as:

```
id = iα * cos(θ) + iβ * sin(θ)
iq = -iα * sin(θ) + iβ * cos(θ)
```

In this rotating frame, the direct-axis (d) component represents the flux-producing component of the stator current, while the quadrature-axis (q) component represents the torque-producing component. By controlling these two DC quantities independently, precise control of the motor's flux and torque can be achieved.

## The Phase-Locked Loop (PLL)

A Phase-Locked Loop is a closed-loop feedback control system that synchronizes its output signal's phase and frequency with an input signal. It consists of three primary components:

*   **Phase Detector (PD):** This component compares the phase of the input signal with the phase of the output signal and generates an error signal proportional to the phase difference.
*   **Loop Filter (LF):** The error signal from the PD is filtered by the loop filter, which is typically a Proportional-Integral (PI) controller. The PI controller's role is to drive the steady-state phase error to zero.
*   **Voltage-Controlled Oscillator (VCO):** The filtered error signal is fed to the VCO, which adjusts the frequency and phase of its output signal to minimize the phase error.

## SRF-PLL for Position and Speed Estimation

The SRF-PLL leverages the principles of the synchronous reference frame and the PLL to estimate the rotor's position and speed. The fundamental idea is to use the back-electromotive force (back-EMF) of the motor, which is a sinusoidal signal whose phase is directly related to the rotor's position.

### Mathematical Model

In a sensorless FOC scheme, the stator voltages in the α-β stationary reference frame can be expressed as:

```
Vα = R*iα + L * d(iα)/dt - Eα
Vβ = R*iβ + L * d(iβ)/dt - Eβ
```

Where:
*   `Vα`, `Vβ` are the stator voltages in the α-β frame.
*   `iα`, `iβ` are the stator currents in the α-β frame.
*   `R` is the stator resistance.
*   `L` is the stator inductance.
*   `Eα`, `Eβ` are the back-EMF components in the α-β frame.

The back-EMF components are sinusoidal and can be expressed as:

```
Eα = -ωe * λm * sin(θe)
Eβ = ωe * λm * cos(θe)
```

Where:
*   `ωe` is the electrical angular velocity of the rotor.
*   `λm` is the flux linkage of the permanent magnets.
*   `θe` is the electrical angle of the rotor.

The goal of the SRF-PLL is to estimate `θe`. This is achieved by transforming the measured stator currents and voltages into the estimated rotating reference frame (d̂, q̂). The key is to drive the q-axis component of the estimated back-EMF to zero.

The estimated back-EMF in the estimated d̂-q̂ frame is obtained by applying the Park transform with the estimated angle `θ̂e`:

```
Ed̂ = Eα * cos(θ̂e) + Eβ * sin(θ̂e)
Eq̂ = -Eα * sin(θ̂e) + Eβ * cos(θ̂e)
```

Substituting the expressions for `Eα` and `Eβ`:

```
Eq̂ = ωe * λm * sin(θe) * sin(θ̂e) + ωe * λm * cos(θe) * cos(θ̂e)
Eq̂ = ωe * λm * cos(θe - θ̂e)
```

When the estimated angle `θ̂e` is equal to the actual angle `θe`, the term `cos(θe - θ̂e)` becomes 1, and `Eq̂` is proportional to the electrical speed. However, the SRF-PLL aims to drive the q-axis component of a specific error signal to zero.

### The PLL in Action

1.  **Phase Detection:** The q-axis component of the stator voltage in the estimated rotating frame (`Vq̂`) is used as the error signal. The objective is to regulate this `Vq̂` to zero.

2.  **Loop Filter (PI Controller):** The `Vq̂` error signal is fed into a PI controller. The PI controller generates an output that represents the estimated electrical angular velocity (`ω̂e`).

    ```
    ω̂e = Kp * Vq̂ + Ki * ∫Vq̂ dt
    ```

    Where `Kp` and `Ki` are the proportional and integral gains of the PI controller.

3.  **Voltage-Controlled Oscillator (Integrator):** The estimated angular velocity (`ω̂e`) is then integrated to obtain the estimated electrical angle (`θ̂e`).

    ```
    θ̂e = ∫ω̂e dt
    ```

This estimated angle `θ̂e` is then used in the Park transformation for the next control cycle. The closed-loop nature of the PLL continuously adjusts `θ̂e` to force `Vq̂` to zero, at which point the estimated angle accurately tracks the actual rotor angle.

### Speed and Position Extraction

*   **Position:** The output of the integrator in the PLL, `θ̂e`, directly provides the estimated electrical position of the rotor.
*   **Speed:** The output of the PI controller, `ω̂e`, provides the estimated electrical angular velocity of the rotor. This can be easily converted to mechanical speed.

## Conclusion

The SRF-PLL is a powerful and elegant mathematical construct that lies at the heart of many modern sensorless motor control systems. By transforming the complex AC dynamics of a motor into a simpler DC problem within a rotating reference frame, and then employing a phase-locked loop to track the rotor's back-EMF, it provides a robust and accurate method for estimating both position and speed. Understanding the underlying mathematics of the Clarke and Park transforms, along with the operation of the PI controller within the PLL, is crucial for designing and implementing high-performance sensorless motor drives.