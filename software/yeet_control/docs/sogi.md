
# Signal Filtering and Decomposition Using the SOGI

## Overview

This document provides a comprehensive overview of the Second-Order Generalized Integrator (SOGI), a powerful adaptive filter used in power electronics and signal processing.

We will cover two key topics:
1.  **The Fundamental SOGI Filter:** An exploration of the mathematical principles, transfer functions, and core concepts behind a single SOGI, which is used to isolate a fundamental frequency and generate orthogonal (quadrature) signals.
2.  **The Dual-SOGI for Positive Sequence Extraction:** A detailed analysis of how two SOGI filters can be combined in a "Dual-SOGI" structure to isolate the positive sequence component from an unbalanced three-phase signal, a critical task for robust grid-tied converter control.

---

## Part 1: The Second-Order Generalized Integrator (SOGI) Filter

### 1.1 Introduction

The Second-Order Generalized Integrator (SOGI) is a powerful adaptive filter widely used in power electronics and signal processing, particularly for grid-connected converters and motor control applications. Its primary function is to isolate the fundamental frequency component of a distorted input signal while simultaneously generating an in-phase and a quadrature (90-degree phase-shifted) version of that component.

This section details the mathematical foundation of the SOGI filter, explaining its structure, equations, and the reasoning behind its effectiveness.

### 1.2 Core Concept: A Resonant Filter

At its heart, the SOGI is a band-pass filter with a resonant frequency set to the desired fundamental frequency (e.g., 50 or 60 Hz for grid applications). It is designed to have a high gain at this specific frequency while attenuating all other frequencies, effectively filtering out noise, harmonics, and DC offsets.

The key innovation of the SOGI is its structure, which not only filters the input signal but also generates two orthogonal output signals. These are commonly referred to as the direct (`v'`) and quadrature (`qv'`) outputs, which correspond to the α and β components in a stationary reference frame used in power systems analysis (like the Clarke transform).

### 1.3 Mathematical Structure and Transfer Functions

The SOGI is a second-order linear time-invariant (LTI) system. Its structure is based on a feedback loop that creates a resonance effect. Let the input signal be `v` and the resonant (or center) angular frequency be `ω'`. The SOGI produces two outputs: `v'`, which is in-phase with the fundamental component of the input, and `qv'`, which is in quadrature (lagging by 90 degrees).

The error signal, `ε`, is defined as:
`ε = v - v'`

The direct output `v'` is the integral of the scaled error:
`v' = ∫(k * ω' * ε) dt`

The quadrature output `qv'` is the integral of the scaled direct output:
`qv' = ∫(ω' * v') dt`

#### 1.3.1 Transfer Functions in the s-Domain

To analyze the filter's frequency response and stability, we use the Laplace transform to move to the continuous-time s-domain. The transfer functions from the input `V(s)` to the two outputs, `V'(s)` and `QV'(s)`, can be derived.

The relationship between the error `ε(s)` and the direct output `V'(s)` is:
`V'(s) = (k * ω' / s) * ε(s) = (k * ω' / s) * (V(s) - V'(s))`

Solving for `V'(s)` leads to the **in-phase transfer function**, `D(s)`:

**`D(s) = V'(s) / V(s) = (k * ω' * s) / (s² + k * ω' * s + ω'²)`**

This is the classic transfer function of a **band-pass filter**. At the resonant frequency `s = jω'`, the gain is maximized, and the phase shift is zero.

The quadrature output `QV'(s)` is the integral of `ω' * V'(s)`. Substituting the expression for `V'(s)` gives the **quadrature transfer function**, `Q(s)`:

**`Q(s) = QV'(s) / V(s) = (k * ω'²) / (s² + k * ω' * s + ω'²)`**

This is the classic transfer function of a **low-pass filter**. At `s = jω'`, the magnitude of this transfer function is equal to that of the band-pass filter, but its phase is shifted by -90 degrees.

#### 1.3.2 The Reasoning Behind the Orthogonality

The genius of the SOGI lies in how it generates these two orthogonal signals. The relationship `QV'(s) = (ω' / s) * V'(s)` shows that the quadrature output is mathematically the integral of the scaled in-phase output. In the frequency domain, integration (`1/s`) corresponds to a -90 degree phase shift. The SOGI structure cleverly implements this integration to produce a perfectly phase-shifted signal at the center frequency `ω'`.

### 1.4 Key Parameters

*   **`ω'` (Center Angular Frequency):** This is the resonant frequency of the filter. It is set to the fundamental frequency of the signal you want to extract (e.g., `2 * π * 50 rad/s`).
*   **`k` (Damping Factor / Gain):** This parameter is crucial for tuning the filter's performance. It controls the **bandwidth** of the filter. A smaller `k` results in a narrower bandwidth (better harmonic rejection, slower response), while a larger `k` widens the bandwidth (faster response, poorer harmonic rejection). A common starting value for `k` is `sqrt(2)`.

### 1.5 Application in Phase-Locked Loops (SOGI-PLL)

The SOGI filter is the foundational block of the SOGI-PLL, which is widely used for grid synchronization.
1.  **Filtering and Orthogonal Signal Generation:** A single-phase grid voltage is fed into the SOGI, which outputs the filtered α and β components (`v_α = v'` and `v_β = qv'`).
2.  **Park Transformation:** These `αβ` signals are transformed into a rotating `dq` frame using an estimated angle `θ'`.
3.  **Phase-Locked Loop:** The `v_q` component is used as the error input to a PI controller. The PLL works to drive `v_q` to zero.
4.  **Frequency and Angle Estimation:** When locked, the output of the PI controller is the estimated grid frequency (`ω`), and the integral of this frequency is the estimated phase angle (`θ'`).

---

## Part 2: The Dual-SOGI for Positive Sequence Extraction

### 2.1 Introduction and Objective

In three-phase power systems, an unbalanced voltage condition leads to the presence of a **positive sequence component** (desired) and a **negative sequence component** (undesired). The Dual-SOGI Positive Sequence Extractor is a control structure designed to take an unbalanced three-phase voltage (represented in the stationary `αβ` frame) and isolate its fundamental positive sequence component. This provides a clean, balanced signal ideal for a standard Phase-Locked Loop (PLL).

### 2.2 Core Concepts

#### 2.2.1 Symmetrical Components in the `αβ` Frame

Any unbalanced `αβ` vector, `Uαβ(t)`, is the sum of its positive (`+`) and negative (`-`) sequence components:
`Uα(t) = Uα+(t) + Uα-(t)`
`Uβ(t) = Uβ+(t) + Uβ-(t)`

*   **Positive Sequence (`+`):** Rotates counter-clockwise. In complex space: `Uαβ+(t) = A+ * e^(j(ωt + φ))`
*   **Negative Sequence (`-`):** Rotates clockwise. In complex space: `Uαβ-(t) = A- * e^(-j(ωt + θ))`

#### 2.2.2 The SOGI as a Quadrature Signal Generator (QSG)

As established in Part 1, a SOGI tuned to frequency `ω` takes an input and produces an in-phase output (`v'`) and a quadrature output (`qv'`). For this derivation, we assume the quadrature signal is leading by 90°.

### 2.3 The Dual-SOGI Structure and Derivation

The system uses two SOGI blocks synchronized to the same frequency `ω'`:
*   **Top SOGI (`SOGI_α`)**: Takes input `Uα` and produces outputs `Uα'` and `Uα''`.
*   **Bottom SOGI (`SOGI_β`)**: Takes input `Uβ` and produces outputs `Uβ'` and `Uβ''`.

The final outputs are calculated via cross-coupling:
`Uα_out = 0.5 * (Uα' - Uβ'')`
`Uβ_out = 0.5 * (Uβ' + Uα'')`

*(Note: The 0.5 scaling factor is included here to compensate for the doubling in amplitude that occurs, as shown below.)*

We can prove its function by analyzing its response to positive and negative sequences separately.

#### 2.3.1 Response to a Pure Positive Sequence Input

Let the input be `Uαβ+`, where `Uα(t) = A+cos(ωt)` and `Uβ(t) = A+sin(ωt)`.
1.  `SOGI_α` outputs: `Uα' = A+cos(ωt)` and `Uα'' = A+sin(ωt)`
2.  `SOGI_β` outputs: `Uβ' = A+sin(ωt)` and `Uβ'' = -A+cos(ωt)`

Applying the cross-coupling (before scaling):
*   `Uα' - Uβ'' = (A+cos(ωt)) - (-A+cos(ωt)) = 2A+cos(ωt)`
*   `Uβ' + Uα'' = (A+sin(ωt)) + (A+sin(ωt)) = 2A+sin(ωt)`

**Result:** The positive sequence is preserved and doubled in amplitude.

#### 2.3.2 Response to a Pure Negative Sequence Input

Let the input be `Uαβ-`, where `Uα(t) = A-cos(ωt)` and `Uβ(t) = -A-sin(ωt)`.
1.  `SOGI_α` outputs: `Uα' = A-cos(ωt)` and `Uα'' = A-sin(ωt)`
2.  `SOGI_β` outputs: `Uβ' = -A-sin(ωt)` and `Uβ'' = A-cos(ωt)`

Applying the cross-coupling:
*   `Uα' - Uβ'' = (A-cos(ωt)) - (A-cos(ωt)) = 0`
*   `Uβ' + Uα'' = (-A-sin(ωt)) + (A-sin(ωt)) = 0`

**Result:** The negative sequence is completely rejected.

#### 2.3.3 Derivation using Complex Phasors (Alternative View)

This provides a more elegant proof. Let the input vector be `U = Uα + jUβ`. The cross-coupling logic `U_out = (Uα' - Uβ'') + j(Uβ' + Uα'')` can be rewritten as:

`U_out = (Uα' + jUα'') - j(Uβ' + jUβ'')`

1.  **For Positive Sequence `U+ = A+e^(jωt)`:**
    *   `SOGI_α` on `Uα` produces `U+`.
    *   `SOGI_β` on `Uβ` produces `-jU+`.
    *   `U_out = (U+) - j(-jU+) = U+ - j²U+ = U+ + U+ = 2U+`

2.  **For Negative Sequence `U- = A-e^(-jωt)`:**
    *   `SOGI_α` on `Uα` produces `A-e^(jωt)`. (SOGI always creates a counter-clockwise quadrature signal).
    *   `SOGI_β` on `Uβ` produces `jA-e^(jωt)`.
    *   `U_out = (A-e^(jωt)) - j(jA-e^(jωt)) = A-e^(jωt) - A-e^(jωt) = 0`

The result is identical: the positive sequence is doubled, and the negative sequence is eliminated.

### 2.4 Conclusion

The Dual-SOGI structure with output cross-coupling acts as a linear operator that effectively **rejects the negative sequence component** of an `αβ` voltage vector while **passing and scaling the positive sequence component**. The final scaling block of `0.5` compensates for the doubling in amplitude. This provides a clean, balanced, fundamental positive-sequence vector ideal for robust PLL implementation in unbalanced grid conditions.



### 1. The Concept of Static Error in a SOGI

A SOGI is fundamentally a resonator, like a very precise digital tuning fork. It is designed to have a **perfect response** at its configured **center frequency (`ω₀`)**:

*   **In-Phase Output (`v'`):** At the center frequency, the gain is exactly 1 (0 dB) and the phase shift is exactly 0 degrees.
*   **Quadrature Output (`v_q'`):** At the center frequency, the gain is exactly 1 (0 dB) and the phase shift is exactly -90 degrees.

The **"static error"** is what happens to the magnitude and phase of the outputs when the **input signal's frequency (`ω`) is different from the SOGI's center frequency (`ω₀`)**. It's the steady-state error after all initial transients have died out.

We need to compute two things:
1.  **Magnitude Error:** How much the output amplitude deviates from the input amplitude.
2.  **Phase Error:** How much the output's phase deviates from the ideal 0° (for the in-phase signal) or -90° (for the quadrature signal).

### 2. The Mathematical Foundation (Transfer Functions)

To compute the error, we need the SOGI's transfer functions. For an input voltage `v(s)`, the two outputs `v'(s)` (in-phase) and `v_q'(s)` (quadrature) are:

*   **In-Phase Transfer Function `D(s)`:**
    `D(s) = v'(s) / v(s) = (k * ω₀ * s) / (s² + k * ω₀ * s + ω₀²)`

*   **Quadrature Transfer Function `Q(s)`:**
    `Q(s) = v_q'(s) / v(s) = (k * ω₀²) / (s² + k * ω₀ * s + ω₀²)`

Where:
*   `s` is the Laplace variable.
*   `ω₀` is the SOGI's center angular frequency (in rad/s), so `ω₀ = 2 * π * f₀`.
*   `ω` is the input signal's actual angular frequency (in rad/s), so `ω = 2 * π * f_input`.
*   `k` is the SOGI's damping factor or gain. A common value for good bandwidth and stability is `k = sqrt(2)`.

### 3. How to Compute the Error at a Specific Frequency

To find the static error, we evaluate the frequency response of these transfer functions by substituting `s = jω`.

#### Step 1: Substitute `s = jω`

This gives us the behavior of the filter for a sinusoidal input at frequency `ω`.

*   `D(jω) = (j * k * ω₀ * ω) / ( (jω)² + j * k * ω₀ * ω + ω₀² ) = (j * k * ω₀ * ω) / ( (ω₀² - ω²) + j * k * ω₀ * ω )`

#### Step 2: Calculate Magnitude Response

The magnitude response `|D(jω)|` tells you the gain of the filter at the input frequency `ω`.

`|D(jω)| = |j * k * ω₀ * ω| / |(ω₀² - ω²) + j * k * ω₀ * ω|`

`|D(jω)| = (k * ω₀ * ω) / sqrt( (ω₀² - ω²)² + (k * ω₀ * ω)² )`

The **Magnitude Error** is the difference between the ideal gain (1.0) and the actual gain.
**`Error_Magnitude = 1.0 - |D(jω)|`**

A positive error means the output is attenuated; a negative error means it's amplified.

#### Step 3: Calculate Phase Response

The phase response `∠D(jω)` tells you the phase shift of the filter at the input frequency `ω`.

`∠D(jω) = ∠(j * k * ω₀ * ω) - ∠((ω₀² - ω²) + j * k * ω₀ * ω)`

`∠D(jω) = 90° - atan2(k * ω₀ * ω, ω₀² - ω²) `

The **Phase Error** is this value directly, as the ideal phase shift is 0°.
**`Error_Phase = 90° - atan2(k * ω₀ * ω, ω₀² - ω²) `**

*(Note: `atan2(y, x)` is the two-argument arctangent, which correctly handles all quadrants. Many programming languages provide this function.)*

---

### 4. Practical Example

Let's compute the static error for a SOGI under these conditions:

*   **SOGI Center Frequency (`f₀`):** 50 Hz (standard grid)
*   **Actual Input Frequency (`f_input`):** 52 Hz (a grid deviation)
*   **Damping Factor (`k`):** `sqrt(2)` ≈ 1.414

**Calculations:**

1.  **Convert to rad/s:**
    *   `ω₀ = 2 * π * 50` ≈ 314.16 rad/s
    *   `ω = 2 * π * 52` ≈ 326.73 rad/s

2.  **Calculate intermediate terms:**
    *   `ω₀²` ≈ 98696
    *   `ω²` ≈ 106752
    *   `ω₀² - ω²` = -8056
    *   `k * ω₀ * ω` ≈ 1.414 * 314.16 * 326.73 ≈ 145118

3.  **Compute Magnitude Error:**
    *   `|D(jω)| = 145118 / sqrt( (-8056)² + (145118)² )`
    *   `|D(jω)| = 145118 / sqrt( 64898736 + 21059234724 )`
    *   `|D(jω)| = 145118 / 145341.4` ≈ **0.9985**
    *   **Magnitude Error** = `1.0 - 0.9985` = **0.0015** or **0.15% attenuation**. The output amplitude will be 99.85% of the input amplitude.

4.  **Compute Phase Error:**
    *   `Error_Phase = 90° - atan2(145118, -8056)`
    *   `atan2(y, x)` where `y` is positive and `x` is negative is in the 2nd quadrant.
    *   `atan2(145118, -8056)` ≈ **93.18°**
    *   **Phase Error** = `90° - 93.18°` = **-3.18°**. The output signal will lag the input signal by about 3.2 degrees.

### Conclusion

The static error is an inherent property of the SOGI's frequency response. This is precisely why a SOGI is used inside a **Phase-Locked Loop (PLL)**. The PLL's job is to:
1.  Detect the phase error (using a PI controller on the quadrature output, for example).
2.  Use that error to estimate the true grid frequency (`ω`).
3.  Continuously update the SOGI's center frequency (`ω₀`) to match the estimated frequency (`ω`).

When the PLL is locked, `ω₀ ≈ ω`, and the static errors (both magnitude and phase) are driven to zero, allowing the SOGI to function perfectly as intended.