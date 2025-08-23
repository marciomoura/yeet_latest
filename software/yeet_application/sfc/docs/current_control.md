# Dynamic Model of the LCI DC-Link Current Control Loop

## 1. Objective

The goal of this document is to derive the complete open-loop transfer function, `G_OL(s)`, for the LCI's main DC-link current control loop. This transfer function represents the "plant" as seen by the PI controller. It models the dynamic relationship between a change in the controller's output (the rectifier firing angle command, `α`) and the signal fed back to the controller (the filtered DC-link current measurement, `I_dc,f`).

The open-loop transfer function `G_OL(s)` is the combined transfer function of the three blocks in the forward path: **Rectifier, DC-Link Circuit, and Measurement Filter.**

$$ G_{OL}(s) = \frac{\Delta I_{dc,f}(s)}{\Delta \alpha(s)} = G_{rectifier}(s) \cdot G_{plant}(s) \cdot G_{filter}(s) $$

## 2. Derivation of Component Transfer Functions

### 2.1. The Actuator: Source-Side Converter (Rectifier) - `G_rectifier(s)`

The rectifier converts the firing angle command `α` into a corresponding average DC voltage `V_dc`. This process involves a variable gain and a transport delay.

*   **Gain (K_r):** The average DC voltage is `V_dc = V_{do} \cos(\alpha)`, where `V_{do}` is the ideal no-load DC voltage. We linearize this function around a steady-state operating point `α_0` to find the small-signal gain:
    $$ K_r = \frac{d(V_{dc})}{d\alpha} \bigg|_{\alpha=\alpha_0} = -V_{do} \sin(\alpha_0) $$
    **Note:** The gain `K_r` is not constant. It varies with the operating point, being largest at `α=90°` and smallest near `α=0°` or `α=180°`. Your controller must be robust to these gain changes.

*   **Delay (T_d):** Thyristor converters can only change their output voltage at discrete firing instants. This introduces a small transport delay. For a 6-pulse, 60 Hz system, the average delay is:
    $$ T_d = \frac{1}{2 \cdot p \cdot f} = \frac{1}{2 \cdot 6 \cdot 60} \approx 1.39 \, \text{ms} $$
    This delay is modeled as `e^{-sT_d}`. For linear analysis, we often use a first-order Padé approximation:
    $$ e^{-sT_d} \approx \frac{1 - sT_d/2}{1 + sT_d/2} $$

*   **Combined Rectifier Transfer Function:**
    $$ G_{rectifier}(s) = K_r \cdot e^{-sT_d} $$

### 2.2. The Plant: DC-Link Circuit - `G_plant(s)`

The DC-link itself is modeled as a series RL circuit. The voltage equation is:
$$ V_{dc,rect}(t) - V_{dc,inv}(t) = R_{dc} I_{dc}(t) + L_{dc} \frac{dI_{dc}(t)}{dt} $$
For control purposes, the back-EMF from the inverter (`V_dc,inv`) is treated as a disturbance. The transfer function from the rectifier voltage (`V_dc`) to the link current (`I_dc`) is found by taking the Laplace transform:
$$ G_{plant}(s) = \frac{\Delta I_{dc}(s)}{\Delta V_{dc}(s)} = \frac{1}{sL_{dc} + R_{dc}} $$
This is a standard first-order low-pass system which can be written as:
$$ G_{plant}(s) = \frac{1/R_{dc}}{(\frac{L_{dc}}{R_{dc}})s + 1} = \frac{K_{dc}}{\tau_{dc}s + 1} $$
Where:
*   **DC Gain (K_dc):** `1 / R_dc`
*   **Time Constant (τ_dc):** `L_dc / R_dc`. The large inductor in an LCI makes this the dominant (slowest) time constant in the system.

### 2.3. The Sensor: Measurement Filter - `G_filter(s)`

You specified a 2nd order low-pass filter to process the DC-link current measurement. This is essential for removing switching ripple and noise before the signal is fed to the PI controller. The standard transfer function for a second-order filter is:
$$ G_{filter}(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2} $$
Where:
*   **Natural Frequency (ω_n):** This determines the filter's cutoff frequency. It must be chosen high enough to not interfere with the control loop's desired bandwidth, but low enough to effectively attenuate noise.
*   **Damping Ratio (ζ):** This determines the filter's response shape. A value of `ζ = 0.707` gives a maximally flat (Butterworth) response, while lower values can cause overshoot.

## 3. The Complete Open-Loop Transfer Function

Combining the three components gives the final open-loop transfer function from the firing angle command to the filtered current measurement.

$$ G_{OL}(s) = G_{rectifier}(s) \cdot G_{plant}(s) \cdot G_{filter}(s) $$

Substituting the individual expressions:

$$ G_{OL}(s) = \left[ -V_{do} \sin(\alpha_0) \cdot e^{-sT_d} \right] \cdot \left[ \frac{1}{sL_{dc} + R_{dc}} \right] \cdot \left[ \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2} \right] $$

This can be written more cleanly as:

$$ G_{OL}(s) = \frac{-V_{do} \sin(\alpha_0) \cdot \omega_n^2}{(sL_{dc} + R_{dc})(s^2 + 2\zeta\omega_n s + \omega_n^2)} e^{-sT_d} $$

## 4. The PI Controller Model

You are using a PI controller with parameters `Kp` (proportional gain) and `Ti` (integral time). Its transfer function is:
$$ G_c(s) = K_p \left( 1 + \frac{1}{T_i s} \right) = \frac{K_p (T_i s + 1)}{T_i s} $$
The total loop gain, which is used for stability analysis (e.g., Bode or Nyquist plots), would be `L(s) = G_c(s) \cdot G_{OL}(s)`.

## 5. Summary and Implications for Control Design

*   **System Order:** Your open-loop system is a **third-order system** (one pole from the DC-link, two poles from the filter) with an additional **pure time delay**.
*   **Variable Gain:** The system gain is proportional to `sin(α_0)`. This means the loop is "hotter" (higher gain) at firing angles near 90 degrees and more sluggish at very low or very high firing angles. Your PI tuning must provide adequate stability margins across the entire range of operation.
*   **Phase Lag:** The combination of the DC-link time constant (`τ_dc`), the filter, and the transport delay (`T_d`) introduces significant phase lag at higher frequencies. This fundamentally limits the achievable bandwidth of your current loop. Attempting to make the controller too fast (high `Kp`, low `Ti`) will lead to instability.
*   **Tuning Strategy:** A common approach is to tune the PI controller to cancel the slowest pole of the system, which is the DC-link pole at `s = -1/τ_dc`. This is achieved by setting the controller's zero to match the plant's pole, i.e., by setting **`Ti = τ_dc = L_dc / R_dc`**. The gain `Kp` is then adjusted to achieve the desired phase margin and bandwidth. 

---

## 6. Example: Applying System Data to the Model

In this section, we will apply the provided system parameters to the derived transfer function. This will give us a concrete numerical model for your specific LCI system, which can be used for simulation and controller tuning in tools like MATLAB/Simulink or Python.

### 6.1. Given System Parameters

First, let's list the known values:

*   **System Frequency (f):** 50 Hz
*   **Source-Side AC Voltage (V_LL_grid):** 2090 V (RMS, Line-to-Line)
*   **DC-Link Inductance (L_dc):** 8 mH = 0.008 H
*   **DC-Link Resistance (R_dc):** 0.05 Ω
*   **Converter Pulse Number (p):** 6 (This is a standard assumption for LCI starters)

### 6.2. Calculation of Model Parameters

Now, we will calculate each parameter for the transfer function blocks.

#### **a) Rectifier Parameters (`G_rectifier`)**

1.  **Ideal No-Load DC Voltage (V_do):**
    This is the maximum possible DC voltage from the rectifier.
    $$ V_{do} = \frac{3\sqrt{2}}{\pi} V_{LL,grid} \approx 1.35 \cdot V_{LL,grid} $$
    $$ V_{do} = 1.35 \cdot 2090 \, \text{V} = 2821.5 \, \text{V} \approx \mathbf{2822 \, V} $$

2.  **Rectifier Gain (K_r):**
    The gain depends on the operating point `α_0`. We will leave it in symbolic form. Remember that the firing angle `α` in the formula must be in radians.
    $$ K_r = -V_{do} \sin(\alpha_0) = \mathbf{-2822 \sin(\alpha_0) \, [V/rad]} $$
    *   *Example at startup:* At low speed, the back-EMF from the motor is low, so `V_dc` must be low. This requires `α_0` to be close to 90° (`π/2` rad). Here, `sin(90°) = 1`, and the gain is at its maximum: `K_r ≈ -2822 V/rad`.
    *   *Example near full speed:* As the motor speeds up, the back-EMF increases, and `α_0` is reduced (e.g., to 30°) to provide more voltage. The gain becomes `K_r = -2822 \sin(30°) = -1411 V/rad`.

3.  **Transport Delay (T_d):**
    The average delay for a 6-pulse, 50 Hz converter.
    $$ T_d = \frac{1}{2 \cdot p \cdot f} = \frac{1}{2 \cdot 6 \cdot 50 \, \text{Hz}} = \frac{1}{600} \approx 0.00167 \, \text{s} = \mathbf{1.67 \, ms} $$

#### **b) DC-Link Plant Parameters (`G_plant`)**

1.  **Plant DC Gain (K_dc):**
    $$ K_{dc} = \frac{1}{R_{dc}} = \frac{1}{0.05 \, \Omega} = \mathbf{20 \, A/V} $$

2.  **Plant Time Constant (τ_dc):**
    This is the dominant time constant of the system.
    $$ \tau_{dc} = \frac{L_{dc}}{R_{dc}} = \frac{0.008 \, \text{H}}{0.05 \, \Omega} = \mathbf{0.16 \, s} \quad (160 \, \text{ms}) $$

The transfer function for the DC-Link is therefore:
$$ G_{plant}(s) = \frac{20}{0.16s + 1} $$

#### **c) Measurement Filter Parameters (`G_filter`)**

The parameters for the 2nd order low-pass filter (`ω_n`, `ζ`) were not provided. These are design choices. For this example, let's select typical values for a filter that needs to remove 6-pulse ripple (300 Hz) without adding excessive phase lag at the control bandwidth.

*   Let's choose a **Butterworth** response (`ζ = 0.707`) for a flat passband.
*   Let's set the natural frequency `ω_n` to give a cutoff around 100 Hz (`ω_n = 2π * 100 = 628` rad/s).

So, the filter transfer function is:
$$ G_{filter}(s) = \frac{628^2}{s^2 + 2(0.707)(628)s + 628^2} = \frac{394384}{s^2 + 888s + 394384} $$

### 6.3. Final Open-Loop Transfer Function (with Numerical Values)

Now we assemble the complete open-loop transfer function `G_OL(s)`:

$$ G_{OL}(s) = \left[ -2822 \sin(\alpha_0) \cdot e^{-0.00167s} \right] \cdot \left[ \frac{20}{0.16s + 1} \right] \cdot \left[ \frac{394384}{s^2 + 888s + 394384} \right] $$

Combining the constant terms:
$$ G_{OL}(s) = \frac{-2.23 \times 10^{10} \sin(\alpha_0)}{(0.16s + 1)(s^2 + 888s + 394384)} e^{-0.00167s} $$

This is the complete, numerical open-loop transfer function for your system.

### 6.4. PI Controller Tuning Recommendation

Your PI Controller has the form `G_c(s) = K_p (1 + 1/(T_i s))`. A highly effective and common strategy for this type of system is **pole-zero cancellation**.

1.  **Set the Integral Time (T_i):**
    We set the controller's zero to cancel the plant's dominant pole. This simplifies the system dynamics significantly.
    $$ T_i = \tau_{dc} = \frac{L_{dc}}{R_{dc}} $$
    $$ \mathbf{T_i = 0.16 \, s} $$

2.  **Determine the Proportional Gain (K_p):**
    With this value of `T_i`, the PI controller becomes `G_c(s) = K_p \frac{(0.16s + 1)}{0.16s}`. The `(0.16s + 1)` terms in the controller and plant will cancel out. The loop gain is now simplified, and you can choose `K_p` based on the remaining dynamics (the integrator, the filter, and the delay) to achieve a desired phase margin (typically 45-60 degrees) for stability and performance. `K_p` will be the primary tuning parameter to adjust the speed of response.

### Summary Table of the Example Model

| Parameter | Symbol | Calculated Value | Description |
| :--- | :--- | :--- | :--- |
| Ideal DC Voltage | `V_do` | 2822 V | Max rectifier output voltage |
| Rectifier Gain | `K_r` | `-2822 sin(α_0)` V/rad | Variable gain of the actuator |
| Transport Delay | `T_d` | 1.67 ms | Inherent delay of the converter |
| DC-Link Gain | `K_dc` | 20 A/V | Steady-state current per volt |
| DC-Link Time Constant | `τ_dc` | 160 ms | Dominant system time constant |
| Recommended Integral Time | `T_i` | 160 ms | For pole-zero cancellation tuning |