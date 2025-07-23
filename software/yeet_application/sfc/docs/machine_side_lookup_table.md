# LCI Control: Constant Extinction Angle (γ) Strategy

## 1. Overview

The primary control strategy for the machine-side converter (inverter) in the Load Commutated Inverter (LCI) system is **Constant Extinction Angle Control**, also known as "Constant Gamma (γ) Control." This is a robust, feed-forward method that ensures safe and efficient operation of the synchronous machine as a motor during the startup sequence.

The core objective is to adjust the inverter's firing angle, **beta (β)**, in real-time to maintain a constant, safe **extinction angle (γ)** across all operating speeds and loads.

### Key Terms:

*   **Beta (β):** The inverter **firing advance angle**. It is the angle by which a thyristor is fired *before* the natural commutation point of the machine's back-EMF.
*   **Gamma (γ):** The **extinction angle**. This is the safety margin, in electrical degrees, that ensures an outgoing thyristor has fully turned off and recovered its blocking capability before it becomes forward-biased again. A failure to maintain a positive gamma results in a **commutation failure**.
*   **Mu (μ):** The **commutation overlap angle**. The angle during which two thyristors in different phases are conducting simultaneously as the DC current transfers from one to the other.

These angles are physically related by the equation: `β = γ + μ`.

## 2. Control Philosophy

The control system aims to achieve two competing goals:

1.  **Safety:** Guarantee that `γ` never falls below a minimum safe value (`γ_min`), preventing commutation failures. A typical `γ_min` is set between 15° and 20°.
2.  **Efficiency:** Operate the machine at the highest possible power factor. The machine-side power factor is directly related to the firing angle by `cos(φ) ≈ cos(β)`. To maximize `cos(β)`, `β` must be as small as possible.

The Constant Gamma Control strategy resolves this conflict by operating the inverter at the smallest possible `β` that still guarantees the minimum safe `γ`.

## 3. Implementation: 2D Lookup Table (LUT)

The relationship between the required firing angle `β`, machine state (speed and current), and the desired extinction angle `γ` is predictable and can be pre-calculated. This makes a 2D Lookup Table an ideal implementation.

### 3.1. Theoretical Foundation

The strategy is based on the fundamental steady-state equation for an LCI inverter:

```
cos(β) = cos(γ) + d_xM
```

where `d_xM` is the per-unit inductive voltage drop caused by commutation. This term is a function of the DC link current and the machine's commutation reactance at a given frequency:

```
d_xM = (3/π) * X_comm_pu_eff * I_d_pu
```

The effective commutation reactance `X_comm_pu_eff` scales with frequency:

```
X_comm_pu_eff = X_comm_pu_base * (f_M / f_M_base)
```

Combining these, we get the final equation used to populate the lookup table:

```
β = arccos( cos(γ_setpoint) + ( (3/π) * X_comm_pu_base * (f_M / f_M_base) ) * (I_d / I_d_base) )
```

### 3.2. Lookup Table Structure

The control is implemented as a 2D table where:

*   **Input Axis 1 (Rows):** Machine Speed / Frequency (`f_M`)
*   **Input Axis 2 (Columns):** DC Link Current (`I_d`)
*   **Output (Cell Value):** The required firing angle `β` in degrees.

### 3.3. Workflow

The implementation is a two-stage process:

#### Stage 1: Offline Generation

A script (`generate_beta_lookup_table.py` or similar) is run once during the design phase.
1.  **Load System Parameters:** It imports the master system parameters, including `X_comm_pu_base`, `f_M_base`, and `I_d_base`.
2.  **Define Control Policy:** A target `γ_setpoint` (e.g., 18.0°) is defined.
3.  **Define Axes:** Vectors representing the discrete points for the speed and current axes are created.
4.  **Calculate Table:** The script iterates through every combination of speed and current on the axes, calculates the corresponding `β` using the formula above, and stores it in a 2D matrix.
5.  **Save Data:** The axes vectors and the 2D `β` matrix are saved to a data file (e.g., `.mat` or `.csv`).

#### Stage 2: Online Execution (in the Controller)

The real-time controller executes the following logic at each time step:
1.  **Load Data:** At initialization, the controller loads the pre-calculated `β` table and axes data into memory.
2.  **Measure Inputs:** The controller measures the current machine frequency `f_M_measured` and DC link current `I_d_measured`.
3.  **Lookup & Interpolate:** It uses these two measured values as coordinates to find the appropriate `β` from the table. **Bilinear interpolation** is used to calculate a smooth `β` value for operating points that fall between the grid points of the table.
4.  **Apply Output:** The resulting `β` is sent to the gate pulse generator for the machine-side converter.

## 4. Advantages and Disadvantages

### Advantages:

*   **Robust and Reliable:** As a feed-forward method, it is not prone to the instability issues of a fast-acting closed-loop controller.
*   **Optimal Power Factor:** By operating at the minimum safe extinction angle, it inherently maximizes the machine's power factor for any given condition.
*   **Computationally Efficient:** A 2D lookup with interpolation is a very fast operation, making it suitable for controllers with limited processing power.

### Disadvantages:

*   **Model-Dependent:** Its accuracy is directly tied to the accuracy of the machine parameters (especially commutation reactance) used for the offline calculation.
*   **Not Adaptive:** It cannot automatically compensate for parameter drift due to temperature, magnetic saturation, or other real-world effects. A fixed safety margin in `γ` is typically included to account for this.