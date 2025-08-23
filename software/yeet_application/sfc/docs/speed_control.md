# Comprehensive Guide to the LCI Speed Controller for Soft-Starter Applications

## 1. Role in the LCI Drive System

The speed control loop is the **outer master loop** in the cascaded control architecture of a Line-Commutated Inverter (LCI) drive. Its primary responsibility is to regulate the mechanical speed of the synchronous motor, ensuring it precisely follows a desired speed reference. While the inner current loop executes the high-speed task of torque production, the speed loop provides the intelligence, dictating how much torque is required to achieve the overall speed objective.

In **soft-starter applications**, the role of the speed controller is particularly critical. It must guide the motor from standstill to its target speed along a carefully defined trajectory, ensuring a smooth, controlled ramp-up that minimizes both electrical stress on the grid and mechanical stress on the motor and its load (like a gas turbine).

---

## 2. Cascaded Control Architecture

The LCI drive employs a two-loop cascaded structure for robust performance:

*   **Inner Loop (Current Control):** A fast, high-bandwidth loop that regulates DC link current ($I_d$), which is directly proportional to motor torque ($T_e$). It receives its setpoint from the outer speed loop.
*   **Outer Loop (Speed Control):** A slower, lower-bandwidth loop that regulates motor speed ($\omega_m$). It calculates the necessary torque (as a current reference) required to make the measured speed match the speed reference.

This separation allows the loops to be tuned independently, ensuring stability. The current loop handles the fast dynamics of the electrical system, while the speed loop manages the much slower dynamics of the mechanical system (motor and load inertia).


```
                  (+)           (+)
         +-----+     +----------+    Id_ref     +-----+     +-----------+   cos(α)_cmd   +--------+   α
ω_ref -->|  Σ  |---->| Speed PI |-------------->|  Σ  |---->| Current PI|--------------->| arccos |---+
         +--^--+     +----------+               +--^--+     +-----------+                +--------+   |
            | (-)                                  | (-)                                              |
            |                                      |                                                  V
            |                                      |                                        +-----------------+
            |                                      |                                        |                 |
            |                                      +----------------------------------------+   PLANT MODEL   |
            |                                                                               |  (Electrical &  |
            +-------------------------------------------------------------------------------+   Mechanical)   |
                                                                                            |                 |
                                                                                            +--------+--------+
                                                                                                |        |
                                                                                                |       (ω_m)
                                                                                                (Id)
```
*Note: The `arccos` block is a key element that linearizes the rectifier's response. By having the current PI controller output a voltage command (which is then converted to a firing angle `α`), the highly non-linear gain of the rectifier is cancelled out, allowing the current controller to operate with constant, robust gains.*

---

## 3. Architecture and Mathematics of the Speed Controller

The speed controller is fundamentally a Proportional-Integral (PI) controller.

#### **Step 1: Error Calculation**
The controller continuously calculates the error between the desired speed and the measured speed.

$$ e_s(t) = \omega_{ref}(t) - \omega_m(t) $$

*   $e_s(t)$: The speed error at time $t$.
*   $\omega_{ref}(t)$: The desired speed reference at time $t$.
*   $\omega_m(t)$: The measured motor speed feedback at time $t$.

#### **Step 2: The PI Controller**
The PI controller processes this error signal to compute the required DC link current reference, $I_{d,ref}$.

$$ I_{d,ref}(t) = K_p e_s(t) + K_i \int_0^t e_s(\tau) d\tau $$

*   **Proportional Term ($K_p e_s(t)$):** This term provides an immediate response that is proportional to the current error. A larger error results in a larger torque command. It is responsible for the primary corrective action.
*   **Integral Term ($K_i \int e_s(\tau) d\tau$):** This term accumulates the error over time. Its purpose is to eliminate any steady-state error, ensuring that the motor speed eventually settles exactly at the reference value. It corrects for constant disturbances, such as a steady load torque.

#### **Step 3: Feedforward Term (Optional)**
To improve the dynamic response and reduce the burden on the feedback controller, a feedforward term can be added. This term pre-emptively calculates the torque required to accelerate the motor according to the reference profile.

The total current reference becomes:
$$ I_{d,ref}(t) = \text{PI Output} + I_{d,ff}(t) = \left( K_p e_s(t) + K_i \int_0^t e_s(\tau) d\tau \right) + \frac{J}{K_t} \frac{d\omega_{ref}(t)}{dt} $$

*   $I_{d,ff}(t)$: The feedforward current component.
*   $J$: The total inertia of the motor and its connected load (in kg·m²).
*   $K_t$: The motor's torque constant (in Nm/A).
*   $\frac{d\omega_{ref}(t)}{dt}$: The rate of change (acceleration) of the speed reference.

By directly commanding the current needed to accelerate the known system inertia, the feedforward term allows the PI controller to focus on correcting for disturbances and model inaccuracies, leading to much tighter tracking of the speed ramp.

#### **Step 4: Anti-Windup Mechanism**
The output of the speed controller, $I_{d,ref}$, is fed directly into the current controller, which has strict limits ($I_{d,min}$, $I_{d,max}$). If the speed controller requests a current that exceeds these limits, the actual current will be saturated. Without a protective mechanism, the integral term of the speed PI controller would continue to accumulate (or "wind up"), leading to a large overshoot once the error is resolved.

**Anti-windup** is essential. A common technique is **integrator clamping**. When the output $I_{d,ref}$ is saturated, the integrator is temporarily disabled, preventing it from accumulating further. It is re-enabled once the output is no longer saturated.

---

## 4. Speed Reference Generation for Soft-Starters

For a soft-starter, the speed reference $\omega_{ref}(t)$ is not a simple step command. It is a carefully profiled trajectory designed to ensure smooth acceleration. This involves several key processing steps:

1.  **Limiting:** The raw speed setpoint is first clamped between a configurable minimum and maximum value ($\omega_{min}$, $\omega_{max}$). This ensures the reference always stays within safe operational bounds.
2.  **Rate Limiting (Slew Rate):** To prevent sudden jumps in the reference, the change in speed is limited to a maximum acceleration and deceleration rate. This is the core of the "ramp" generation.

A typical profile consists of one or more linear ramps:

$$ \omega_{ref}(t) = \omega_{start} + A \cdot t $$

*   $\omega_{start}$: The speed at the beginning of the ramp.
*   $A$: The acceleration rate (in rad/s²), constrained by the rate limiter.
*   $t$: Time elapsed since the start of the ramp.

The control system generates this profile dynamically. For example, a simple soft-start profile might be:
1.  **Hold at zero speed** during motor magnetization.
2.  **Ramp 1:** Accelerate at a low rate $A_1$ to overcome breakaway torque and establish rotation.
3.  **Ramp 2:** Accelerate at a higher rate $A_2$ through the main operating range.
4.  **Ramp 3:** Reduce acceleration to a lower rate $A_3$ to smoothly approach the final target speed and avoid overshoot.

The speed controller's job is to ensure the actual motor speed $\omega_m(t)$ follows this generated profile $\omega_{ref}(t)$ as closely as possible.

---

## 5. Speed Feedback and Tuning

#### Speed Measurement ($\omega_m$)
Accurate and clean speed feedback is crucial. It is typically obtained from a **resolver** or **encoder** mounted on the motor shaft. However, the signal that reaches the speed controller is not an instantaneous, perfect measurement of the true mechanical speed. It is the result of a multi-stage signal processing chain that introduces significant, and often dominant, dynamic lag.

### 5.1 The Speed Measurement Path as a Dynamic System

It is a common and critical mistake to assume the speed feedback is instantaneous. The measurement path itself is a dynamic system that acts as a low-pass filter, introducing phase lag that directly impacts the stability of the speed controller. This path typically consists of two key stages:

1.  **SRF-PLL for Frequency Estimation:** The Synchronous Reference Frame Phase-Locked Loop (SRF-PLL) is used to estimate the grid's electrical frequency from the motor's back-EMF or sensor position. The PLL is not a static calculator; it is a closed-loop control system in its own right. As such, it has its own dynamic response, which closely resembles a **second-order low-pass filter**. The bandwidth and damping of this "filter" are determined by the tuning of the PLL's internal PI controller.

2.  **Additional Low-Pass Filtering:** The raw frequency estimate from the PLL is often noisy. To provide a clean signal to the speed controller, an additional explicit low-pass filter (typically first or second-order) is applied.

The crucial insight is that these two stages are **cascaded**. The total dynamic behavior of the measurement path is the product of these individual stages. This cumulative filtering action is often the primary factor that limits how fast the speed controller can be.

#### Tuning the Speed Loop
The speed loop must be tuned to be significantly **slower** than both the inner current loop and, critically, the speed measurement path. A common rule of thumb is for the speed loop's bandwidth to be 5 to 10 times lower than the bandwidth of the components it depends on. The significant phase lag introduced by the measurement chain means that if the speed controller is tuned to be too aggressive (too high bandwidth), it will become unstable, regardless of how well the current loop is performing.

---

## 6. Controller Tuning and Bandwidth Allocation

The stability and performance of the entire LCI drive depend on the correct tuning of the cascaded control loops. The fundamental principle is **timescale separation**: the inner loop must be significantly faster than the outer loop. This ensures that the inner loop can be modeled as a simple, fast-responding gain from the perspective of the slower outer loop.

#### **Inner Loop: Current Controller**

*   **Objective:** To make the DC link current respond as quickly as possible to the reference from the speed controller.
*   **Governing Dynamics:** The primary factor limiting the current loop's response is the electrical time constant of the DC link, defined by its inductance ($L_{dc}$) and resistance ($R_{dc}$).
*   **Typical Bandwidth:** A typical current loop bandwidth is in the range of **100 to 500 rad/s** (approximately 15 to 80 Hz).

#### **Outer Loop: Speed Controller**

*   **Objective:** To regulate the mechanical speed of the motor and load.
*   **Governing Dynamics:** The speed loop's response is dominated by the mechanical time constant of the system, which is determined by the total **inertia** of the motor and the connected load (e.g., the gas turbine).
*   **Typical Bandwidth:** To ensure stability, the speed loop must be significantly slower than the current loop. A common rule of thumb is to set the speed loop bandwidth to be **5 to 10 times lower** than the current loop bandwidth. A typical speed loop bandwidth is in the range of **5 to 50 rad/s** (approximately 0.8 to 8 Hz).

#### **Tuning Methodology**

1.  **Tune the Current Loop First:** The speed loop is disabled. The current loop is tuned by applying a step change to the current reference ($I_{d,ref}$) and adjusting its $K_p$ and $K_i$ gains to achieve a fast, stable response with minimal overshoot.
2.  **Tune the Speed Loop:** Once the current loop is tuned and stable, it can be considered a single, fast-responding block. The speed loop is then enabled, and its $K_p$ and $K_i$ gains are adjusted by applying a step change to the speed reference ($\omega_{ref}$) to achieve the desired acceleration and settling time.

---

## 7. High-Fidelity Speed Controller Plant Model for Tuning

To systematically tune the speed PI controller, we need a high-fidelity transfer function model of the system it commands. This "plant" represents everything in the loop *except* for the speed controller itself. The model describes the dynamic relationship between the speed controller's output (the current reference, $I_{d,ref}$) and its feedback input (the measured motor speed, $\omega_{m,meas}$).

#### **1. The Closed-Loop Current Controller ($G_{CL,I}(s)$)**
From the perspective of the slow outer speed loop, the entire fast inner current loop is represented by its **actual closed-loop transfer function** derived from the current loop's design.

$$ G_{CL,I}(s) = \frac{I_d(s)}{I_{d,ref}(s)} = \frac{G_{OL,I}(s)}{1 + G_{OL,I}(s)} $$

#### **2. Control Task Execution Delay ($G_{delay}(s)$)**
The discrete execution of the speed control algorithm introduces a pure transport delay.

$$ G_{delay}(s) = e^{-s T_{delay}} $$

#### **3. Torque Production ($K_t$)**
The electromagnetic torque ($T_e$) is proportional to the DC link current ($I_d$).

$$ T_e(s) = K_t \cdot I_d(s) $$

#### **4. Linearized Mechanical System ($G_{mech}(s)$)**
For a gas turbine with a quadratic load torque ($T_{load} = k \cdot \omega^2$), the non-linear dynamics are linearized around a steady-state operating speed, $\omega_0$.

$$ G_{mech}(s) = \frac{\Delta\omega(s)}{\Delta T_e(s)} = \frac{1}{J s + B_{eff}} $$

*   $J$: Total system inertia (motor rotor + turbine) in kg·m².
*   $k$: The quadratic load constant in Nm/(rad/s)².
*   $B_{eff} = 2k\omega_0$: The effective viscous damping coefficient at the operating speed $\omega_0$.

#### **5. Speed Measurement Path Dynamics ($G_{meas}(s)$)**
This block represents the entire signal processing chain from the true mechanical speed ($\omega_m$) to the final measured signal ($\omega_{m,meas}$) used by the controller. As discussed in Section 5.1, this is a cascade of the SRF-PLL and an additional filter.

The SRF-PLL's closed-loop response from actual frequency to estimated frequency is accurately modeled as a second-order low-pass filter:
$$ G_{PLL}(s) = \frac{\omega_{n,PLL}^2}{s^2 + 2\zeta_{PLL}\omega_{n,PLL} s + \omega_{n,PLL}^2} $$
*   $\omega_{n,PLL}$: The natural frequency of the PLL, set by its PI tuning.
*   $\zeta_{PLL}$: The damping ratio of the PLL, also set by its PI tuning.

The additional filter is also modeled as a second-order low-pass filter:
$$ G_{filter}(s) = \frac{\omega_{n,filt}^2}{s^2 + 2\zeta_{filt}\omega_{n,filt} s + \omega_{n,filt}^2} $$
*   $\omega_{n,filt}$: The cutoff frequency of the explicit filter.
*   $\zeta_{filt}$: The damping ratio of the explicit filter (e.g., $\zeta=0.707$ for a Butterworth response).

The total transfer function for the measurement path is the product of these two, resulting in a **fourth-order system**:
$$ G_{meas}(s) = G_{PLL}(s) \cdot G_{filter}(s) = \left[ \frac{\omega_{n,PLL}^2}{s^2 + 2\zeta_{PLL}\omega_{n,PLL} s + \omega_{n,PLL}^2} \right] \cdot \left[ \frac{\omega_{n,filt}^2}{s^2 + 2\zeta_{filt}\omega_{n,filt} s + \omega_{n,filt}^2} \right] $$
This fourth-order lag is often the dominant dynamic that limits the achievable bandwidth of the speed control loop.

#### **Total Open-Loop Plant Model ($G_{OL,speed}(s)$)**

The complete plant model is the product of these stages:

$$ G_{OL,speed}(s) = G_{CL,I}(s) \cdot G_{delay}(s) \cdot K_t \cdot G_{mech}(s) \cdot G_{meas}(s) $$

$$ G_{OL,speed}(s) = \left[ \frac{G_{OL,I}(s)}{1 + G_{OL,I}(s)} \right] \cdot \left[ e^{-s T_{delay}} \right] \cdot \left[ K_t \right] \cdot \left[ \frac{1}{J s + B_{eff}} \right] \cdot \left[ G_{PLL}(s) \cdot G_{filter}(s) \right] $$

The resulting transfer function, $G_{OL,speed}(s)$, has units of **[rad/s / A]**, representing the change in speed output for a given change in current reference input. This model is used in simulation tools to calculate the speed controller's PI gains.

---

## 8. Alternative: Per-Unit (pu) System Modeling for Universal Tuning

While the SI-unit model in Section 7 is physically intuitive, a **per-unit (pu) system** offers a more standardized and universal approach. It normalizes all variables against a base value (e.g., nominal speed, nominal torque), making the control model independent of the machine's specific power or voltage rating. This is standard practice in high-power drives engineering.

#### **Per-Unit Mechanical System ($G_{mech,pu}(s)$)**
The per-unit equation of motion uses the inertia constant, **H** (in seconds), which is a more practical and scalable measure of inertia than $J$.

$$ G_{mech,pu}(s) = \frac{\Delta\omega_{pu}(s)}{\Delta T_{e,pu}(s)} = \frac{1}{2H s + D_{eff}} $$

*   **$D_{eff} = 2k_{pu}\omega_{0,pu}$**: The effective per-unit viscous damping coefficient.

#### **Simplification of Torque Production**
In a consistent per-unit system, the relationship between per-unit electromagnetic torque and per-unit DC link current becomes a direct 1-to-1 mapping. The physical torque constant block ($K_t$) is replaced with a simple gain of **1.0**.

#### **Total Per-Unit Open-Loop Plant Model ($G_{OL,speed,pu}(s)$)**

$$ G_{OL,speed,pu}(s) = G_{CL,I}(s) \cdot G_{delay}(s) \cdot (1.0) \cdot G_{mech,pu}(s) \cdot G_{meas}(s) $$

$$ G_{OL,speed,pu}(s) = \left[ \frac{G_{OL,I}(s)}{1 + G_{OL,I}(s)} \right] \cdot \left[ e^{-s T_{delay}} \right] \cdot \left[ \frac{1}{2H s + D_{eff}} \right] \cdot \left[ G_{PLL}(s) \cdot G_{filter}(s) \right] $$

Note that the transfer function for the measurement path, $G_{meas}(s)$, remains unchanged as its dynamics are defined by absolute frequencies and time constants, which are independent of the per-unit system's base values.

---

## 9. Advanced Considerations: Gain Scheduling

The linearization in Section 7 shows that the mechanical plant's effective damping (`B_eff`) is not constant but increases linearly with speed. A single set of PI gains tuned for one speed (e.g., 50% nominal) may be too sluggish at high speed or too aggressive at low speed.

To optimize performance across the entire starting ramp, an advanced technique called **Gain Scheduling** is often employed. In this strategy, the speed controller's gains ($K_p$ and $K_i$) are not fixed values. Instead, they are functions of the measured speed ($\omega_m$), automatically adjusting to maintain a consistent stability margin and dynamic response as the plant characteristics change. This ensures robust performance from breakaway to full speed.