# Steady-State Equations for LCI Systems

[← Back to Documentation Index](index.md)

This documentation provides a comprehensive list and derivation of key steady-state equations for Load-Commutated Inverter (LCI) systems. The analysis begins with a first-principles derivation of the DC voltage and then expands to cover all major operating characteristics. The system is modeled using the standard 6-pulse thyristor converter, which is the fundamental building block for both the source-side rectifier and the load-side inverter.

## 1. Fundamental Assumptions for Analysis

To derive the fundamental equations, we make the following standard assumptions:
*   **Balanced AC Source**: The AC voltage source (grid or machine) is perfectly balanced and sinusoidal.
*   **Ideal Thyristors**: The thyristors are treated as ideal switches with zero forward voltage drop, infinite blocking voltage, and instantaneous switching.
*   **Continuous, Ripple-Free DC Current**: The DC link inductor ($L_d$) is large enough to maintain a perfectly smooth DC current ($I_d$) without ripple.
*   **Commutating Inductance**: The AC side has a per-phase inductance ($L_c$) which prevents instantaneous changes in current. This is crucial for understanding commutation.

---

## 2. Derivation of Average DC Voltage

The core of the LCI is the relationship between the AC side voltage and the resulting DC side voltage, which is controlled by the thyristor firing angle.

### 2.1 Ideal DC Voltage (Without Commutation Overlap)

First, we derive the theoretical average DC voltage from a 6-pulse bridge, assuming no AC side inductance ($L_c = 0$) and thus no commutation overlap.

#### Circuit and Principle
The circuit consists of six thyristors in a full-wave bridge. At any time, one thyristor from the top group (T1, T3, T5) and one from the bottom group (T4, T6, T2) conduct. The converter logic fires the thyristor connected to the most positive phase for the top group and the most negative phase for the bottom group. This ensures the output DC voltage, $v_d$, is constructed from segments of the AC **line-to-line voltages**. The output voltage waveform repeats every 60° ($\pi/3$ radians).

#### Voltage Definitions
Let the line-to-neutral (phase) voltages be:
$ v_{aN} = V_{ph,peak} \sin(\omega t) $
$ v_{bN} = V_{ph,peak} \sin(\omega t - 2\pi/3) $
$ v_{cN} = V_{ph,peak} \sin(\omega t + 2\pi/3) $

The output is formed by the line-to-line voltages, e.g., $v_{ab}$:
$ v_{ab} = v_{aN} - v_{bN} = \sqrt{3} V_{ph,peak} \sin(\omega t + \pi/6) = V_{LL,peak} \sin(\omega t + \pi/6) $

#### Firing Angle (α)
The firing angle $\alpha$ is the delay from the point where a thyristor would naturally conduct in a diode bridge. For thyristor T1, this point occurs when phase 'a' becomes the most positive, which corresponds to the crossover of the line voltages $v_{ab}$ and $v_{cb}$. This happens at **$\omega t = \pi/6$ (30°)**. Therefore, T1 is fired at $\omega t = \pi/6 + \alpha$.

#### Integration
The average DC voltage ($V_d$) is found by integrating the instantaneous voltage over one ripple period ($\pi/3$ or 60°). Let's analyze the interval when T1 and T6 are conducting, so $v_d = v_{ab}$. This interval runs from the firing of T1 to the firing of the next thyristor in the top group (T3), which is 60° later.

The integration limits are from $\omega t = \pi/6 + \alpha$ to $\omega t = (\pi/6 + \pi/3) + \alpha = \pi/2 + \alpha$.

$$ V_d = \frac{1}{\pi/3} \int_{\pi/6 + \alpha}^{\pi/2 + \alpha} v_{ab}(t) \, d(\omega t) $$
$$ V_d = \frac{3}{\pi} \int_{\pi/6 + \alpha}^{\pi/2 + \alpha} V_{LL,peak} \sin(\omega t + \pi/6) \, d(\omega t) $$
$$ V_d = \frac{3 V_{LL,peak}}{\pi} \left[ -\cos(\omega t + \pi/6) \right]_{\pi/6 + \alpha}^{\pi/2 + \alpha} $$
$$ V_d = \frac{3 V_{LL,peak}}{\pi} \left( -\cos\left(\frac{2\pi}{3} + \alpha\right) + \cos\left(\frac{\pi}{3} + \alpha\right) \right) $$
Using the identity $\cos(A) - \cos(B) = -2\sin(\frac{A+B}{2})\sin(\frac{A-B}{2})$, the term in the parenthesis simplifies to $\cos(\alpha)$.
$$ V_d = \frac{3 V_{LL,peak}}{\pi} \cos(\alpha) $$
Since the peak line-to-line voltage is $V_{LL,peak} = \sqrt{2} V_{LL,rms}$, we arrive at the standard formula. We define the **ideal no-load DC voltage ($V_{do}$)** as the voltage at $\alpha=0$:
$$ V_{do} = \frac{3\sqrt{2}}{\pi} V_{LL,rms} \approx 1.35 \cdot V_{LL,rms} $$
This simplifies the general expression to its most common form:
$$ V_d = V_{do} \cos(\alpha) $$

### 2.2 Voltage Drop from Commutation Overlap ($\Delta V_{dx}$)

In a real system, the AC-side inductance ($L_c$) prevents current from transferring instantaneously between phases. This period, where two thyristors conduct simultaneously, is called **commutation overlap**. During this overlap, the two conducting phases are briefly short-circuited, causing the output DC voltage to be lower than in the ideal case. This results in an average voltage drop, $\Delta V_{dx}$.

The voltage drop is derived by calculating the "lost" volt-second area during each commutation and averaging it over the cycle.
1.  **KVL during Commutation**: Applying KVL across the two commutating phases (e.g., 'a' and 'b') with inductance $L_c$ gives: $v_a - L_c \frac{di_a}{dt} = v_b - L_c \frac{di_b}{dt}$.
2.  **Lost Volt-Second Area**: The instantaneous voltage loss during overlap is $\Delta v = L_c \frac{di}{dt}$. Integrating this over the commutation time (during which the current changes from 0 to $I_d$) gives the lost volt-second area for one event: $A_{lost} = \int L_c di = L_c I_d$.
3.  **Average Voltage Drop**: There are 6 commutations per cycle. The total average voltage drop is the lost area per commutation multiplied by the number of commutations per second ($6f$).
$$ \Delta V_{dx} = (L_c I_d) \cdot (6f) = (L_c I_d) \cdot \frac{6\omega}{2\pi} = \frac{3\omega L_c}{\pi} I_d $$
This term represents a resistive-like voltage drop, where the equivalent resistance is $R_{eq} = 3\omega L_c / \pi$.

---

## 3. Complete DC Voltage and Current Equations

Combining the ideal voltage and the commutation drop gives the complete steady-state equations for the LCI system.

*   **Rectifier (Source-Side) DC Voltage ($V_d$)**
    This is the actual average DC voltage from the source-side converter.
    $$ V_d = V_{do,S} \cos(\alpha_R) - \Delta V_{dx,R} = \frac{3\sqrt{2}}{\pi}V_{LL,S} \cos(\alpha_R) - \frac{3 \omega_S L_s}{\pi}I_d $$
    *   $V_{LL,S}$: Source-side RMS line-to-line voltage.
    *   $\alpha_R$: Rectifier firing angle ($0^\circ < \alpha_R < 90^\circ$).
    *   $\omega_S$: Source angular frequency (e.g., $2\pi \cdot 50$ Hz).
    *   $L_s$: Source-side commutating inductance per phase.
    *   $I_d$: Average DC link current.

*   **Inverter (Load-Side) DC Voltage ($V_{di}$)**
    This is the average DC back-voltage from the load-side converter, which opposes the rectifier voltage.
    $$ V_{di} = V_{do,M} \cos(\beta) - \Delta V_{dx,I} = \frac{3\sqrt{2}}{\pi}V_{LL,M} \cos(\beta) - \frac{3 \omega_M L_m}{\pi}I_d $$
    *   $V_{LL,M}$: Machine terminal RMS line-to-line voltage.
    *   $\beta$: Inverter advance angle ($\beta = 180^\circ - \alpha_I$, where $\alpha_I$ is the inverter firing angle).
    *   $\omega_M$: Machine operating angular frequency.
    *   $L_m$: Machine-side commutating inductance per phase.

*   **DC Link Current ($I_d$)**
    The steady-state current is determined by the voltage difference across the total resistance in the DC link, including the DC link choke resistance ($R_d$) and the equivalent resistances from commutation.
    $$ I_d = \frac{V_d - V_{di}}{R_d} $$
    Substituting the full expressions reveals how the commutation inductances act as equivalent DC resistances:
    $$ I_d = \frac{V_{do,S} \cos(\alpha_R) - V_{do,M} \cos(\beta)}{R_d + \frac{3}{\pi}(\omega_S L_s + \omega_M L_m)} $$

---

## 4. Commutation and Angle Equations

These equations define the physical limits and timing of thyristor commutation.

*   **Commutation Overlap Angle ($u$)**
    The overlap angle is the duration (in electrical angle) during which two thyristors in the same bridge conduct simultaneously.
    $$ \cos(\alpha) - \cos(\alpha + u) = \frac{2 \omega L_c I_d}{\sqrt{2/3} V_{do}} = \frac{\sqrt{2} \omega L_c I_d}{V_{LL,peak}} = \frac{2 X_c I_d}{\sqrt{6} V_{ph}} $$
    This is often solved for the angle itself:
    $$ u = \arccos\left(\cos(\alpha) - \frac{2 \omega L_c I_d}{\sqrt{2/3} V_{do}}\right) - \alpha $$
    *   This general formula applies to both rectifier ($u_R$, using $\alpha_R, \omega_S, L_s$) and inverter ($u_I$, using $\beta, \omega_M, L_m$ after substituting $\alpha_I = 180 - \beta$).

*   **Extinction Angle ($\gamma$)**
    This is the most critical parameter for inverter operation. It is the angle from the end of commutation to the next zero crossing of the commutating voltage. It must be greater than the thyristor turn-off time ($t_q$) for the device to regain its blocking capability.
    $$ \gamma = \beta - u_I $$
    *   **Condition for Safe Commutation:** $\gamma > \omega_M t_q$

---

## 5. Current, Power, and Torque Equations

These equations describe the flow of power through the system and its conversion to mechanical torque.

*   **AC Fundamental Current ($I_{AC,fund}$)**
    The fundamental RMS component of the quasi-square wave AC current is directly related to the DC current.
    $$ I_{AC,fund} = \frac{\sqrt{6}}{\pi} I_d \approx 0.78 I_d $$

*   **Real Power (P)**
    Power is most easily calculated on the DC link and then related to the AC sides (assuming negligible converter losses).
    *   **Rectifier Output Power:** $P_d = V_d I_d$
    *   **Inverter Input Power:** $P_{di} = V_{di} I_d$
    *   **AC Input/Output Power:** $P_{AC} = \sqrt{3} V_{LL} I_{AC,fund} \cos(\phi) \approx P_d$ or $P_{di}$

*   **Reactive Power (Q)**
    LCI converters consume reactive power on the source side and require it from the load side.
    $$ Q_{AC} = \sqrt{3} V_{LL} I_{AC,fund} \sin(\phi) \approx P \tan(\phi) $$
    *   On the **source side**, the power factor angle $\phi_S$ is approximately $\alpha_R$, so $Q_S > 0$ (lagging, consumed).
    *   On the **load side**, the power factor angle $\phi_M$ is approximately $\beta$, so the machine must supply reactive power to the inverter.

*   **Electrical Torque ($T_e$)**
    Torque is directly proportional to the power delivered to the machine and inversely proportional to its speed.
    $$ T_e = \frac{P_{di}}{\omega_{mech}} = \frac{V_{di} I_d}{\omega_M / (p/2)} $$
    A very useful approximation highlights the core relationship between current, flux, and torque:
    $$ T_e \propto \Phi_M I_d \cos(\beta) $$
    *   $\omega_{mech}$: Mechanical speed of the machine [rad/s].
    *   $p$: Number of poles in the synchronous machine.
    *   $\Phi_M$: Machine magnetic flux (held constant in constant torque region).

---

## 6. AC Side Voltage and Power Factor

These equations are crucial for interfacing with the utility grid and understanding the machine's operating state.

*   **Estimated Machine Stator Voltage ($V_{LL,M}$)**
    The machine's terminal voltage is a function of its internal EMF ($E_f$, proportional to speed and field current) and the voltage drop across its synchronous reactance ($X_m$). Using a phasor diagram:
    $$ \vec{E_f} = \vec{V_t} + j I_a X_m $$
    *   $\vec{V_t}$: Machine terminal voltage phasor.
    *   $I_a$: Machine armature current phasor.
    *   This equation underscores that $V_{LL,M}$ is not fixed but depends on the machine's operating point (speed, load, and excitation).

*   **Source-Side Power Factor ($\cos(\phi_S)$)**
    The power factor seen by the utility grid is always lagging. A good approximation is the cosine of the firing angle, but a more precise formula includes the effect of commutation overlap.
    $$ \cos(\phi_S) \approx \cos(\alpha_R) $$
    More accurately:
    $$ \cos(\phi_S) = \frac{\cos(\alpha_R) + \cos(\alpha_R + u_R)}{2} $$

*   **Load-Side (Machine) Power Factor ($\cos(\phi_M)$)**
    For load commutation to be possible, the machine must be over-excited to operate at a leading power factor, supplying reactive power to the inverter. A common approximation is:
    $$ \cos(\phi_M) \approx \cos(\beta) \quad \text{(Leading)} $$
---

[← Back to Documentation Index](index.md)