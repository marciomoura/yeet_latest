# Steady-State Equations for LCI Systems

[← Back to Documentation Index](index.md)

This documentation provides a comprehensive list of key steady-state equations for Load-Commutated Inverter (LCI) systems, assuming standard 6-pulse converter configurations on both source and load sides.

## **1. DC Link Voltage and Current Equations**

These equations form the core of the converter's operation, linking the AC source to the AC load.

*   **Ideal No-Load DC Voltage ($V_{do}$)**
    This is the theoretical maximum average DC voltage from a 6-pulse bridge, assuming no commutation overlap. It's the base for all DC voltage calculations.
    $$ V_{do} = \frac{3\sqrt{2}}{\pi} V_{LL} $$
    *   $V_{LL}$: The RMS line-to-line voltage on the AC side of the converter.

*   **Rectifier (Source-Side) DC Voltage ($V_d$)**
    This is the actual average DC voltage produced by the source-side converter, accounting for the firing angle ($\alpha_R$) and the voltage drop from commutation overlap ($\Delta V_{dx,R}$).
    $$ V_d = V_{do,S} \cos(\alpha_R) - \Delta V_{dx,R} = \frac{3\sqrt{2}}{\pi}V_{LL,S} \cos(\alpha_R) - \frac{3 \omega_S L_s}{\pi}I_d $$
    *   $V_{LL,S}$: Source-side RMS line-to-line voltage.
    *   $\alpha_R$: Rectifier firing angle ($0^\circ < \alpha_R < 90^\circ$).
    *   $\omega_S$: Source angular frequency (e.g., $2\pi \cdot 60$ Hz).
    *   $L_s$: Source-side commutating inductance per phase.
    *   $I_d$: Average DC link current.

*   **Inverter (Load-Side) DC Voltage ($V_{di}$)**
    This is the average DC back-voltage produced by the load-side converter, which operates against the machine's back-EMF. Note the positive sign on the commutation term, as the overlap effect *reduces* the back-voltage, thus aiding current flow.
    $$ V_{di} = V_{do,M} \cos(\beta) + \Delta V_{dx,I} = \frac{3\sqrt{2}}{\pi}V_{LL,M} \cos(\beta) + \frac{3 \omega_M L_m}{\pi}I_d $$
    *   $V_{LL,M}$: Machine terminal RMS line-to-line voltage.
    *   $\beta$: Inverter advance angle ($\beta = 180^\circ - \alpha_I$, where $\alpha_I$ is the inverter firing angle).
    *   $\omega_M$: Machine operating angular frequency (proportional to speed).
    *   $L_m$: Machine-side commutating inductance per phase (typically the subtransient reactance, $X_d''$).

*   **DC Link Current ($I_d$)**
    The steady-state current is determined by the voltage difference across the DC link resistance ($R_d$).
    $$ I_d = \frac{V_d - V_{di}}{R_d} $$
    Substituting the above gives the complete expression:
    $$ I_d = \frac{(\frac{3\sqrt{2}}{\pi}V_{LL,S} \cos(\alpha_R)) - (\frac{3\sqrt{2}}{\pi}V_{LL,M} \cos(\beta))}{R_d + \frac{3}{\pi}(\omega_S L_s + \omega_M L_m)} $$

---

## **2. Commutation and Angle Equations**

These equations define the physical limits and timing of thyristor commutation.

*   **Commutation Overlap Angle ($u$)**
    The overlap angle is the duration (in electrical angle) during which two thyristors in the same bridge conduct simultaneously, shorting two AC lines through the commutating inductance.
    $$ \cos(\alpha) - \cos(\alpha + u) = \frac{2 \omega L_c I_d}{\sqrt{2/3} V_{do}} = \frac{\sqrt{2} \omega L_c I_d}{V_{LL,peak}} = \frac{2 X_c I_d}{\sqrt{6} V_{ph}} $$
    This is often solved for the angle itself:
    $$ u = \arccos\left(\cos(\alpha) - \frac{2 \omega L_c I_d}{\sqrt{2/3} V_{do}}\right) - \alpha $$
    *   This general formula applies to both rectifier ($u_R$, using $\alpha_R, \omega_S, L_s$) and inverter ($u_I$, using $\alpha_I, \omega_M, L_m$).

*   **Extinction Angle ($\gamma$)**
    This is the most critical parameter for inverter operation. It is the angle from the end of commutation to the next zero crossing of the commutating voltage. It must be greater than the thyristor turn-off time ($t_q$) for the device to regain its blocking capability.
    $$ \gamma = \beta - u_I $$
    *   **Condition for Safe Commutation:** $\gamma > \omega_M t_q$

*   **Commutation Time ($t_c$)**
    The physical time it takes for commutation to complete.
    $$ t_c = \frac{u}{\omega} $$
    *   For the inverter, $t_{c,I} = u_I / \omega_M$.

---

## **3. Current, Power, and Torque Equations**

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
    *   On the **source side**, $\phi_S \approx \alpha_R$, so $Q_S > 0$ (lagging, consumed).
    *   On the **load side**, $\phi_M \approx \beta$, so $Q_M < 0$ (leading, supplied by the over-excited machine).

*   **Electrical Torque ($T_e$)**
    Torque is directly proportional to the power delivered to the machine and inversely proportional to its speed.
    $$ T_e = \frac{P_{di}}{\omega_{mech}} = \frac{V_{di} I_d}{\omega_M / (p/2)} $$
    A very useful approximation highlights the core relationship between current, flux, and torque:
    $$ T_e \propto \Phi_M I_d \cos(\beta) $$
    *   $\omega_{mech}$: Mechanical speed of the machine [rad/s].
    *   $p$: Number of poles in the synchronous machine.
    *   $\Phi_M$: Machine magnetic flux (held constant in constant torque region).

---

## **4. AC Side Voltage and Power Factor**

These equations are crucial for interfacing with the utility grid and understanding the machine's operating state.

*   **Estimated Machine Stator Voltage ($V_{LL,M}$)**
    The machine's terminal voltage is a function of its internal EMF ($E_f$, proportional to speed and field current) and the voltage drop across its synchronous reactance ($X_m$). Using a phasor diagram:
    $$ \vec{E_f} = \vec{V_t} + j I_a X_m $$
    *   $\vec{V_t}$: Machine terminal voltage phasor.
    *   $I_a$: Machine armature current phasor.
    *   This equation underscores that $V_{LL,M}$ is not fixed but depends on the machine's operating point (speed, load, and excitation).

*   **Source-Side Power Factor ($\cos(\phi_S)$)**
    The power factor seen by the utility grid is always lagging. A good approximation is simply the cosine of the firing angle, but a more precise formula includes the effect of commutation overlap.
    $$ \cos(\phi_S) \approx \cos(\alpha_R) $$
    More accurately:
    $$ \cos(\phi_S) = \frac{\cos(\alpha_R) + \cos(\alpha_R + u_R)}{2} $$

*   **Load-Side (Machine) Power Factor ($\cos(\phi_M)$)**
    For load commutation to be possible, the machine must be over-excited to operate at a leading power factor, supplying reactive power to the inverter.
    $$ \cos(\phi_M) \approx \cos(\beta) \quad \text{(Leading)} $$

---

[← Back to Documentation Index](index.md)