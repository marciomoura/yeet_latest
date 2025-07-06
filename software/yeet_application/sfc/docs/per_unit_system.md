# Per-Unit System for LCI Drives

[← Back to Documentation Index](index.md)

## **1. Introduction and System Overview**

A Load-Commutated Inverter (LCI) drive is a high-power AC-AC converter used to control the speed and torque of large synchronous machines. In the context of a gas turbine, it performs two primary roles:
*   **Starter:** It operates as a variable frequency drive (VFD) to slowly ramp up the turbine's speed from standstill to its self-sustaining speed.
*   **Drive:** In some applications (e.g., helper motor), it can continue to provide torque to assist the main process.

The basic structure consists of two 6-pulse thyristor converters connected back-to-back via a large DC link inductor.

A per-unit system is exceptionally useful for analyzing such systems because it:
*   **Normalizes variables**, allowing for the comparison of systems with different voltage and power ratings.
*   **Simplifies equations** by eliminating constants like $\pi$, $\sqrt{2}$, and $\sqrt{3}$.
*   Provides **direct insight into the machine's operating state**, as values like voltage, current, and speed are expressed as a fraction of their rated values.

---

## **2. Definition of Base Quantities**

The selection of base quantities is the most critical step. For an LCI drive, the system is designed around the **synchronous machine**, so its ratings are chosen as the primary base values.

### **2.1 AC Side (Machine) Base Quantities**

| Quantity           | Symbol          | Definition                                                 | Explanation                                                                   |
| :----------------- | :-------------- | :--------------------------------------------------------- | :---------------------------------------------------------------------------- |
| **Base Power**     | $S_{base}$      | Rated 3-phase apparent power of the synchronous machine.   | $S_{base} = S_{rated, M}$                                                     |
| **Base Voltage**   | $V_{base}$      | Rated line-to-line RMS voltage of the synchronous machine. | $V_{base} = V_{rated, LL, M}$                                                 |
| **Base Current**   | $I_{base}$      | Derived from base power and base voltage.                  | $I_{base} = \frac{S_{base}}{\sqrt{3} V_{base}}$                               |
| **Base Impedance** | $Z_{base}$      | Derived from base voltage and base current.                | $Z_{base} = \frac{V_{base}}{\sqrt{3} I_{base}} = \frac{V_{base}^2}{S_{base}}$ |
| **Base Frequency** | $\omega_{base}$ | Rated electrical angular frequency of the machine.         | $\omega_{base} = 2 \pi f_{rated, M}$                                          |

### **2.2 DC Link Base Quantities**

The DC link base quantities are derived from the AC base quantities to ensure power invariance ($P_{base, AC} = P_{base, DC}$).

| Quantity            | Symbol       | Definition                                                                                          | Explanation                                                                                                                                     |
| :------------------ | :----------- | :-------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------------- |
| **Base DC Voltage** | $V_{d,base}$ | The ideal, no-load average DC voltage produced by a 6-pulse converter fed with the base AC voltage. | $V_{d,base} = V_{do,base} = \frac{3\sqrt{2}}{\pi} V_{base}$                                                                                     |
| **Base DC Current** | $I_{d,base}$ | Derived from base power and base DC voltage.                                                        | $I_{d,base} = \frac{S_{base}}{V_{d,base}} = \frac{S_{base}}{(\frac{3\sqrt{2}}{\pi})V_{base}} = \frac{\pi}{3\sqrt{2}} \frac{S_{base}}{V_{base}}$ |

---

## **3. Per-Unit Steady-State Equations**

With the base quantities defined, we can now express the standard LCI equations in per-unit form. Per-unit quantities are denoted with lowercase letters (e.g., $v_d = V_d / V_{d,base}$).

### **3.1 Source-Side Converter (Rectifier) Equation**

The standard equation for the average DC voltage of a 6-pulse rectifier, including the voltage drop due to commutation overlap from source inductance ($L_s$), is:
$$ V_d = \frac{3\sqrt{2}}{\pi}V_{LL,S} \cos(\alpha_R) - \frac{3\omega_S L_s}{\pi}I_d $$
Where:
*   $V_{LL,S}$ is the source-side line-to-line voltage.
*   $\alpha_R$ is the rectifier firing angle.
*   $\omega_S$ is the source angular frequency.
*   $I_d$ is the DC link current.

To convert this to per-unit, we divide every term by $V_{d,base}$:
$$ v_d = \frac{V_d}{V_{d,base}} = \frac{\frac{3\sqrt{2}}{\pi}V_{LL,S}}{V_{d,base}}\cos(\alpha_R) - \frac{\frac{3\omega_S L_s}{\pi}I_d}{V_{d,base}} $$

Let's analyze each term:
*   **First Term:** $\frac{\frac{3\sqrt{2}}{\pi}V_{LL,S}}{V_{d,base}} = \frac{\frac{3\sqrt{2}}{\pi}V_{LL,S}}{\frac{3\sqrt{2}}{\pi}V_{base}} = \frac{V_{LL,S}}{V_{base}} = v_{s,pu}$
*   **Second Term (Commutation Drop):** This term simplifies elegantly in the p.u. system. The voltage drop due to commutation reactance ($X_s = \omega_S L_s$) can be shown to be:
    $$ \frac{\frac{3\omega_S L_s}{\pi}I_d}{V_{d,base}} = \frac{\pi}{6} \left(\frac{\omega_S}{\omega_{base}}\right) \left(\frac{\omega_{base}L_s}{Z_{base}}\right) \left(\frac{I_d}{I_{d,base}}\right) = \frac{\pi}{6} f_{s,pu} \cdot x_{s,pu} \cdot i_{d,pu} $$

The final per-unit equation for the rectifier is:
$$ v_d = v_{s,pu} \cos(\alpha_R) - \left( \frac{\pi}{6} f_{s,pu} \cdot x_{s,pu} \right) i_{d,pu} $$
Since the source frequency is typically fixed at the base frequency, $f_{s,pu} = 1$.
$$ \boxed{v_d = v_{s,pu} \cos(\alpha_R) - \frac{\pi}{6} x_{s,pu} \cdot i_{d,pu}} $$

### **3.2 Load-Side Converter (Inverter) Equation**

The inverter operates against the back-EMF of the synchronous machine. Its DC voltage ($V_{di}$) opposes the rectifier's voltage. The inverter firing angle is typically expressed as $\beta = 180^\circ - \alpha_I$, where $\alpha_I$ is the inverter firing angle. For safe commutation, the extinction angle $\gamma$ must be greater than the thyristor turn-off time ($\gamma = 180^\circ - \alpha_I - u = \beta - u > \omega t_q$).

The equation for the inverter's back-voltage is:
$$ V_{di} = \frac{3\sqrt{2}}{\pi}V_{LL,M} \cos(\beta) + \frac{3\omega_M L_m}{\pi}I_d $$
Where:
*   $V_{LL,M}$ is the machine terminal line-to-line voltage (proportional to speed and flux).
*   $\beta$ is the inverter firing angle relative to the machine voltage zero crossing.
*   $\omega_M$ is the machine's operating electrical frequency.
*   $L_m$ is the machine's commutating inductance.

Converting to per-unit by dividing by $V_{d,base}$:
$$ v_{di} = \frac{V_{di}}{V_{d,base}} = \frac{V_{LL,M}}{V_{base}}\cos(\beta) + \frac{\frac{3\omega_M L_m}{\pi}I_d}{V_{d,base}} $$
The terms simplify similarly:
*   **First Term:** $\frac{V_{LL,M}}{V_{base}} = v_{m,pu}$. Note that $v_{m,pu}$ is proportional to the machine's per-unit speed, $f_{m,pu} = \omega_M / \omega_{base}$, assuming constant flux.
*   **Second Term (Commutation Rise):** The voltage drop from commutation on the inverter side *aids* the DC current flow, hence the positive sign.
    $$ \frac{\frac{3\omega_M L_m}{\pi}I_d}{V_{d,base}} = \frac{\pi}{6} \left(\frac{\omega_M}{\omega_{base}}\right) \left(\frac{\omega_{base}L_m}{Z_{base}}\right) \left(\frac{I_d}{I_{d,base}}\right) = \frac{\pi}{6} f_{m,pu} \cdot x_{m,pu} \cdot i_{d,pu} $$

The final per-unit equation for the inverter is:
$$ \boxed{v_{di} = v_{m,pu} \cos(\beta) + \frac{\pi}{6} f_{m,pu} \cdot x_{m,pu} \cdot i_{d,pu}} $$

### **3.3 Overall DC Link Equation**

In steady-state, the rectifier voltage overcomes the inverter's back-voltage and the voltage drop across the DC link resistance ($R_d$).
$$ V_d = V_{di} + I_d R_d $$
In per-unit form:
$$ v_d = v_{di} + i_{d,pu} \cdot r_{d,pu} $$
where $r_{d,pu} = \frac{R_d}{Z_{d,base}}$ and $Z_{d,base} = V_{d,base}/I_{d,base}$.

Substituting the expressions for $v_d$ and $v_{di}$:
$$ v_{s,pu} \cos(\alpha_R) - \frac{\pi}{6} x_{s,pu} i_{d,pu} = v_{m,pu} \cos(\beta) + \frac{\pi}{6} f_{m,pu} x_{m,pu} i_{d,pu} + i_{d,pu} r_{d,pu} $$

Rearranging to solve for the per-unit DC current ($i_{d,pu}$), we get the fundamental governing equation of the LCI drive in a per-unit system:
$$ \boxed{i_{d,pu} = \frac{v_{s,pu} \cos(\alpha_R) - v_{m,pu} \cos(\beta)}{\frac{\pi}{6} (x_{s,pu} + f_{m,pu} x_{m,pu}) + r_{d,pu}}} $$

---

## **4. Key Insights and Application**

This final equation provides powerful insights into the control and operation of the LCI drive:

1.  **Current Control:** The DC link current ($i_{d,pu}$), which is directly related to the machine's torque, is controlled by the difference between the rectifier's ideal voltage ($v_{s,pu} \cos(\alpha_R)$) and the inverter's ideal back-voltage ($v_{m,pu} \cos(\beta)$). The primary method of controlling current is to rapidly adjust the rectifier firing angle $\alpha_R$.

2.  **Speed Control:** The machine speed is directly proportional to $f_{m,pu}$ and $v_{m,pu}$. To change speed, the controller adjusts the inverter firing angle $\beta$ (often keeping the extinction angle $\gamma$ constant for efficiency and safety) and provides a new current reference $i_{d,ref}$ for the rectifier's control loop.

3.  **Low-Speed Operation Challenge:** As speed approaches zero ($f_{m,pu} \to 0$), the machine's back-EMF ($v_{m,pu}$) also approaches zero. This makes natural line commutation on the inverter side impossible, as there is no voltage source to commutate the thyristors. The equation shows that the denominator becomes small, and control becomes difficult. This is why LCI drives use special "DC link current pulsing" modes at very low speeds (< 5-10% of rated speed) to force commutation.

4.  **Power Factor:** The firing angles $\alpha_R$ and $\beta$ directly determine the displacement power factor on both the source and machine sides. Because both converters operate with delayed firing angles, LCI drives inherently have a lagging power factor on the utility side and require the synchronous machine to operate at a leading power factor (over-excited) to provide reactive power for commutation.

---

[← Back to Documentation Index](index.md)