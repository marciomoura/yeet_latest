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

The selection of base quantities is the most critical step. For an LCI drive, it is often practical to define separate base systems for the grid side and the machine side, as their ratings may differ. The DC link quantities are then defined to interface between the two.

### **2.1 Grid-Side AC Base Quantities (CLS)**

The grid-side converter's base quantities are derived from the transformer secondary ratings and the rated DC link current. This system does not rely on a pre-defined base power.

| Quantity           | Symbol          | Definition                                                              | Formula                                                                       |
| :----------------- | :-------------- | :---------------------------------------------------------------------- | :---------------------------------------------------------------------------- |
| **Base Voltage**   | $V_{base,S}$    | Rated peak phase voltage of the transformer secondary.                  | $V_{base,S} = \frac{\sqrt{2}}{\sqrt{3}} V_{rated,LL}$              |
| **Base AC Current**| $I_{base,S}$    | Peak of the fundamental AC current corresponding to the base DC current.| $I_{base,S} = \frac{2\sqrt{3}}{\pi} I_{d,base}$                                |
| **Base Impedance** | $Z_{base,S}$    | Derived from base voltage and base current.                             | $Z_{base,S} = \frac{V_{base,S}}{I_{base,S}}$                                  |
| **Base Frequency** | $\omega_{base,S}$ | Rated electrical angular frequency of the grid.                         | $\omega_{base,S} = 2 \pi f_{grid}$                                            |
| **Base Inductance**| $L_{base,S}$    | Derived from base impedance and base frequency.                         | $L_{base,S} = \frac{Z_{base,S}}{\omega_{base,S}}$                             |

### **2.2 Machine-Side AC Base Quantities (CMS)**

The machine-side converter (CMS) has its own independent per-unit system, defined by the synchronous machine's nameplate ratings.

| Quantity                | Symbol              | Definition                                                 | Formula                                                                       |
| :---------------------- | :------------------ | :--------------------------------------------------------- | :---------------------------------------------------------------------------- |
| **Base Power**          | $S_{base,M}$        | Rated 3-phase apparent power of the synchronous machine.   | $S_{base,M} = S_{rated, M}$                                                   |
| **Base Voltage**        | $V_{base,M}$        | Rated peak phase voltage of the synchronous machine.       | $V_{base,M} = \frac{V_{rated, LL, M}}{\sqrt{3}} \sqrt{2}$                      |
| **Base Current**        | $I_{base,M}$        | Derived from base power and base voltage.                  | $I_{base,M} = \frac{2}{3} \frac{S_{base,M}}{V_{base,M}}$                      |
| **Base Impedance**      | $Z_{base,M}$        | Derived from base voltage and base power.                  | $Z_{base,M} = \frac{3}{2} \frac{V_{base,M}^2}{S_{base,M}}$                    |
| **Base Elec. Frequency**| $\omega_{base,M}$   | Rated electrical angular frequency of the machine.         | $\omega_{base,M} = 2 \pi f_{rated, M}$                                        |
| **Base Mech. Speed**    | $\Omega_{base,M}$   | Rated mechanical angular speed of the machine.             | $\Omega_{base,M} = \frac{\omega_{base,M}}{p/2}$ (p = poles)                    |
| **Base Inductance**     | $L_{base,M}$        | Derived from base impedance and base frequency.            | $L_{base,M} = \frac{Z_{base,M}}{\omega_{base,M}}$                             |
| **Base Torque**         | $T_{base}$          | Derived from base power and base mechanical speed.         | $T_{base} = \frac{S_{base,M}}{\Omega_{base,M}}$                               |
| **Base Flux Linkage**   | $\Psi_{base}$       | Base peak phase flux linkage (stator).                     | $\Psi_{base} = \frac{V_{base,M}}{\omega_{base,M}}$                             |

### **2.3 DC Link Base Quantities**

The DC link base quantities are defined relative to the grid-side AC quantities.

| Quantity            | Symbol       | Definition                                                                                          | Explanation                                                                                                                                     |
| :------------------ | :----------- | :-------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------------- |
| **Base DC Voltage** | $V_{d,base}$ | The ideal, no-load average DC voltage produced by a 6-pulse converter fed with the base AC voltage. | $V_{d,base} = \frac{3}{\pi} V_{base,S}$                                                                                                          |
| **Base DC Current** | $I_{d,base}$ | The rated DC link current, consistent with the grid-side definition.                                | $I_{d,base} = I_{d,rated}$                                                                                                                      |
| **Base DC Impedance**| $Z_{d,base}$ | Derived from base DC voltage and current.                                                           | $Z_{d,base} = \frac{V_{d,base}}{I_{d,base}}$                                                                                                    |

---

[← Back to Documentation Index](index.md)