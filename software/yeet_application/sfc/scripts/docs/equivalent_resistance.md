Based on the system characteristics provided, the nominal DC link current that will flow given these specific firing angles and circuit parameters is approximately **519 A**.

Although the `kBaseDCLinkCurrent` is rated for **1044 A**, the specific firing angles provided ($\alpha_{grid}=13^{\circ}$ and $\gamma_{machine}=26.84^{\circ}$) create a voltage balance that limits the current to roughly half the rated capacity.

Here is the step-by-step calculation based on the steady-state equilibrium of the LCI drive.

### 1. The Governing Equation
In steady-state, the DC link current ($I_{d}$) is determined by the difference between the Rectifier's driving voltage and the Inverter's counter-voltage, divided by the total loop impedance (including physical resistance and the effective resistance of commutation overlap).

$$ I_d = \frac{V_{d,rect} + V_{d,inv}}{R_{phys} + R_{comm}} $$
*(Note: In this convention, $V_{d,inv}$ is a negative value representing the counter-EMF)*

### 2. Voltage Calculations
First, we calculate the ideal no-load DC voltages ($V_{d0}$) and apply the firing angles.

**Rectifier (Grid Side):**
*   $V_{d0,g} = \frac{3\sqrt{2}}{\pi} \times V_{LL,g} = 1.3505 \times 2090 \text{ V} = 2822.5 \text{ V}$
*   Operating Voltage: $V_{rect} = V_{d0,g} \cos(13^{\circ}) = 2822.5 \times 0.9744 = \mathbf{2750.2 \text{ V}}$

**Inverter (Machine Side):**
*   $V_{d0,m} = \frac{3\sqrt{2}}{\pi} \times V_{LL,m} = 1.3505 \times 1980 \text{ V} = 2673.9 \text{ V}$
*   Operating Voltage: $V_{inv} = V_{d0,m} \cos(153.16^{\circ}) = 2673.9 \times (-0.8923) = \mathbf{-2385.9 \text{ V}}$
    *(Note: $\alpha = 180 - 26.84 = 153.16^{\circ}$)*

**Net Driving Voltage:**
$$ V_{net} = 2750.2 \text{ V} + (-2385.9 \text{ V}) = \mathbf{364.3 \text{ V}} $$

### 3. Impedance Calculations
We must calculate the total resistance opposing the DC current.

**A. Physical Resistance ($R_{phys}$)**
In a 6-pulse bridge, current always flows through two phases in series. Therefore, the phase resistances are doubled in the DC equivalent circuit.
$$ R_{phys} = (2 \times R_{grid}) + R_{dc\_link} + (2 \times R_{machine}) $$
$$ R_{phys} = (2 \times 0.1552) + 0.010 + (2 \times 0.0788) = \mathbf{0.478 \ \Omega} $$

**B. Commutation "Resistance" ($R_{comm}$)**
The voltage loss due to commutation overlap acts as an equivalent resistance $R_c = \frac{3}{\pi}\omega L = 6 f L$.
*   Grid Side: $R_{c,g} = 6 \times 50 \text{ Hz} \times 0.4941 \text{ mH} = 0.148 \ \Omega$
*   Machine Side: $R_{c,m} = 6 \times 50 \text{ Hz} \times 0.2508 \text{ mH} = 0.075 \ \Omega$
*   Total $R_{comm} = 0.148 + 0.075 = \mathbf{0.223 \ \Omega}$

**Total Effective Resistance:**
$$ R_{total} = 0.478 \ \Omega + 0.223 \ \Omega = \mathbf{0.701 \ \Omega} $$

### 4. Final Current Calculation

$$ I_d = \frac{364.3 \text{ V}}{0.701 \ \Omega} \approx \mathbf{519.7 \text{ A}} $$

### Summary
With the system configured at these specific angles:
*   **Nominal Current:** **~520 A**
*   **Rated Current:** **1044 A**

To reach the rated current of 1044 A, the control system would need to either advance the rectifier angle (closer to $0^{\circ}$) or, more likely, increase the inverter extinction angle $\beta$ (running closer to $140^{\circ}$ instead of $153^{\circ}$) to reduce the back-pressure from the machine.