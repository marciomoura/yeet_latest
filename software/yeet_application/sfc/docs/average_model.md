Developing an average-value model is the standard and most effective method for designing and tuning the control loops of a power converter system like an LCI. This approach allows you to focus on the system's dynamics at the time scale of the controllers (milliseconds) rather than the much faster switching events (microseconds), dramatically speeding up simulations while retaining the necessary accuracy for controller design.

Here is a comprehensive guide to developing the average plant model for an LCI drive, including the electrical and mechanical dynamics, and how to set it up for closed-loop controller evaluation, with special considerations for soft-starter operation.

### **Introduction: The Purpose of an Average Model**

An average model replaces the high-frequency switched behavior of the thyristor bridges with their equivalent average voltage equations. The core idea is that the speed and current controllers have bandwidths much lower than the converter's switching frequency ($6f$ or $12f$). Therefore, they only respond to the *average* value of the DC link quantities, not the high-frequency ripple.

**Benefits:**
*   **Fast Simulation:** Allows for rapid testing and iteration of controller gains (PI parameters).
*   **Linearization:** The resulting model can be linearized around an operating point to apply classical control theory (Bode plots, root locus) for systematic controller design.
*   **Focus on Core Dynamics:** Isolates the fundamental electrical and mechanical time constants that govern the system's response.

### **System Architecture: Cascaded Control Structure**

The standard control topology for an LCI drive is a cascaded structure. This is what our model will be designed to test.

*   **Outer Speed Loop:** A PI controller compares the desired speed ($\omega_{ref}$) with the measured machine speed ($\omega_m$) and generates a command for the required DC link current ($I_{d,ref}$). This sets the required torque.
*   **Inner Current Loop:** A faster PI controller compares $I_{d,ref}$ with the actual DC link current ($I_d$) and adjusts the rectifier firing angle ($\alpha$) to regulate the current to its setpoint.

Our "plant model" is the combination of the **Electrical Subsystem** and the **Mechanical Subsystem**.

---

### **Part 1: The Dynamic Electrical Subsystem Model**

This subsystem models the DC link. Its input is the firing angle $\alpha$ and its output is the DC link current $I_d$. It is the "plant" for the inner current controller.

The dynamic behavior of the DC link is described by applying KVL to the average voltages:
$$ V_d(t) = L_d \frac{dI_d(t)}{dt} + R_d I_d(t) + V_{di}(t) $$
Where:
*   $L_d$ and $R_d$ are the inductance and resistance of the DC link choke.
*   $V_d(t)$ is the average DC voltage from the rectifier.
*   $V_{di}(t)$ is the average DC counter-voltage from the inverter.

Rearranging this into state-space form gives the central differential equation for the DC link current:
$$ \frac{dI_d}{dt} = \frac{1}{L_d} \left( V_d(t) - V_{di}(t) - R_d I_d(t) \right) $$

Now, we substitute the average models for the converters:

**1. Rectifier Average Model ($V_d$)**
The average voltage from the rectifier, including the resistive effect of commutation, is:
$$ V_d(t) = V_{do} \cos(\alpha(t)) - \left(\frac{3 X_s}{\pi}\right) I_d(t) $$
*   $V_{do} = \frac{3\sqrt{2}}{\pi} V_{LL,grid}$ (a constant).
*   $\alpha(t)$ is the control input from the current controller.
*   $X_s$ is the grid-side commutation reactance.

**2. Inverter Average Model ($V_{di}$)**
The average counter-voltage from the inverter, including its commutation effect, is:
$$ V_{di}(t) = V_{dio}(t) \cos(\gamma) + \left(\frac{3 X_{mach}(\omega_m)}{\pi}\right) I_d(t) $$
*   $\gamma$ is the extinction angle, which is typically kept constant by a separate, fast controller. For this model, we can assume $\gamma$ is fixed at its setpoint (e.g., 15°).
*   $V_{dio}(t) = \frac{3\sqrt{2}}{\pi} E_{LL,mach}(t)$. The machine's internal voltage $E_{LL,mach}$ is directly proportional to its speed: $E_{LL,mach}(t) = K_V \cdot \omega_m(t)$, where $K_V$ is the machine's voltage constant (in V/(rad/s)).
*   Therefore, $V_{dio}(t) = \left(\frac{3\sqrt{2}}{\pi}K_V\right) \omega_m(t) = K_{inv} \cdot \omega_m(t)$.
*   The machine-side commutation reactance is also speed-dependent: $X_{mach}(\omega_m) = L_{mach} \cdot \omega_m(t)$.

---

### **Part 2: The Dynamic Mechanical Subsystem Model**

This subsystem models the rotational dynamics of the motor and its connected load. Its input is the DC link current $I_d$ (which creates torque) and its output is the machine's mechanical speed $\omega_m$. It is the "plant" for the outer speed controller.

The model is based on Newton's second law for rotation:
$$ J \frac{d\omega_m(t)}{dt} = T_e(t) - T_{load}(t) - B \omega_m(t) $$
Where:
*   $J$ is the total moment of inertia of the rotating system (motor + load, e.g., gas turbine).
*   $T_e(t)$ is the electromagnetic torque produced by the motor.
*   $T_{load}(t)$ is the opposing load torque from the driven equipment.
*   $B$ is the viscous friction coefficient, representing speed-dependent losses.

Rearranging into state-space form:
$$ \frac{d\omega_m}{dt} = \frac{1}{J} \left( T_e(t) - T_{load}(t) - B \omega_m(t) \right) $$

**1. Electromagnetic Torque ($T_e$)**
The torque is directly proportional to the DC link current, which is the core principle of LCI control:
$$ T_e(t) = K_t I_d(t) $$
*   $K_t$ is the motor's torque constant (in Nm/A). This constant links the electrical and mechanical subsystems.

### **Part 3: Load Torque Models for Soft-Starter Applications**

The load torque $T_{load}(t)$ is critical for accurately simulating a soft-start. For a gas turbine, the load torque is highly nonlinear and speed-dependent.

*   **Breakaway Torque:** A high static friction torque that must be overcome to initiate rotation.
*   **Windage and Compression:** As speed increases, the torque required to turn the turbine increases due to air resistance and compression.

A common simplified model for a pump or fan, which can be adapted for a turbine, is:
$$ T_{load}(t) = T_{breakaway} + K_{load} \omega_m(t)^2 $$
*   $T_{breakaway}$ is the static breakaway torque.
*   $K_{load}$ is a constant representing the load factor.

For a high-fidelity simulation, a look-up table based on empirical data from the turbine manufacturer should be used to represent $T_{load}$ as a function of $\omega_m$.

---

### **Part 4: Assembling the Complete Closed-Loop Model**

With the electrical and mechanical subsystems defined, we can assemble the full closed-loop simulation in a tool like MATLAB/Simulink or Python.

**Signal Flow:**
1.  **Speed Controller:**
    *   Inputs: $\omega_{ref}$ (from a ramp generator), $\omega_m$ (from the mechanical plant).
    *   Calculates: Speed error $e_s = \omega_{ref} - \omega_m$.
    *   Output: $I_{d,ref}$ (PI controller output).
2.  **Current Controller:**
    *   Inputs: $I_{d,ref}$ (from speed controller), $I_d$ (from the electrical plant).
    *   Calculates: Current error $e_c = I_{d,ref} - I_d$.
    *   Output: $\alpha$ (PI controller output, after linearization and feedforward).
3.  **Electrical Plant (DC Link Model):**
    *   Inputs: $\alpha$ (from current controller), $\omega_m$ (from mechanical plant, for back-EMF).
    *   State Equation: $\frac{dI_d}{dt} = \frac{1}{L_d} (V_d - V_{di} - R_d I_d)$.
    *   Output: $I_d$.
4.  **Mechanical Plant (Motor & Load Model):**
    *   Input: $I_d$ (from electrical plant).
    *   State Equation: $\frac{d\omega_m}{dt} = \frac{1}{J} (K_t I_d - T_{load} - B \omega_m)$.
    *   Output: $\omega_m$.

This creates a complete, coupled differential equation model that accurately represents the dynamics of the entire LCI drive system.

---

### **Part 5: Reconstructing AC-Side Quantities for Protection Testing**

While the average model focuses on DC quantities for control design, we can accurately reconstruct the fundamental AC currents and voltages. This is crucial for testing protection algorithms that operate on AC measurements.

**1. Grid and Machine AC Currents**

Based on Fourier analysis of the idealized 6-pulse converter waveforms, the AC line current is directly related to the DC link current.

*   **Fundamental RMS AC Current ($I_{ac,1,rms}$):**
    The fundamental component of the line current on either the grid or machine side is given by:
    $$ I_{ac,1,rms}(t) = \frac{\sqrt{6}}{\pi} I_d(t) \approx 0.78 I_d(t) $$
    This value is essential for protection relays that filter for the fundamental component.

*   **Total RMS AC Current ($I_{ac,rms}$):**
    For thermal protection and overcurrent relays that respond to the true RMS value (including harmonics), the total RMS current is:
    $$ I_{ac,rms}(t) = \sqrt{\frac{2}{3}} I_d(t) \approx 0.816 I_d(t) $$

**2. Grid-Side AC Terminal Voltage**

The AC voltage at the rectifier terminals is distorted by commutation notches. While the average model doesn't replicate the notches themselves, it accounts for the *effect* of commutation as a voltage drop. The fundamental voltage at the terminal is what a protection relay would typically measure.

We can estimate the terminal voltage by considering the voltage drop across the source reactance $X_s$. However, a simpler and often sufficient approach for protection studies is to monitor the main grid voltage ($V_{LL,grid}$) and the reconstructed AC current ($I_{ac,rms}$). The protection logic would then check for overcurrent conditions.

**3. Machine-Side AC Terminal Voltage**

The machine terminal voltage is a function of the inverter's operation and the machine's back-EMF. We can estimate the fundamental RMS line-to-line voltage at the machine terminals ($V_{LL,M,1,rms}$) from the inverter's average DC voltage ($V_{di}$):
$$ V_{di}(t) \approx \frac{3\sqrt{2}}{\pi} V_{LL,M,1,rms}(t) \cos(\gamma) $$
*   Note the use of $\gamma$ (extinction angle) instead of $\beta$. We assume the inverter is controlled to maintain a constant $\gamma$.

Rearranging for the machine terminal voltage:
$$ V_{LL,M,1,rms}(t) \approx \frac{\pi}{3\sqrt{2}} \frac{V_{di}(t)}{\cos(\gamma)} $$
This allows you to simulate the machine voltage as it varies with speed and load, which is critical for testing machine-side undervoltage or overvoltage protection.

**Simulation Implementation:**
These calculations can be added as an additional block to your simulation. After the main state equations for $I_d$ and $\omega_m$ are solved at each time step, you can compute the AC quantities for use in your protection algorithm models.

### **Conclusion: Using the Model for Tuning**

This average-value model is the ideal platform for tuning the PI gains of the speed and current controllers.

*   **Current Loop Tuning:** The inner loop should be tuned first, with the outer speed loop disabled. A step change is applied to $I_{d,ref}$, and the PI gains ($K_{p,c}, K_{i,c}$) are adjusted to achieve a fast response with minimal overshoot (typically aiming for a bandwidth of 100-500 rad/s).
*   **Speed Loop Tuning:** Once the current loop is stable, the speed loop is closed. A step change is applied to $\omega_{ref}$, and the PI gains ($K_{p,s}, K_{i,s}$) are adjusted for a smooth response with no overshoot (typically aiming for a bandwidth of 10-50 rad/s, at least 5-10 times slower than the current loop).