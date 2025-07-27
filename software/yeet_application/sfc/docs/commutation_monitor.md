An algorithm for detecting commutation notches in 3-phase thyristor bridges has been developed. This algorithm generates a real-time boolean signal, `is_commuting`, that identifies the precise period when any of the three phases are undergoing commutation. By operating within a per-unit system, the algorithm ensures scalability and straightforward integration into existing control systems. The core of the functionality lies in its use of thyristor firing pulses to arm the detection mechanism, followed by direct measurement of line voltage behavior to confirm the start and end of the commutation process.

### **Updated Documentation: Commutation Notch Detection**

#### **1. Overview**

This section details the implementation of the commutation notch detection algorithm. The primary purpose of this feature is to generate a boolean signal, `is_commuting`, which is active (`true`) during the commutation period of any of the three phases of the thyristor bridge. The detection is initiated by a thyristor firing pulse and confirmed by the characteristic drop in line-to-line voltage that occurs during the transfer of current from one thyristor to the next. The algorithm is designed to work within a per-unit (p.u.) system for all voltage measurements.

#### **2. Algorithm Description**

The algorithm executes the following steps in a continuous loop to ensure real-time detection:

##### **Step 1: Conversion to Line Voltage**

The algorithm ingests the three-phase voltage measurements (`Va`, `Vb`, `Vc`) and converts them into three line-to-line voltage measurements. The conversion is performed as follows:

*   `Vab = (Va - Vb) / sqrt(3)`
*   `Vbc = (Vb - Vc) / sqrt(3)`
*   `Vca = (Vc - Va) / sqrt(3)`

These calculations are based on instantaneous voltage values in the per-unit system.

##### **Step 2: Arming the Detection on Firing Pulse**

The algorithm detects the rising edge of each of the six thyristor firing pulses. When a pulse relevant to a specific line-to-line commutation event is detected, an internal "armed" flag for that line is set to `true`. This signifies that a commutation is expected to begin.

*   **Vab Commutation (shorting Vca):** Armed by firing T1 or T4.
*   **Vbc Commutation (shorting Vab):** Armed by firing T3 or T6.
*   **Vca Commutation (shorting Vbc):** Armed by firing T2 or T5.

##### **Step 3: Detecting Commutation Start**

Once a line's detection is armed, the algorithm monitors the corresponding line voltage that is expected to be short-circuited. The commutation is considered to have started when the absolute value of this line voltage drops below the `commutation_start_threshold`. When this occurs, the `is_commuting` state for that line is set to `true`.

##### **Step 4: Activating the Master `is_commuting` Signal**

The master `is_commuting` boolean signal is a logical OR of the individual commutation states of the three line voltages. It is set to `true` as soon as any line enters the commutation state.

##### **Step 5: Detecting the End of Commutation**

The commutation period ends when the current has fully transferred to the new thyristor, and the line voltage is restored. The algorithm identifies the end of the commutation by monitoring the voltage level of the line that was previously in commutation.

The `is_commuting` state for a specific line will be set to `false` only when its voltage rises above the `commutation_end_threshold`. At this point, the "armed" flag for that line is also reset. The master `is_commuting` signal will become `false` only when all lines have exited the commutation state.

### **Implementation Interfaces**

The following outlines the necessary interfaces for the commutation detection algorithm.

#### **Module: `sfc_commutation_monitor`**

**Description:**
A persistent class that maintains the state of each line voltage to detect commutation notches based on firing pulses and line voltage levels.

**Inputs:**

*   `phase_voltages` (`abc<voltage_pu_t>`): A data structure containing the instantaneous per-unit measurements of the three phase voltages.
*   `pulses` (`control::six_pulse_generator::pulses`): A data structure containing the boolean state of the six thyristor firing pulses.

**Outputs:**

*   `is_commuting` (boolean): A signal that is `true` if any of the three line voltages are currently in a commutation notch. Otherwise, it is `false`.

**Configurable Parameters:**

*   `config` (struct): A data structure containing the configuration parameters for the detection algorithm.
    *   `commutation_start_threshold` (float): The per-unit voltage level below which commutation is considered to have started, after the detection has been armed by a firing pulse.
        *   **Default:** 0.05 (i.e., 5%)
    *   `commutation_end_threshold` (float): The per-unit voltage level that the line voltage must cross for the commutation to be considered complete.
        *   **Default:** 0.10 (i.e., 10%)

**Internal State Variables:**

*   `_is_commuting_ab`, `_is_commuting_bc`, `_is_commuting_ca` (boolean): Internal flags for the commutation state of each line.
*   `_is_armed_ab`, `_is_armed_bc`, `_is_armed_ca` (boolean): Internal flags indicating that a firing pulse has occurred and the monitor is waiting for the voltage to drop.
*   `_p1_detector`...`_p6_detector` (`edge_detector`): Internal edge detectors for each firing pulse.

#### **Function Prototype**

The `update` method of the `sfc_commutation_monitor` class is called within the main control loop.

```cpp
bool sfc_commutation_monitor::update(const abc<voltage_pu_t>& phase_voltages, const control::six_pulse_generator::pulses& pulses);
```

**Description:**
This function is called at each time step of the control algorithm. It takes the latest phase voltage measurements and firing pulses, updates the internal state of the detector, and returns the current `is_commuting` status.

**Implementation Logic (Pseudocode):**

```
// For each line (e.g., Vab commutation, which shorts Vca):
// 1. Detect rising edge of relevant firing pulses (e.g., T1 or T4)
has_fired = rising_edge(T1) or rising_edge(T4)

// 2. Update line commutation state
if has_fired
    internal_state.is_armed_ab = true
end

if internal_state.is_armed_ab == true and internal_state.is_commuting_ab == false
    // Armed, waiting for voltage to drop
    if abs(Vca) < config.commutation_start_threshold
        internal_state.is_commuting_ab = true
    end
else if internal_state.is_commuting_ab == true
    // Commuting, waiting for voltage to recover
    if abs(Vca) > config.commutation_end_threshold
        internal_state.is_commuting_ab = false
        internal_state.is_armed_ab = false // Disarm
    end
end

// 3. Repeat for other lines (Vbc, Vca)

// 4. Determine the master 'is_commuting' signal
is_commuting = internal_state.is_commuting_ab OR internal_state.is_commuting_bc OR internal_state.is_commuting_ca

return is_commuting
```