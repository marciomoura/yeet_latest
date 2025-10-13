# Static Frequency Converter (SFC) Sequencing for Gas Turbine Starting

## Overview

This document describes the operational sequencing and control architecture for a Load-Commutated Inverter (LCI) based Static Frequency Converter used to start large gas turbines with synchronous motors. The control logic is based on pre-defined states and acceleration profiles, not on continuously variable commands or signal toggling.

---

## Table of Contents

1. [Control Architecture](#control-architecture)
2. [Signal Logic and Communication](#signal-logic-and-communication)
3. [Startup Sequence Example](#startup-sequence-example)
4. [Acceleration Profile Configuration](#acceleration-profile-configuration)
5. [LCI-Specific Operational Considerations](#lci-specific-operational-considerations)
6. [Fault Conditions and Protective Responses](#fault-conditions-and-protective-responses)
7. [Commissioning Guidelines](#commissioning-guidelines)

---

## Control Architecture

### **The Core Concept: Master and Slave Control**

The system operates with a clear master-slave hierarchy:

*   **TCS (Turbine Control System):** The **master controller** or "brain." It orchestrates the entire startup sequence for the gas turbine (e.g., "purge for 3 minutes at 20% speed," "ignite at 60% speed"). It makes all high-level decisions based on time, temperature, pressure, and speed feedback from the turbine.

*   **SFC (Static Frequency Converter):** The **slave controller** or "muscle." It doesn't know *why* it needs to reach 20% speed, only *how* to achieve it safely. Its software contains pre-configured profiles for acceleration, deceleration, speed holding, and protective limits. The SFC is responsible for:
    - DC link current regulation (via line-side rectifier firing angle control)
    - Motor torque and speed control (via machine-side inverter firing angle control)
    - Field excitation management
    - Commutation control and monitoring
    - Real-time protective functions

The digital signals (DI1-DI10) are simple, unambiguous commands the TCS sends to the SFC to select which pre-defined operational mode to execute.

**Internal SFC Control Structure**: The SFC implements a multi-loop control architecture with:
- **Outer loop**: Speed control (when in "keep" modes)
- **Inner loop**: DC link current control
- **Firing angle calculation**: Separate alpha angles for rectifier and inverter based on control demands and commutation margin requirements

---

## Signal Logic and Communication

### **How the Signals Work: State Changes, Not Toggles**

When a signal transitions from **0 (Logic Low) to 1 (Logic High)**, the TCS is issuing a new command or enabling a state. The signal will typically **remain at 1** for the entire duration that the command is active. It is **not** toggled on and off to incrementally adjust speed.

**Incorrect Interpretation (Toggling):**
```
TCS toggles DI8: 0→1→0→1→0 ... each pulse adds 1% speed
❌ This is inefficient, timing-dependent, and error-prone
```

**Correct Interpretation (Profile-Based):**
```
TCS: DI8 goes 0→1 (and stays 1)
SFC: "Acceleration command active. Execute pre-programmed ramp."
     Continues accelerating per internal profile until DI8 returns to 0
✓ This is deterministic, safe, and repeatable
```

**Important**: Only **one acceleration/deceleration command** should be active at any time. Simultaneous conflicting commands (e.g., `DI8` and `DI3` both high) should trigger a fault condition.

### **Feedback from SFC to TCS**

While the TCS sends commands to the SFC via digital inputs, the SFC provides critical feedback via:
- **Analog outputs**: Actual motor speed (RPM or % of rated), DC link current, field current
- **Digital outputs**: Status signals (Ready, Running, Fault, At-Speed, etc.)
- **Communication bus** (optional): Detailed diagnostics, temperatures, operating mode

This closed-loop communication ensures the TCS always knows the actual system state and can make informed decisions about the next sequence step.

---

## Startup Sequence Example

### **Typical Gas Turbine Starting Sequence Using DI1-DI10**

This example illustrates a complete starting sequence from rest to self-sustaining turbine operation.

**Initial State:** 
- Turbine at rest, shaft stationary
- SFC powered on, pre-checks complete, system ready
- All TCS command signals (DI1-DI10) at **0**
- Main circuit breakers open

---

### **Step 1: Initiate Startup Sequence**

*   **Action:** Plant operator issues "Start" command to TCS
*   **TCS Signal:** `DI1: OPERATION ON CMD` → **1**
*   **SFC Response:** 
    - Performs final system checks (coolant flow, temperature limits, interlocks)
    - Closes main circuit breakers
    - Energizes control power to converters
    - Enters "Ready to Start" state
    - Waits for next command
    - `DI1` remains **1** throughout the entire startup

---

### **Step 1b: Field Excitation Initialization**

*   **Action:** Before mechanical rotation, establish magnetic field in the synchronous motor
*   **SFC Response:**
    - Ramps up field current to initial starting level (typically 20-40% of rated field current)
    - Monitors field voltage and current for faults
    - Confirms sufficient excitation for torque production
*   **Purpose:** Ensures adequate magnetic flux for:
    - Initial torque generation
    - Back-EMF generation for load commutation
    - Stable motor operation during acceleration
*   **Duration:** 1-3 seconds
*   **Critical**: Without proper field excitation, the LCI cannot commutate and the motor cannot produce torque

---

### **Step 2: Begin Purge Cycle (Acceleration)**

*   **Action:** TCS initiates turbine purge cycle, requiring shaft rotation at 20% nominal speed
*   **TCS Signal:** `DI8: ACCELERATION REQUEST` → **1**
*   **SFC Response:** 
    - Activates "Purge Acceleration" profile (Profile 1)
    - **Low-Speed Operation**: At very low speeds (0-10% typically), motor back-EMF is insufficient for natural load commutation. SFC operates in:
        - **Pulse mode** (very low frequency, discrete firing pulses), OR
        - **Forced commutation mode** (using auxiliary commutation circuits)
    - **Transition to Natural Commutation**: Around 10-15% speed, motor back-EMF becomes adequate. SFC transitions to natural (load) commutation mode
    - Smoothly ramps DC link current and adjusts inverter firing angles
    - Continuously monitors commutation margin to prevent failures
    - Continues ramping as long as `DI8` = **1**
    - Reports actual speed to TCS every cycle
*   **Typical Parameters**: 
    - Acceleration time: 120 seconds (0-100% basis)
    - Torque limit: 40% of rated
    - Current limit: 50% of rated
    - S-curve: 30% (gentle ease-in/ease-out)

---

### **Step 3: Hold Purge Speed (Keep)**

*   **Action:** TCS detects turbine has reached target purge speed (20% via speed feedback). Must hold this speed for timed purge cycle (typically 2-5 minutes)
*   **TCS Signals:** 
    - `DI8: ACCELERATION REQUEST` → **0**
    - `DI5: LOW SPEED KEEP REQUEST` → **1**
*   **SFC Response:** 
    - Detects mode transition from "Accelerate" to "Keep"
    - Activates **closed-loop speed controller**:
        - Measures actual motor speed continuously (via encoder/resolver)
        - Compares to setpoint (captured speed at moment of transition)
        - Adjusts DC link current to maintain constant speed against load disturbances
    - Regulates firing angles for both rectifier and inverter
    - Maintains commutation margin monitoring
    - Holds this state as long as `DI5` = **1**
*   **Speed Regulation**: Typically ±0.5% of setpoint during hold
*   **Load Compensation**: Automatically compensates for bearing friction, windage, and any generator load

---

### **Step 4: Accelerate to Ignition Speed**

*   **Action:** Purge timer complete. TCS initiates acceleration to ignition speed (typically 50-60% of rated)
*   **TCS Signals:** 
    - `DI5: LOW SPEED KEEP REQUEST` → **0**
    - `DI3: HIGH SPEED SPIN REQUEST` → **1**
*   **SFC Response:** 
    - Detects mode change, activates "Ignition Acceleration" profile (Profile 3)
    - This profile is typically **faster** than purge ramp:
        - Acceleration time: 45 seconds (0-100% basis)
        - Higher torque limit: 80% of rated
        - Objective: Pass quickly through any mechanical resonant frequencies
    - Natural load commutation is now fully established (sufficient back-EMF)
    - Continues accelerating until commanded to stop
*   **Note**: Uses a different digital input (`DI3` not `DI8`) to select a distinct acceleration profile. This ensures the correct ramp rate and limits are applied for this phase.

---

### **Step 5: Hold for Ignition (Keep)**

*   **Action:** Ignition speed reached (50-60%). TCS holds speed constant while managing fuel valve opening, spark plug firing, and flame detection
*   **TCS Signals:** 
    - `DI3: HIGH SPEED SPIN REQUEST` → **0**
    - `DI4: HIGH SPEED KEEP REQUEST` → **1**
*   **SFC Response:** 
    - Transitions to high-speed hold mode
    - Maintains precise speed control (±0.3% typical)
    - Provides stable platform for combustion initiation
    - Duration: 5-30 seconds (until flame confirmed)
*   **Critical Function**: Speed stability during this phase is crucial for successful ignition. Variations can cause fuel-air ratio mismatches and failed light-off attempts.

---

### **Step 6: Full Acceleration to Self-Sustaining Speed**

*   **Action:** Ignition successful, flame stable. Turbine now generating power internally but requires assistance to reach self-sustaining speed (typically 70-75% of rated speed)
*   **TCS Signals:** 
    - `DI4: HIGH SPEED KEEP REQUEST` → **0**
    - `DI6: SFC ACCELERATION REQUEST` → **1**
*   **SFC Response:** 
    - Activates "Assist Acceleration" profile (Profile 4) - **most aggressive**:
        - Acceleration time: 30 seconds (0-100% basis)
        - Torque limit: 100% of rated
        - Current limit: 100% of rated
        - S-curve: 15% (minimal smoothing for fastest response)
    - Turbine is now warm, combustion established
    - SFC provides maximum support to reach self-sustaining condition
    - Field current may be increased to maximum for highest torque capability
*   **Load Sharing**: As turbine develops its own torque from combustion, SFC automatically reduces its contribution. The speed controller maintains constant speed while torque responsibility shifts from SFC to turbine.

---

### **Step 7: Disconnect and Sequence Complete**

*   **Action:** Turbine reaches self-sustaining speed (70-75% typically). Combustion torque now exceeds all losses. SFC no longer required.
*   **TCS Signals:** 
    - `DI6: SFC ACCELERATION REQUEST` → **0**
    - `DI2: OPERATION OFF CMD` → **1**
*   **SFC Response:** 
    - Receives shutdown command
    - Executes controlled ramp-down sequence:
        1. Gradually reduces inverter firing angles (torque reduction over 1-2 seconds)
        2. Ramps down DC link current to zero
        3. Reduces field excitation to zero
        4. Opens main circuit breakers
        5. De-energizes power converters
    - Enters safe standby mode
    - **Maintains `DI1: OPERATION ON` monitoring** - if it goes to 0, emergency stop
*   **Turbine Status**: Now operating independently on combustion power, continuing acceleration to idle speed (typically 95-100%) under its own fuel control

---

### **Sequence Summary**

This startup sequence demonstrates that the TCS uses digital inputs as **mode selectors** to trigger pre-programmed, thoroughly tested operational profiles in the SFC. This architecture provides:
- **Repeatability**: Every startup follows identical profiles
- **Safety**: All limits and ramps are pre-validated
- **Simplicity**: TCS doesn't need to know the details of firing angle control, commutation, or power electronics
- **Reliability**: Proven, deterministic behavior every time

---

## Acceleration Profile Configuration


This section explains how acceleration profiles are designed, parameterized, and commissioned within the SFC software.

---

### **1. Who Dictates the Acceleration? The TCS or the SFC?**

This is a partnership with clearly defined roles:

**TCS Dictates "WHAT" and "WHEN":**
*   Acts as master controller
*   Decides *when* to accelerate and *which type* of acceleration is needed
*   Issues commands via digital signals (e.g., `DI8` for purge acceleration)
*   Monitors progress and sequences to next step when conditions are met

**SFC Dictates "HOW":**
*   Contains the actual acceleration profile algorithms
*   Executes pre-programmed control instructions
*   Manages real-time firing angle calculations for rectifier and inverter
*   Controls DC link current, field excitation, and commutation
*   Enforces all safety limits (torque, current, voltage)
*   Responsible for millisecond-level control decisions

**Analogy**: Like a car's cruise control:
- You (TCS): "Accelerate to 60 MPH now"
- Engine Controller (SFC): Manages throttle, fuel, transmission to achieve it smoothly
- You don't tap the pedal repeatedly; one command triggers the complete process

**Internal SFC Operation**: When a profile is active, the SFC translates the speed reference ramp into:
- Target DC link current (from outer speed loop or direct ramp reference)
- Rectifier firing angle (alpha_rect) to achieve that current
- Inverter firing angle (alpha_inv) to produce required motor torque while maintaining adequate commutation margin
- Field current adjustment for optimal flux control

---

### **2. Is the Acceleration Constant? No, It's a "Profile"**

Acceleration is almost never a simple constant rate. Instead, it follows a carefully engineered **acceleration profile** or **ramp function**.

**Basic Profile Types:**

1. **Linear Ramp** (Simple but less ideal):
   - Constant acceleration rate: e.g., +0.5 Hz/second
   - Step change in acceleration at start and stop causes mechanical shock

2. **S-Curve Ramp** (Industry Standard):
   - **Ease-In**: Acceleration starts very gently (low jerk)
   - **Linear Section**: Ramps at main acceleration rate
   - **Ease-Out**: Gently reduces acceleration near target (smooth arrival)
   - Eliminates step changes in acceleration
   - Minimizes mechanical stress and vibration

**Why S-Curves Matter:**

The controlled profile protects multi-million dollar equipment from:
- **Mechanical shock** ("jerk" = rate of change of acceleration)
- **Shaft torsional oscillations**
- **Gearbox tooth stress**
- **Coupling misalignment**
- **Bearing loading spikes**

**Mathematical Implementation:**
```
Position (speed) = f(time)
Velocity (acceleration) = f'(time)  
Acceleration (rate) = f''(time)
Jerk = f'''(time) ← This is what S-curves control
```

S-curve ramps ensure jerk is finite and bounded, rather than infinite at step changes.



---

### **3. Is it Parametrized in the Software? Yes, Absolutely**

This is the key to system flexibility. Acceleration profiles are **not hard-coded**. They are defined by parameter sets accessible to commissioning engineers via configuration software or HMI.

**Common Profile Parameters:**

| Parameter                 | Units              | Typical Range | Description                                                                    |
| ------------------------- | ------------------ | ------------- | ------------------------------------------------------------------------------ |
| **Acceleration Time**     | seconds            | 20-300        | Time to accelerate from 0% to 100% speed (on 100% basis)                       |
| **Deceleration Time**     | seconds            | 10-120        | Time to decelerate from 100% to 0%                                             |
| **S-Curve Percentage**    | %                  | 0-50          | Portion of ramp time used for ease-in and ease-out (0% = linear)               |
| **Torque Limit**          | % of rated         | 20-150        | Maximum electromagnetic torque (protects shaft from overstress)                |
| **Current Limit**         | % of rated or Amps | 50-120        | Maximum DC link current (protects power electronics)                           |
| **Initial Torque Boost**  | %                  | 0-50          | Extra torque pulse at startup to overcome static friction ("breakaway torque") |
| **Boost Duration**        | seconds            | 0.1-2.0       | How long the initial torque boost is applied                                   |
| **Minimum Speed**         | RPM or %           | 0-10          | Speed below which forced commutation is used                                   |
| **Field Current Profile** | % of rated         | 20-100        | Field excitation level during this profile (may vary with speed)               |

**Critical Safety Parameters:**

- **Torque Limit**: The SFC will **never** exceed this value, even if the TCS requests faster acceleration. Prevents shaft damage.
- **Current Limit**: Protects thyristors/IGBTs from thermal damage. The SFC will automatically slow the acceleration if approaching this limit.
- **Commutation Margin Limit**: (LCI-specific) Minimum extinction angle. If violated, the SFC immediately reduces firing angle advance to prevent commutation failure.

**Commissioning Process:**

Engineers tune these parameters based on:
1. Turbine manufacturer specifications (maximum shaft torque, acceleration rate)
2. Mechanical resonance testing (identify critical speed ranges)
3. Thermal modeling of power electronics
4. Field trials with instrumented shaft torque measurements
5. Iterative optimization for fastest safe startup

---

### **4. How Many Acceleration Profiles May There Be?**

There can be **multiple distinct profiles** (typically 3-6), each tied to a specific digital input signal or operational mode. The SFC software maps each TCS command to a specific parameter set.

**Typical Profile Configuration for Gas Turbine Starting:**

#### **Profile 1: "Creep Speed" (Maintenance Mode)**
*   **Associated Signal:** `DI7: COLD-DRAG START COMMAND`
*   **Purpose:** Manual maintenance mode - slowly rotate shaft for inspection
*   **Characteristics:**
    - Accel Time: 300 seconds
    - S-Curve: 40%
    - Torque Limit: 15%
    - Target Speed: 5-10% maximum
    - Extremely gentle, lowest stress possible

#### **Profile 2: "Purge Acceleration"**
*   **Associated Signal:** `DI8: ACCELERATION REQUEST`
*   **Purpose:** Initial acceleration to purge speed
*   **Characteristics:**
    - Accel Time: 120 seconds
    - S-Curve: 30%
    - Torque Limit: 40%
    - Current Limit: 50%
    - Initial Torque Boost: 30% (overcome static friction)
    - Conservative ramp for cold turbine protection

#### **Profile 3: "Ignition Acceleration"**
*   **Associated Signal:** `DI3: HIGH SPEED SPIN REQUEST`
*   **Purpose:** Accelerate from purge to ignition speed
*   **Characteristics:**
    - Accel Time: 45 seconds
    - S-Curve: 20%
    - Torque Limit: 80%
    - Objective: Pass quickly through mechanical resonant frequencies
    - More aggressive than purge, turbine is warmer

#### **Profile 4: "Assist Acceleration"**
*   **Associated Signal:** `DI6: SFC ACCELERATION REQUEST`
*   **Purpose:** Final push to self-sustaining speed after ignition
*   **Characteristics:**
    - Accel Time: 30 seconds
    - S-Curve: 15%
    - Torque Limit: 100%
    - Current Limit: 100%
    - Most aggressive profile
    - Turbine is hot, combustion established
    - Maximum power electronics capability utilized

#### **Profile 5: "Emergency Deceleration"**
*   **Purpose:** Controlled shutdown on fault or emergency stop
*   **Characteristics:**
    - Decel Time: 10-20 seconds
    - Torque limited to avoid reverse torque shock
    - Field excitation maintained throughout for controlled braking

---

**Profile Selection Logic:**

```
IF DI7 = 1 THEN Profile = Creep_Speed
ELSE IF DI8 = 1 THEN Profile = Purge_Accel
ELSE IF DI3 = 1 THEN Profile = Ignition_Accel  
ELSE IF DI6 = 1 THEN Profile = Assist_Accel
ELSE IF DI2 = 1 THEN Profile = Emergency_Decel
ELSE Profile = Hold_Current_Speed
```

**Analogy**: The TCS is the orchestra conductor, telling the SFC *which piece of music to play* and *when*. The SFC is the skilled musician with the sheet music (parametrized profiles) ready, executing perfectly every time it's commanded.

---

### **5. The Ramp Function Generator: Software Implementation**

This section explains the internal software mechanism that generates the smooth acceleration profiles.

#### **The Standard Method: Ramp Function Generator Block**

The "Ramp Function Generator" is a standard software block within the SFC. Its job: take a sudden "step" command for speed (e.g., "go from 0% to 20% now") and transform it into a gradual, controlled ramp.

**Key Point**: This is configured using simple parameters, **NOT** a complex lookup table.

Here’s how it works conceptually:

1.  **The Input Command:** The SFC receives a command that changes the *target speed*. For example, when the TCS activates `DI8: ACCELERATION REQUEST`, the SFC's internal logic might change its target speed from 0 RPM to 1200 RPM (the purge speed). This is an instantaneous step change *inside the software*.
2.  **The Ramp Generator:** This step change is fed into the Ramp Function Generator.
3.  **The Parameters:** The engineer has pre-configured this generator with parameters like:
    *   **Acceleration Time:** The time it should take to go from 0% to 100% speed (e.g., `120.0 seconds`). The software uses this to calculate a rate.
    *   **S-Curve Percentage (or Jerk Control):** A parameter (e.g., `25%`) that defines how much to "round off" the corners of the ramp to make the start and end of acceleration smoother.
4.  **The Output:** The output of the ramp generator is a new, smoothly changing speed reference that goes up second-by-second. This smooth reference is what the SFC's motor control loop actually tries to follow.

**Visualizing the Process:**

*   **TCS Command (Input):** A sudden jump from 0 RPM to 1200 RPM.
*   **Ramp Generator Output (What the motor sees):** A smooth line starting at 0 RPM and gradually increasing over many seconds until it reaches 1200 RPM.

---

## LCI-Specific Operational Considerations

This section addresses aspects unique to Load-Commutated Inverter systems that are critical for successful operation.

---

### **1. Commutation Modes and Transition**

**The Commutation Challenge:**

LCI systems rely on the motor's back-EMF for natural (load) commutation of the machine-side inverter thyristors. At low speeds, insufficient back-EMF makes natural commutation impossible.

**Low-Speed Operation (0-10% typical):**

The SFC must use alternative strategies:

1. **Pulse Mode (Discrete Firing)**:
   - Very low frequency operation
   - Individual, discrete firing pulses
   - Long intervals between pulses
   - Each pulse provides a small torque impulse

2. **Forced Commutation**:
   - Auxiliary commutation circuits (capacitors, auxiliary thyristors)
   - Actively turns off conducting thyristors
   - More complex but allows smoother low-speed operation

**Transition to Natural Commutation:**

- **Threshold Speed**: Typically 10-15% of rated speed
- **Detection**: SFC monitors back-EMF magnitude
- **Smooth Transition**: Gradually shifts from forced to natural commutation over a speed range
- **Hysteresis**: Different thresholds for increasing vs decreasing speed to prevent oscillation

**Commutation Margin Monitoring:**

Throughout operation, the SFC continuously monitors:
- **Extinction Angle (γ)**: Time available for thyristor turn-off
- **Minimum Safe Value**: Typically γ_min = 18-25° electrical
- **If γ < γ_min**: Immediately reduce inverter firing angle advance to prevent commutation failure
- **Protection Priority**: Commutation protection overrides all other control demands

---

### **2. Firing Angle Control Strategy**

**Rectifier Side (Line-Side Converter):**

- **Primary Function**: Control DC link current
- **Firing Angle (α_rect)**: Typically 5-90°
- **Control Law**: α_rect = f(I_dc_error, V_grid)
- **Operation**: 
  - Small α → High DC voltage → High current
  - Large α → Low DC voltage → Low current

**Inverter Side (Machine-Side Converter):**

- **Primary Function**: Control motor torque while maintaining commutation margin
- **Firing Angle (α_inv)**: Typically 90-150° (leading)
- **Control Law**: α_inv = f(Torque_demand, Speed, γ_min)
- **Balancing Act**:
  - More advance (larger α) → More torque, but less commutation margin
  - Less advance (smaller α) → More commutation margin, but less torque

**Coordinated Control:**

The SFC coordinates both converters:
```
Speed_Error → [Speed Controller] → Torque_Demand
                                         ↓
Torque_Demand → [Current Reference] → I_dc_ref
                                         ↓
I_dc_ref → [Current Controller] → α_rect (Rectifier)
                                         ↓
Torque_Demand + Speed → [Angle Calculator] → α_inv (Inverter)
                                              (subject to γ_min constraint)
```

---

### **3. Field Excitation Control**

**Purpose of Field Control:**

The synchronous motor's field winding creates the magnetic field necessary for:
- Electromagnetic torque production
- Back-EMF generation for commutation
- Power factor control

**Field Current Profiles:**

| Operating Phase             | Field Current     | Rationale                                                                   |
| --------------------------- | ----------------- | --------------------------------------------------------------------------- |
| **Startup (0-20% speed)**   | 20-40% rated      | Lower field reduces required SFC current, eases commutation at low back-EMF |
| **Acceleration (20-70%)**   | 60-80% rated      | Increased field for higher torque capability                                |
| **High Speed (70-100%)**    | 80-100% rated     | Maximum field for maximum torque assist                                     |
| **Field Weakening (>100%)** | <100%, decreasing | If overspeed is needed, field is weakened to reduce back-EMF                |

**Field Ramping:**

- **Never step-change field current** - always ramp smoothly
- **Typical ramp rates**: 5-20% per second
- **Coordination**: Field changes must be coordinated with firing angle adjustments

---

### **4. Speed Feedback and Closed-Loop Control**

**Speed Measurement:**

- **Encoder**: High-resolution digital (1024-4096 pulses/rev typical)
- **Resolver**: Analog, very robust, common in industrial applications
- **Update Rate**: Every control cycle (500 µs - 2 ms typical)

**Speed Control Loop (During "Keep" Modes):**

```
Speed_Setpoint ─────────┬─[─] → [PI Controller] → Torque_Demand
                        │   -
Actual_Speed (measured)─┘

Torque_Demand → [Firing Angle Control] → α_inv
```

**Typical PI Controller Parameters:**
- **Kp** (Proportional Gain): 0.5 - 5.0 (dimensionless, per-unit)
- **Ki** (Integral Gain): 0.1 - 2.0 (1/second)
- **Integration Limits**: ±100% torque

**Speed Regulation Performance:**
- **Steady-State Accuracy**: ±0.2-0.5% of setpoint
- **Load Disturbance Rejection**: Returns to setpoint within 1-2 seconds

---

## Fault Conditions and Protective Responses

The SFC continuously monitors for fault conditions and implements immediate protective actions.

---

### **Critical Fault Conditions**

#### **1. Commutation Failure**

**Detection:**
- Thyristor current doesn't reach zero within expected time
- DC link overcurrent spike
- Phase current imbalance

**Response:**
- Immediate reduction of inverter firing angle advance
- If repeated (>3 failures within 10 seconds): Emergency shutdown
- Log fault with timestamp and operating conditions

#### **2. Overcurrent (DC Link)**

**Threshold:** Typically 120-150% of rated current

**Response:**
- Immediate firing angle retardation (both rectifier and inverter)
- If current doesn't decrease within 50ms: Open circuit breaker
- Torque reference limited to prevent recurrence

#### **3. Overvoltage / Undervoltage (DC Link)**

**Thresholds:** 
- Overvoltage: >110-120% rated
- Undervoltage: <70-80% rated

**Response:**
- Adjust rectifier firing angle to compensate
- If outside control range: Fault and shutdown

#### **4. Loss of Speed Feedback**

**Detection:**
- Encoder signal loss
- Implausible speed values
- Speed sensor disagreement (if redundant sensors)

**Response:**
- Switch to open-loop control mode (if safe)
- OR: Controlled deceleration and shutdown
- Cannot continue in "Keep" mode without speed feedback

#### **5. Grid Disturbances**

**Conditions:**
- Voltage sag/swell
- Phase loss
- Frequency deviation

**Response:**
- Adjust firing angles to compensate
- Reduce current limits if necessary
- If severe: Controlled shutdown

#### **6. TCS Communication Loss**

**Detection:**
- Watchdog timer expires (no signal changes for >5 seconds)
- Communication bus timeout

**Response:**
- Hold current operating state for brief period (5-10 seconds)
- If not restored: Controlled shutdown to safe state
- Do not allow startup without TCS communication

---

### **Emergency Stop Sequence**

When `DI2: OPERATION OFF` is activated OR any critical fault occurs:

**Timing and Actions:**

| Time               | Action                                  | Purpose                                            |
| ------------------ | --------------------------------------- | -------------------------------------------------- |
| **t = 0 ms**       | Fault detected / Stop command received  |                                                    |
| **t = 0-10 ms**    | Reduce inverter firing angle advance    | Reduce electromagnetic torque immediately          |
| **t = 10-1000 ms** | Ramp down DC link current (controlled)  | Prevent mechanical shock while stopping power flow |
| **t = 1000 ms**    | Set field current to zero               | Remove magnetic field                              |
| **t = 1200 ms**    | Open main circuit breakers              | Electrically isolate the system                    |
| **t = 1500 ms**    | De-energize control power to converters | Safe state achieved                                |

**Post-Fault Actions:**
- Log detailed fault data (conditions, measurements, timestamps)
- Prevent restart until fault is acknowledged by operator or TCS
- Require explicit reset sequence before allowing next startup

---

## Commissioning Guidelines

This section provides practical guidance for commissioning the SFC system with a gas turbine.

---

### **Pre-Commissioning Checks**

**Electrical:**
- [ ] Verify grid voltage and frequency are within specifications
- [ ] Confirm circuit breaker ratings and operation
- [ ] Test emergency stop circuits
- [ ] Verify encoder/resolver installation and direction
- [ ] Check field winding resistance and insulation

**Mechanical:**
- [ ] Confirm shaft alignment (motor to turbine)
- [ ] Verify bearing lubrication systems operational
- [ ] Check for mechanical binding or excessive friction
- [ ] Install shaft torque monitoring (if available)

**Control System:**
- [ ] TCS communication established and tested
- [ ] All digital input/output signals verified
- [ ] Analog feedback signals calibrated
- [ ] Safety interlocks tested

---

### **Profile Tuning Procedure**

**Step 1: Conservative Initial Parameters**

Start with very conservative profiles:
- Long acceleration times (180+ seconds)
- Low torque limits (30%)
- High S-curve percentages (40%)

**Step 2: Baseline Test**

- Execute complete startup sequence with conservative parameters
- Monitor:
  - Shaft torque (if instrumented)
  - DC link current
  - Motor temperature rise
  - Vibration levels
  - Commutation margins

**Step 3: Iterative Optimization**

Gradually adjust parameters:
1. Reduce acceleration time by 10-20%
2. Test and monitor
3. Verify no mechanical resonances excited
4. Check thermal margins on power electronics
5. Repeat until optimal performance achieved

**Step 4: Limit Testing**

- Verify torque limit enforcement (command higher acceleration, confirm torque caps)
- Test current limit enforcement
- Verify commutation failure protection activates correctly

**Step 5: Emergency Stop Testing**

- Test emergency stop from various speeds
- Confirm controlled deceleration
- Verify no mechanical shock or overspeed

**Step 6: Endurance Testing**

- Multiple consecutive startups
- Monitor for thermal accumulation
- Verify repeatability

---

### **Documentation Requirements**

For each commissioned system, document:

1. **Final Parameter Sets**: All profile parameters for each mode
2. **Test Results**: Data from limit testing and optimization
3. **Baseline Performance**: Speed traces, current profiles, torque measurements
4. **Fault Log**: Any faults encountered during commissioning and resolution
5. **Safety Verification**: Evidence that all protective functions operate correctly

---

## Conclusion

This document has provided a comprehensive overview of Static Frequency Converter sequencing for gas turbine starting applications, with specific focus on Load-Commutated Inverter systems. The key takeaways are:

1. **Master-Slave Architecture**: TCS commands "what" and "when"; SFC executes "how"
2. **Profile-Based Control**: Pre-configured, parametrized profiles ensure repeatable, safe operation
3. **LCI-Specific Considerations**: Commutation management, firing angle control, and field excitation are critical
4. **Safety First**: Multiple layers of protective functions and fault responses
5. **Commissioning is Critical**: Proper tuning and testing ensure reliable operation

The profile-based approach provides the optimal balance of flexibility, safety, and reliability required for starting multi-million dollar gas turbine assets.

---

**Document Version:** 1.0  
**Last Updated:** October 12, 2025  
**Author:** SFC Control System Documentation
