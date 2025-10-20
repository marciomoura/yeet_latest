# Grid-Side Control I/O Mapping

This document provides a comprehensive mapping of hardware I/O signals to the grid-side control system for the SFC (Static Frequency Converter) embedded application.

## Overview

The grid-side controller manages the line-side rectifier of the LCI system. It receives supervisory commands, processes grid measurements, and generates thyristor firing pulses to regulate DC link current.

**Key Functions:**
- State machine control (startup/shutdown sequencing)
- Grid voltage/current measurement processing
- Current control loop (DC link regulation)
- Thyristor firing pulse generation
- Cross-communication with machine-side controller

---

## 1. Digital Inputs (Supervisory Commands)

Digital inputs provide supervisory control signals from the system operator or higher-level controller (HMI/SCADA).

| Hardware DI | Task I/O Index                 | Signal Name          | Purpose                                                        | Active State | Update Rate |
| ----------- | ------------------------------ | -------------------- | -------------------------------------------------------------- | ------------ | ----------- |
| DI1         | `digital_inputs.signals[0]`    | Auxiliaries Ready    | Confirms auxiliary systems (cooling, lube oil) are operational | High         | 1 kHz       |
| DI2         | `digital_inputs.signals[1]`    | Breakers Closed      | Confirms grid breakers are closed and ready                    | High         | 1 kHz       |
| DI3         | `digital_inputs.signals[2]`    | Start Command        | Command to begin operation                                     | High         | 1 kHz       |
| DI4         | `digital_inputs.signals[3]`    | Stop Command         | Command to stop operation (controlled shutdown)                | High         | 1 kHz       |
| DI5         | `digital_inputs.signals[4]`    | Emergency Shutdown   | Emergency shutdown request (immediate stop)                    | High         | 1 kHz       |
| DI6         | `digital_inputs.signals[5]`    | Current Extinguished | Confirms DC link current is zero (safety interlock)            | High         | 1 kHz       |
| DI7-DI16    | `digital_inputs.signals[6-15]` | *Reserved*           | Available for application-specific inputs                      | -            | 1 kHz       |

**State Machine Sequence:**
1. `Auxiliaries Ready` = High (cooling, oil systems operational)
2. `Breakers Closed` = High (grid connection established)
3. `Start Command` = High → System enters OPERATION_ON state
4. During operation: Monitor interlocks
5. `Stop Command` = High → Controlled shutdown sequence
6. `Current Extinguished` = High → Safe to open breakers

---

## 2. Analog Inputs (Grid Measurements)

Analog inputs provide real-time measurements of grid voltages and currents via 16-bit ADC (AD7606).

### 2.1 Hardware Channel Mapping

| Hardware AI | Task I/O Index              | Signal Name        | Physical Quantity                | Transducer Ratio   | ADC Range            | Update Rate |
| ----------- | --------------------------- | ------------------ | -------------------------------- | ------------------ | -------------------- | ----------- |
| AI1 (X1-1)  | `analog_inputs.channels[0]` | Grid Voltage V_AB  | Line-to-line voltage AB          | 100V → 3.5V        | ±10V (±200V primary) | 5 kHz       |
| AI2 (X1-2)  | `analog_inputs.channels[1]` | Grid Voltage V_BC  | Line-to-line voltage BC          | 100V → 3.5V        | ±10V (±200V primary) | 5 kHz       |
| AI3 (X1-3)  | -                           | Grid Voltage V_CA  | Line-to-line voltage CA (unused) | 100V → 3.5V        | ±10V (±200V primary) | -           |
| AI4 (X1-4)  | -                           | *White Noise*      | Test signal generator (unused)   | -                  | ±10V                 | -           |
| AI5 (X1-5)  | `analog_inputs.channels[4]` | Grid Current I_A   | Phase A current                  | 1A → 5V or 5A → 5V | ±10V (±7.5A primary) | 5 kHz       |
| AI6 (X1-6)  | -                           | Grid Current I_B   | Phase B current (reconstructed)  | 1A → 5V or 5A → 5V | ±10V (±7.5A primary) | -           |
| AI7 (X1-7)  | `analog_inputs.channels[6]` | Grid Current I_C   | Phase C current                  | 1A → 5V or 5A → 5V | ±10V (±7.5A primary) | 5 kHz       |
| AI8-AI12    | -                           | *Spare/Excitation* | Reserved for future use          | -                  | -                    | -           |

### 2.2 ADC Conversion

**ADC Specifications:**
- Resolution: 16-bit signed (±32767 counts)
- Input range: ±10V analog
- Conversion: `10V → 32767 counts` (0.305 mV/count)

**Voltage Transducer:**
- Primary range: ±200V (line-to-line)
- Secondary output: 100V → 3.5V
- Full-scale primary: ±200V → ±7V secondary → ±22937 counts

**Current Transducer:**
- Primary range: ±7.5A (phase current)
- Secondary output: 1A → 5V (or 5A → 5V depending on CT ratio)
- Full-scale primary: 7.5A → 5V secondary → 16384 counts

### 2.3 Control Signal Flow

```
Hardware ADC → task_inputs_t → mock_*_counts → grid_measurements_runner → grid_side_observer
                                                                        → dc_link_measurements_runner
```

**Processing Chain:**
1. **Hardware ADC DMA** → Populates `task_inputs_t.analog_inputs.channels[]`
2. **200µs Task** → Writes ADC counts to `mock_v_ab_counts`, `mock_i_a_counts`, etc.
3. **Grid Measurements Runner** → Converts ADC counts to per-unit voltages/currents
4. **DC Link Measurements Runner** → Computes DC current from grid-side AC currents
5. **Grid-Side Observer** → Estimates grid voltage magnitude and phase angle
6. **Current Controller** → Generates firing angle reference based on DC current error

---

## 3. Thyristor Firing Pulses (Primary Control Output)

Thyristor firing pulses control the 6-pulse rectifier bridge on the grid side.

### 3.1 PWM Output Mapping

| Hardware PWM | Task I/O Signal            | Thyristor | Bridge Arm | Phase   | Update Rate |
| ------------ | -------------------------- | --------- | ---------- | ------- | ----------- |
| PWM1 (PC6)   | `thyristor_pulses.a_plus`  | T1        | Positive   | Phase A | 5 kHz       |
| PWM2 (PC7)   | `thyristor_pulses.c_minus` | T6        | Negative   | Phase C | 5 kHz       |
| PWM3 (PC8)   | `thyristor_pulses.b_plus`  | T3        | Positive   | Phase B | 5 kHz       |
| PWM4 (PC9)   | `thyristor_pulses.a_minus` | T4        | Negative   | Phase A | 5 kHz       |
| PWM5 (PB0)   | `thyristor_pulses.c_plus`  | T5        | Positive   | Phase C | 5 kHz       |
| PWM6 (PB1)   | `thyristor_pulses.b_minus` | T2        | Negative   | Phase B | 5 kHz       |

### 3.2 Pulse Generation Control Flow

```
Current Controller → Grid-Side Modulator → Grid-Side Pulse Generator → task_outputs_t → Hardware PWM
```

**Generation Chain:**
1. **Current Controller** → Computes firing angle α based on DC current error (PI control)
2. **Grid-Side Modulator** → Applies operating mode logic, limits, and commutation coordination
3. **Grid-Side Pulse Generator** → Generates synchronized 6-pulse pattern with angle α delay
4. **200µs Task** → Extracts firing pulses and writes to `task_outputs_t.thyristor_pulses`
5. **Hardware PWM Driver** → Applies pulses to thyristor gate drivers

### 3.3 Firing Angle Control

**Operating Modes:**
- **Natural Commutation (α = 15-90°)**: High-speed operation, machine back-EMF commutates thyristors
- **Forced Commutation (α = 90-165°)**: Low-speed operation, requires coordinated machine-side commutation

**Typical Firing Sequence (60° intervals):**
```
0°:   T1 fires (A+)
60°:  T2 fires (B-)
120°: T3 fires (B+)
180°: T4 fires (A-)
240°: T5 fires (C+)
300°: T6 fires (C-)
```

---

## 4. Digital Outputs (Status Indicators)

Digital outputs provide system status to the operator interface (HMI) and interlocking logic.

| Hardware DO | Task I/O Index                  | Signal Name              | Purpose                                              | Active State | Update Rate |
| ----------- | ------------------------------- | ------------------------ | ---------------------------------------------------- | ------------ | ----------- |
| DO1         | `digital_outputs.signals[0]`    | System Running           | Pulse releases are active (operation in progress)    | High         | 1 kHz       |
| DO2         | `digital_outputs.signals[1]`    | Current Zero Status      | DC link current is below threshold                   | High         | 1 kHz       |
| DO3         | `digital_outputs.signals[2]`    | Controller Saturated     | Current controller has reached output limits         | High         | 1 kHz       |
| DO4         | `digital_outputs.signals[3]`    | Forced Commutation Mode  | System operating in forced commutation (low speed)   | High         | 1 kHz       |
| DO5         | `digital_outputs.signals[4]`    | Natural Commutation Mode | System operating in natural commutation (high speed) | High         | 1 kHz       |
| DO6         | `digital_outputs.signals[5]`    | Fiber Link Status        | Fiber-optic link to machine-side is healthy          | High         | 1 kHz       |
| DO7-DO17    | `digital_outputs.signals[6-16]` | *Reserved*               | Available for application-specific outputs           | -            | 1 kHz       |

**Status Indicator Usage:**
- **DO1 (System Running)**: Indicates state machine is in OPERATION_ON state
- **DO2 (Current Zero)**: Used for safe breaker opening during shutdown
- **DO3 (Controller Saturated)**: Indicates current reference cannot be achieved (voltage/speed limitation)
- **DO4/DO5 (Operating Mode)**: Provides operational mode feedback for diagnostics
- **DO6 (Fiber Link)**: Critical for detecting communication failures between boards

---

## 5. Fiber-Optic Communication (Cross-Coupling)

The fiber-optic interface provides high-speed, deterministic communication between grid-side and machine-side controllers.

### 5.1 Receive (From Machine-Side)

**Update Rate:** 1 kHz (1 ms task)

| Payload Index | Signal Name               | Type  | Units    | Purpose                                                            |
| ------------- | ------------------------- | ----- | -------- | ------------------------------------------------------------------ |
| `payload[0]`  | Measured Speed            | float | per-unit | Machine rotor speed for operating mode determination               |
| `payload[1]`  | DC Link Current Reference | float | per-unit | Current setpoint from speed/torque controller cascade              |
| `payload[2]`  | Machine Voltage Magnitude | float | per-unit | Machine back-EMF for current controller feedforward                |
| `payload[3]`  | Force Commutation Request | float | boolean  | Request for grid-side commutation assistance (low-speed operation) |

**Signal Flow:**
```
Machine-Side Controller → Fiber TX → Fiber RX → Grid-Side mock signals → Grid-Side Control
```

### 5.2 Transmit (To Machine-Side)

**Update Rate:** 1 kHz (1 ms task)

| Payload Index | Signal Name                    | Type  | Units       | Purpose                                                   |
| ------------- | ------------------------------ | ----- | ----------- | --------------------------------------------------------- |
| `payload[0]`  | Operating Mode                 | float | enumeration | Current operating mode (forced/natural commutation)       |
| `payload[1]`  | Is Current Zero                | float | boolean     | DC link current zero status for machine-side interlocking |
| `payload[2]`  | Force Commutation Acknowledged | float | boolean     | Grid-side acknowledgment of commutation assistance        |
| `payload[3]`  | Pulse Releases                 | float | boolean     | Grid-side state machine pulse enable status               |

**Signal Flow:**
```
Grid-Side Control → Grid-Side Status Outputs → Fiber TX → Fiber RX → Machine-Side Controller
```

---

## 6. Cross-Coupled Control Signals

These signals coordinate control between grid-side and machine-side subsystems.

### 6.1 Machine-Side → Grid-Side

| Signal Name               | Source                 | Destination            | Purpose                                   |
| ------------------------- | ---------------------- | ---------------------- | ----------------------------------------- |
| Measured Speed            | Machine-Side Observer  | Operation Mode Monitor | Determines operating mode based on speed  |
| DC Link Current Reference | Torque Controller      | Current Controller     | Setpoint for DC link current regulation   |
| Machine Voltage Magnitude | Machine-Side Observer  | Current Controller     | Feedforward compensation for load voltage |
| Force Commutation Request | Machine-Side Modulator | Grid-Side Modulator    | Coordinates commutation at low speeds     |

### 6.2 Grid-Side → Machine-Side

| Signal Name                    | Source                 | Destination            | Purpose                           |
| ------------------------------ | ---------------------- | ---------------------- | --------------------------------- |
| Operating Mode                 | Operation Mode Monitor | Machine-Side Modulator | Synchronizes modulation strategy  |
| Is Current Zero                | Operation Mode Monitor | Machine-Side Modulator | Interlocking for safe commutation |
| Force Commutation Acknowledged | Grid-Side Modulator    | Machine-Side Modulator | Confirms commutation assistance   |
| Pulse Releases                 | State Machine          | Machine-Side Modulator | Master enable for both sides      |

---

## 7. Timing and Execution Rates

| Task           | Period | Frequency | Signals Processed                      | Purpose                             |
| -------------- | ------ | --------- | -------------------------------------- | ----------------------------------- |
| **200µs Task** | 200 µs | 5 kHz     | Analog inputs, thyristor pulses        | High-frequency inner control loop   |
| **1ms Task**   | 1 ms   | 1 kHz     | Digital I/O, fiber-optic communication | Medium-frequency outer control loop |
| **10ms Task**  | 10 ms  | 100 Hz    | State machine                          | Low-frequency supervisory control   |

**Execution Priority:**
1. **Highest:** 200µs task (current control loop - must meet hard real-time deadline)
2. **Medium:** 1ms task (digital I/O and communication)
3. **Lowest:** 10ms task (state machine and monitoring)

---

## 8. Safety Interlocks and Fault Handling

### 8.1 Startup Interlocks
- Auxiliaries must be ready before breakers can close
- Breakers must be closed before start command is accepted
- DC link current must be zero before energizing

### 8.2 Operational Interlocks
- Fiber-optic link must be healthy for coordinated operation
- Current zero status must be confirmed before commutation transitions
- Operating mode must match speed range (automatic detection)

### 8.3 Shutdown Interlocks
- Stop command initiates controlled ramp-down
- Current must reach zero before breakers can open
- Breakers must open before auxiliaries are disabled

### 8.4 Emergency Shutdown
- Emergency shutdown request bypasses normal sequencing
- Firing pulses are immediately disabled
- System enters safe state (all outputs off)

---

## 9. Typical Operating Sequence

### 9.1 Startup
1. Operator enables auxiliaries (cooling, oil pumps) → **DI1 = High**
2. System confirms auxiliaries ready → Wait for stabilization
3. Operator closes grid breakers → **DI2 = High**
4. System confirms breakers closed and current is zero
5. Operator issues start command → **DI3 = High**
6. State machine transitions to OPERATION_ON → **DO1 = High** (System Running)
7. Grid-side enables pulse releases → Thyristor firing begins
8. Machine-side ramps speed reference → System accelerates

### 9.2 Normal Operation
- Grid-side regulates DC link current based on machine-side reference
- Operating mode automatically transitions based on machine speed:
  - **Low speed (<10% rated):** Forced commutation mode → **DO4 = High**
  - **High speed (>25% rated):** Natural commutation mode → **DO5 = High**
- Cross-coupled signals maintain coordination via fiber-optic link

### 9.3 Shutdown
1. Operator issues stop command → **DI4 = High**
2. Machine-side ramps down speed reference → System decelerates
3. DC link current reduces to zero → **DO2 = High** (Current Zero)
4. System confirms current extinguished → **DI6 = High**
5. State machine disables pulse releases → **DO1 = Low**
6. Operator opens breakers → **DI2 = Low**
7. Operator disables auxiliaries → **DI1 = Low**
8. System returns to IDLE state

---

## 10. Troubleshooting Guide

### Common Issues

**Problem:** Thyristor pulses not generating
- **Check:** DO1 (System Running) is High
- **Check:** Fiber link status (DO6) is healthy
- **Check:** Operating mode is appropriate for current speed

**Problem:** Current controller saturated (DO3 = High)
- **Cause:** Insufficient grid voltage for requested current
- **Cause:** Machine speed too low for natural commutation
- **Action:** Reduce current reference or increase machine voltage

**Problem:** Fiber link failure (DO6 = Low)
- **Cause:** Physical fiber damage or disconnection
- **Cause:** Communication protocol mismatch
- **Action:** Check fiber integrity and board synchronization

**Problem:** System stuck in forced commutation mode
- **Cause:** Machine speed below transition threshold
- **Cause:** Machine voltage magnitude too low
- **Action:** Verify machine-side observer speed estimate

---

## Revision History

| Date       | Version | Author       | Changes                                        |
| ---------- | ------- | ------------ | ---------------------------------------------- |
| 2025-10-20 | 1.0     | AI Assistant | Initial documentation of grid-side I/O mapping |
