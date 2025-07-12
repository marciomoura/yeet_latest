# Pulse Generator (Thyristor) - 6-Pulse Configuration

[← Back to Documentation Index](index.md)

This document describes the Pulse Generator (Thyristor) component for 6-pulse thyristor bridge configurations in Load-Commutated Inverter (LCI) systems. The pulse generator provides precise firing control for thyristor-based converters operating in both rectifier and inverter modes.

## Overview

The 6-pulse thyristor pulse generator creates gate signals for six thyristors arranged in a three-phase bridge configuration. Each thyristor is fired with a specific phase sequence and timing to achieve controlled power conversion with minimal harmonic distortion.

## Physical Configuration

### 6-Pulse Bridge Topology

```
6-Pulse Thyristor Bridge Configuration (IEEE Standard)

    Phase A      Phase B      Phase C
       |            |            |
   ┌───▼───┐    ┌───▼───┐    ┌───▼───┐
   │   T1  │    │   T3  │    │   T5  │  ← Upper leg thyristors
   │  ╱▷   │    │  ╱▷   │    │  ╱▷   │    (cathode → anode)
   └───┬───┘    └───┬───┘    └───┬───┘
       │            │            │
   ────┼────────────┼────────────┼──── DC+ Bus
       │            │            │
       │            │            │
   ────┼────────────┼────────────┼──── DC- Bus
       │            │            │
   ┌───▼───┐    ┌───▼───┐    ┌───▼───┐
   │   T4  │    │   T6  │    │   T2  │  ← Lower leg thyristors
   │  ╱▷   │    │  ╱▷   │    │  ╱▷   │    (cathode → anode)
   └───┬───┘    └───┬───┘    └───┬───┘
       │            │            │
       ▼            ▼            ▼
    Phase A      Phase B      Phase C

Firing Sequence: T1 → T2 → T3 → T4 → T5 → T6 → T1...
Natural Angles:  30° → 90° → 150° → 210° → 270° → 330°

Legend:
  ╱▷  = Thyristor symbol (gate controlled switch)
  ──  = DC bus connections
  │   = AC phase connections
  ▼   = AC output terminals
```

### Thyristor Numbering Convention
Following IEEE standards for 6-pulse bridges:
- **T1**: Phase A upper thyristor
- **T2**: Phase C lower thyristor  
- **T3**: Phase B upper thyristor
- **T4**: Phase A lower thyristor
- **T5**: Phase C upper thyristor
- **T6**: Phase B lower thyristor

## Input/Output Interface

### Inputs (From Control System)

| Input Signal          | Programming Name        | Symbol        | Type        | Range      | Description                                          |
| :-------------------- | :---------------------- | :------------ | :---------- | :--------- | :--------------------------------------------------- |
| **Firing Angle**      | `firing_angle`          | α             | `double`    | 0° to 180° | Control angle that determines output voltage/current |
| **Reference Voltage** | `reference_voltage_abc` | $V_{ref,abc}$ | `double[3]` | ±1.0 pu    | Three-phase reference voltages for synchronization   |
| **Enable Signal**     | `converter_enable`      | -             | `boolean`   | 0/1        | Master enable/disable for pulse generation           |
| **Pulse Block**       | `pulse_block`           | -             | `boolean`   | 0/1        | Emergency pulse blocking signal                      |

### Outputs (To Thyristor Gates)

| Output Signal    | Programming Name    | Symbol    | Type         | Description                                  |
| :--------------- | :------------------ | :-------- | :----------- | :------------------------------------------- |
| **Gate Pulses**  | `gate_pulses_t1_t6` | $g_{1-6}$ | `boolean[6]` | Individual gate signals for thyristors T1-T6 |
| **Pulse Status** | `pulse_status`      | -         | `uint8`      | Current firing status and diagnostics        |
| **Sync Status**  | `sync_status`       | -         | `boolean`    | Phase lock status indicator                  |

## Operating Principles

### 1. Phase-Locked Loop (PLL) Synchronization

The pulse generator must maintain precise synchronization with the AC voltage system to ensure proper thyristor commutation. The PLL functionality includes:

**Grid Voltage Tracking:**
- Continuously monitors three-phase grid voltages to extract phase and frequency information
- Uses Clarke and Park transformations to convert ABC voltages to rotating dq reference frame
- Maintains phase lock by driving the q-component of grid voltage to zero

**Frequency Estimation:**
- Tracks grid frequency variations using a PI controller
- Adapts to frequency deviations from nominal (50/60 Hz) values
- Provides robust operation during grid disturbances and frequency fluctuations

**Phase Angle Generation:**
- Generates continuous grid transformation angle (θ_S) for synchronization
- Handles angle wrapping and maintains continuity across 0°/360° transitions
- Provides reference angle for firing sequence calculations

### 2. Firing Angle Control

The pulse generator calculates precise firing times for each thyristor based on the control angle and natural commutation sequence:

**Natural Firing Sequence:**
- Follows IEEE standard 6-pulse firing order: T1→T2→T3→T4→T5→T6→T1...
- Each thyristor fires at 60° intervals relative to its commutating voltage
- Base firing angles are: T1(30°), T2(270°), T3(90°), T4(210°), T5(150°), T6(330°)

**Control Angle Application:**
- Delays each thyristor firing by the control angle (α) from its natural firing point
- For rectifier mode: 0° ≤ α ≤ 90° (reduces DC output voltage)
- For inverter mode: 90° ≤ α ≤ 180° (expressed as advance angle β = 180° - α)

**Timing Precision:**
- Maintains firing angle accuracy within ±0.5° under all operating conditions
- Accounts for angle wrapping when control angles exceed 360°
- Provides deterministic timing for real-time control applications

### 3. Pulse Generation Logic

The system generates gate pulses with precise timing and safety interlocks:

**Pulse Width Control:**
- Generates gate pulses with configurable width (typically 5° to 30°)
- Ensures minimum pulse width for reliable thyristor turn-on
- Prevents excessive pulse width that could cause device stress

**Safety Interlocks:**
- Immediately blocks all pulses when converter disable signal is active
- Responds to emergency pulse blocking commands within one control cycle
- Maintains safe state during fault conditions or system shutdown

**Pulse Sequencing:**
- Ensures only one thyristor per leg conducts at any time
- Prevents shoot-through conditions in the bridge configuration
- Coordinates timing between upper and lower thyristors

## Control Modes

### 1. Rectifier Mode (Grid-Side Converter)
- **Firing Angle Range**: 0° ≤ α ≤ 90°
- **Purpose**: Convert AC grid power to DC
- **Control Variable**: `grid_firing_angle`
- **DC Voltage**: $V_d = V_{LL} \cdot \frac{3\sqrt{3}}{2\pi} \cos(\alpha)$

### 2. Inverter Mode (Machine-Side Converter)
- **Firing Angle Range**: 90° ≤ α ≤ 180°
- **Advance Angle**: β = 180° - α
- **Purpose**: Convert DC power to variable-frequency AC
- **Control Variable**: `machine_advance_angle`
- **Minimum β**: Typically 15° to 20° for safe commutation

## Protection and Safety Features

### 1. Minimum Pulse Width Enforcement

The pulse generator must enforce safe operating limits to ensure reliable thyristor operation:

**Pulse Width Constraints:**
- Maintains minimum pulse width (typically 5°) to guarantee thyristor turn-on
- Limits maximum pulse width (typically 30°) to prevent device thermal stress
- Automatically adjusts pulse width when control system requests out-of-range values
- Provides diagnostic indication when pulse width limiting is active

### 2. Commutation Failure Protection

Critical safety feature that prevents thyristor damage during inverter operation:

**Extinction Angle Monitoring:**
- Continuously calculates extinction angle (γ) based on current operating conditions
- Compares extinction angle against minimum safe threshold (typically 15° to 20°)
- Triggers protective action when extinction angle approaches unsafe levels

**Protective Actions:**
- Automatically increases advance angle (β) to restore safe extinction angle
- Reduces DC link current reference to decrease commutation stress
- Activates alarm signals to notify control system of commutation risk
- Blocks inverter pulses if extinction angle cannot be maintained

### 3. Overcurrent Protection

Fast-acting protection to prevent equipment damage during fault conditions:

**Current Monitoring:**
- Continuously monitors DC link current against configurable limits
- Responds to overcurrent conditions within one control cycle (< 50 μs)
- Distinguishes between normal load transients and fault conditions

**Protection Response:**
- Immediately blocks all gate pulses when current exceeds trip level
- Maintains pulse blocking until current returns to safe levels
- Provides time-delayed restart capability after fault clearance
- Logs fault events for diagnostic and maintenance purposes

### 4. Phase Loss Detection

Ensures safe operation during grid disturbances:

**Grid Monitoring:**
- Detects loss of one or more grid phases through voltage amplitude monitoring
- Identifies grid unbalance conditions that could affect commutation
- Monitors grid frequency deviations beyond acceptable limits

**Response Strategy:**
- Reduces converter output during minor grid disturbances
- Blocks operation during severe grid faults or phase loss
- Provides smooth restart capability when grid conditions normalize

## Integration with Simulink Plant Model

### 1. Interface Variables (Following Dictionary Conventions)

The pulse generator interfaces with the control system and plant model using standardized signal names from the project dictionary:

**Input Signals from Control System:**
- **Control Angles**: `grid_firing_angle` (α_R for rectifier), `machine_advance_angle` (β for inverter)
- **Grid Synchronization**: `grid_voltage_abc_phase` (V_S,abc), `grid_frequency` (f_S)
- **Enable/Disable**: `converter_enable`, `pulse_block` for safety interlocks
- **Protection Inputs**: `dc_link_current` (I_d), `machine_extinction_angle` (γ)

**Output Signals to Plant Model:**
- **Gate Signals**: `grid_gate_pulses_t1_t6[6]` for rectifier, `machine_gate_pulses_t1_t6[6]` for inverter
- **Status Information**: `pulse_status` for diagnostics, `sync_status` for PLL lock indication
- **Synchronization Angles**: `grid_transform_angle` (θ_S), `machine_transform_angle` (θ_M)

### 2. Functional Integration Requirements

**Real-Time Operation:**
- Must execute within 10 μs time step constraint for real-time simulation
- Provides deterministic response times suitable for hardware-in-the-loop testing
- Maintains numerical stability across wide operating ranges

**Simulink Compatibility:**
- Interfaces directly with Simulink Power System thyristor blocks
- Compatible with Universal Bridge configurations used in LCI models
- Supports both continuous and discrete-time simulation modes

**Multi-Instance Support:**
- Supports multiple independent pulse generators for different plant models
- Enables parallel simulation of various LCI configurations
- Maintains isolation between different converter instances

### 3. Configuration and Parameterization

**Tunable Parameters:**
- PLL controller gains (Kp_pll, Ki_pll) for different grid conditions
- Pulse width settings for various thyristor types
- Protection thresholds for overcurrent and extinction angle limits
- Operating frequency ranges for grid and machine-side converters

**Calibration Requirements:**
- Phase angle offsets to account for measurement delays and hardware timing
- Gain adjustments for different voltage scaling in the plant model
- Temperature compensation for thyristor characteristics
- Timing calibration for specific simulation time steps

## Performance Specifications

### Timing Requirements
- **Response Time**: < 50 μs (control angle to gate pulse)
- **Pulse Width Accuracy**: ±0.5°
- **Phase Lock Accuracy**: ±0.1°
- **Jitter**: < 1 μs

### Operating Limits
- **Firing Angle Range**: 0° to 180°
- **Frequency Range**: 0.1 Hz to 100 Hz
- **Temperature Range**: -40°C to +85°C
- **Supply Voltage**: 12V ±10%

## Validation and Testing

### 1. Functional Verification

**Phase Sequence Verification:**
- Confirms correct thyristor firing order (T1→T2→T3→T4→T5→T6) under all conditions
- Validates 60° spacing between consecutive thyristor pulses
- Verifies proper pulse timing across full firing angle range (0° to 180°)

**Firing Angle Accuracy:**
- Tests firing angle precision across the complete operating range
- Validates control angle response time and linearity
- Confirms proper angle wrapping behavior at 0°/360° boundaries

**Pulse Width Consistency:**
- Verifies pulse width remains constant across different firing angles
- Tests pulse width limiting functions for safety compliance
- Validates pulse width accuracy under varying grid frequency conditions

**Protection Response Verification:**
- Tests overcurrent protection response time (target: < 50 μs)
- Validates extinction angle monitoring and protective action
- Confirms pulse blocking effectiveness during fault conditions

### 2. Integration Testing

**Simulink Plant Model Compatibility:**
- Verifies seamless interface with generated C++ plant models
- Tests signal integrity between pulse generator and thyristor models
- Validates timing synchronization between control and plant models

**Real-Time Performance Validation:**
- Confirms execution time meets 10 μs time step requirement
- Tests deterministic behavior under maximum computational load
- Validates numerical stability during extended operation periods

**Multi-Instance Operation:**
- Tests parallel operation of multiple pulse generators
- Verifies isolation between different converter instances
- Confirms resource utilization remains within system limits

### 3. System-Level Testing

**Harmonic Analysis:**
- Analyzes output current and voltage harmonic content
- Verifies compliance with IEEE standards for power quality
- Tests harmonic performance across different operating points

**Grid Disturbance Response:**
- Tests operation during voltage sags, swells, and frequency variations
- Validates phase loss detection and response
- Confirms proper restart behavior after grid fault clearance

**Fault Injection Testing:**
- Simulates various fault scenarios (overcurrent, commutation failure, etc.)
- Verifies protection system response and recovery behavior
- Tests diagnostic capability and fault reporting accuracy

### 4. Performance Benchmarking

**Timing Specifications:**
- **Response Time**: Control angle change to gate pulse output < 50 μs
- **Pulse Width Accuracy**: ±0.5° across full operating range
- **Phase Lock Accuracy**: ±0.1° during steady-state operation
- **Pulse Jitter**: < 1 μs under all operating conditions

**Operational Verification:**
- **Firing Angle Range**: Full 0° to 180° coverage with 0.1° resolution
- **Frequency Range**: 0.1 Hz to 100 Hz operation capability
- **Dynamic Response**: Step response settling time < 100 μs
- **Stability Margins**: Stable operation with ±20% parameter variations

## References

1. IEEE Std 519-2014: Harmonic Control in Electric Power Systems
2. "Power Electronics: Converters, Applications, and Design" - Mohan, Undeland, Robbins
3. Simulink Power System Blockset Documentation
4. Project SFC Dictionary and Coding Standards

---

[← Back to Documentation Index](index.md)
