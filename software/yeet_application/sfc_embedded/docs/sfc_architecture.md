# Static Frequency Converter (SFC) Embedded Architecture

## Overview

The Static Frequency Converter (SFC) embedded control system is split into two independent microcontroller applications that work cooperatively to control a doubly-fed induction machine (DFIM) drive system. The system enables bidirectional power flow between a fixed-frequency grid and a variable-frequency machine.

**Key Specifications:**
- **Grid Side:** 2090V RMS line-to-line, 50 Hz
- **Machine Side:** 1980V RMS line-to-line, variable frequency (0-100 Hz)
- **DC Link:** 1044A rated current
- **Rated Power:** 2950 kW
- **Machine:** 2-pole pair induction machine
- **Control Platform:** Dual STM32F407G microcontrollers (ARM Cortex-M4)

---

## 1. System Architecture

### 1.1 Dual-Controller Configuration

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         SFC Control System                               │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                ┌───────────────────┴────────────────────┐
                │                                        │
    ┌───────────▼──────────┐              ┌─────────────▼────────────┐
    │  Grid-Side MCU       │◄────────────►│  Machine-Side MCU        │
    │  (STM32F407G)        │ Communication│  (STM32F407G)            │
    │                      │   Interface  │                          │
    │  • State Machine     │              │  • Speed Control         │
    │  • Current Control   │              │  • Current Ref Gen       │
    │  • Grid Measurements │              │  • Machine Measurements  │
    │  • Grid Modulator    │              │  • Machine Modulator     │
    └──────────┬───────────┘              └────────────┬─────────────┘
               │                                       │
    ┌──────────▼──────────┐              ┌────────────▼─────────────┐
    │  Grid-Side Thyristor│              │ Machine-Side Thyristor   │
    │  Bridge (6-pulse)   │              │ Bridge (6-pulse)         │
    └──────────┬──────────┘              └────────────┬─────────────┘
               │                                      │
               │          ┌──────────────┐            │
               └──────────►   DC Link    ◄────────────┘
                          │  (Inductor)  │
                          └──────────────┘
```

### 1.2 Control Hierarchy

Both controllers operate with a **multi-rate control architecture**:

| Task | Rate | Period | Purpose |
|------|------|--------|---------|
| **Task 1** | 5 kHz | 200 µs | High-frequency: Measurements, observers, modulation, pulse generation |
| **Task 2** | 1 kHz | 1 ms | Medium-frequency: Controllers (current/machine) |
| **Task 3** | 100 Hz | 10 ms | Low-frequency: Speed control, operation mode monitoring |

---

## 2. Grid-Side Controller

### 2.1 Purpose

The **grid-side controller** is the **master** in the system hierarchy. It is responsible for:
- **System startup and shutdown sequencing** via state machine
- **DC link current regulation** to maintain stable power transfer
- **Grid synchronization** and voltage measurement
- **System-level coordination** and protection

### 2.2 Features Implemented

#### 2.2.1 State Machine (`sfc_state_machine_runner`)
- **Purpose:** Orchestrates system startup, operation, and shutdown
- **States:** 
  - `IDLE` - System off, all outputs disabled
  - `PRECHARGE` - Pre-charging DC link capacitors
  - `READY` - Ready to operate, thyristors armed
  - `RUNNING` - Active power conversion
  - `FAULT` - Fault condition detected
  - `SHUTDOWN` - Controlled shutdown sequence
- **Outputs:**
  - `pulse_releases` - Enable/disable thyristor firing
  - `freeze_integrator` - Anti-windup for controllers
  - Status flags for system coordination

#### 2.2.2 Current Controller (`sfc_current_controller_runner`)
- **Purpose:** Regulate DC link current to maintain power balance
- **Type:** PI controller with anti-windup
- **Inputs:**
  - `dc_link_current_reference` - From machine-side speed controller
  - `dc_link_current_measurement` - From DC link current sensor
  - `machine_side_voltage_magnitude` - For feedforward compensation
  - `freeze_integrator` - Anti-windup signal from state machine
  - `modulator_freeze_integrator` - Anti-windup from modulator
- **Output:**
  - `grid_side_voltage_reference` - Commanded grid-side voltage

#### 2.2.3 Grid-Side Measurements (`sfc_grid_side_measurements_runner`)
- **Purpose:** Acquire and process grid voltage and DC link current
- **Sensors:**
  - 2x voltage sensors (V_ab, V_bc) - Grid line voltages
  - 2x current sensors (I_a, I_c) - Grid phase currents
  - Synchronized to thyristor commutation events
- **Processing:**
  - ABC → αβ transformation
  - RMS voltage calculation
  - Fundamental component extraction
  - DC link current measurement and filtering

#### 2.2.4 Grid-Side Observer (`sfc_grid_side_observer_runner`)
- **Purpose:** Estimate grid voltage angle and synchronize to grid
- **Type:** Phase-locked loop (PLL) based observer
- **Outputs:**
  - Estimated grid voltage angle
  - Estimated grid frequency
  - Synchronization status

#### 2.2.5 Grid-Side Modulator (`sfc_grid_side_modulator_runner`)
- **Purpose:** Generate thyristor firing angles to achieve desired voltage
- **Type:** Phase-controlled rectifier/inverter modulation
- **Operating Modes:**
  - `RECTIFIER_MODE` - Power flow from grid to machine
  - `INVERTER_MODE` - Power flow from machine to grid
- **Inputs:**
  - `voltage_reference` - From current controller
  - `grid_angle` - From observer
  - `enable` - From state machine
  - `operating_mode` - From operation mode monitor
  - `request_force_commutation` - From machine-side
- **Outputs:**
  - `firing_angles` - 6 thyristor firing angles
  - `force_commutation_acknowledged` - Acknowledgment to machine-side

#### 2.2.6 Grid-Side Pulse Generator (`sfc_grid_side_pulse_generator_runner`)
- **Purpose:** Convert firing angles to hardware PWM signals
- **Output:** 6 digital pulse signals for gate drivers

#### 2.2.7 DC Link Measurements (`sfc_dc_link_measurements_runner`)
- **Purpose:** Measure and filter DC link current
- **Sensor:** DC link current transducer (Hall effect or shunt)
- **Processing:** Filtering, scaling, per-unit conversion

#### 2.2.8 Operation Mode Monitor (`sfc_operation_mode_monitor_runner`)
- **Purpose:** Determine power flow direction and mode transitions
- **Inputs:**
  - DC link current (magnitude and direction)
  - Machine speed
- **Output:** 
  - `operating_mode` - RECTIFIER_MODE or INVERTER_MODE

### 2.3 Grid-Side Input Signals

| Signal | Source | Type | Description |
|--------|--------|------|-------------|
| `auxiliaries_are_on` | Supervisory | `bool` | Auxiliary systems ready |
| `breakers_are_on` | Supervisory | `bool` | Main breakers closed |
| `start_command` | Supervisory | `bool` | Command to start system |
| `stop_command` | Supervisory | `bool` | Command to stop system |
| `shutdown_request` | Supervisory | `bool` | Emergency shutdown |
| `is_current_extinguished` | Hardware | `bool` | Zero-current detection |
| `v_ab_counts` | ADC | `adc_counts` | Grid voltage AB measurement |
| `v_bc_counts` | ADC | `adc_counts` | Grid voltage BC measurement |
| `i_a_counts` | ADC | `adc_counts` | Grid phase A current |
| `i_c_counts` | ADC | `adc_counts` | Grid phase C current |
| `grid_side_frequency` | Timer | `frequency_t` | Grid frequency reference |
| `grid_side_angle` | Timer | `angle_wrapped` | Grid angle reference |
| `grid_pulses` | Hardware | `six_pulse_generator::pulses` | Thyristor commutation events |
| `dc_link_current_reference` | Machine-side | `current_pu_t` | Commanded DC current |
| `machine_side_voltage_magnitude` | Machine-side | `voltage_pu_t` | Machine voltage for feedforward |
| `request_force_commutation` | Machine-side | `bool` | Force commutation request |

### 2.4 Grid-Side Output Signals

| Signal | Destination | Type | Description |
|--------|-------------|------|-------------|
| `pulse_releases` | Machine-side | `bool` | Enable thyristor firing |
| `operating_mode` | Machine-side | `sfc_operating_mode_t` | Power flow direction |
| `dc_link_current_pu` | Machine-side | `current_pu_t` | Measured DC link current |
| `force_commutation_acknowledged` | Machine-side | `bool` | Commutation ack |
| `freeze_integrator` | Machine-side | `bool` | Anti-windup signal |
| `current_controller_is_saturated` | Machine-side | `bool` | Saturation status |
| `modulator_freeze_integrator` | Machine-side | `bool` | Modulator freeze signal |
| `gate_pulses[6]` | Gate drivers | `bool[6]` | Thyristor firing signals |

---

## 3. Machine-Side Controller

### 3.1 Purpose

The **machine-side controller** is the **slave** in the system hierarchy. It is responsible for:
- **Speed regulation** of the induction machine
- **Machine-side voltage/current control**
- **Generating DC link current reference** for grid-side
- **Machine voltage measurement and estimation**

### 3.2 Features Implemented

#### 3.2.1 Speed Controller (`sfc_speed_controller_runner`)
- **Purpose:** Regulate machine speed to commanded reference
- **Type:** PI controller with anti-windup
- **Inputs:**
  - `speed_reference_pu` - Commanded speed (supervisory)
  - `measured_speed_pu` - From machine observer
  - `enable_speed_control` - From grid-side state machine
  - `freeze_integrator` - Anti-windup from grid-side
  - `current_controller_is_saturated` - Anti-windup from machine controller
  - `modulator_freeze_integrator` - Anti-windup from modulator
- **Output:**
  - `dc_link_current_reference` - Commanded DC link current → sent to grid-side

#### 3.2.2 Machine-Side Controller (`sfc_machine_side_controller_runner`)
- **Purpose:** Control machine-side converter voltage/current
- **Type:** Current/voltage controller
- **Inputs:**
  - `current_reference` - From speed controller
  - `dc_link_current` - From grid-side (measured)
  - Machine voltage feedback
- **Outputs:**
  - `machine_voltage_reference` - Commanded machine voltage
  - `current_controller_is_saturated` - Saturation flag

#### 3.2.3 Machine-Side Measurements (`sfc_machine_side_measurements_runner`)
- **Purpose:** Acquire and process machine voltages
- **Sensors:**
  - 2x voltage sensors (V_ab, V_bc) - Machine line voltages
  - **Note:** No current sensors on machine side (sensorless control)
- **Processing:**
  - ABC → αβ transformation
  - RMS voltage calculation
  - Synchronization to machine commutation

#### 3.2.4 Machine-Side Observer (`sfc_machine_side_observer_runner`)
- **Purpose:** Estimate machine speed and rotor angle
- **Type:** Sensorless speed observer (model-based or PLL)
- **Inputs:**
  - Machine voltages
  - Machine electrical angle reference
- **Outputs:**
  - `measured_speed_pu` - Estimated machine speed
  - Estimated rotor position

#### 3.2.5 Machine-Side Modulator (`sfc_machine_side_modulator_runner`)
- **Purpose:** Generate thyristor firing angles for machine-side bridge
- **Type:** Phase-controlled inverter/rectifier modulation
- **Inputs:**
  - `voltage_reference` - From machine controller
  - `machine_angle` - From observer
  - `enable` - From grid-side (pulse_releases)
  - `operating_mode` - From grid-side
  - `force_commutation_acknowledged` - From grid-side
- **Outputs:**
  - `firing_angles` - 6 thyristor firing angles
  - `request_force_commutation` - Request to grid-side
  - `modulator_freeze_integrator` - Anti-windup signal

#### 3.2.6 Machine-Side Pulse Generator (`sfc_machine_side_pulse_generator_runner`)
- **Purpose:** Convert firing angles to hardware PWM signals
- **Output:** 6 digital pulse signals for gate drivers

### 3.3 Machine-Side Input Signals

| Signal | Source | Type | Description |
|--------|--------|------|-------------|
| `speed_reference_pu` | Supervisory | `speed_pu_t` | Commanded machine speed |
| `pulse_releases` | Grid-side | `bool` | Enable from state machine |
| `operating_mode` | Grid-side | `sfc_operating_mode_t` | Rectifier/Inverter mode |
| `dc_link_current_pu` | Grid-side | `current_pu_t` | Measured DC current |
| `force_commutation_acknowledged` | Grid-side | `bool` | Commutation ack from grid |
| `freeze_integrator` | Grid-side | `bool` | Anti-windup for speed controller |
| `current_controller_is_saturated` | Grid-side | `bool` | Saturation from grid current controller |
| `modulator_freeze_integrator` | Grid-side | `bool` | Anti-windup from grid modulator |
| `enable_speed_control` | Grid-side | `bool` | Enable speed control loop |
| `v_ab_counts` | ADC | `adc_counts` | Machine voltage AB measurement |
| `v_bc_counts` | ADC | `adc_counts` | Machine voltage BC measurement |
| `machine_side_frequency` | Timer | `frequency_t` | Machine frequency reference |
| `machine_side_angle` | Timer | `angle_wrapped` | Machine angle reference |
| `machine_pulses` | Hardware | `six_pulse_generator::pulses` | Thyristor commutation events |

### 3.4 Machine-Side Output Signals

| Signal | Destination | Type | Description |
|--------|-------------|------|-------------|
| `dc_link_current_reference` | Grid-side | `current_pu_t` | Commanded DC current |
| `machine_side_voltage_magnitude` | Grid-side | `voltage_pu_t` | Machine voltage magnitude |
| `request_force_commutation` | Grid-side | `bool` | Force commutation request |
| `measured_speed_pu` | Grid-side | `speed_pu_t` | Estimated machine speed |
| `current_controller_is_saturated` | Internal | `bool` | Saturation flag |
| `gate_pulses[6]` | Gate drivers | `bool[6]` | Thyristor firing signals |

---

## 4. Key Differences: Grid-Side vs Machine-Side

| Aspect | Grid-Side | Machine-Side |
|--------|-----------|--------------|
| **Role** | Master (coordinator) | Slave (follows grid-side) |
| **Primary Control Objective** | DC link current regulation | Machine speed regulation |
| **State Machine** | ✅ Yes - Controls system sequencing | ❌ No - Receives enable signals |
| **Sensors** | 2x Voltage + 2x Current (4 ADCs) | 2x Voltage only (2 ADCs) |
| **Current Measurement** | Direct DC link current sensor | Receives from grid-side |
| **Speed Measurement** | Receives from machine-side | Sensorless observer |
| **External Commands** | 6 state machine inputs | 1 speed reference |
| **Inter-Controller Inputs** | 4 signals from machine-side | 8 signals from grid-side |
| **Control Hierarchy** | Outer loop (power flow) | Inner loop (speed/torque) |
| **Operating Mode Determination** | ✅ Yes - Monitors and decides | ❌ No - Receives from grid-side |
| **Pulse Release Authority** | ✅ Yes - Controls both sides | ❌ No - Receives from grid-side |
| **Anti-Windup Coordination** | Sends freeze signals | Receives freeze signals |
| **Force Commutation** | Acknowledges requests | Initiates requests |

---

## 5. Signal Flow Architecture

### 5.1 Supervisory Layer → Controllers

```
┌──────────────────────────┐
│  Supervisory Controller  │
│  (SCADA / HMI)           │
└────────┬─────────────────┘
         │
         ├─────────► Grid-Side: auxiliaries_are_on
         ├─────────► Grid-Side: breakers_are_on
         ├─────────► Grid-Side: start_command
         ├─────────► Grid-Side: stop_command
         ├─────────► Grid-Side: shutdown_request
         │
         └─────────► Machine-Side: speed_reference_pu
```

### 5.2 Grid-Side → Machine-Side Communication

```
┌─────────────────┐                        ┌──────────────────┐
│  Grid-Side MCU  │───────────────────────►│ Machine-Side MCU │
└─────────────────┘                        └──────────────────┘

Signals sent from Grid to Machine:
├── pulse_releases ──────────────────────► Enables thyristor firing
├── operating_mode ──────────────────────► RECTIFIER_MODE / INVERTER_MODE
├── dc_link_current_pu ──────────────────► Measured DC link current
├── force_commutation_acknowledged ──────► Ack for commutation request
├── freeze_integrator ───────────────────► Anti-windup for speed controller
├── current_controller_is_saturated ─────► Anti-windup signal
├── modulator_freeze_integrator ─────────► Anti-windup from modulator
└── enable_speed_control ────────────────► Enable speed control loop
```

### 5.3 Machine-Side → Grid-Side Communication

```
┌──────────────────┐                       ┌─────────────────┐
│ Machine-Side MCU │──────────────────────►│  Grid-Side MCU  │
└──────────────────┘                       └─────────────────┘

Signals sent from Machine to Grid:
├── dc_link_current_reference ───────────► Commanded DC current (from speed controller)
├── machine_side_voltage_magnitude ──────► Machine voltage for feedforward
├── request_force_commutation ───────────► Request forced commutation
└── measured_speed_pu ────────────────────► Estimated machine speed (for mode monitor)
```

### 5.4 Complete Signal Flow Diagram

```
                    SUPERVISORY CONTROLLER
                            │
            ┌───────────────┴───────────────┐
            │                               │
     ┌──────▼──────┐                 ┌──────▼──────┐
     │  GRID-SIDE  │◄───────────────►│ MACHINE-SIDE│
     │  CONTROLLER │  8 signals →   │  CONTROLLER │
     │             │  ← 4 signals   │             │
     └──────┬──────┘                 └──────┬──────┘
            │                               │
     ┌──────▼──────┐                 ┌──────▼──────┐
     │Grid Sensors │                 │Mach Sensors │
     │• V_ab, V_bc │                 │• V_ab, V_bc │
     │• I_a, I_c   │                 │(no current) │
     │• DC current │                 │             │
     └──────┬──────┘                 └──────┬──────┘
            │                               │
     ┌──────▼──────┐                 ┌──────▼──────┐
     │  Grid-Side  │                 │Machine-Side │
     │  Thyristor  │                 │  Thyristor  │
     │  Bridge     │                 │  Bridge     │
     └──────┬──────┘                 └──────┬──────┘
            │                               │
            │        ┌───────────┐          │
            └───────►│  DC LINK  │◄─────────┘
                     └───────────┘
```

---

## 6. Control Loop Timing

### 6.1 Task 1 (200 µs / 5 kHz) - Fastest Control Loop

**Grid-Side:**
1. ADC sampling triggered by timer
2. Grid voltage and current measurements
3. DC link current measurement
4. Grid-side observer (PLL)
5. Grid-side modulator
6. Pulse generator
7. Update gate driver signals

**Machine-Side:**
1. ADC sampling triggered by timer
2. Machine voltage measurements
3. Machine-side observer
4. Machine-side modulator
5. Pulse generator
6. Update gate driver signals

### 6.2 Task 2 (1 ms / 1 kHz) - Medium Control Loop

**Grid-Side:**
1. Current controller execution
2. Operation mode monitor
3. Communication with machine-side

**Machine-Side:**
1. Machine-side controller execution
2. Communication with grid-side

### 6.3 Task 3 (10 ms / 100 Hz) - Slowest Control Loop

**Grid-Side:**
1. State machine execution
2. System diagnostics
3. Status reporting

**Machine-Side:**
1. Speed controller execution
2. Generate DC current reference
3. Status reporting

---

## 7. Embedded Implementation Details

### 7.1 Build Configuration

The project uses **CMake presets** to configure separate builds:

```bash
# Grid-Side Controller
cmake --preset=embedded-grid
cmake --build --preset=embedded-grid-release

# Machine-Side Controller
cmake --preset=embedded-machine
cmake --build --preset=embedded-machine-release
```

**Build Outputs:**
- Grid-Side: `build-embedded-grid/bin/Release/yeet_embedded_app_grid.elf`
- Machine-Side: `build-embedded-machine/bin/Release/yeet_embedded_app_machine.elf`

### 7.2 Task Structure

Both controllers implement three interrupt-driven tasks:

```cpp
extern "C" {
    void yeet_embedded_init(void);  // Called once at startup
    void yeet_task_1(void);         // 5 kHz - High priority
    void yeet_task_2(void);         // 1 kHz - Medium priority
    void yeet_task_3(void);         // 100 Hz - Low priority
}
```

### 7.3 Signal Connections

All control signals use the **yeet signal framework** which provides:
- Type-safe signal connections
- Compile-time validation
- Runtime assertion on unconnected signals (prevents crashes)
- Zero-overhead in release builds

```cpp
// Example: Connect grid-side output to machine-side input
yeet::signals_connector::connect_output_to_input(
    grid_side_pulse_releases_output,
    machine_side_modulator.get_inputs().enable
);
```

### 7.4 Mock Signals for Standalone Testing

In the current embedded-only build (without hardware-in-the-loop simulation), **mock signals** are used to simulate:
- **Grid-side:** Machine-side controller outputs
- **Machine-side:** Grid-side controller outputs and supervisory commands

This allows each controller to run standalone for testing.

---

## 8. Inter-Controller Communication (Future)

### 8.1 Current Implementation
- Mock signal outputs simulate the other controller
- Standalone operation for development and testing

### 8.2 Production Implementation (To Be Implemented)
Communication between grid-side and machine-side will use one of:

1. **SPI** (Serial Peripheral Interface)
   - High-speed synchronous communication
   - Suitable for 1-10 Mbps data rates
   - Low latency (~10-50 µs)

2. **CAN Bus** (Controller Area Network)
   - Robust in noisy industrial environments
   - Built-in error checking
   - Up to 1 Mbps

3. **Shared Memory via Dual-Port RAM**
   - Fastest option if hardware supports
   - Zero-copy data transfer
   - Sub-microsecond latency

**Required Data Transfer:**
- Grid → Machine: 8 signals (~20 bytes) @ 1 kHz minimum
- Machine → Grid: 4 signals (~12 bytes) @ 1 kHz minimum

---

## 9. Safety and Protection Features

### 9.1 Anti-Windup Coordination
- Speed controller receives saturation status from current controller
- Speed controller receives freeze signals from state machine and modulator
- Prevents integrator windup during transients and faults

### 9.2 State Machine Interlocks (Grid-Side)
- Ensures safe startup sequence
- Monitors breaker status before enabling pulses
- Implements controlled shutdown on faults

### 9.3 Force Commutation Coordination
- Machine-side can request force commutation
- Grid-side acknowledges and coordinates
- Prevents commutation failures in low-voltage conditions

### 9.4 Pulse Release Authority
- Only grid-side state machine can enable thyristor firing
- Machine-side receives enable signal
- Centralized fault handling

---

## 10. Per-Unit System

Both controllers use the **same per-unit base values** for consistency:

```cpp
yeet::application::sfc::sfc_per_unit_system(
    voltage_grid_base:    2090.0 V RMS,
    frequency_grid_base:  50.0 Hz,
    current_dc_base:      1044.0 A,
    power_base:           2950.0 kW,
    voltage_machine_base: 1980.0 V RMS,
    frequency_machine_base: 50.0 Hz,
    pole_pairs:           2
);
```

This ensures all internal calculations are normalized and consistent between controllers.

---

## 11. Development and Testing Workflow

### 11.1 Build Both Controllers
```bash
# Configure and build grid-side
cmake --preset=embedded-grid
cmake --build --preset=embedded-grid-debug

# Configure and build machine-side
cmake --preset=embedded-machine
cmake --build --preset=embedded-machine-debug
```

### 11.2 Flash to Hardware
```bash
# Use VS Code tasks or command line
# Grid-Side
./STM32_Programmer_CLI.exe -c port=SWD -w yeet_embedded_app_grid.elf -v -rst

# Machine-Side (different board or same board after reprogramming)
./STM32_Programmer_CLI.exe -c port=SWD -w yeet_embedded_app_machine.elf -v -rst
```

### 11.3 Debugging
- Each controller can be debugged independently
- Mock signals allow standalone operation
- Use ST-Link GDB server for on-chip debugging

---

## 12. Future Enhancements

### 12.1 Communication Interface
- Implement actual SPI/CAN communication between controllers
- Remove mock signals
- Add communication health monitoring

### 12.2 Hardware Integration
- Connect to actual ADC inputs (currently mocked)
- Implement gate driver interfaces
- Add protection circuits monitoring

### 12.3 Advanced Features
- Fault diagnostics and logging
- Parameter tuning interface via CAN
- Real-time performance monitoring
- Adaptive control algorithms

---

## 13. References

- **Grid-Side Implementation:** `software/yeet_application/sfc_embedded/src/grid_side_tasks.cpp`
- **Machine-Side Implementation:** `software/yeet_application/sfc_embedded/src/machine_side_tasks.cpp`
- **Grid-Side Control Class:** `software/yeet_application/sfc_integration/include/sfc_integration/sfc_grid_side_control.hpp`
- **Machine-Side Control Class:** `software/yeet_application/sfc_integration/include/sfc_integration/sfc_machine_side_control.hpp`
- **Build Configuration:** `CMakePresets.json`
- **Task Configuration:** `.vscode/tasks.json`

---

## Glossary

- **SFC:** Static Frequency Converter
- **DFIM:** Doubly-Fed Induction Machine
- **PLL:** Phase-Locked Loop
- **RMS:** Root Mean Square
- **ADC:** Analog-to-Digital Converter
- **PWM:** Pulse Width Modulation
- **Per-Unit (pu):** Normalized value relative to base quantity
- **Thyristor:** Semiconductor switching device (SCR)
- **Commutation:** Transfer of current from one thyristor to another
- **Anti-Windup:** Technique to prevent integrator saturation

---

**Document Version:** 1.0  
**Last Updated:** October 7, 2025  
**Authors:** Yeet Embedded Control Team  
**Status:** Implementation Complete ✅
