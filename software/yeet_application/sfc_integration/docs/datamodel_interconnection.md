# SFC Datamodel Interconnection Documentation

## Overview

This document describes the complete signal flow and interconnection architecture for the Static Frequency Converter (SFC) / Load-Commutated Inverter (LCI) control system. The system implements a hierarchical control structure with multiple interconnected components that work together to control power flow from a grid-side AC source to a variable-speed AC machine through a DC link.

## System Architecture

### Control Hierarchy

The SFC control system follows a cascaded control structure:

1. **Outer Loop**: Speed Control
2. **Inner Loop**: Current Control
3. **Modulation Layer**: Firing Pulse Generation
4. **Physical Layer**: Plant Model

### Component Categories

1. **Control Components**
   - Speed Controller
   - Current Controller
   - Machine-Side Firing Angle Lookup Table

2. **Measurement Components**
   - Grid-Side Measurements & Filtering
   - Machine-Side Measurements & Filtering
   - DC Link Measurements & Filtering

3. **Observer Components**
   - Grid-Side Observer (PLL-based)
   - Machine-Side Observer (Adaptive)

4. **Modulation Components**
   - Grid-Side Modulator
   - Machine-Side Modulator

5. **Plant Model**
   - SFC 6-pulse to 6-pulse converter model
   - Mechanical load model

## Detailed Signal Flow

### 1. Speed Control Loop

```
Speed Reference → Speed Controller → Current Reference → Current Controller
        ↑                                      |
        |                                      ↓
Machine Speed ← Machine Observer ← Machine Measurements
```

**Signal Path**:
1. External speed reference (`speed_reference_pu`) enters the Speed Controller
2. Machine-Side Observer estimates actual speed from voltage/current measurements
3. Speed Controller computes error and generates current reference
4. Current reference is passed to the Current Controller

**Key Connections**:
- `machine_observer.estimated_speed_pu` → `speed_controller.measured_speed_pu`
- `speed_controller.current_reference` → `current_controller.current_reference_pu`

### 2. Current Control Loop

```
Current Reference → Current Controller → Firing Angle Reference
         ↑                                       |
         |                                       ↓
   DC Current ← DC Link Measurements      Grid Modulator
```

**Signal Path**:
1. Current reference from Speed Controller (or external in torque mode)
2. DC Link Measurements provides actual DC current
3. Current Controller computes firing angle to regulate DC current
4. Firing angle reference sent to Grid-Side Modulator

**Key Connections**:
- `dc_link_measurements.dc_link_current_pu` → `current_controller.dc_link_current_measured_pu`
- `current_controller.firing_angle_reference` → `grid_modulator.firing_angle_reference`

### 3. Grid-Side Synchronization Chain

```
Grid Voltages → Grid Measurements → Grid Observer → Grid Modulator → Firing Pulses
                                           |                              |
                                           ↓                              ↓
                                    Angle & Frequency                Plant Model
```

**Signal Path**:
1. Raw grid voltages from Plant Model
2. Grid Measurements scales and filters signals
3. Grid Observer (PLL) extracts phase angle and frequency
4. Grid Modulator generates synchronized firing pulses

**Key Connections**:
- `plant.grid_voltage_abc` → `grid_measurements.line_voltages_raw`
- `grid_measurements.grid_voltage_abc_pu` → `grid_observer.grid_voltage_abc`
- `grid_observer.estimated_angle` → `grid_modulator.grid_voltage_angle`
- `grid_observer.estimated_frequency` → `grid_modulator.grid_frequency`
- `grid_modulator.firing_pulses` → `plant.grid_side_firing_pulses`

### 4. Machine-Side Control Chain

```
DC Current → Lookup Table → Firing Angle → Machine Modulator → Firing Pulses
                                  |                                     |
                                  ↓                                     ↓
Machine Measurements → Machine Observer → Rotor Position          Plant Model
                              |
                              ↓
                        Speed Estimate
```

**Signal Path**:
1. Machine-Side Lookup Table determines optimal firing angle based on DC current
2. Machine Observer estimates rotor position and speed from machine measurements
3. Machine Modulator combines lookup table firing angle with rotor position to generate position-synchronized firing pulses
4. Machine firing pulses are sent to the plant model

**Key Connections**:
- `dc_link_measurements.dc_link_current_pu` → `lookup_table.dc_link_current_pu`
- `lookup_table.firing_angle` → `machine_modulator.lookup_table_firing_angle`
- `machine_measurements.machine_voltage_abc_pu` → `machine_observer.machine_voltage_pu`
- `machine_observer.estimated_reference_angle` → `machine_modulator.rotor_position_electrical`
- `machine_modulator.firing_pulses` → `plant.machine_side_firing_pulses`

### 5. Measurement Processing Chains

All measurement chains follow a similar pattern:

```
Raw Sensor Values → Scaling → Per-Unit Conversion → Filtering → Control Algorithms
```

**Grid-Side Measurements**:
- Inputs: `line_voltages_raw`, `line_currents_raw`, `dc_link_voltage_raw`
- Outputs: `grid_voltage_abc_pu`, `grid_current_abc_pu`, `dc_voltage_pu`
- Features: Harmonic filtering (5th, 7th, 11th, 13th), THD calculation

**Machine-Side Measurements**:
- Inputs: `machine_voltages_raw`, `machine_currents_raw`, `machine_speed_raw`
- Outputs: `machine_voltage_abc_pu`, `machine_current_abc_pu`, `machine_speed_pu`
- Features: Adaptive filtering based on machine frequency, ripple calculation

**DC Link Measurements**:
- Inputs: `dc_voltage`, `dc_current`
- Outputs: `dc_voltage_pu`, `dc_current_pu`, `dc_power_pu`
- Features: Low-pass filtering for ripple removal

### 6. Feedback and Feedforward Paths

**Current Controller Feedforward**:
```
Grid Observer → Ideal DC Voltage → Current Controller
Machine Observer → Back-EMF Estimate → Current Controller
```

These feedforward paths improve dynamic response:
- `grid_observer.ideal_dc_voltage` → `current_controller.ideal_dc_voltage_pu`
- `machine_observer.estimated_back_emf_pu` → `current_controller.inverter_back_emf_machine_pu`

**Commutation Feedback**:
```
Firing Pulses → Observers → Commutation Detection → Modulators
```

Observers use firing pulse information to detect commutation events:
- `grid_modulator.firing_pulses` → `grid_observer.grid_side_pulses`
- `machine_modulator.firing_pulses` → `machine_observer.machine_side_pulses`

### 7. Complete Firing Pulse Generation Chain

**Grid-Side Firing Pulse Path**:
```
Current Controller → Firing Angle Ref → Grid Modulator → Grid Firing Pulses → Plant
                                              ↑
                                        Grid Observer
                                              ↑
                                      Grid Measurements
                                              ↑
                                         Plant Model
```

**Machine-Side Firing Pulse Path**:
```
DC Current → Lookup Table → Firing Angle → Machine Modulator → Machine Firing Pulses → Plant
                                                    ↑
                                            Machine Observer
                                                    ↑
                                          Machine Measurements
                                                    ↑
                                               Plant Model
```

**Key Signal Connections for Firing Pulses**:
- `current_controller.firing_angle_reference` → `grid_modulator.firing_angle_reference`
- `grid_observer.estimated_angle` → `grid_modulator.grid_voltage_angle`
- `grid_modulator.firing_pulses` → `plant.grid_side_firing_pulses`
- `lookup_table.firing_angle` → `machine_modulator.lookup_table_firing_angle`
- `machine_observer.estimated_reference_angle` → `machine_modulator.rotor_position_electrical`
- `machine_modulator.firing_pulses` → `plant.machine_side_firing_pulses`

## Complete System Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           SFC Control System                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Speed Ref ──┐                                                              │
│              ↓                                                              │
│       ┌──────────────┐         ┌─────────────────┐      ┌──────────────┐  │
│       │Speed Control │────────►│Current Control  │─────►│Grid Modulator│  │
│       └──────▲───────┘         └────────▲────────┘      └──────┬───────┘  │
│              │                           │                      │          │
│              │                     DC Current              Grid Angle      │
│              │                           │                      │          │
│       ┌──────┴───────┐         ┌────────┴────────┐      ┌─────┴────────┐  │
│       │Machine Speed │         │DC Link Meas.    │      │Grid Observer │  │
│       └──────▲───────┘         └────────▲────────┘      └─────▲────────┘  │
│              │                           │                     │          │
│       ┌──────┴───────┐                  │               ┌─────┴────────┐  │
│       │Machine Obs.  │                  │               │Grid Meas.    │  │
│       └──────▲───────┘                  │               └─────▲────────┘  │
│              │                           │                     │          │
│       ┌──────┴───────┐                  │                     │          │
│       │Machine Meas. │◄─────────────────┼─────────────────────┼──────────│
│       └──────────────┘                  │                     │          │
│                                          │                     │          │
│       ┌──────────────┐                  │                     │          │
│       │Machine-Side  │                  │                     │          │
│       │Lookup Table  │◄─────────────────┘                     │          │
│       └──────┬───────┘                                        │          │
│              │ Firing Angle                                   │          │
│              ↓                                                │          │
│       ┌──────────────┐                                        │          │
│       │Machine       │                                        │          │
│       │Modulator     │                                        │          │
│       └──────┬───────┘                                        │          │
│              │ Machine Pulses                                 │          │
│              ↓                                                │          │
├──────────────┼─────────────────┬──────────────────────────────┼──────────┤
│              │                 │ Grid Pulses                  │          │
│              │                 ↓                              │          │
│       ┌──────┴─────────────────┴──────────────────────────────┴────────┐  │
│       │                        Plant Model                              │  │
│       │  - 6-pulse Grid-Side Converter                                 │  │
│       │  - DC Link                                                     │  │
│       │  - 6-pulse Machine-Side Converter                              │  │
│       │  - Synchronous Machine + Mechanical Load                       │  │
│       └─────┬───────────────────┬───────────────────────────────────────┘  │
│             │                   │                                          │
│             │                   └──────────────────────────────────────────│
│             └──────────────────────────────────────────────────────────────│
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Implementation Details

### Connector Classes

Each signal connection is implemented using dedicated connector classes that ensure type safety and clear signal routing:

1. **Control Loop Connectors**
   - `machine_observer_to_speed_controller_connector`
   - `speed_controller_to_current_controller_connector`
   - `dc_link_to_current_controller_connector`
   - `current_controller_to_grid_modulator_connector`

2. **Observer Chain Connectors**
   - `grid_measurements_to_grid_observer_connector`
   - `grid_observer_to_grid_modulator_connector`
   - `machine_measurements_to_machine_observer_connector`

3. **Plant Interface Connectors**
   - `plant_to_grid_measurements_connector`
   - `plant_to_machine_measurements_connector`
   - `plant_to_dc_link_measurements_connector`
   - `modulators_to_plant_connector`

4. **Machine-Side Firing Chain Connectors**
   - `dc_link_to_lookup_table_connector`
   - `lookup_table_to_machine_modulator_connector`
   - `machine_observer_to_machine_modulator_connector`

### Sampling Rates

Different components operate at different sampling rates for optimal performance:

- **Plant Model**: 10 μs (100 kHz)
- **Measurements & Filtering**: 100 μs (10 kHz)
- **Observers**: 100 μs (10 kHz)
- **Current Controller**: 100 μs (10 kHz)
- **Speed Controller**: 1 ms (1 kHz)
- **Modulators**: 100 μs (10 kHz)
- **Lookup Table**: 100 μs (10 kHz)

### Per-Unit System

All control algorithms operate in per-unit to ensure numerical stability and hardware independence. The per-unit system maintains separate bases for:

- **Grid-Side**: Based on grid voltage and frequency
- **DC Link**: Based on rated DC current
- **Machine-Side**: Based on machine power and ratings

### Key Design Patterns

1. **Signal Flow Datamodel**: Each component has clearly defined inputs and outputs
2. **Type Safety**: Strong typing using yeet::control units prevents unit errors
3. **Hierarchical Control**: Cascaded loops with clear precedence
4. **Observer Pattern**: Measurements are processed through observers before use
5. **Feedforward Enhancement**: Strategic feedforward paths improve response
6. **Dual Firing Pulse Generation**: Separate optimized paths for grid and machine-side modulation

## Operational Modes

### Speed Control Mode
- Speed Controller generates current reference
- Current Controller regulates DC link current
- Machine-Side Lookup Table optimizes firing angle based on DC current
- Both grid and machine modulators generate synchronized firing pulses

### Torque Control Mode
- External current reference bypasses Speed Controller
- Direct torque control through current regulation
- Machine-Side Lookup Table continues to optimize firing angle

### Startup Sequence
1. Measurements initialize and validate sensors
2. Observers acquire lock on grid/machine voltages
3. Lookup table initializes with default firing angles
4. Modulators enable firing pulses
5. Controllers activate in sequence (current → speed)

## Safety Interlocks

The interconnection includes several safety features:

1. **Observer Lock Detection**: Firing pulses disabled until observers converge
2. **Measurement Validation**: Invalid measurements trigger safe states
3. **Saturation Indicators**: Controllers report saturation status
4. **Commutation Monitoring**: Detection of failed commutations
5. **Lookup Table Limits**: Firing angles bounded within safe operating ranges

## Testing and Validation

The modular interconnection design enables:

1. **Component Testing**: Each component can be tested in isolation
2. **Integration Testing**: Connector classes can be verified independently
3. **Closed-Loop Testing**: Complete system behavior validation
4. **Hardware-in-Loop**: Same datamodels used for HIL testing
5. **Firing Pulse Verification**: Individual modulation paths can be validated

## Conclusion

This interconnection architecture provides a robust, type-safe, and maintainable framework for SFC/LCI control. The clear separation of concerns, well-defined interfaces, and hierarchical structure enable both high performance control and systematic testing/validation. The dual firing pulse generation paths ensure optimal control of both grid-side rectification and machine-side inversion, while the comprehensive feedback and feedforward mechanisms provide excellent dynamic response and stability.