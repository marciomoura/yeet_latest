# SFC Machine-Side Modulator

[← Back to Documentation Index](index.md)

This document describes the `sfc_machine_side_modulator`, a component that manages the firing angle for the machine-side converter (inverter). Its function is to select the correct firing strategy based on machine speed, switching between a lookup-table-based angle for normal operation and a fixed, position-based angle for startup.

## 1. Overview

The machine-side modulator acts as a mode-dependent switch for the inverter's firing angle. It ensures the inverter correctly synchronizes with the motor's back-EMF at high speed and correctly positions the rotor for torque production at low speed.

### Key Responsibilities:
-   **Mode Selection**: Automatically determines if the system should be in `Load-Commutated Mode` or `Pulse Mode` based on machine speed.
-   **Firing Angle Selection**:
    -   In **Load-Commutated Mode**, it passes through the firing angle provided by the `sfc_machine_side_firing_angle_lookup_table`.
    -   In **Pulse Mode**, it overrides the lookup table and calculates a fixed firing angle based on the absolute rotor position to generate maximum torque.

## 2. Operating Modes

### 2.1. Load-Commutated Mode (High-Speed)

When machine speed is sufficient for natural commutation, the modulator uses a sophisticated open-loop strategy.

-   **Strategy**: Pass-through from lookup table.
-   **Operation**: The modulator simply outputs the firing angle (`β`) provided as an input, which is pre-calculated by the `sfc_machine_side_firing_angle_lookup_table`. This angle ensures a safe commutation margin (extinction angle γ) is maintained.

### 2.2. Pulse Mode (Low-Speed / Startup)

At low speeds, the modulator's role simplifies to that of a distributor, selecting the correct thyristor pair to fire based on rotor position.

-   **Strategy**: Override and calculate based on position.
-   **Operation**: The modulator calculates a firing angle that corresponds to the thyristor pair best aligned to produce torque for the current rotor position. This angle is typically fixed (e.g., 0 or 90 degrees, depending on the frame of reference) relative to the rotor flux. The modulator holds this angle constant for the duration of the DC link current pulse created by the grid-side converter.

## 3. Architecture and Interface

```cpp
// Proposed interface for sfc_machine_side_modulator.hpp
class sfc_machine_side_modulator {
public:
    explicit sfc_machine_side_modulator(duration_t sampling_time);

    // --- Configuration Methods ---
    void configure_mode_handover_speed_pu(speed_pu_t handover_speed);
    void configure_pulse_mode_firing_angle_rad(angle_t fixed_angle);

    // --- Main Execution Method ---
    void update(
        bool enable,
        speed_pu_t measured_speed_pu,
        angle_t absolute_rotor_position_rad,
        angle_t firing_angle_from_lut
    );

    // --- State Reset Method ---
    void reset();

    // --- Getter Methods ---
    angle_t get_firing_angle_rad() const noexcept;
    operating_mode_t get_current_operating_mode() const noexcept;
};
```

## 4. Implementation Details

### 4.1. Mode Transition Logic

The modulator switches between operating modes based on the `measured_speed_pu` compared to the configured `handover_speed_pu` threshold:

- **Transition to Load-Commutated Mode**: When `measured_speed_pu >= handover_speed_pu`
- **Transition to Pulse Mode**: When `measured_speed_pu < handover_speed_pu`

### 4.2. Pulse Mode Firing Angle Calculation

In Pulse Mode, the firing angle is calculated based on the rotor position to select the optimal thyristor pair for maximum torque production. The calculation considers:

1. **Rotor Sector Determination**: The absolute rotor position is divided into 6 sectors (60° each)
2. **Thyristor Pair Selection**: Each sector corresponds to a specific thyristor pair
3. **Fixed Angle Application**: A configurable fixed angle (typically 0°) is applied relative to the selected pair

### 4.3. Real-Time Constraints

The modulator operates with strict real-time constraints:
- All methods marked `noexcept` for deterministic execution
- No dynamic memory allocation during runtime
- Predictable execution time for all `update()` calls

## 5. Usage Example

```cpp
#include "sfc_machine_side_modulator.hpp"

// Create and configure the modulator
sfc_machine_side_modulator modulator(duration_t(100e-6)); // 100µs sampling
modulator.configure_mode_handover_speed_pu(speed_pu_t{0.1}); // 10% speed threshold
modulator.configure_pulse_mode_firing_angle_rad(angle_t{0.0}); // 0° fixed angle

// In the control loop
modulator.update(
    enable_signal,
    measured_speed_pu,
    absolute_rotor_position_rad,
    firing_angle_from_lookup_table
);

// Get the output firing angle
angle_t machine_firing_angle = modulator.get_firing_angle_rad();
```

## 6. Integration Notes

The machine-side modulator is designed to integrate seamlessly with:
- `sfc_machine_side_firing_angle_lookup_table` for Load-Commutated Mode operation
- Rotor position feedback systems for Pulse Mode operation
- The overall SFC control architecture for mode coordination

The modulator ensures smooth transitions between operating modes while maintaining the safety and efficiency requirements of the LCI drive system.