# SFC Grid-Side Modulator

[← Back to Documentation Index](index.md)

This document describes the `sfc_grid_side_modulator`, a component that manages the firing angle for the grid-side converter (rectifier). Its primary role is to select the correct firing strategy based on machine speed, switching between a closed-loop external controller for normal operation and an internal forced-commutation strategy for startup.

## 1. Overview

The grid-side modulator acts as a mode-dependent switch for the rectifier's firing angle. It ensures that the DC link current is controlled appropriately for both high-speed, efficient operation and low-speed, forced-commutation startup.

### Key Responsibilities:
-   **Mode Selection**: Automatically determines if the system should be in `Load-Commutated Mode` or `Pulse Mode` based on machine speed.
-   **Firing Angle Selection**:
    -   In **Load-Commutated Mode**, it passes through the firing angle calculated by the main external `sfc_current_controller`.
    -   In **Pulse Mode**, it overrides the external controller and generates its own firing angles to create controlled current pulses and force commutation.
-   **Handover Management**: Provides signals to ensure a smooth, bumpless transfer between modes, particularly by indicating when the external PI controller must be reset.

## 2. Operating Modes

### 2.1. Load-Commutated Mode (High-Speed)

When machine speed is sufficient for natural commutation, the modulator defers to the main current control loop.

-   **Strategy**: Pass-through.
-   **Operation**: The modulator simply outputs the firing angle (`α_grid`) provided by the external `sfc_current_controller`. This external controller is responsible for regulating the DC link current to the setpoint from the speed loop. For details, see the [LCI Current Controller documentation](current_control.md).

### 2.2. Pulse Mode (Low-Speed / Startup)

At low speeds, the modulator takes direct control of the grid-side firing angle to implement a forced-commutation sequence.

-   **Strategy**: Override and internal state machine control.
-   **Operation**: The modulator uses an internal state machine to pulse the DC link current.
    1.  **Current Pulse (`pulse_on` state)**: An internal, dedicated PI controller calculates a firing angle to rapidly build DC link current to the desired setpoint. This PI controller is **only active during Pulse Mode**.
    2.  **Forced Commutation (`force_commutation` state)**: To commutate the machine-side inverter, the modulator outputs a fixed, deep inverting angle (e.g., 150°) to drive the DC link current to zero, regardless of the PI controller's output.

## 3. Critical Note on External PI Controller

The main external `sfc_current_controller` is designed for continuous operation. Its integral term can wind up to invalid values during Pulse Mode.

**ACTION REQUIRED**: The higher-level control task that orchestrates the system **must** reset the external `sfc_current_controller` immediately upon transitioning from Pulse Mode back to Load-Commutated Mode. The `sfc_grid_side_modulator` provides a `is_requesting_main_pi_reset()` flag to signal when this reset is necessary. Failure to do so will result in large current overshoots.

## 4. Architecture and Interface

```cpp
// Proposed interface for sfc_grid_side_modulator.hpp
class sfc_grid_side_modulator {
public:
    explicit sfc_grid_side_modulator(duration_t sampling_time);

    // --- Configuration Methods ---
    void configure_mode_handover_speed_pu(speed_pu_t handover_speed);
    void configure_pulse_mode_pi_gains(double kp, duration_t ti);
    void configure_pulse_mode_current_thresholds(current_pu_t zero_threshold, current_pu_t setpoint_reached_threshold);
    void configure_force_commutation_angle_rad(angle_t angle);

    // --- Main Execution Method ---
    void update(
        bool enable,
        speed_pu_t measured_speed_pu,
        current_pu_t current_setpoint_pu,
        current_pu_t measured_dc_current_pu,
        angle_t firing_angle_from_main_pi
    );

    // --- State Reset Method ---
    void reset();

    // --- Getter Methods ---
    angle_t get_firing_angle_rad() const noexcept;
    operating_mode_t get_current_operating_mode() const noexcept;
    pulse_mode_state_t get_pulse_mode_state() const noexcept;
    bool is_requesting_main_pi_reset() const noexcept;
};
```