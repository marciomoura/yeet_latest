# SFC Grid-Side Modulator

[← Back to Documentation Index](index.md)

This document describes the `sfc_grid_side_modulator`, the component that acts as the **master controller** for the forced commutation sequence in the LCI system.

## 1. Overview

The grid-side modulator is the central coordinator for the commutation process at low speeds. During normal operation, it allows the main DC link current controller to determine the firing angle. However, when a commutation is requested by the `sfc_machine_side_modulator`, it takes full command of the system. It forces the grid-side converter into inversion to extinguish the DC link current and, once the current is zero, sends a critical acknowledgment signal back to the machine-side modulator.

### Key Responsibilities:
-   **Commutation Master**: Manages the stateful handshake protocol for forced commutation.
-   **Firing Angle Override**: Upon receiving a `request_force_commutation` signal, it immediately overrides the firing angle from the main current controller.
-   **Forced Inversion**: It applies a pre-configured, deep inverting firing angle (e.g., 150°) to the grid-side converter to rapidly drive the DC link current to zero.
-   **Controller Reset Management**: While overriding the firing angle, it asserts a signal to hold the main current controller's integrator in reset, preventing integral windup.
-   **Commutation Acknowledgment**: After confirming the DC link current is zero, it asserts a `force_commutation_acknowledged` signal. This is the "all clear" for the machine-side modulator.
-   **Relinquishing Control**: It returns control to the main current controller immediately after sending the acknowledgment, without waiting for the machine-side request to be cleared.

## 2. Forced Commutation Handshake Protocol

The modulator executes a precise, state-driven sequence:

1.  **Request**: It continuously monitors the `request_force_commutation` input from the machine-side modulator.
2.  **Invert**: When the request is asserted, the grid-side modulator immediately takes control, forces the firing angle into deep inversion, and requests a reset of the main PI controller.
3.  **Confirm Zero Current**: It waits for the `is_current_zero` input to become true.
4.  **Acknowledge & Release**: Once zero current is confirmed, it asserts the `force_commutation_acknowledged` output signal for a single cycle and immediately returns to normal operation, handing control of the firing angle back to the PI controller.

## 3. Architecture and Interface

```cpp
// Proposed interface for sfc_grid_side_modulator.hpp
class sfc_grid_side_modulator {
public:
    explicit sfc_grid_side_modulator();

    // --- Configuration Methods ---
    void configure_force_commutation_angle_rad(angle_t angle);

    // --- Main Execution Method ---
    void update(
        bool request_force_commutation,   // From machine-side modulator
        bool is_current_zero,
        angle_t firing_angle_from_main_pi
    );

    // --- State Reset Method ---
    void reset();

    // --- Getter Methods ---
    angle_t get_firing_angle_rad() const noexcept;
    bool get_force_commutation_acknowledged() const noexcept; // To machine-side modulator
    bool get_is_requesting_main_pi_reset() const noexcept;
};
```

## 4. Implementation Details

### 4.1. Forced Commutation State Machine

The core of the logic is a simple two-state machine:
1.  **`NORMAL_OPERATION`**: The default state. It passes through the firing angle from the PI controller. If `request_force_commutation` becomes true, it transitions to `FORCE_INVERSION`.
2.  **`FORCE_INVERSION`**: The modulator has taken control. It outputs the deep inversion angle and requests a PI reset. It monitors the DC link current. If `is_current_zero` becomes true, it asserts `force_commutation_acknowledged` for one cycle and transitions back to `NORMAL_OPERATION`.

### 4.2. Future Safety Enhancement: Handshake Timer

While the current design is efficient, a potential failure mode exists if the `is_current_zero` signal never arrives after a request is made. To enhance safety, a timer can be added to monitor the duration of the `FORCE_INVERSION` state. If the state persists for longer than a configurable timeout period, a system alarm or fault should be triggered. This functionality can be added in the future without changing the core state machine logic.

### 4.3. Critical Note on External PI Controller

The `get_is_requesting_main_pi_reset()` flag **must** be used by the higher-level control task to hold the external `sfc_current_controller`'s integrator in reset during the `FORCE_INVERSION` state. Failure to do so will cause dangerous DC link current overshoots when control is handed back.
