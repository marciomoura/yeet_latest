# SFC Machine-Side Modulator

[← Back to Documentation Index](index.md)

This document describes the `sfc_machine_side_modulator`, a component that manages the firing pulses for the machine-side converter (inverter). Its primary role is to select the correct firing strategy based on the commanded operating mode and to actively manage the commutation sequence when operating at low speeds.

## 1. Overview

The machine-side modulator is the initiator of the commutation process in forced-commutation (pulse) mode and acts as a simple angle follower in natural-commutation mode. It translates the rotor's physical position into precisely timed requests and firing commands to ensure correct torque production.

### Key Responsibilities:
-   **Mode Management**: Operates in one of two externally commanded modes: `Natural Commutation` or `Forced Commutation`.
-   **Commutation Initiation**: In `Forced Commutation` mode, it detects when the motor rotor crosses into a new 60-degree electrical sector and generates a `request_force_commutation` signal. This signal instructs the grid-side converter to extinguish the DC link current.
-   **Pulse Gating**: After requesting commutation, it blocks its own firing pulses and waits for confirmation that the DC link current is zero.
-   **Pulse Release**: Once zero current is confirmed, it releases the firing pulses for the next thyristor pair in the sequence, completing the commutation.
-   **Firing Angle Selection**:
    -   In **Natural Commutation Mode**, it passes through the firing angle (`β`) provided by the `sfc_machine_side_firing_angle_lookup_table`.
    -   In **Forced Commutation Mode**, it uses a fixed firing angle calculated from the absolute rotor position to generate maximum torque.

## 2. Operating Modes

### 2.1. Natural Commutation Mode (High-Speed)

When machine speed is sufficient for the motor's back-EMF to commutate the thyristors, the modulator operates in a simple pass-through mode.

-   **Strategy**: Continuously enable firing pulses using the angle from the lookup table.
-   **Operation**: The modulator continuously enables the six-pulse generator and outputs the firing angle (`β`) provided as an input. The `request_force_commutation` signal is never asserted.

### 2.2. Forced Commutation Mode (Low-Speed / Startup)

At low speeds where there is no back-EMF, the modulator orchestrates a "handshake" protocol to force the current to zero, allowing the thyristors to turn off.

-   **Strategy**: Act as the master for the commutation event.
-   **Operation**:
    1.  The modulator tracks the rotor's 60-degree electrical sector.
    2.  When the rotor transitions to a new sector, the modulator asserts the `request_force_commutation` output for one cycle.
    3.  Simultaneously, it **blocks** its own firing pulses by de-asserting the `release_pulses` output.
    4.  It then monitors the `is_current_zero` input.
    5.  Once `is_current_zero` becomes true and remains true for a configurable hold time, the modulator re-asserts `release_pulses`, firing the correct thyristor pair for the new sector.

## 3. Implementation Details

### 3.1. Forced Commutation State Machine

The core of the `Forced Commutation` logic is a state machine:
1.  **`RUNNING`**: The modulator is releasing pulses for the current sector. It monitors the rotor position. If a sector change is detected, it asserts `request_force_commutation` for one cycle, de-asserts `release_pulses`, and transitions to `WAITING_FOR_ZERO_CURRENT`.
2.  **`WAITING_FOR_ZERO_CURRENT`**: The modulator waits for the `is_current_zero` input to become true. When it does, it starts a timer and transitions to `CONFIRMING_ZERO_CURRENT`.
3.  **`CONFIRMING_ZERO_CURRENT`**: The modulator waits for the configured `zero_current_hold_time` to elapse. If the current remains zero for the entire duration, it re-asserts `release_pulses` and transitions back to `RUNNING`. If the current becomes non-zero, it transitions back to `WAITING_FOR_ZERO_CURRENT`.

### 3.2. Firing Angle Calculation

-   **Natural Commutation**: `firing_angle = firing_angle_from_lut`.
-   **Forced Commutation**: The firing angle is calculated based on the rotor sector to align with the optimal torque-producing position. This is typically a fixed value (e.g., 90 degrees relative to the phase A flux).

## 4. Integration Notes

The machine-side modulator is designed to integrate seamlessly with:
- `sfc_machine_side_firing_angle_lookup_table` for Load-Commutated Mode operation
- Rotor position feedback systems for Pulse Mode operation
- The overall SFC control architecture for mode coordination

The modulator ensures smooth transitions between operating modes while maintaining the safety and efficiency requirements of the LCI drive system.