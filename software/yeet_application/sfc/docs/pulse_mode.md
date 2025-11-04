# Pulse Mode (Forced Commutation) Logic

[← Back to Documentation Index](index.md)

This document provides a high-level conceptual overview of the "Pulse Mode" or "Forced Commutation" technique used in the LCI drive at low speeds. It describes the coordinated handshake protocol between the machine-side and grid-side modulators.

## 1. The Challenge: Commutation at Low Speed

A Load-Commutated Inverter (LCI) relies on the motor's back-EMF (voltage) to naturally turn off its thyristors. At low speeds or standstill, there is insufficient back-EMF, so a "forced" method is required to extinguish the DC link current, allowing the machine-side thyristors to turn off and the next pair in the sequence to be fired.

## 2. Conceptual Overview: Roles and Responsibilities

The solution is a highly-coordinated, stateful handshake protocol between the two converters, orchestrated by the grid-side modulator.

*   **`sfc_machine_side_modulator` (The Initiator):**
    *   Its primary job is to track the rotor's position.
    *   When the rotor moves to the next electrical sector, it knows a commutation is required.
    *   It **initiates** the event by sending a `request_force_commutation` signal to the grid-side.

*   **`sfc_grid_side_modulator` (The Master / Coordinator):**
    *   This component contains the supervisory logic and acts as the **master** of the handshake.
    *   It listens for the request from the machine side.
    *   When the request is received, it **coordinates** the entire event by taking control of the DC link.

*   **`sfc_current_controller` (The Workhorse):**
    *   Its normal job is to regulate the DC link current (`Idc`) to match the torque demand by adjusting the grid-side firing angle.
    *   During forced commutation, its output is temporarily ignored and its integrator **must** be held in reset to prevent windup.

*   **Zero-Current Detector (ZCD):**
    *   A hardware circuit or software comparator that provides the critical feedback signal: `is_current_zero`.

## 3. Communication Flow: Step-by-Step Sequence

Here is the chronological sequence of events for a single forced commutation event.

**Phase 1: Steady-State (Running)**
*   **Grid-Side:** The `sfc_grid_side_modulator` is in `NORMAL_OPERATION`. It passes the firing angle from the `sfc_current_controller` to the pulse generator, maintaining the desired DC link current.
*   **Machine-Side:** The `sfc_machine_side_modulator` is in its `RUNNING` state, releasing pulses for a specific thyristor pair. It is monitoring the rotor position.

**Phase 2: Commutation Initiation**
*   **Step 1: Machine-Side Detects Sector Change.** The rotor moves into the next 60-degree electrical sector.
*   **Step 2: Machine-Side Requests Commutation.** The `sfc_machine_side_modulator`:
    *   Asserts its `request_force_commutation` output to `true`.
    *   Immediately blocks its own pulses by setting `release_pulses` to `false`.
    *   Transitions to its `WAITING_FOR_ACK` state.

**Phase 3: Grid-Side Takes Control**
*   **Step 3: Grid-Side Detects Request.** The `sfc_grid_side_modulator` sees `request_force_commutation` is `true` and transitions to its `FORCE_INVERSION` state.
*   **Step 4: Grid-Side Forces Inversion.** The `sfc_grid_side_modulator` immediately:
    *   Overrides the firing angle from the current controller and outputs a deep inverting angle (e.g., 150°).
    *   Asserts `is_requesting_main_pi_reset` to `true` to prevent controller windup.

**Phase 4: Handshake and Pulse Release**
*   **Step 5: Current Extinguishes.** The inverted DC link voltage rapidly drives the current to zero. The Zero-Current Detector asserts `is_current_zero` to `true`.
*   **Step 6: Grid-Side Acknowledges.** The `sfc_grid_side_modulator` sees that the current is zero. It:
    *   Asserts its `force_commutation_acknowledged` output to `true` for one cycle.
    *   Immediately transitions back to `NORMAL_OPERATION`, de-asserting the PI reset request and handing firing angle control back to the current controller.
*   **Step 7: Machine-Side Releases New Pulses.** The `sfc_machine_side_modulator` sees `force_commutation_acknowledged` is `true`. It:
    *   Immediately re-asserts `release_pulses` to `true`, firing the next thyristor pair.
    *   De-asserts its `request_force_commutation` signal to `false`.
    *   Transitions back to its `RUNNING` state.

The system is now back in Phase 1, delivering current through the new motor winding, and the handshake is complete.