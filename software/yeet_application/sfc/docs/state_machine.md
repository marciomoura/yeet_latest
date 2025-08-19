# LCI Converter Operational State Machine

## 1. Overview

The LCI (Load-Commuted Inverter) operational state machine is the central logic governing the converter's lifecycle. It ensures a safe, sequential, and predictable transition through various operational phases, from initial power-up to active running and controlled shutdown. The state machine manages the activation of auxiliary systems, the closing of circuit breakers, and the release of modulation based on external commands and system feedback.

## 2. State Descriptions

The state machine consists of the following distinct operational states:

### `IDLE`
- **Description**: The default, de-energized state. All auxiliary systems and main power circuits are off. This is the initial state upon system startup.
- **Entry Condition**: System power-on or `reset()` command.
- **Exit Condition**: `auxiliaries_on_command` is received.

### `AUXILIARIES_ON`
- **Description**: The first step in the startup sequence. Auxiliary systems (e.g., cooling fans, control power) are activated, but the main power circuit remains de-energized.
- **Entry Condition**: An `auxiliaries_on_command` is received from the `IDLE` state.
- **Exit Condition**: Transitions to `BREAKERS_OFF` on `breakers_on_command` or back to `IDLE` on `auxiliaries_off_command`.

### `BREAKERS_OFF`
- **Description**: The system is ready for the main circuit breakers to be closed. Auxiliaries are running, and all conditions for energizing the main circuit are met.
- **Entry Condition**: A `breakers_on_command` is received from the `AUXILIARIES_ON` state.
- **Exit Condition**: Transitions to `BREAKERS_ON` once feedback confirms the breakers are closed (`breakers_closed`).

### `BREAKERS_ON`
- **Description**: The main circuit breakers are closed, and the power circuit is energized. The system is in a standby mode, ready to accept a start command, but the converter is not yet firing.
- **Entry Condition**: `breakers_closed` feedback is received from the `BREAKERS_OFF` state.
- **Exit Condition**: Transitions to `OPERATION_OFF` upon receiving a `start_command`.

### `OPERATION_OFF`
- **Description**: The final standby state before active operation. The system is fully energized and ready, but modulation is disabled. This state acts as a gate to prevent accidental operation.
- **Entry Condition**: A `start_command` is received from the `BREAKERS_ON` state.
- **Key Signals**:
    - `modulation_release` = `FALSE`
    - `anti_windup_active` = `FALSE`
- **Exit Condition**: Transitions to `OPERATION_ON` upon `operation_on_command` AND `system_ready`.

### `OPERATION_ON`
- **Description**: The main operational state. The converter is actively running and controlling the load (e.g., motor).
- **Entry Condition**: An `operation_on_command` is received, and the `system_ready` flag is true.
- **Key Signals**:
    - `modulation_release` = `TRUE`
    - `anti_windup_active` = `FALSE`
- **Exit Condition**: Transitions to `SHUTDOWN_REQUESTED` upon any stop command, protection trip, or shutdown request.

### `SHUTDOWN_REQUESTED`
- **Description**: A controlled shutdown sequence is in progress. This state ensures the system current is ramped down to zero safely before disabling modulation.
- **Entry Condition**: A `shutdown_request`, `protection_trip`, or `manual_stop` occurs.
- **Key Signals**:
    - `modulation_release` = `TRUE` (Modulation remains active to control the current ramp-down).
    - `anti_windup_active` = `TRUE` (Freezes controller integrators to prevent wind-up).
- **Actions**: The `SHUTDOWN_CONTROL()` logic is executed, which typically involves forcing the firing angle to a value that drives the DC link current to zero.
- **Exit Condition**: Transitions to `OPERATION_OFF` once `zero_current_detected` becomes true.

## 3. State Transition Summary

**Startup Sequence:**
`IDLE` → (aux on) → `AUXILIARIES_ON` → (breakers cmd) → `BREAKERS_OFF` → (breakers closed) → `BREAKERS_ON` → (start) → `OPERATION_OFF` → (operation on) → `OPERATION_ON`

**Shutdown Path:**
`OPERATION_ON` → (shutdown/protection) → `SHUTDOWN_REQUESTED` → (zero current) → `OPERATION_OFF`

## 4. Shutdown Sequence Logic

The shutdown sequence is a critical safety feature designed to de-energize the system gracefully.

1.  **Initiation**: A `shutdown_request`, `protection_trip`, or `manual_stop` command transitions the state machine from `OPERATION_ON` to `SHUTDOWN_REQUESTED`.
2.  **Controller Freeze**: The `anti_windup_active` signal is asserted to `TRUE`. This freezes the integrators of any upstream controllers (e.g., speed controller) to prevent them from winding up while the current is being ramped down.
3.  **Modulation Active**: The `modulation_release` signal remains `TRUE`. This is crucial, as the firing angle control is required to actively drive the current to zero.
4.  **Current Ramp-Down**: The `SHUTDOWN_CONTROL()` logic takes over, typically forcing the line-side converter's firing angle to its end position (e.g., ~150 degrees). This applies a negative voltage to the DC link, rapidly and safely draining its energy and reducing the current.
5.  **Completion**: Once the `zero_current_detected` feedback signal becomes `TRUE`, the state machine transitions to `OPERATION_OFF`, and `modulation_release` is finally set to `FALSE`, disabling the converter pulses.