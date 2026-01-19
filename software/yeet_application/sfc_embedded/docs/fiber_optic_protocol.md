# Fiber-Optic Communication Protocol (v2.0)

## Overview

This document describes the optimized bit-packed communication protocol used for high-speed, deterministic inter-board communication between the grid-side and machine-side controllers via fiber-optic link.

## Design Goals

- **Minimal Bandwidth**: Use bit-packing to reduce payload size and maximize transmission efficiency.
- **Deterministic**: Fixed payload structure for predictable real-time performance.
- **Resilient**: Incorporates communication health handshake and hysteresis-based watchdog logic (at application level).
- **Type-Safe**: Explicit encoding/decoding with clear bit assignments using `sfc::fiber_optic::` C++ namespace.

## Protocol Specifications

### Grid → Machine Link (Control Command)

**Transmission Rate**: 1 kHz (every 1 ms)  
**Payload Size**: 3 uint32_t words

| Word | Description                       | Type           | Units |
| ---- | --------------------------------- | -------------- | ----- |
| 0    | DC link current measurement       | IEEE 754 float | pu    |
| 1    | Grid voltage magnitude            | IEEE 754 float | pu    |
| 2    | Packed Control Flags and State ID | Bit-field      | -     |

#### Word 2: Packed Control Flags and State ID

| Bits  | Description                    | Values                       |
| ----- | ------------------------------ | ---------------------------- |
| 0     | Is forced commutation mode     | 0=false, 1=true              |
| 1     | Force commutation acknowledged | 0=false, 1=true              |
| 2     | Release pulses                 | 0=false, 1=true              |
| 3     | Grid release protections       | 0=false, 1=true              |
| 4     | Machine release protections    | 0=false, 1=true              |
| 5     | Communication ok               | 0=false, 1=true (Handshake)  |
| 6-24  | Reserved                       | -                            |
| 25-31 | State Machine ID               | 0-127 (Current system state) |

---

### Machine → Grid Link (Feedback Status)

**Transmission Rate**: 1 kHz (every 1 ms)  
**Payload Size**: 4 uint32_t words

| Word | Description               | Type           | Units |
| ---- | ------------------------- | -------------- | ----- |
| 0    | Measured machine speed    | IEEE 754 float | pu    |
| 1    | DC link current reference | IEEE 754 float | pu    |
| 2    | Machine voltage magnitude | IEEE 754 float | pu    |
| 3    | Packed Status Flags       | Bit-field      | -     |

#### Word 3: Packed Status Flags

| Bits | Description                | Values                                 |
| ---- | -------------------------- | -------------------------------------- |
| 0    | Force commutation request  | 0=false, 1=true                        |
| 1    | Reserved                   | -                                      |
| 2    | Machine protection tripped | 0=false, 1=true                        |
| 3    | Protection inhibit         | 0=false (ready), 1=true (awaiting ack) |
| 4    | Machine trip acknowledged  | 0=false, 1=true                        |
| 5    | Communication ok           | 0=false, 1=true (Handshake)            |
| 6-31 | Reserved                   | -                                      |

---

## Communication Health and Handshake

### The `communication_ok` Bit (Bit 5)

Both sides include a `communication_ok` bit in their packed words. This bit is used to perform a mutual handshake:
1. **Local Logic**: Each board monitors `fiber_optic_rx.data_ready`.
2. **Watchdog**: If data is valid and frequent, the `sfc_communication_watchdog` (application level) transitions to "OK" state.
3. **Feedback**: When a board's local watchdog is "OK", it sets its outgoing `communication_ok` bit to `1`.
4. **Mutual Confirmation**: Real "System Comm OK" requires both local validation AND the remote side reporting `communication_ok = 1`.

### Hysteresis / Resilience

The application layer `sfc_communication_watchdog` allows for resilience against occasional packet drops (1-2 consecutive packets):
- **Increment on Fail**: If a packet is missing in a 1ms cycle, a counter increments.
- **Decrement on Success**: If a packet arrives, the counter decrements (down to zero).
- **Hard Timeout**: If the counter exceeds a threshold (e.g., 5-10ms), the communication is declared "BAD".
- **Self-Healing**: Once communication resumes and stays stable for a threshold of consecutive valid packets, the watchdog clears the fault.

## Implementation Details

### Type Punning (Floats)

In `yeet_application`, `std::memcpy` is used to map 32-bit float patterns to `uint32_t` words. This ensures standards compliance for C++17.

```cpp
// Encoding
float val = 1.0f;
std::memcpy(&payload[word_idx], &val, sizeof(float));

// Decoding
float out;
std::memcpy(&out, &payload[word_idx], sizeof(float));
```

### STM32 Integration Loop

To ensure deterministic behavior and prevent stale data processing, the STM32 main task loop initializes `data_ready` to `false` at the start of every cycle:

```c
void task_1ms(void) {
    // 1. Reset input state
    task_inputs.fiber_optic_rx.data_ready = false;

    // 2. Poll hardware for new packet
    if (FiberProto_ReceivePacket(...) == FIBER_PROTO_OK) {
        task_inputs.fiber_optic_rx.data_ready = true;
    }

    // 3. Execute Control Task
    yeet_task_1000us(&task_inputs, &task_outputs);
}
```

## Revision History

| Version | Date       | Changes                                                                |
| ------- | ---------- | ---------------------------------------------------------------------- |
| 1.0     | 2025-10-20 | Initial version (archived)                                             |
| 2.0     | 2026-01-20 | Updated to 3/4 word payloads, added bit 5 handshake, clarified bit IDs |

