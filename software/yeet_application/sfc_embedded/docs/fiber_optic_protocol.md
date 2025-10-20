# Fiber-Optic Communication Protocol

## Overview

This document describes the optimized bit-packed communication protocol used for high-speed, deterministic inter-board communication between the grid-side and machine-side controllers via fiber-optic link.

## Design Goals

- **Minimal Bandwidth**: Use bit-packing to reduce payload size and maximize transmission efficiency
- **Deterministic**: Fixed payload structure for predictable real-time performance
- **Extensible**: Reserved bits allow future expansion without protocol breaking changes
- **Type-Safe**: Explicit encoding/decoding with clear bit assignments

## Protocol Specifications

### Grid → Machine Link

**Transmission Rate**: 1 kHz (every 1 ms)  
**Payload Size**: 1 uint32_t word (reduced from 4 words - **75% bandwidth savings!**)

#### Word 0: Packed Status and Control Flags

| Bits  | Description                    | Type                          | Values          |
| ----- | ------------------------------ | ----------------------------- | --------------- |
| 0-7   | Operating Mode                 | `sfc_operating_mode_t` (enum) | 0-255           |
| 8     | Is Current Zero                | boolean                       | 0=false, 1=true |
| 9     | Force Commutation Acknowledged | boolean                       | 0=false, 1=true |
| 10    | Pulse Releases                 | boolean                       | 0=false, 1=true |
| 11-31 | Reserved                       | -                             | (future use)    |

**Encoding Example (Grid-Side TX):**
```cpp
uint32_t packed_flags = 0;
packed_flags |= (static_cast<uint32_t>(operating_mode) & 0xFF);  // Bits 0-7
packed_flags |= (is_current_zero ? 1U : 0U) << 8;                // Bit 8
packed_flags |= (force_comm_ack ? 1U : 0U) << 9;                 // Bit 9
packed_flags |= (pulse_releases ? 1U : 0U) << 10;                // Bit 10
```

**Decoding Example (Machine-Side RX):**
```cpp
const uint32_t packed_flags = payload[0];
auto operating_mode = static_cast<sfc_operating_mode_t>(packed_flags & 0xFF);  // Bits 0-7
bool is_current_zero = ((packed_flags >> 8) & 0x01) != 0;                      // Bit 8
bool force_comm_ack = ((packed_flags >> 9) & 0x01) != 0;                       // Bit 9
bool pulse_releases = ((packed_flags >> 10) & 0x01) != 0;                      // Bit 10
```

#### Future Expansion (Words 1-3)

Reserved for additional signals such as:
- DC link current measurement (float)
- Grid voltage magnitude (float)
- Additional status flags
- State machine ID

---

### Machine → Grid Link

**Transmission Rate**: 1 kHz (every 1 ms)  
**Payload Size**: 4 uint32_t words

#### Word 0: Measured Speed [pu]
- **Type**: IEEE 754 single-precision float (as uint32_t bits)
- **Range**: -2.0 to +2.0 pu typical
- **Encoding**: `std::memcpy(&payload[0], &speed_value, sizeof(float))`

#### Word 1: DC Link Current Reference [pu]
- **Type**: IEEE 754 single-precision float (as uint32_t bits)
- **Range**: 0.0 to +2.0 pu typical
- **Encoding**: `std::memcpy(&payload[1], &current_value, sizeof(float))`

#### Word 2: Machine Voltage Magnitude [pu]
- **Type**: IEEE 754 single-precision float (as uint32_t bits)
- **Range**: 0.0 to +1.5 pu typical
- **Encoding**: `std::memcpy(&payload[2], &voltage_value, sizeof(float))`

#### Word 3: Packed Control Flags

| Bits | Description               | Type    | Values          |
| ---- | ------------------------- | ------- | --------------- |
| 0    | Force Commutation Request | boolean | 0=false, 1=true |
| 1-31 | Reserved                  | -       | (future use)    |

**Encoding Example (Machine-Side TX):**
```cpp
// Float values
std::memcpy(&payload[0], &speed_value, sizeof(float));
std::memcpy(&payload[1], &current_value, sizeof(float));
std::memcpy(&payload[2], &voltage_value, sizeof(float));

// Packed flags
uint32_t packed_flags = 0;
packed_flags |= (force_comm_req ? 1U : 0U);  // Bit 0
payload[3] = packed_flags;
```

**Decoding Example (Grid-Side RX):**
```cpp
// Float values
float speed_pu_value, current_ref_value, voltage_mag_value;
std::memcpy(&speed_pu_value, &payload[0], sizeof(float));
std::memcpy(&current_ref_value, &payload[1], sizeof(float));
std::memcpy(&voltage_mag_value, &payload[2], sizeof(float));

// Packed flags
const uint32_t packed_flags = payload[3];
bool force_comm_request = (packed_flags & 0x01) != 0;  // Bit 0
```

---

## Performance Benefits

### Bandwidth Optimization

| Direction      | Old Protocol       | New Protocol       | Savings |
| -------------- | ------------------ | ------------------ | ------- |
| Grid → Machine | 4 words (16 bytes) | 1 word (4 bytes)   | **75%** |
| Machine → Grid | 4 words (16 bytes) | 4 words (16 bytes) | 0%      |

**Total bidirectional bandwidth reduced by 37.5%** (from 32 bytes to 20 bytes per cycle)

### Future Expansion Capacity

With the current protocol:
- **Grid → Machine**: 3 additional uint32_t words available (12 bytes)
- **Machine → Grid**: 31 additional boolean flags in word 3

This allows adding:
- Additional analog measurements (DC current, voltages)
- More status flags (faults, warnings, diagnostics)
- State machine information
- Timing/synchronization data

---

## Implementation Notes

### Type Punning for Float Transmission

IEEE 754 floats are transmitted by copying their bit representation into `uint32_t` words using `std::memcpy()`. This is the **only standards-compliant way** to perform type punning in C++:

```cpp
// CORRECT: Standards-compliant type punning
float value = 1.234f;
uint32_t bits;
std::memcpy(&bits, &value, sizeof(float));

// WRONG: Undefined behavior (violates strict aliasing)
uint32_t bits = *reinterpret_cast<uint32_t*>(&value);  // DON'T DO THIS
```

### Bit Field Access

Bit fields are accessed using bitwise operations:
- **Extract**: `(word >> bit_position) & mask`
- **Set**: `word |= (value << bit_position)`
- **Clear**: `word &= ~(mask << bit_position)`

### Enum Encoding

Enumerations are cast to `uint32_t` and masked to fit their bit field:
```cpp
// Encoding (8-bit enum field)
uint32_t encoded = static_cast<uint32_t>(enum_value) & 0xFF;

// Decoding
auto decoded = static_cast<EnumType>(word & 0xFF);
```

---

## Error Handling

### Checksum (Future Enhancement)

Currently, no checksum or CRC is implemented. The fiber-optic physical layer provides bit error detection. For additional safety, a CRC-8 or CRC-16 could be added to reserved bits.

### Payload Length Validation

Both sides validate `payload_length` before decoding:
```cpp
if (inputs->fiber_optic_rx.payload_length >= expected_length) {
    // Safe to decode
}
```

### Link Status

The `fiber_optic_rx.link_ok` flag indicates physical layer health. If false, received data should be ignored and safe defaults used.

---

## Revision History

| Version | Date       | Changes                                    | Author         |
| ------- | ---------- | ------------------------------------------ | -------------- |
| 1.0     | 2025-10-20 | Initial bit-packed protocol implementation | Copilot + User |
