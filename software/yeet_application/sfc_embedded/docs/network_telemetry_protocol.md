# Network Telemetry Protocol

## Overview

The Network Telemetry Protocol provides efficient, scalable transmission of real-time control system data over TCP/IP. Designed for monitoring and debugging SFC (Static Frequency Converter) systems during development and commissioning.

## Design Principles

### 1. **Scalability**
- Hierarchical signal grouping (measurements, observers, controllers, state machines)
- Bit flags to enable/disable groups per packet
- Easy to add new signal groups without breaking existing parsers
- Supports both grid-side and machine-side controllers

### 2. **Efficiency**
- Binary protocol with fixed-size structures
- Selective transmission: only send requested signal groups
- Typical packet sizes:
  - Full grid-side telemetry: ~112 bytes (header + all groups)
  - Measurements only: 44 bytes (header + measurements)
  - Controller + state: 48 bytes (header + controller + state machine)

### 3. **Type Safety**
- Modern C++17 implementation
- Strong typing with enums and structs
- Compile-time size validation (`static_assert`)

### 4. **Ease of Use**
- Builder pattern for populating packets
- Single function call to encode/decode
- Clear separation between protocol and application logic

## Protocol Structure

### Packet Format

```
┌─────────────────────────────────┐
│  Header (16 bytes)               │  ← Always present
├─────────────────────────────────┤
│  Measurements Group (28 bytes)   │  ← Optional (flag bit 0)
├─────────────────────────────────┤
│  Observer Group (12 bytes)       │  ← Optional (flag bit 1)
├─────────────────────────────────┤
│  Modulator Group (20 bytes)      │  ← Optional (flag bit 2)
├─────────────────────────────────┤
│  Controller Group (20 bytes)     │  ← Optional (flag bit 3)
├─────────────────────────────────┤
│  State Machine Group (12 bytes)  │  ← Optional (flag bit 4)
└─────────────────────────────────┘
```

### Header Structure (16 bytes)

| Field            | Type   | Size | Description                                |
| ---------------- | ------ | ---- | ------------------------------------------ |
| packet_type      | uint8  | 1    | 0x01=Grid-side, 0x02=Machine-side          |
| protocol_version | uint8  | 1    | Protocol version (currently 1)             |
| payload_length   | uint16 | 2    | Total payload size (after header)          |
| flags            | uint32 | 4    | Bit flags for present signal groups        |
| timestamp_ms     | uint32 | 4    | Controller timestamp [ms]                  |
| sequence_number  | uint32 | 4    | Incrementing sequence (detect packet loss) |

### Signal Groups

#### 1. Measurements Group (28 bytes, flag bit 0)

| Field                     | Type  | Description                 |
| ------------------------- | ----- | --------------------------- |
| grid_voltage_magnitude_pu | float | Grid voltage magnitude [pu] |
| dc_link_current_pu        | float | DC link current [pu]        |
| dc_link_voltage_pu        | float | DC link voltage [pu]        |
| grid_voltage_a_pu         | float | Phase A voltage [pu]        |
| grid_voltage_b_pu         | float | Phase B voltage [pu]        |
| grid_voltage_c_pu         | float | Phase C voltage [pu]        |
| dc_link_power_pu          | float | DC link power [pu]          |

#### 2. Observer Group (12 bytes, flag bit 1)

| Field                  | Type  | Description                   |
| ---------------------- | ----- | ----------------------------- |
| estimated_frequency_pu | float | Estimated grid frequency [pu] |
| estimated_angle_rad    | float | Estimated grid angle [rad]    |
| pll_error_pu           | float | PLL tracking error [pu]       |

#### 3. Modulator Group (20 bytes, flag bit 2)

| Field                      | Type  | Description                        |
| -------------------------- | ----- | ---------------------------------- |
| firing_angle_reference_rad | float | Commanded firing angle [rad]       |
| firing_angle_actual_rad    | float | Actual firing angle (after limits) |
| force_commutation_mode     | uint8 | 0=natural, 1=forced                |
| force_commutation_ack      | uint8 | Acknowledgment flag                |
| reserved_1                 | uint8 | Reserved                           |
| reserved_2                 | uint8 | Reserved                           |
| alpha_min_rad              | float | Minimum firing angle limit [rad]   |
| alpha_max_rad              | float | Maximum firing angle limit [rad]   |

#### 4. Controller Group (20 bytes, flag bit 3)

| Field                | Type  | Description                     |
| -------------------- | ----- | ------------------------------- |
| current_reference_pu | float | Current reference [pu]          |
| current_measured_pu  | float | Measured current [pu]           |
| current_error_pu     | float | Control error [pu]              |
| controller_output_pu | float | PI controller output [pu]       |
| operating_mode       | uint8 | 0=forced, 1=natural commutation |
| controller_saturated | uint8 | Saturation flag                 |
| freeze_integrator    | uint8 | Integrator freeze flag          |
| reserved             | uint8 | Reserved                        |

#### 5. State Machine Group (12 bytes, flag bit 4)

| Field             | Type   | Description                       |
| ----------------- | ------ | --------------------------------- |
| current_state     | uint8  | Current state enum value          |
| previous_state    | uint8  | Previous state (transition track) |
| pulse_releases    | uint8  | Pulses enabled flag               |
| fault_active      | uint8  | Fault flag                        |
| state_duration_ms | uint32 | Time in current state [ms]        |
| fault_code        | uint32 | Fault code (if fault_active)      |

## Usage Examples

### Example 1: Transmit Full Telemetry (All Signal Groups)

```cpp
#include "sfc_embedded/network_telemetry_builder.hpp"
#include "sfc_integration/sfc_grid_side_control.hpp"

// In grid_side_tasks.cpp or similar
void transmit_full_telemetry() {
    using namespace yeet::application::sfc::telemetry;
    
    // Create builder
    grid_side_telemetry_builder builder(get_grid_side_control());
    
    // Build complete packet
    auto packet = builder.build_full_packet(get_time_ms());
    
    // Encode to binary
    uint8_t buffer[max_packet_size];
    size_t size = encode_grid_side_telemetry(packet, buffer);
    
    // Transmit over TCP
    network_outputs.network_tx.tx_buffer = buffer;
    network_outputs.network_tx.tx_length = size;
    network_outputs.network_tx.transmit_request = true;
}
```

### Example 2: Transmit Only Measurements and Controller (Reduced Bandwidth)

```cpp
void transmit_essential_telemetry() {
    using namespace yeet::application::sfc::telemetry;
    
    grid_side_telemetry_builder builder(get_grid_side_control());
    
    // Build packet with only essential data
    uint32_t flags = telemetry_flags::measurements | telemetry_flags::controller;
    auto packet = builder.build_packet(get_time_ms(), flags);
    
    // Encode (results in 16 + 28 + 20 = 64 bytes)
    uint8_t buffer[128];
    size_t size = encode_grid_side_telemetry(packet, buffer);
    
    // Transmit
    tcp_send(buffer, size);
}
```

### Example 3: Decode Received Telemetry (Host Side)

```cpp
// On host PC receiving telemetry
void process_received_telemetry(const uint8_t* rx_data, size_t rx_size) {
    using namespace yeet::application::sfc::telemetry;
    
    grid_side_telemetry_packet packet;
    
    if (decode_grid_side_telemetry(rx_data, rx_size, packet)) {
        printf("Timestamp: %u ms, Sequence: %u\n",
               packet.header.timestamp_ms,
               packet.header.sequence_number);
        
        // Check which groups are present
        if (packet.header.flags & telemetry_flags::measurements) {
            printf("Grid Voltage: %.3f pu\n",
                   packet.measurements.grid_voltage_magnitude_pu);
            printf("DC Current: %.3f pu\n",
                   packet.measurements.dc_link_current_pu);
        }
        
        if (packet.header.flags & telemetry_flags::controller) {
            printf("Current Ref: %.3f pu, Measured: %.3f pu\n",
                   packet.controller.current_reference_pu,
                   packet.controller.current_measured_pu);
        }
        
        if (packet.header.flags & telemetry_flags::state_machine) {
            printf("State: %u, Pulses: %s\n",
                   packet.state_machine.current_state,
                   packet.state_machine.pulse_releases ? "ON" : "OFF");
        }
    }
}
```

## Integration with Existing Network Code

### In `grid_side_tasks.cpp` - Add to `yeet_network()`:

```cpp
void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs) {
    // ... existing command processing ...
    
    // Telemetry transmission (every cycle or on request)
    static grid_side_telemetry_builder telemetry_builder(get_grid_side_control());
    
    // Build packet with desired signal groups
    auto packet = telemetry_builder.build_full_packet(get_1ms_counter());
    
    // Encode to network TX buffer
    size_t size = encode_grid_side_telemetry(packet, outputs->network_tx.tx_buffer);
    outputs->network_tx.tx_length = static_cast<uint16_t>(size);
    outputs->network_tx.socket_id = 0;
    outputs->network_tx.transmit_request = true;
}
```

## Bandwidth Analysis

### Full Packet Size
- Header: 16 bytes
- Measurements: 28 bytes
- Observer: 12 bytes
- Modulator: 20 bytes
- Controller: 20 bytes
- State Machine: 12 bytes
- **Total: 108 bytes**

### Transmission Rates

| Update Rate | Bandwidth (Full Packet) | Bandwidth (Measurements Only) |
| ----------- | ----------------------- | ----------------------------- |
| 10 Hz       | 8.64 kbps               | 3.52 kbps                     |
| 100 Hz      | 86.4 kbps               | 35.2 kbps                     |
| 1 kHz       | 864 kbps                | 352 kbps                      |

**Recommended**: 100 Hz for full telemetry (well within 100 Mbps Ethernet capacity)

## Future Extensions

### Machine-Side Telemetry
- Add `machine_side_telemetry_packet` struct
- Include excitation controller, speed controller, torque controller groups
- Reuse same encoding/decoding functions with different packet type

### Diagnostic Telemetry
- Add `diagnostic_group` with cycle times, CPU load, error counters
- Useful for performance monitoring

### Configuration Protocol
- Add bidirectional protocol for parameter tuning
- Host sends configuration commands, controller acknowledges

### Waveform Streaming
- High-frequency signal groups for scope-like functionality
- Circular buffer for pre-trigger capture

## Implementation Checklist

- [x] Protocol specification (network_telemetry.hpp)
- [x] Encoding/decoding implementation (network_telemetry.cpp)
- [x] Builder for grid-side (network_telemetry_builder.hpp/.cpp)
- [ ] Fix builder to use correct signal names from datamodels
- [ ] Integrate into grid_side_tasks.cpp `yeet_network()`
- [ ] Add transmission rate control (e.g., 100 Hz decimation)
- [ ] Create Python host-side decoder for visualization
- [ ] Add machine-side telemetry (similar pattern)
- [ ] Add diagnostic telemetry group
- [ ] Test with real hardware and network

## Notes

- **Endianness**: Currently assumes little-endian (x86, ARM Cortex-M). Add byte-swapping if communicating with big-endian hosts.
- **Alignment**: All structs are naturally aligned for ARM Cortex-M4 (4-byte boundaries).
- **Thread Safety**: Builder is not thread-safe. Create one instance per control task or protect with mutex.
