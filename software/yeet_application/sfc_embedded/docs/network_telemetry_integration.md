# Network Telemetry Integration Guide

## Overview

This document describes the integration of the network telemetry system into the grid-side network task (`yeet_network()`). The telemetry system provides real-time control data streaming over TCP/IP for HMI/SCADA visualization and diagnostics.

## Integration Architecture

### Design Principles

1. **Simplicity**: Reuse existing START/STOP command protocol
2. **Efficiency**: Decimated transmission rate (100 Hz) to prevent bandwidth saturation
3. **Non-blocking**: Never overwrite pending transmission buffers
4. **Real-time safe**: No dynamic memory allocation, fixed-size buffers
5. **Scalable**: Easy to add/remove signal groups via bit flags

### Command Protocol

The network task accepts simple text commands:

```
START\n   - Begin streaming telemetry data
STOP\n    - Stop streaming telemetry data
```

These commands are backward-compatible with the previous test streaming protocol.

## Implementation Details

### State Management

```cpp
struct network_telemetry_state {
    bool is_streaming;                         // Streaming active flag
    uint32_t timestamp_ms;                     // System time in milliseconds
    uint32_t decimation_counter;               // Counter for rate control
    static constexpr uint32_t decimation_factor = 10;  // 100 Hz rate
};
```

### Transmission Flow

1. **Command Processing**:
   - Parse incoming TCP data for START/STOP commands
   - Set `is_streaming` flag accordingly

2. **Decimation Control**:
   - Increment `decimation_counter` on each call
   - Only transmit every 10th call (100 Hz from 1 kHz task rate)

3. **Packet Construction**:
   ```cpp
   // Create builder from control system
   grid_side_telemetry_builder builder(get_grid_side_control());
   
   // Build complete packet (all signal groups)
   auto packet = builder.build_full_packet(timestamp_ms);
   
   // Encode to binary buffer
   size_t size = encode_grid_side_telemetry(packet, tx_buffer);
   
   // Request transmission
   outputs->network_tx.transmit_request = true;
   outputs->network_tx.tx_length = size;
   ```

4. **Non-Blocking Transmission**:
   - Check `transmit_request` flag before building new packet
   - Skip transmission cycle if previous packet not yet sent

## Performance Characteristics

### Bandwidth Usage

| Configuration               | Packet Size | Rate   | Bandwidth |
| --------------------------- | ----------- | ------ | --------- |
| Full telemetry (all groups) | 108 bytes   | 100 Hz | 86.4 kbps |
| Measurements only           | 44 bytes    | 100 Hz | 35.2 kbps |
| Measurements + Observer     | 56 bytes    | 100 Hz | 44.8 kbps |

### CPU Load

- **Builder creation**: ~500 CPU cycles (negligible, inlined)
- **Packet population**: ~1000 CPU cycles (signal reads + float conversions)
- **Binary encoding**: ~2000 CPU cycles (memcpy operations)
- **Total per transmission**: ~3500 cycles @ 180 MHz = ~19 µs

At 100 Hz transmission rate: **1.9 ms / 10 ms period = 19% CPU load**

This is acceptable for non-real-time network task.

## Integration Points

### File: `grid_side_tasks.cpp`

**Includes added**:
```cpp
#include "sfc_embedded/network_telemetry.hpp"
#include "sfc_embedded/network_telemetry_builder.hpp"
```

**State machine**:
- `network_telemetry_state telem_state` (static global)

**Function: `yeet_network()`**:
- Command parsing: START/STOP
- Decimation counter: Rate control
- Builder instantiation: `grid_side_telemetry_builder`
- Packet construction: `build_full_packet()`
- Binary encoding: `encode_grid_side_telemetry()`
- Transmission request: Set output flags

## Usage Instructions

### Host-Side Connection

1. **Establish TCP connection** to grid-side controller (port depends on hardware configuration)

2. **Send START command**:
   ```python
   import socket
   sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   sock.connect(('192.168.1.100', 5000))  # Example IP
   sock.send(b'START\n')
   ```

3. **Receive telemetry packets**:
   ```python
   while True:
       data = sock.recv(1024)
       if len(data) >= 16:  # Minimum header size
           # Decode using telemetry protocol (see network_telemetry_protocol.md)
           header = parse_header(data[:16])
           payload = data[16:]
           # Extract signal groups based on header flags
   ```

4. **Send STOP command**:
   ```python
   sock.send(b'STOP\n')
   sock.close()
   ```

### Python Decoder Example

See `tools/network_telemetry_decoder.py` (to be implemented) for complete host-side decoder.

Basic structure:
```python
def decode_telemetry(data):
    # Unpack header (16 bytes)
    packet_type, version, payload_len, flags, timestamp, seq = \
        struct.unpack('>BBHIII', data[:16])
    
    offset = 16
    
    # Check flags and decode groups
    if flags & TELEM_FLAG_MEASUREMENTS:
        measurements = struct.unpack('>7f', data[offset:offset+28])
        offset += 28
    
    if flags & TELEM_FLAG_OBSERVER:
        observer = struct.unpack('>3f', data[offset:offset+12])
        offset += 12
    
    # ... continue for other groups
    
    return decoded_data
```

## Configuration Options

### Selecting Signal Groups

To transmit only specific signal groups (reduce bandwidth):

```cpp
// Instead of build_full_packet(), use build_packet() with flags:
using namespace yeet::application::sfc::telemetry::telemetry_flags;

auto packet = builder.build_packet(
    timestamp_ms,
    measurements | observer  // Only measurements + observer
);
```

Available flags:
- `measurements` - Grid voltages, DC link current/voltage/power
- `observer` - PLL frequency, angle, error
- `modulator` - Firing angles, limits, commutation mode
- `controller` - Current reference, measured, error, saturation
- `state_machine` - State, faults, pulse releases

### Adjusting Transmission Rate

To change transmission rate, modify `decimation_factor`:

```cpp
static constexpr uint32_t decimation_factor = 20;  // 50 Hz transmission
```

Recommended rates:
- **10 Hz**: Low bandwidth monitoring (decimation = 100)
- **100 Hz**: Standard telemetry (decimation = 10) **[DEFAULT]**
- **1 kHz**: High-speed diagnostics (decimation = 1, not recommended for continuous operation)

## Testing Procedure

1. **Build and flash firmware**:
   ```powershell
   cmake --build --preset=embedded-cn-grid-release
   # Flash using ST-Link or J-Link
   ```

2. **Connect via network** (requires hardware network interface configured)

3. **Send START command** via TCP client (e.g., netcat, Python script)

4. **Verify packet reception**:
   - Check header magic bytes: `0x01` (grid-side telemetry)
   - Check protocol version: `0x01`
   - Validate payload length matches flags

5. **Parse and display data** using Python decoder

6. **Send STOP command** to cease transmission

## Future Enhancements

1. **Hardware timestamp**: Replace software counter with actual hardware timer (TIM2/TIM3)
2. **Python decoder GUI**: Real-time plotting and visualization tool
3. **UDP support**: Lower overhead for high-rate streaming
4. **Selective group commands**: Allow host to request specific signal groups
5. **Machine-side telemetry**: Extend to machine-side controller

## Troubleshooting

### No data received after START

- **Check TCP connection**: Verify `inputs->network_rx.tcp_connected` is true
- **Check streaming flag**: Set breakpoint on `telem_state.is_streaming`
- **Check decimation**: Ensure sufficient time has elapsed (100 ms for decimation=10)

### Corrupted packets

- **Validate header**: Check packet type, version, payload length
- **Check buffer size**: Ensure `tx_buffer` is large enough (minimum 108 bytes)
- **Verify encoding**: Compare encoded size with expected size

### High CPU load

- **Reduce transmission rate**: Increase `decimation_factor`
- **Select fewer signal groups**: Use `build_packet()` with specific flags
- **Profile builder**: Check which `populate_*` methods are expensive

### Signal values are zero

- **Check control system**: Ensure grid-side control is running (200µs, 1ms, 10ms tasks)
- **Check signal connections**: Verify internal signals are connected in `sfc_grid_side_control`
- **Check per-unit conversion**: Verify per-unit system is correctly configured

## References

- `network_telemetry_protocol.md` - Complete protocol specification
- `network_telemetry.hpp` - Protocol structures and encoding functions
- `network_telemetry_builder.hpp` - Packet construction helpers
- `sfc_io_definitions.md` - I/O interface signal definitions
