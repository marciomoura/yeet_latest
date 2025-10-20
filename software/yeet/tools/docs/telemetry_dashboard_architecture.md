# SFC Telemetry Dashboard Architecture

## Overview

The SFC Telemetry Dashboard provides a real-time monitoring interface for the Static Frequency Converter control system. It displays live telemetry data from the embedded grid-side controller with signals organized into logical functional groups.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    Embedded Controller                           │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐       │
│  │ Control Loop │──>│ Telemetry    │──>│ TCP Server   │       │
│  │ (100 Hz)     │   │ Builder      │   │ (Port 7000)  │       │
│  └──────────────┘   └──────────────┘   └──────┬───────┘       │
└─────────────────────────────────────────────────┼──────────────┘
                                                  │
                                    Binary Telemetry Protocol
                                    (72-byte packets @ 100 Hz)
                                                  │
┌─────────────────────────────────────────────────▼──────────────┐
│                    Host PC - Dashboard                          │
│  ┌────────────────────────────────────────────────────┐        │
│  │  telemetry_dashboard.py (Main Application)         │        │
│  │  ┌──────────────┐  ┌──────────────┐  ┌─────────┐  │        │
│  │  │ Connection   │  │ Telemetry    │  │ Signal  │  │        │
│  │  │ Controls     │  │ Controls     │  │ Displays│  │        │
│  │  └──────────────┘  └──────────────┘  └─────────┘  │        │
│  └────────────┬───────────────┬───────────────┬───────┘        │
│               │               │               │                 │
│  ┌────────────▼──────┐ ┌──────▼───────┐ ┌────▼────────────┐   │
│  │ Network Client    │ │ Signal       │ │ Protocol        │   │
│  │ - TCP connection  │ │ Manager      │ │ Decoder         │   │
│  │ - Receive thread  │ │ - Storage    │ │ - Binary parse  │   │
│  │ - Statistics      │ │ - Retrieval  │ │ - Validation    │   │
│  └───────────────────┘ └──────────────┘ └─────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## Module Structure

### Core Modules

#### 1. `telemetry_protocol.py`
**Purpose**: Binary protocol definition and decoding

**Key Components**:
- `PacketType`: Enum for packet type identification
- `TelemetryFlags`: Bit flags for signal group presence
- `TelemetryPacketHeader`: 16-byte fixed header structure
- Signal group dataclasses:
  - `GridMeasurementsGroup` (20 bytes)
  - `GridObserverGroup` (8 bytes)
  - `GridModulatorGroup` (12 bytes)
  - `GridControllerGroup` (12 bytes)
  - `StateMachineGroup` (4 bytes)
- `GridSideTelemetryPacket`: Complete packet container
- `TelemetryDecoder`: Binary data decoder

**Responsibilities**:
- Define protocol structures matching embedded C++ code
- Decode binary packets from network stream
- Validate packet headers and payload sizes
- Handle variable-length packets based on flags

**Example Usage**:
```python
decoder = TelemetryDecoder()
packet = decoder.decode(raw_bytes)
if packet and packet.measurements:
    voltage = packet.measurements.grid_voltage_magnitude_pu
    current = packet.measurements.dc_link_current_pu
```

#### 2. `telemetry_signal_manager.py`
**Purpose**: Signal data storage and management

**Key Components**:
- `SignalValue`: Single timestamped signal value
- `SignalHistory`: Thread-safe rolling buffer for time-series data
- `SignalGroup`: Collection of related signals
- `SignalManager`: Central coordinator for all signals

**Responsibilities**:
- Store signal histories in fixed-size rolling buffers
- Organize signals into logical groups (measurements, observer, etc.)
- Provide thread-safe access to signal data
- Extract data from telemetry packets and update buffers
- Support both latest-value and time-series retrieval

**Example Usage**:
```python
manager = SignalManager(buffer_size=10000)

# Update from packet
manager.update_from_packet(packet)

# Get latest value
voltage = manager.get_latest_value('measurements', 'grid_voltage_magnitude_pu')

# Get time series
times, values = manager.get_time_series('measurements', 'dc_link_current_pu')
```

#### 3. `telemetry_network_client.py`
**Purpose**: TCP communication and packet reception

**Key Components**:
- `TelemetryNetworkClient`: Main client class
- Background receive thread
- Packet callback system
- Reception statistics tracking

**Responsibilities**:
- Manage TCP socket connection to embedded controller
- Run continuous receive loop in background thread
- Buffer partial packets and extract complete packets
- Invoke callback for each decoded packet
- Track statistics (packets received/lost, bytes, errors)
- Send text commands (START/STOP) to controller

**Example Usage**:
```python
def on_packet_received(packet):
    print(f"Timestamp: {packet.header.timestamp_ms} ms")

client = TelemetryNetworkClient('192.168.2.100', 7000)
client.set_packet_callback(on_packet_received)

if client.connect():
    client.send_command('START')
    # Packets received in background...
    time.sleep(10)
    client.send_command('STOP')
    client.disconnect()
```

#### 4. `telemetry_dashboard.py`
**Purpose**: Main GUI application

**Key Components**:
- `SignalDisplayWidget`: Individual signal display (LCD-style)
- `SignalGroupWidget`: Group of related signal displays
- `DashboardWindow`: Main application window

**Responsibilities**:
- Provide user interface for connection management
- Display all signals in organized groups
- Update displays at high rate (20 Hz)
- Show connection statistics
- Handle user interactions (connect, start/stop, clear)
- Coordinate network client and signal manager

**Example Usage**:
```bash
# Run dashboard
python telemetry_dashboard.py --host 192.168.2.100 --port 7000

# With custom buffer size
python telemetry_dashboard.py --host 192.168.2.100 --port 7000 --buffer-size 20000
```

## Signal Organization

### Signal Groups

All telemetry signals are organized into logical groups matching the control system architecture:

#### 1. **Grid & DC Measurements**
- Grid Voltage Magnitude (pu)
- Grid Voltage Phase A (pu)
- Grid Voltage Phase B (pu)
- Grid Voltage Phase C (pu)
- DC Link Current (pu)

#### 2. **Grid Observer**
- Estimated Frequency (pu)
- Estimated Angle (rad)

#### 3. **Grid-Side Modulator**
- Firing Angle Reference (rad)
- Firing Angle Actual (rad)
- Force Commutation Acknowledgment (boolean)

#### 4. **Current Controller**
- Current Error (pu)
- Controller Output (pu)
- Operating Mode (0=forced, 1=natural)
- Controller Saturated (boolean)
- Freeze Integrator (boolean)

#### 5. **State Machine**
- Current State (enum)
- Pulse Releases (boolean)

## Threading Model

The dashboard uses multiple threads for responsive operation:

### Main UI Thread
- **Purpose**: Handle user interactions and update displays
- **Responsibilities**:
  - Process Qt events (button clicks, window management)
  - Update signal displays (20 Hz timer)
  - Update statistics display (2 Hz timer)
- **Thread-Safety**: Reads signal data via thread-safe manager methods

### Network Receive Thread
- **Purpose**: Continuous packet reception from embedded controller
- **Responsibilities**:
  - Receive binary data from TCP socket
  - Buffer partial packets until complete
  - Decode packets using `TelemetryDecoder`
  - Invoke packet callback
  - Update reception statistics
- **Thread-Safety**: Writes to signal manager using thread-safe methods

### Synchronization Strategy
- Signal manager uses locks for all data access
- Network statistics use dedicated lock
- UI thread only reads data (never writes)
- Network thread only writes data (never reads)
- No circular dependencies or deadlock potential

## Data Flow

### Complete Flow Sequence

```
1. Embedded Controller (100 Hz control loop)
   └─> Populate telemetry packet from control system
   └─> Encode to binary (network_telemetry_builder.cpp)
   └─> Transmit via TCP (grid_side_tasks.cpp)
   
2. Network Receive Thread
   └─> Receive binary data from TCP socket
   └─> Buffer until complete packet available
   └─> Decode packet (TelemetryDecoder)
   └─> Invoke callback with decoded packet
   
3. Packet Callback (network thread context)
   └─> Extract signal values from packet
   └─> Update SignalManager buffers (thread-safe)
   └─> Update reception statistics
   
4. UI Update Timer (20 Hz, main thread)
   └─> Query SignalManager for latest values
   └─> Update SignalDisplayWidget for each signal
   └─> Update connection statistics labels
```

## Protocol Details

### Binary Packet Structure

```
┌────────────────────────────────────────────┐
│ Header (16 bytes, always present)          │
├────────────────────────────────────────────┤
│ - packet_type (uint8)                      │
│ - protocol_version (uint8)                 │
│ - payload_length (uint16)                  │
│ - flags (uint32) - indicates groups below  │
│ - timestamp_ms (uint32)                    │
│ - sequence_number (uint32)                 │
├────────────────────────────────────────────┤
│ Measurements Group (20 bytes, if flag set) │
│ - grid_voltage_magnitude_pu (float)        │
│ - grid_voltage_a_pu (float)                │
│ - grid_voltage_b_pu (float)                │
│ - grid_voltage_c_pu (float)                │
│ - dc_link_current_pu (float)               │
├────────────────────────────────────────────┤
│ Observer Group (8 bytes, if flag set)      │
│ - estimated_frequency_pu (float)           │
│ - estimated_angle_rad (float)              │
├────────────────────────────────────────────┤
│ Modulator Group (12 bytes, if flag set)    │
│ - firing_angle_reference_rad (float)       │
│ - firing_angle_actual_rad (float)          │
│ - force_commutation_ack (uint8)            │
│ - reserved_1 (uint8)                       │
│ - reserved_2 (uint8)                       │
│ - reserved_3 (uint8)                       │
├────────────────────────────────────────────┤
│ Controller Group (12 bytes, if flag set)   │
│ - current_error_pu (float)                 │
│ - controller_output_pu (float)             │
│ - operating_mode (uint8)                   │
│ - controller_saturated (uint8)             │
│ - freeze_integrator (uint8)                │
│ - reserved (uint8)                         │
├────────────────────────────────────────────┤
│ State Machine Group (4 bytes, if flag set) │
│ - current_state (uint8)                    │
│ - pulse_releases (uint8)                   │
│ - reserved_1 (uint8)                       │
│ - reserved_2 (uint8)                       │
└────────────────────────────────────────────┘

Total Size: 16 + 20 + 8 + 12 + 12 + 4 = 72 bytes
           (when all groups present)
```

### Endianness
- **Little-endian** for all multi-byte fields
- Matches ARM Cortex-M4 native byte order
- Python `struct` module uses `<` prefix for little-endian

### Packet Loss Detection
- Each packet contains incrementing `sequence_number`
- Client detects gaps in sequence numbers
- Lost packet count tracked in statistics
- Handles 32-bit wraparound correctly

## Extension Points

The architecture is designed for easy extension:

### Adding New Signal Groups

1. **Embedded Side** (`network_telemetry.hpp`):
   ```cpp
   struct new_signal_group {
       float signal_1;
       float signal_2;
       // ...
   };
   ```

2. **Protocol Module** (`telemetry_protocol.py`):
   ```python
   @dataclass
   class NewSignalGroup:
       signal_1: float
       signal_2: float
   ```

3. **Signal Manager** (`telemetry_signal_manager.py`):
   ```python
   new_group = SignalGroup('new_group', 'New Group Display Name')
   new_group.add_signal(SignalHistory('signal_1', 'unit', 'Display Name', buffer_size))
   ```

4. **Dashboard UI** (`telemetry_dashboard.py`):
   ```python
   new_widget = SignalGroupWidget('new_group', 'New Group Display Name')
   new_widget.add_signal_display('signal_1', 'Display Name', 'unit')
   ```

### Adding Plotting Capabilities

The `SignalHistory` class already stores time-series data in rolling buffers. To add plotting:

1. Create `TelemetryPlotter` widget using `pyqtgraph`
2. Query `SignalManager.get_time_series()` for data
3. Update plot in separate timer (e.g., 10 Hz)
4. Add plot widget to dashboard layout

### Adding Data Logging

1. Create `TelemetryLogger` class
2. Register as packet callback alongside signal manager
3. Write packets to file (CSV, HDF5, or binary)
4. Include timestamp, sequence number, and all signals

### Adding Remote Control

The network client already sends text commands. To extend:

1. Define new commands in embedded `network_command_parser.cpp`
2. Add UI controls to dashboard for new commands
3. Call `network_client.send_command()` with new command strings

## Performance Characteristics

### Resource Usage

**Memory** (Host PC):
- Signal manager: ~400 KB per 10,000 samples (25 signals × 16 bytes/sample)
- Network buffers: ~8 KB
- UI objects: ~50 KB
- Total: < 1 MB for typical configuration

**CPU** (Host PC):
- Network thread: < 1% (100 Hz packet rate)
- UI update: < 2% (20 Hz display update)
- Total: < 5% on typical modern CPU

**Network Bandwidth**:
- Packet size: 72 bytes
- Rate: 100 Hz
- Bandwidth: 57.6 kbps (negligible for modern networks)

### Update Rates

- **Control loop**: 100 Hz (embedded)
- **Telemetry transmission**: 100 Hz (with decimation factor 10)
- **Network reception**: Continuous (background thread)
- **UI signal displays**: 20 Hz (configurable via `UI_UPDATE_INTERVAL_MS`)
- **Statistics display**: 2 Hz (configurable via `STATS_UPDATE_INTERVAL_MS`)

### Latency

- **End-to-end**: < 50 ms typical (control loop → UI display)
  - Embedded encoding: < 1 ms
  - Network transmission: < 10 ms (LAN)
  - Decoding + update: < 1 ms
  - UI refresh: 0-50 ms (depends on timer phase)

## Testing Strategy

### Unit Testing

Test each module independently:

1. **Protocol Decoder**:
   - Valid packet decoding
   - Invalid packet rejection
   - Partial packet handling
   - All signal group combinations

2. **Signal Manager**:
   - Buffer overflow handling
   - Thread-safe access
   - Signal retrieval
   - Statistics calculation

3. **Network Client**:
   - Connection management
   - Command transmission
   - Packet loss detection
   - Statistics tracking

### Integration Testing

1. **Loopback Test**: Run embedded simulator on same PC, connect dashboard
2. **Hardware Test**: Connect to actual embedded controller
3. **Stress Test**: High packet rate, verify no drops or UI freezing
4. **Long-Run Test**: 24-hour continuous operation

### User Acceptance Testing

1. Connect to embedded hardware
2. Start telemetry streaming
3. Verify all signals update in real-time
4. Check statistics (loss rate < 0.1%)
5. Stop streaming, verify signals freeze
6. Clear data, verify displays reset

## Troubleshooting

### No Connection
- **Check**: Controller IP address and port number
- **Check**: Network cable connected
- **Check**: Firewall settings (allow TCP port 7000)
- **Test**: `ping <controller-ip>` from command line

### No Data After START Command
- **Check**: START command sent successfully (see log)
- **Check**: Controller state machine allows telemetry
- **Check**: Network receive thread running (check logs)
- **Debug**: Enable verbose logging (`logging.DEBUG` level)

### High Packet Loss
- **Check**: Network congestion or poor WiFi signal
- **Check**: CPU usage on controller (control loop priority)
- **Check**: Statistics for specific sequence number patterns
- **Solution**: Use wired Ethernet connection

### UI Freezing
- **Check**: Callback function not blocking (should be fast)
- **Check**: UI update rate not too high (reduce if needed)
- **Solution**: Move heavy processing out of UI thread

## Future Enhancements

### Phase 2: Plotting & Visualization
- Add real-time waveform plots for selected signals
- Oscilloscope-style triggering and cursors
- FFT analysis for frequency domain view
- XY plots for phase relationships

### Phase 3: Data Logging
- Record telemetry to file (CSV, HDF5)
- Playback recorded data for post-analysis
- Export selected time windows
- Automatic logging triggers (fault conditions)

### Phase 4: Advanced Features
- Multiple controller support (grid + machine side)
- Parameter tuning interface (send configuration commands)
- Alarm thresholds and notifications
- Remote firmware update capability

## References

- **Embedded Protocol**: `software/yeet_application/sfc_embedded/docs/network_telemetry_protocol.md`
- **Integration Guide**: `software/yeet_application/sfc_embedded/docs/network_telemetry_integration.md`
- **Architecture**: `software/yeet_application/sfc_embedded/docs/network_command_telemetry_architecture.md`
- **PyQt5 Documentation**: https://www.riverbankcomputing.com/static/Docs/PyQt5/
- **Python struct module**: https://docs.python.org/3/library/struct.html
