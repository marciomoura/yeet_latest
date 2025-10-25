# SFC Telemetry Dashboard

Real-time monitoring dashboard for the Static Frequency Converter (SFC) control system.

## Overview

The telemetry dashboard provides live visualization of control system signals from the embedded grid-side controller. Signals are organized into logical groups (measurements, observer, controller, etc.) and displayed in digital LCD-style formats with automatic real-time updates.

## Features

✅ **Real-Time Display**: 20 Hz update rate for all signals  
✅ **Organized Groups**: Signals grouped by function (5 groups, 17 signals total)  
✅ **Network Statistics**: Packet loss detection and monitoring  
✅ **Clean Architecture**: Modular design with separate protocol, network, and UI layers  
✅ **Thread-Safe**: Proper synchronization between network and UI threads  
✅ **Extensible**: Ready for plotting, logging, and control features  

## Quick Start

### Installation

```bash
# Install Python dependency
pip install PyQt5

# Navigate to tools directory
cd software/yeet/tools
```

### Run Dashboard

```bash
# Default connection (192.168.2.100:7000)
python telemetry_dashboard.py

# Custom IP and port
python telemetry_dashboard.py --host 192.168.1.100 --port 7000

# Large buffer for extended recording
python telemetry_dashboard.py --buffer-size 50000
```

### Basic Usage

1. **Connect**: Enter controller IP/port and click "Connect"
2. **Start Streaming**: Click "Start Streaming" to begin telemetry
3. **Monitor**: Watch signals update in real-time at 20 Hz
4. **Stop**: Click "Stop Streaming" when done
5. **Disconnect**: Click "Disconnect" to close connection

## Signal Groups

### Grid & DC Measurements (5 signals)
- Grid Voltage Magnitude (pu)
- Grid Voltage Phase A, B, C (pu)
- DC Link Current (pu)

### Grid Observer (2 signals)
- Estimated Frequency (pu)
- Estimated Angle (rad)

### Grid-Side Modulator (3 signals)
- Firing Angle Reference (rad)
- Firing Angle Actual (rad)
- Force Commutation Acknowledgment

### Current Controller (5 signals)
- Current Error (pu)
- Controller Output (pu)
- Operating Mode (0=forced, 1=natural)
- Controller Saturated flag
- Freeze Integrator flag

### State Machine (2 signals)
- Current State (enum)
- Pulse Releases flag

## File Structure

```
software/yeet/tools/
├── telemetry_dashboard.py          # Main GUI application
├── telemetry_protocol.py           # Binary protocol decoder
├── telemetry_signal_manager.py     # Signal storage and retrieval
├── telemetry_network_client.py     # TCP network communication
├── test_telemetry_system.py        # Component tests
└── docs/
    ├── telemetry_dashboard_architecture.md      # Architecture guide
    ├── telemetry_dashboard_user_guide.md        # User manual
    └── telemetry_dashboard_implementation_summary.md  # Implementation details
```

## Module Overview

### `telemetry_dashboard.py` (680 lines)
Main PyQt5 GUI application with connection controls, signal displays, and statistics.

**Key Classes**:
- `SignalDisplayWidget`: LCD-style digital display for individual signals
- `SignalGroupWidget`: Group of related signal displays  
- `DashboardWindow`: Main application window

### `telemetry_protocol.py` (382 lines)
Binary protocol definition and decoder matching embedded C++ structures.

**Key Classes**:
- `TelemetryPacketHeader`: 16-byte fixed header
- Signal group dataclasses (measurements, observer, modulator, controller, state_machine)
- `TelemetryDecoder`: Binary packet decoder

### `telemetry_signal_manager.py` (420 lines)
Thread-safe signal data storage with rolling buffers.

**Key Classes**:
- `SignalHistory`: Rolling buffer for time-series data
- `SignalGroup`: Collection of related signals
- `SignalManager`: Central coordinator for all signals

### `telemetry_network_client.py` (320 lines)
TCP network communication with background receive thread.

**Key Classes**:
- `TelemetryNetworkClient`: TCP connection, packet reception, statistics

## Testing

Run component tests:

```bash
python test_telemetry_system.py
```

Tests verify:
- ✅ Protocol encoder/decoder
- ✅ Signal manager storage and retrieval
- ✅ Partial packet decoding
- ✅ Invalid packet rejection

Expected output:
```
🎉 All tests PASSED! Dashboard components working correctly.
```

## Architecture

```
┌─────────────────────────────────────────┐
│       Embedded Controller               │
│  Control Loop → Telemetry → TCP Server │
└────────────────┬────────────────────────┘
                 │ 72-byte packets @ 100 Hz
┌────────────────▼────────────────────────┐
│         Host PC - Dashboard             │
│  ┌─────────────────────────────────┐   │
│  │  telemetry_dashboard.py         │   │
│  │  (Main GUI Application)         │   │
│  └──┬───────────┬──────────────┬───┘   │
│     │           │              │        │
│  ┌──▼──────┐ ┌─▼──────────┐ ┌─▼──────┐│
│  │Network  │ │  Signal    │ │Protocol││
│  │Client   │ │  Manager   │ │Decoder ││
│  └─────────┘ └────────────┘ └────────┘│
└─────────────────────────────────────────┘
```

## Performance

- **Memory**: ~500 KB (10,000 sample buffer)
- **CPU**: <5% on typical PC
- **Network**: 57.6 kbps @ 100 Hz
- **End-to-End Latency**: <50 ms

## Documentation

- **Architecture Guide**: `docs/telemetry_dashboard_architecture.md`  
  Complete system architecture, threading model, protocol details, extension points

- **User Guide**: `docs/telemetry_dashboard_user_guide.md`  
  Usage instructions, troubleshooting, signal interpretation, FAQ

- **Implementation Summary**: `docs/telemetry_dashboard_implementation_summary.md`  
  Implementation details, design decisions, file structure

## Future Enhancements

### Phase 2: Plotting & Visualization
- Real-time waveform plots using pyqtgraph
- Multiple signals on same plot
- FFT analysis for frequency domain
- XY plots for phase relationships

### Phase 3: Data Logging
- Record telemetry to file (CSV, HDF5)
- Playback recorded data
- Export selected time windows
- Automatic logging on fault conditions

### Phase 4: Advanced Features
- Multiple controller support (grid + machine)
- Parameter tuning interface
- Alarm thresholds and notifications
- Remote firmware update

## Dependencies

- **Python**: 3.7+ (3.9+ recommended)
- **PyQt5**: GUI framework (`pip install PyQt5`)
- **Standard Library**: struct, socket, threading, dataclasses, logging

## Troubleshooting

### Connection Failed
- Verify controller is powered and network interface up
- Check IP address and port number (default: 192.168.2.100:7000)
- Test with: `ping <controller-ip>`

### No Data After START
- Check controller state machine allows telemetry
- Verify control loop is running
- Enable debug logging in code

### High Packet Loss
- Use wired Ethernet instead of WiFi
- Check network congestion
- Verify controller CPU not overloaded

See user guide for detailed troubleshooting.

## License

Part of the YEET project - Internal use only.

## Contact

For issues or questions, contact the YEET development team.

---

**Status**: ✅ Production Ready  
**Version**: 1.0  
**Last Updated**: October 2025
