# Monitor Client

A professional telemetry monitoring and parameter configuration application for embedded LCI (Load-Commutated Inverter) control systems.

## Features

- **Real-Time Telemetry Monitoring**: Visualize control signals in real-time
- **Three Synchronized Oscilloscope Plots**: Display waveforms across multiple channels
- **Digital Dashboard**: View current signal values organized by functional groups
- **Parameter Discovery**: Automatically discover all configurable parameters
- **Live Parameter Editing**: Modify parameters in real-time with validation
- **Component Filtering**: Filter parameters by component (e.g., GRID, MACHINE)
- **Search Functionality**: Quickly find parameters by name
- **Connection Management**: Robust connection handling with automatic reconnection
- **Data Export**: Save telemetry data for offline analysis

## Architecture

```
monitor_client/
├── __init__.py          # Package initialization
├── __main__.py          # Entry point for python -m monitor_client
├── core/                # Network communication and protocols
│   ├── __init__.py
│   ├── telemetry_network_client.py    # TCP telemetry streaming
│   ├── telemetry_protocol.py          # Telemetry packet definitions
│   ├── parameter_network_client.py    # Parameter discovery/configuration
│   └── parameter_protocol.py          # Parameter packet definitions
├── ui/                  # User interface components
│   ├── __init__.py
│   └── main_window.py   # Main application window
└── utils/               # Utility modules
    ├── __init__.py
    └── telemetry_signal_manager.py    # Signal data management
```

## Installation

### Prerequisites

- Python 3.7 or later
- PyQt5
- pyqtgraph
- numpy

### Install Dependencies

```bash
pip install PyQt5 pyqtgraph numpy
```

Or if using the workspace requirements:

```bash
pip install -r python_requirements.txt
```

## Usage

### Quick Start

From the `software/yeet/tools/` directory:

```bash
python monitor.py
```

This will launch the monitor client with default settings:
- Host: 192.168.2.100
- Port: 7000
- Time Window: 5.0 seconds

### Command-Line Options

```bash
python monitor.py --host 192.168.2.100 --port 7000 --window 5.0 --verbose
```

**Options:**
- `--host HOST`: MCU IP address (default: 192.168.2.100)
- `--port PORT`: MCU telemetry port (default: 7000)
- `--window SECONDS`: Oscilloscope time window in seconds (default: 5.0)
- `--verbose`: Enable verbose logging for debugging

### Alternative Launch Methods

**As a Python module:**

```bash
python -m monitor_client --host 192.168.2.100
```

**Direct execution:**

```bash
cd software/yeet/tools/monitor_client
python -m monitor_client
```

## User Interface

### Dashboard Panel (Top Left)

Displays real-time digital values for all telemetry signals, organized into functional groups:
- **Grid-Side Control**: Line-side converter signals
- **Machine-Side Control**: Machine-side converter signals
- **DC Link**: DC link current and voltage
- **System Status**: Overall system state and health

### Oscilloscope Panel (Top Right)

Three synchronized subplots showing real-time waveforms:
- **Plot 1**: Primary control signals
- **Plot 2**: Secondary control signals
- **Plot 3**: Additional monitoring signals

Features:
- Auto-scrolling time window
- Zoom and pan with mouse
- Signal selection dropdowns
- Configurable time window

### Parameter Panel (Bottom)

Live parameter configuration interface:
- **Search Bar**: Filter parameters by name
- **Component Filter**: Filter by component (GRID, MACHINE, etc.)
- **Parameter Table**: Shows name, value, type, and units
- **Live Editing**: Double-click values to edit
- **Refresh Button**: Reload parameters from MCU

## Network Protocol

### Telemetry Streaming

- **Protocol**: TCP
- **Port**: 7000 (default)
- **Packet Format**: Binary (`GridSideTelemetryPacket`)
- **Update Rate**: Configurable on MCU (typically 1 kHz)

### Parameter Protocol

- **Commands**: COUNT, GET_BY_INDEX, SET_VALUE
- **Transport**: TCP (same connection as telemetry)
- **Format**: Binary with checksums
- **Features**: Automatic discovery, type validation, range checking

## Development

### Project Structure

The monitor client follows a clean separation of concerns:

- **core/**: Low-level network and protocol handling
  - Independent of UI framework
  - Reusable in other applications
  - Handles connection management, packet parsing, checksums

- **ui/**: User interface components
  - PyQt5-based GUI
  - Integrates core modules
  - Implements visualization and interaction

- **utils/**: Shared utilities
  - Signal data management
  - Helper functions

### Adding New Features

1. **New Telemetry Signals**: Update `telemetry_protocol.py` packet definition
2. **New Parameters**: Parameters are auto-discovered; no code changes needed
3. **Custom Plots**: Modify `main_window.py` oscilloscope setup
4. **New UI Components**: Add to `ui/` directory

### Testing

To test the monitor client:

1. Ensure embedded controller is running and connected
2. Launch monitor client: `python monitor.py --verbose`
3. Check connection status in title bar
4. Verify telemetry data is streaming
5. Test parameter discovery and editing

## Troubleshooting

### Connection Failed

- Verify MCU is powered and network cable is connected
- Check IP address: `ping 192.168.2.100`
- Confirm firewall allows TCP connections to port 7000
- Check if another application is using the port

### No Telemetry Data

- Ensure streaming is started (check MCU state)
- Verify MCU telemetry task is running
- Check network traffic with Wireshark
- Enable `--verbose` logging to see packet reception

### Parameter Loading Fails

- Verify parameter handler is initialized on MCU
- Check for timeout errors in verbose log
- Ensure GET_BY_INDEX packet handling is correct
- Try manually sending COUNT command to verify response

### UI Performance Issues

- Reduce time window: `--window 2.0`
- Close other applications using GPU
- Update graphics drivers
- Reduce number of visible signals in oscilloscope

## License

Copyright © 2025 Techperts. All rights reserved.

## Support

For issues, questions, or contributions, please contact the development team.
