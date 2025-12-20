# YEET Platform

**Y**our **E**ngineering **E**cosystem for **T**esting - A software platform for power electronics control systems, particularly Load-Commutated Inverters (LCI) and thyristor-based converters.

## Features

- Real-time control algorithms (PI/PID, filters, PLLs, reference frame transforms)
- Hardware-in-the-loop testing with embedded targets (STM32)
- Closed-loop simulation and validation framework
- C++17 with deterministic, real-time safe execution

## Quick Start

### Build and Test (Host)

```bash
# Configure and build
cmake --preset host-gcc
cmake --build --preset host-gcc-debug

# Run tests
ctest --test-dir build-host-gcc -C Debug
```

### Build Embedded Firmware

```bash
# STM32F407 Discovery board
cmake --preset embedded-gcc
cmake --build --preset embedded-gcc-release
```

## Software Architecture

The YEET platform is organized in layered architecture from foundational libraries to application-specific implementations:

### Core Platform Layers

#### **[YEET Core](software/yeet/docs/index.md)**
Foundational infrastructure for real-time control applications
- Application data models and signal definitions
- Control execution policies and task scheduling
- Network telemetry and command interfaces
- Parameter management and configuration

#### **[YEET Testing](software/yeet_testing/docs/index.md)**
Comprehensive testing and validation framework
- Software-in-the-Loop (SIL) simulation infrastructure
- Automated data sampling and post-processing
- Verification and validation tools
- Google Test integration for CI/CD

#### **[YEET Control](software/yeet_control/docs/index.md)**
Generic, reusable control algorithms and components
- PI/PID controllers with anti-windup
- Digital filters and signal processing
- Phase-Locked Loops (PLLs) for synchronization
- Reference frame transformations (abc, αβ, dq)
- Mechanical load models and pulse generators

### Application Layer - Static Frequency Converter (SFC)

#### **[SFC Application](software/yeet_application/sfc/docs/index.md)**
Load-Commutated Inverter (LCI) control application
- LCI system theory and mathematical models
- Current and speed control algorithms
- Per-unit system and steady-state equations
- Terminology dictionary and design guidelines

#### **[SFC Integration](software/yeet_application/sfc_integration/docs/index.md)**
Platform integration and data model definitions
- Data model interconnection architecture
- Runner implementations for host and embedded platforms
- Signal mapping and execution synchronization
- Testing and simulation interfaces

#### **[SFC Embedded](software/yeet_application/sfc_embedded/docs/index.md)**
Embedded hardware implementation
- STM32 and custom board support
- Real-time I/O and interrupt management
- Hardware abstraction and driver layer
- Timing, memory, and performance optimization

## Prerequisites

- **Host builds**: CMake 3.20+, GCC/Clang, Ninja
- **Embedded builds**: GNU Arm Embedded Toolchain
- **Python tools**: Python 3.8+, PyQt5, pyqtgraph, numpy (for monitor GUI)

## Running the SFC Simulator and Monitor

The YEET platform includes a complete closed-loop simulation environment with plant model and real-time telemetry streaming.

### Starting the Host Simulator

The SFC host simulator runs both grid-side and machine-side controllers with a complete plant model (6-pulse thyristor bridges, synchronous machine, DC link). It exposes dual TCP servers for independent telemetry streaming.

```bash
# Build the simulator
cmake --build --preset host-gcc-debug

# Run with default settings (grid: port 7000, machine: port 7001, 5 kHz control)
./build-host/bin/sfc_host_simulator_with_plant

# Run with verbose logging enabled
./build-host/bin/sfc_host_simulator_with_plant --verbose

# Run with custom settings
./build-host/bin/sfc_host_simulator_with_plant --grid-port 7000 --machine-port 7001 --frequency 10000 --verbose
```

**Simulator Options:**
- `--grid-port PORT` - Grid-side telemetry TCP port (default: 7000)
- `--machine-port PORT` - Machine-side telemetry TCP port (default: 7001)
- `--frequency HZ` - Control loop frequency in Hz (default: 5000.0)
- `--verbose` - Enable detailed console logging (initialization, statistics, warnings)
- `--help, -h` - Show help message

**Note:** Telemetry always runs at 50µs (20 kHz) regardless of control frequency. The simulator runs as fast as possible without real-time pacing.

### Using the Telemetry Monitor

The Python-based monitor provides real-time visualization and parameter configuration for the SFC system.

```bash
# Navigate to tools directory
cd software/yeet/tools

# Run grid-side monitor (default)
python monitor.py --host localhost --port 7000

# Run machine-side monitor
python monitor.py --host localhost --port 7001 --target machine

# Run with custom settings
python monitor.py --host 192.168.2.100 --port 7000 --target grid --window 2000 --decimate 5 --verbose

# Enable telemetry debug logging to file
python monitor.py --host localhost --port 7000 --debug-log ./telemetry_debug.log
```

**Monitor Options:**
- `--host HOST` - Controller IP address (default: 192.168.2.100)
- `--port PORT` - TCP port for telemetry (auto-selected based on --target if not specified)
- `--target {grid|machine}` - Target side: 'grid' (port 7000) or 'machine' (port 7001) [default: grid]
- `--window SIZE` - Number of samples to display on X-axis (default: 1000)
- `--decimate N` - Process 1 out of N packets for reduced CPU load (default: 1, no decimation)
- `--verbose` - Enable verbose console logging (connection events, packet stats)
- `--debug-log FILE` - Enable detailed telemetry decoding logs to specified file (e.g., `./telemetry_debug.log`)

**Monitor Features:**
- **Left Panel:** Parameter list with load button and double-click editing
- **Center:** Four real-time oscilloscope plots (voltage, currents, observer states, DC link control)
- **Right Panel:** Live telemetry dashboard with state machine, measurements, and protections

### Typical Workflow

1. **Start the simulator:**
   ```bash
   ./build-host/bin/sfc_host_simulator_with_plant --verbose
   ```

2. **Connect the grid-side monitor:**
   ```bash
   cd software/yeet/tools
   python monitor.py --host localhost --port 7000 --target grid
   ```

3. **Connect the machine-side monitor** (optional, in separate terminal):
   ```bash
   python monitor.py --host localhost --port 7001 --target machine
   ```

4. **Send start command** via telemetry or parameter interface to begin operation

5. **Monitor real-time plots** and adjust parameters through the GUI as needed

### Troubleshooting

**Simulator Issues:**
- If you see excessive console output, run without `--verbose` flag
- Connection refused: Ensure simulator is running and ports are not in use
- High CPU usage: The simulator runs as fast as possible; this is expected behavior

**Monitor Issues:**
- Connection timeout: Check host IP and port, ensure simulator is running
- Choppy plots: Increase `--decimate` value (e.g., `--decimate 5` processes every 5th packet)
- Decoding errors: Enable debug logging with `--debug-log telemetry.log` to diagnose protocol issues
- Memory usage: Reduce `--window` size to limit buffered samples

## License

[License information to be added]