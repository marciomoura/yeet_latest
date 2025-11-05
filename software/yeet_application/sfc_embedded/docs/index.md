# YEET SFC Embedded Layer Documentation

## Overview

The SFC Embedded Layer provides the hardware abstraction, low-level drivers, and platform-specific implementations required to run the SFC application on embedded targets. This layer handles real-time I/O, interrupt management, and hardware-specific optimizations for deterministic control execution.

## Purpose

The embedded layer is responsible for:
- **Hardware Abstraction**: Platform-independent interface to MCU peripherals
- **Real-Time I/O**: Deterministic reading of sensors and writing to actuators
- **Interrupt Management**: Prioritized interrupt handling for control timing
- **Board Support**: Target-specific initialization and configuration

## Supported Platforms

### STM32F407 Discovery
- Primary development and testing platform
- ARM Cortex-M4F with FPU
- 168 MHz operation
- Comprehensive peripheral set

### Custom Power Electronics Boards
- Grid-side converter control board
- Machine-side converter control board
- Integrated monitoring and protection

## Key Components

### Hardware Abstraction Layer (HAL)

#### GPIO Control
- Digital I/O for enable signals and status indicators
- Fault input monitoring
- LED status indication

#### Analog Input (ADC)
- High-speed current and voltage measurements
- Multi-channel synchronized sampling
- DMA-based data transfer for minimal CPU overhead

#### Timer Peripherals
- PWM generation for gate drive signals
- Encoder interface for position sensing
- Time-base generation for control loops

#### Communication Interfaces
- UART for command and telemetry
- SPI for fast data exchange
- Ethernet for network telemetry (on supported platforms)

### Real-Time Control Execution

#### Interrupt Service Routines (ISRs)
- **High-Priority Control ISR**: Main control loop execution
- **ADC Completion ISR**: Synchronized measurement acquisition
- **Communication ISRs**: Non-blocking telemetry transmission

#### Task Scheduling
- Deterministic execution at fixed sampling rates
- Multi-rate control support
- Minimal jitter and latency

### Board Support Packages (BSP)

#### Initialization
- Clock configuration
- Peripheral initialization
- Memory setup (Flash, RAM, CCM)

#### Resource Mapping
- Pin assignments for I/O
- DMA channel allocation
- Interrupt priority configuration

## I/O Architecture

### Input Signals

#### Analog Measurements
- Grid voltages (3-phase)
- Grid currents (3-phase)
- DC link current
- DC link voltage
- Machine voltages (3-phase)
- Machine currents (3-phase)

#### Digital Inputs
- Enable/disable commands
- Fault signals
- Limit switches
- Emergency stop

#### Encoder/Position
- Rotor position (optional)
- Auxiliary feedback signals

### Output Signals

#### Gate Drive Signals
- Grid-side thyristor firing pulses
- Machine-side thyristor firing pulses
- Pulse timing with microsecond precision

#### Digital Outputs
- Status indicators
- Relay controls
- Communication with auxiliary systems

#### Analog Outputs (optional)
- Test signals
- Analog monitoring

## Real-Time Considerations

### Timing Requirements
- **Control Loop Rate**: 10 kHz typical (100 µs period)
- **Firing Pulse Precision**: ±1 µs accuracy
- **ADC Sampling**: Synchronized to control loop
- **Communication**: Non-blocking, background operation

### Interrupt Priorities
1. **Highest**: Critical fault detection
2. **High**: Main control loop ISR
3. **Medium**: ADC completion and data ready
4. **Low**: Communication and telemetry
5. **Lowest**: Background tasks and monitoring

### Memory Management
- **Flash**: Program code and constants
- **RAM**: Variables and stack
- **CCM (Core-Coupled Memory)**: High-speed critical data
- **DMA Buffers**: Pre-allocated for deterministic operation

## Platform-Specific Optimizations

### ARM Cortex-M4F Features
- **FPU**: Hardware floating-point for control calculations
- **DSP Instructions**: SIMD operations for vector math
- **Bit-Banding**: Atomic bit operations for flags
- **MPU**: Memory protection for safety-critical code

### Clock Configuration
- PLL configuration for maximum performance
- Peripheral clock optimization
- Power management for reduced EMI

## Safety and Protection

### Fault Detection
- Overcurrent detection
- Overvoltage detection
- Communication timeout
- Watchdog timer

### Safe State Management
- Immediate gate drive disable on fault
- Predictable fault response
- Non-volatile fault logging

### Diagnostics
- Built-in self-test (BIST)
- Peripheral health monitoring
- Real-time performance metrics

## Building and Flashing

### Build Configuration

```bash
# Configure for embedded target
cmake --preset embedded-gcc

# Build release firmware
cmake --build --preset embedded-gcc-release

# Build debug firmware with symbols
cmake --build --preset embedded-gcc-debug
```

### Flashing Firmware

```bash
# Flash via ST-Link
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program build-embedded/firmware.elf verify reset exit"

# Flash via UART bootloader
stm32flash -w build-embedded/firmware.bin /dev/ttyUSB0
```

### Debugging

```bash
# Start OpenOCD server
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# Connect with GDB
arm-none-eabi-gdb build-embedded/firmware.elf
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) continue
```

## Integration with Other Layers

### SFC Integration Layer
The embedded layer implements the hardware interface expected by the integration layer:
- Provides real-time I/O functions
- Implements timer callbacks for control execution
- Handles interrupt-driven data acquisition

### SFC Application Layer
The application runs unchanged on embedded hardware through:
- Platform-independent data model
- Deterministic execution timing
- Hardware abstraction

### Board Support (boards/)
Uses board-specific definitions from the `boards/` directory:
- Pin mappings
- Clock configurations
- Peripheral assignments

## Documentation

The embedded layer includes detailed documentation on:
- **[SFC Architecture](sfc_architecture.md)**: Overall embedded system architecture
- **[I/O Definitions](sfc_io_definitions.md)**: Complete signal mapping and specifications
- **[Network Telemetry](network_telemetry_integration.md)**: Embedded telemetry implementation
- **[Fiber Optic Protocol](fiber_optic_protocol.md)**: High-speed communication between boards
- **[Testing with Host Simulator](testing_host_simulator.md)**: Development and validation workflow

## Testing Strategy

### Unit Tests
- Individual driver validation
- Peripheral initialization checks
- I/O boundary testing

### Integration Tests
- Full control loop execution on target
- Timing verification
- Resource usage validation

### Hardware-in-the-Loop (HIL)
- Real-time performance validation
- Fault injection testing
- Long-term stability testing

## Performance Metrics

### CPU Usage
- Control loop: < 30% at 10 kHz
- Communication: < 10% average
- Background tasks: < 5%
- Idle time: > 50% for margin

### Memory Usage
- Flash: ~80 KB typical
- RAM: ~20 KB typical
- Stack: 4 KB per task
- Heap: Not used in real-time code

### Timing Metrics
- Control loop jitter: < 1 µs
- Interrupt latency: < 2 µs
- End-to-end latency: < 5 µs (sensor to actuator)

## See Also

- [YEET Core Library](../../yeet/docs/index.md)
- [SFC Application](../sfc/docs/index.md)
- [SFC Integration](../sfc_integration/docs/index.md)
- [Board Support Packages](../../../../boards/README.md)
