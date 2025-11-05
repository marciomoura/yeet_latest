# YEET SFC Integration Layer Documentation

## Overview

The SFC Integration Layer bridges the gap between the generic control algorithms and the specific hardware or simulation platforms. It defines the data models, runner implementations, and interconnections that enable the SFC application to execute on both host simulators and embedded targets.

## Purpose

The integration layer serves as the "glue code" that:
- **Connects** generic control algorithms to platform-specific I/O
- **Defines** the complete data model for the SFC application
- **Implements** execution runners for different platforms
- **Provides** testability through data model abstraction

## Key Components

### Data Model Definitions

#### SFC Data Model
The complete data model for the Static Frequency Converter application, including:
- Grid-side measurements and controls
- DC link measurements and references
- Machine-side measurements and controls
- Protection and status signals
- Configuration parameters

**Key Files:**
- Data model headers defining signals and parameters
- Interconnection definitions between layers

### Runner Implementations

#### Host Simulator Runner
- Executes SFC control on host platform
- Interfaces with simulation models
- Supports closed-loop testing

#### Embedded Runner
- Executes SFC control on embedded hardware
- Interfaces with real-time I/O
- Optimized for deterministic execution

### Interconnection Architecture

The integration layer implements the architecture defined in the [Data Model Interconnection](datamodel_interconnection.md) document, which establishes:
- **Data flow** between application layers
- **Signal mapping** from hardware to control algorithms
- **Parameter propagation** across layers
- **Execution synchronization** between control tasks

## Architecture Principles

### Platform Independence
- Control algorithms remain platform-agnostic
- Integration layer handles platform-specific details
- Same control code runs on host and embedded targets

### Data Model Abstraction
- All I/O goes through defined data model interfaces
- No direct hardware access in control algorithms
- Testability through data model injection

### Execution Policy
- Deterministic task scheduling
- Multi-rate control execution
- Synchronization between control levels

## Integration with Other Layers

### SFC Application Layer
The integration layer executes the SFC application by:
- Instantiating control algorithms
- Connecting data model signals
- Implementing execution policies

### SFC Embedded Layer
The integration layer interfaces with embedded hardware through:
- Hardware abstraction provided by embedded layer
- Real-time I/O mapping
- Interrupt-driven execution

### YEET Core Library
Uses core infrastructure for:
- Application data model framework
- Control execution policies
- Network telemetry and commands

### YEET Testing Framework
Enables comprehensive testing through:
- Simulator runner implementations
- Data model injection for test scenarios
- Integration with automated verification

## Data Model Structure

The SFC data model is organized into functional groups:

### Grid Side (Source)
- Grid voltages (abc)
- Grid currents (abc)
- Source-side firing angles
- Grid synchronization signals

### DC Link
- DC current measurement
- DC voltage measurement
- DC current reference
- Commutation parameters

### Machine Side (Load)
- Machine voltages (abc)
- Machine currents (abc)
- Load-side firing angles
- Rotor position and speed

### Control and Status
- Enable/disable commands
- Protection status
- Fault conditions
- Operating mode

### Configuration
- Control loop gains
- Filter parameters
- Protection limits
- System ratings

## Runner Execution Flow

### Initialization
1. Configure data model with parameters
2. Initialize control algorithms
3. Setup I/O mapping
4. Enter main control loop

### Control Loop Execution
1. Read inputs from hardware/simulator
2. Update data model signals
3. Execute control algorithms
4. Write outputs to hardware/simulator
5. Handle telemetry and commands

### Shutdown
1. Safe state transition
2. Cleanup resources
3. Final telemetry transmission

## Building and Testing

### Build Configuration

```bash
# Build host simulator with SFC integration
cmake --preset host-gcc-sfc
cmake --build --preset host-gcc-sfc-debug

# Build embedded target with SFC integration
cmake --preset embedded-gcc
cmake --build --preset embedded-gcc-release
```

### Testing

```bash
# Run integration tests
ctest --test-dir build-host-gcc-sfc -C Debug -R sfc_integration

# Run specific integration test
./build-host-gcc-sfc/software/yeet_application/sfc_integration/tests/test_sfc_integration
```

## Usage Example

```cpp
#include <sfc_integration/sfc_data_model.hpp>
#include <sfc_integration/sfc_runner.hpp>

// Create and configure runner
auto runner = SFCRunner();
runner.configure_parameters(params);
runner.initialize();

// Execute control loop
while (running) {
    runner.read_inputs();
    runner.execute_control();
    runner.write_outputs();
}

runner.shutdown();
```

## Documentation

- **[Data Model Interconnection](datamodel_interconnection.md)**: Detailed architecture of data flow and signal mapping

## See Also

- [YEET Core Library](../../yeet/docs/index.md)
- [YEET Testing Framework](../../yeet_testing/docs/index.md)
- [SFC Application](../sfc/docs/index.md)
- [SFC Embedded](../sfc_embedded/docs/index.md)
