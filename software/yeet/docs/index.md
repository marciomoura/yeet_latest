# YEET Core Library Documentation

## Overview

The YEET Core Library provides the foundational infrastructure for building real-time control applications. It defines the architecture for data models, application interfaces, and network communication that enables seamless integration between control algorithms, hardware, and testing frameworks.

## Key Components

### Application Framework

#### Application Data Model
- **Signal System**: Type-safe signal definitions for real-time data exchange
- **Parameter Management**: Runtime-configurable parameters with validation and persistence
- **Data Model Connector**: Links application data models to execution policies

**Key Files:**
- `yeet/signal.hpp` - Signal definitions and metadata
- `yeet/parameter.hpp` - Parameter definitions and handlers
- `yeet/application_datamodel.hpp` - Data model base classes
- `yeet/data_model_connector.hpp` - Connection infrastructure

### Control Execution

#### Task Level Management
- **Control Task Levels**: Hierarchical execution of control loops at different rates
- **Execution Policies**: Deterministic scheduling and synchronization
- **Registry System**: Component registration and lifecycle management

**Key Files:**
- `yeet/control_task_level.hpp` - Task level definitions
- `yeet/control_execution_policy.hpp` - Execution policies
- `yeet/control_task_level_registry.hpp` - Task registration
- `yeet/application_registry.hpp` - Application-level registration

### Network Communication

#### Telemetry and Command
- **Network Telemetry**: High-performance streaming of control signals and parameters
- **Command Parser**: Runtime command interface for control and configuration
- **Network Handler**: Asynchronous I/O management

**Key Files:**
- `yeet/network_telemetry.hpp` - Telemetry definitions
- `yeet/network_command_parser.hpp` - Command parsing
- `yeet/network_handler.hpp` - Network I/O abstraction
- `yeet/network_ios.hpp` - I/O stream utilities

### Utilities

#### Low-Level Access
- **Parameter Accessors**: Direct memory access for real-time parameter updates
- **Storage Types**: Type-safe storage for various data types
- **Metadata System**: Compile-time and runtime metadata for introspection

**Key Files:**
- `yeet/utils/low_level_access.hpp` - Memory access abstractions
- `yeet/utils/parameter_accessors.hpp` - Parameter access utilities
- `yeet/utils/storage_type.hpp` - Storage type definitions
- `yeet/utils/metadata.hpp` - Metadata system

## Architecture Principles

### Real-Time Safety
- **No Dynamic Allocation**: All memory allocated at initialization
- **Deterministic Execution**: Predictable timing for control loops
- **Type Safety**: Compile-time checks for signal and parameter types

### Modularity
- **Layer Separation**: Clear boundaries between application, integration, and embedded layers
- **Component Registration**: Discover and connect components at runtime
- **Interface Abstraction**: Platform-independent interfaces for portability

### Testability
- **Data Model Separation**: Control logic independent of I/O
- **Simulator Integration**: Built-in support for closed-loop testing
- **Network Introspection**: Monitor and debug via telemetry

## Integration with Other Layers

### YEET Testing
The core library provides the data model infrastructure used by the testing framework for:
- Simulator configuration and execution
- Data sampling and post-processing
- Parameter sweeps and optimization

### YEET Control
Control algorithms built with `yeet_control` are integrated through:
- Signal-based I/O for algorithm inputs and outputs
- Parameter handlers for runtime tuning
- Task level assignment for execution scheduling

### YEET Application
Application-specific implementations (like SFC) use the core library to:
- Define application data models
- Register control components
- Implement network interfaces

## Usage Example

```cpp
#include <yeet/application_datamodel.hpp>
#include <yeet/signal.hpp>
#include <yeet/parameter.hpp>

// Define application data model
class MyApplicationDataModel : public yeet::ApplicationDataModel {
public:
    // Define signals
    yeet::Signal<float> motor_speed{"motor_speed", "rad/s"};
    yeet::Signal<float> motor_torque{"motor_torque", "Nm"};
    
    // Define parameters
    yeet::Parameter<float> speed_reference{"speed_reference", "rad/s", 100.0f};
    yeet::Parameter<float> torque_limit{"torque_limit", "Nm", 50.0f};
};
```

## Building and Testing

The YEET core library is built as part of the main YEET platform:

```bash
# Configure build
cmake --preset host-gcc

# Build the library
cmake --build --preset host-gcc-debug

# Run tests
ctest --test-dir build-host-gcc -C Debug
```

## Documentation Standards

- **Signal Naming**: Use `snake_case` for signal and parameter names
- **Units**: Always specify physical units in signal definitions
- **Type Safety**: Leverage C++ type system for compile-time validation
- **Documentation**: Use Doxygen-style comments for public APIs

## See Also

- [YEET Testing Framework](../../yeet_testing/docs/index.md)
- [YEET Control Library](../../yeet_control/docs/index.md)
- [SFC Application](../../yeet_application/sfc/docs/index.md)
