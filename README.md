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

## License

[License information to be added]