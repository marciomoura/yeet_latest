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

## Project Structure

```
software/
├── yeet/              # Core library
├── yeet_control/      # Control algorithms
└── yeet_application/  # Applications (SFC, etc.)
boards/                # Board support packages
```

## Documentation

- [SFC System Documentation](software/yeet_application/sfc/docs/index.md)
- [Control Library Reference](software/yeet_control/docs/index.md)

## Prerequisites

- **Host builds**: CMake 3.20+, GCC/Clang, Ninja
- **Embedded builds**: GNU Arm Embedded Toolchain

## License

[License information to be added]