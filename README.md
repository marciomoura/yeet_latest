# YEET Platform - Power Electronics Engineering Software

**Y**our **E**ngineering **E**cosystem for **T**esting - A comprehensive platform for power electronics development, real-time control systems, and closed-loop validation.

## Overview

YEET is a specialized software platform designed for power electronics engineers working on advanced control systems, particularly Load-Commutated Inverters (LCI), thyristor bridges, and gas turbine soft-starters. The platform provides:

- **Advanced Data Modeling**: Comprehensive modeling capabilities for power electronics systems
- **Real-Time Control Libraries**: High-performance control algorithms optimized for real-time execution
- **Closed-Loop Testing Framework**: Integrated validation and testing tools for control system verification
- **Hardware-in-the-Loop Support**: Integration with microcontrollers and RTOS platforms
- **Simulink Integration**: Seamless code generation and plant model integration

## Key Features

### Power Electronics Focus
- **LCI Drive Systems**: Complete support for Load-Commutated Inverter applications
- **Thyristor Control**: 6-pulse and 12-pulse bridge configurations
- **Gas Turbine Integration**: Specialized soft-starter and helper motor applications
- **Real-Time Performance**: Deterministic execution for high-frequency control loops

### Software Architecture
- **C++ Core Guidelines Compliance**: Modern, maintainable C++ codebase
- **Real-Time Constraints**: Memory-safe, predictable execution paths
- **Modular Design**: Reusable components for control systems and signal processing
- **Cross-Platform Support**: Windows, Linux, and embedded targets

## Getting Started

### Prerequisites

#### Development Environment
1. **Visual Studio Code** with the following extensions:
   - [Clangd Extension](https://github.com/clangd/vscode-clangd)
   - Configure Clangd with: `--query-driver=C:\msys64\mingw64\**\*`

2. **Code Quality Tools**:
   - [Clang-tidy](https://clang.llvm.org/extra/clang-tidy/) for static analysis
   - [CppCheck](https://cppcheck.sourceforge.io/) for additional code validation

#### Windows Development Setup (MSYS2)

Install the required development packages:

```bash
# Core development tools
pacman -S mingw-w64-x86_64-clang
pacman -S mingw-w64-x86_64-gdb
pacman -S mingw-w64-x86_64-cmake
pacman -S mingw-w64-x86_64-ccache
pacman -S mingw-w64-x86_64-gcc
pacman -S mingw-w64-x86_64-lcov
pacman -S mingw-w64-x86_64-perl
```

Add to your system PATH:
```
C:\msys64\usr\bin
C:\msys64\mingw64\bin
```

#### Python Environment

1. Install Python 3.8 or higher
2. For Windows PowerShell, enable script execution:
   ```powershell
   Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
   ```
3. Initialize Python environment in VSCode: `Ctrl + Shift + P` → `Python: Create Environment`

### Building the Project

```bash
# Clone the repository
git clone <repository-url>
cd yeet

# Configure build
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --parallel

# Run tests
ctest --test-dir build
```

## Project Structure

```
yeet/
├── software/
│   ├── yeet_application/
│   │   └── sfc/              # SFC (Static Frequency Converter) applications
│   │       ├── simulink/     # Simulink model integration
│   │       ├── control/      # Control algorithms
│   │       └── docs/         # Technical documentation
│   └── yeet_control/         # Reusable control components
│       ├── filters/          # Digital filters
│       ├── transforms/       # Reference frame transformations
│       └── controllers/      # PID, PI controllers
├── tests/                    # Unit and integration tests
└── docs/                     # Project documentation
```

## Key Applications

### LCI Gas Turbine Starter
- **6-pulse thyristor bridges** for grid and machine side converters
- **Real-time control algorithms** for torque and speed regulation
- **Protection systems** for safe turbine startup sequences
- **Simulink integration** for plant model validation

### Power Electronics Control
- **Digital signal processing** components for real-time applications
- **Reference frame transformations** (ABC ↔ αβ ↔ dq)
- **Advanced filtering** for harmonic mitigation
- **Closed-loop controllers** optimized for power electronics

## Documentation

- **[SFC System Documentation](software/yeet_application/sfc/docs/index.md)** - Complete technical reference for Static Frequency Converter applications, LCI drive systems, and gas turbine starter implementations
- **[Control Library Reference](software/yeet_control/docs/index.md)** - Comprehensive guide to reusable control components, signal processing algorithms, and real-time utilities
- [API Documentation](docs/api/index.md)
- [Development Guidelines](docs/development.md)

## Contributing

This project follows strict coding standards for real-time systems:

1. **C++ Core Guidelines** compliance
2. **Real-time constraints** - no dynamic memory allocation in critical paths
3. **Comprehensive testing** for all new features
4. **Domain expertise** in power electronics and control systems

Please review the [Development Guidelines](docs/development.md) before contributing.

## License

[License information to be added]

## Status

**Active Development** - Core LCI functionality is stable, with ongoing development of:
- Extended microcontroller support
- Enhanced RTOS integration
- Additional power electronics applications
- Advanced control algorithms

For issues, feature requests, or technical discussions, please use the GitHub issue tracker.

## Embedded Target

To configure and compile the embedded target, run:

```bash
cmake -B build-embedded -S . -DCMAKE_BUILD_TYPE=Debug -DYEET_BUILD_EMBEDDED=ON "-DCMAKE_TOOLCHAIN_FILE=boards/stm32f407g-disc1/cmake/gcc-arm-none-eabi.cmake" -G Ninja
cmake --build build-embedded --parallel
```

# Build release version
cmake --preset embedded-release
cmake --build --preset embedded-release

# Flash using STM32CubeProgrammer CLI
```bash
C:\ST\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=SWD -w build-embedded-release/bin/yeet_embedded_app.elf -v -rst
```