# YEET Platform - Power Electronics Engineering Software

**Y**our **E**ngineering **E**cosystem for **T**esting - A comprehensive platform for power electronics development, real-time control systems, and closed-loop validation.

## Overview

YEET is a software platform for power electronics engineers working on advanced control systems, particularly Load-Commutated Inverters (LCI), thyristor bridges, and gas turbine soft-starters.

**Key capabilities:**
- Advanced data modeling for power electronics systems
- Real-time control libraries optimized for deterministic execution
- Closed-loop testing framework for control system verification
- Hardware-in-the-loop support with microcontrollers and RTOS
- Simulink integration for code generation and plant models

## Project Structure

```
yeet/
├── software/
│   ├── yeet/                     # Core library (data structures, utilities)
│   ├── yeet_control/             # Control algorithms (filters, transforms, controllers)
│   ├── yeet_testing/             # Testing utilities and fixtures
│   └── yeet_application/
│       ├── sfc/                  # Static Frequency Converter application
│       ├── sfc_integration/      # Integration tests
│       └── sfc_embedded/         # Embedded ARM target
├── boards/
│   └── stm32f407g-disc1/         # STM32F407 Discovery board support
├── cmake/
│   ├── toolchains/               # Cross-compilation toolchains
│   └── dependencies/             # External dependencies (FetchContent)
└── tests/                        # Unit and integration tests
```

## Build System

### CMake Targets

The build system creates the following main targets:

**Core Libraries:**
- `yeet::yeet` - Core data structures and utilities
- `yeet::yeet_control` - Control algorithms (PI/PID controllers, filters, transforms)
- `yeet::yeet_testing` - Testing utilities and mock objects

**Applications:**
- `yeet::yeet_sfc` - Static Frequency Converter simulation
- `yeet::yeet_sfc_integration` - Integration test suite
- `yeet::yeet_sfc_embedded` - Embedded application logic
- `yeet_embedded_app` - Complete embedded firmware (only when `YEET_BUILD_EMBEDDED=ON`)

**Board Support:**
- `stm32f407g-disc1` - STM32F407 Discovery board library
- `STM32_HAL_Drivers` - STM32 HAL (fetched from ST repositories)
- `USB_Host_Library` - USB Host middleware (fetched from ST repositories)

### CMake Options

```cmake
YEET_BUILD_EMBEDDED      # Build ARM embedded targets (default: OFF)
YEET_BUILD_TESTING       # Build test suite (default: ON)
YEET_BUILD_EXAMPLES      # Build example applications (default: ON)
YEET_CODE_COVERAGE       # Enable coverage reports with gcovr (default: OFF)
YEET_MEMCHECK           # Enable Valgrind memory checks (default: OFF)
YEET_ENABLE_CCACHE      # Use ccache for faster rebuilds (default: ON)
YEET_PRECISION_DOUBLE   # Use double precision floats (default: ON)
YEET_ENABLE_DATA_LOGGING # Enable JSON simulation logging (default: ON)
```

### CMake Presets

Configure and build using presets defined in `CMakePresets.json`:

**Host Builds (native x86_64):**
```bash
# GCC toolchain
cmake --preset host-gcc
cmake --build --preset host-gcc-debug
cmake --build --preset host-gcc-release

# Clang toolchain
cmake --preset host-clang
cmake --build --preset host-clang-debug
cmake --build --preset host-clang-release
```

**Embedded Builds (ARM Cortex-M4):**
```bash
# GCC ARM toolchain
cmake --preset embedded-gcc
cmake --build --preset embedded-gcc-debug
cmake --build --preset embedded-gcc-release
```

Build artifacts are placed in:
- `build-host/bin/` or `build-host/bin/` for host builds
- `build-embedded/bin/` for embedded builds

### Running Tests

```bash
# Configure with testing enabled (default for host builds)
cmake --preset host-gcc

# Build and run all tests
cmake --build --preset host-gcc-debug
ctest --test-dir build-host-gcc -C Debug

# Run specific test suites
./build-host-gcc/bin/yeet_test
./build-host-gcc/bin/yeet_control_tests
./build-host-gcc/bin/yeet_application_sfc_tests
```

### Code Coverage

```bash
# Configure with coverage enabled
cmake --preset host-gcc -DYEET_CODE_COVERAGE=ON

# Build and run tests
cmake --build --preset host-gcc-debug
ctest --test-dir build-host-gcc -C Debug

# Generate HTML coverage report
cmake --build build-host-gcc --target coverage

# Open report: build-host-gcc/coverage/index.html
```

## Prerequisites

### Windows Development (MSYS2)

Install required packages:
```bash
pacman -S mingw-w64-x86_64-clang mingw-w64-x86_64-gcc
pacman -S mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja
pacman -S mingw-w64-x86_64-gdb mingw-w64-x86_64-ccache
pacman -S mingw-w64-x86_64-lcov mingw-w64-x86_64-perl
```

Add to PATH:
```
C:\msys64\usr\bin
C:\msys64\mingw64\bin
```

### ARM Embedded Development

Install [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm):
```bash
# Add to PATH (Windows)
C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\<version>\bin
```

Verify installation:
```bash
arm-none-eabi-gcc --version
```

### Visual Studio Code

Recommended extensions:
- **clangd** - C/C++ language server (configure with `--query-driver=C:\msys64\mingw64\**\*`)
- **CMake Tools** - CMake integration
- **Clang-Format** - Code formatting

## Embedded Development

### Building Firmware

```bash
# Configure for STM32F407 Discovery board
cmake --preset embedded-gcc

# Build Debug firmware
cmake --build --preset embedded-gcc-debug

# Build Release firmware (optimized)
cmake --build --preset embedded-gcc-release
```

The firmware binary is located at:
- Debug: `build-embedded-gcc/bin/Debug/yeet_embedded_app.elf`
- Release: `build-embedded-gcc/bin/Release/yeet_embedded_app.elf`

### Flashing Firmware

Using STM32CubeProgrammer CLI:
```bash
# Flash Debug build
STM32_Programmer_CLI.exe -c port=SWD \
  -w build-embedded-gcc/bin/Debug/yeet_embedded_app.elf \
  -v -rst

# Flash Release build
STM32_Programmer_CLI.exe -c port=SWD \
  -w build-embedded-gcc/bin/Release/yeet_embedded_app.elf \
  -v -rst
```

### Memory Optimization

The Release build uses aggressive optimization:
- `-O3` - Maximum optimization
- `-flto=auto` - Link-Time Optimization with parallel jobs
- `-ffunction-sections -fdata-sections` - Dead code elimination
- `-Wl,--gc-sections` - Linker garbage collection
- `-ffast-math -funroll-loops` - Performance tuning

Check memory usage (displayed automatically during linking):
```
Memory region         Used Size  Region Size  %age Used
             RAM:       10240 B       128 KB      7.81%
           FLASH:       45678 B      1024 KB      4.36%
```

## Applications

### LCI Gas Turbine Starter
- 6-pulse thyristor bridges for grid and machine converters
- Real-time torque and speed regulation
- Protection systems for safe turbine startup
- Simulink integration for plant model validation

### Control Library Components
- Digital filters (low-pass, high-pass, notch)
- Reference frame transformations (ABC ↔ αβ ↔ dq)
- PI/PID controllers optimized for real-time execution
- Phase-Locked Loop (PLL) for grid synchronization

## Documentation

- **[SFC System Documentation](software/yeet_application/sfc/docs/index.md)** - Static Frequency Converter technical reference
- **[Control Library Reference](software/yeet_control/docs/index.md)** - Control components and algorithms
- **[Development Guidelines](docs/development.md)** - Coding standards and contribution guide

## Development Standards

- **C++17** with Core Guidelines compliance
- **Real-time constraints** - no dynamic allocation in control loops
- **Comprehensive testing** - unit tests for all components
- **Static analysis** - clang-tidy and cppcheck integration
- **Code coverage** - minimum 90% line coverage

## License

[License information to be added]

## Status

**Active Development** - Core LCI functionality is stable. Ongoing work:
- Extended microcontroller support
- RTOS integration improvements
- Additional power electronics applications
- Advanced control algorithms

For issues or questions, use the GitHub issue tracker.