# Single-Executable Multi-Controller Refactoring Plan

## Executive Summary

Refactor the SFC embedded application to support **both grid-side and machine-side controllers in a single executable**, enabling:
1. **Host Simulator**: Single process running both controllers with dual TCP servers and integrated plant model
2. **Embedded MCU**: Still supports separate firmware images (one per STM32 board) for production deployment

**Key Challenge**: Resolve function name conflicts where both sides define identical function names (`yeet_task_200us`, `yeet_network`, etc.).

---

## Problem Statement: Function Name Conflicts

### Current Architecture

```
grid_side_tasks.cpp          machine_side_tasks.cpp
┌──────────────────┐        ┌──────────────────┐
│ yeet_init()      │        │ yeet_init()      │  ❌ CONFLICT
│ yeet_task_200us()│        │ yeet_task_200us()│  ❌ CONFLICT
│ yeet_task_1ms()  │        │ yeet_task_1ms()  │  ❌ CONFLICT
│ yeet_task_10ms() │        │ yeet_task_10ms() │  ❌ CONFLICT
│ yeet_network()   │        │ yeet_network()   │  ❌ CONFLICT
└──────────────────┘        └──────────────────┘
        ↓                            ↓
   Cannot link both into same executable!
```

### Solution: Side-Specific Function Prefixes

```
grid_side_tasks.cpp          machine_side_tasks.cpp
┌───────────────────┐        ┌──────────────────────┐
│ grid_init()       │        │ machine_init()       │  ✅ UNIQUE
│ grid_task_200us() │        │ machine_task_200us() │  ✅ UNIQUE
│ grid_task_1ms()   │        │ machine_task_1ms()   │  ✅ UNIQUE
│ grid_task_10ms()  │        │ machine_task_10ms()  │  ✅ UNIQUE
│ grid_network()    │        │ machine_network()    │  ✅ UNIQUE
└───────────────────┘        └──────────────────────┘
        │                            │
        └────────┬───────────────────┘
                 ↓
        Single Executable with both! ✅
```

---

## Architecture Overview

### Single-Executable Host Simulator

```
┌──────────────────────────────────────────────────────────────┐
│  sfc_dual_host_simulator (Single Process)                    │
│                                                               │
│  ┌────────────────────────────────────────────────────────┐  │
│  │                   Main Control Loop                     │  │
│  │                   (200µs / 1ms / 10ms)                  │  │
│  └─────────┬──────────────────────────────────────┬────────┘  │
│            │                                      │            │
│  ┌─────────▼──────────┐              ┌───────────▼─────────┐ │
│  │  Grid-Side Module  │   Fiber      │ Machine-Side Module │ │
│  │                    │◄──Optic─────►│                     │ │
│  │ grid_task_200us()  │   (RAM)      │machine_task_200us() │ │
│  │ grid_task_1ms()    │              │machine_task_1ms()   │ │
│  │ grid_task_10ms()   │              │machine_task_10ms()  │ │
│  │ grid_network()     │              │machine_network()    │ │
│  │                    │              │                     │ │
│  │ TCP Server :7000   │              │ TCP Server :7001    │ │
│  └────────────────────┘              └─────────────────────┘ │
│            │                                      │            │
│            └──────────┬───────────────────────────┘            │
│                       │                                        │
│           ┌───────────▼──────────┐                             │
│           │   Plant Simulator    │                             │
│           │  (In-Process Object) │                             │
│           └──────────────────────┘                             │
└──────────────────────────────────────────────────────────────┘
```

### Embedded Target (Separate Firmware Images)

```
┌─────────────────────┐        ┌──────────────────────┐
│  STM32 Board 1      │        │  STM32 Board 2       │
│  (Grid-Side)        │        │  (Machine-Side)      │
│                     │        │                      │
│  grid_init()        │        │  machine_init()      │
│  grid_task_200us()  │◄──────►│  machine_task_200us()│
│  grid_network()     │ Fiber  │  machine_network()   │
│                     │        │                      │
│  W5200 :7000        │        │  W5200 :7001         │
└─────────────────────┘        └──────────────────────┘
           │                            │
           └──────► Physical Plant ◄────┘
```

---

## Implementation Plan

### Phase 1: Rename Functions with Side Prefixes

#### 1.1 Update Grid-Side Function Names

**File**: `software/yeet_application/sfc_embedded/src/grid_side_tasks.cpp`

**Changes**:
```cpp
// OLD
void yeet_embedded_init(void) { ... }
void yeet_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs) { ... }
void yeet_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs) { ... }
void yeet_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs) { ... }
void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs) { ... }

// NEW
void grid_embedded_init(void) { ... }
void grid_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs) { ... }
void grid_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs) { ... }
void grid_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs) { ... }
void grid_network(const network_inputs_t* inputs, network_outputs_t* outputs) { ... }
```

**File**: `software/yeet_application/sfc_embedded/include/sfc_embedded/grid_side_tasks.hpp`

**Changes**:
```cpp
// Function declarations
void grid_embedded_init(void);
void grid_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs);
void grid_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs);
void grid_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs);
void grid_network(const network_inputs_t* inputs, network_outputs_t* outputs);

// Backward compatibility wrappers for embedded-only builds
#if defined(YEET_EMBEDDED_GRID_SIDE) && !defined(YEET_BUILD_DUAL_CONTROLLER)
inline void yeet_embedded_init(void) { grid_embedded_init(); }
inline void yeet_task_200us(const task_inputs_t* i, task_outputs_t* o) { grid_task_200us(i, o); }
inline void yeet_task_1000us(const task_inputs_t* i, task_outputs_t* o) { grid_task_1000us(i, o); }
inline void yeet_task_10000us(const task_inputs_t* i, task_outputs_t* o) { grid_task_10000us(i, o); }
inline void yeet_network(const network_inputs_t* i, network_outputs_t* o) { grid_network(i, o); }
#endif
```

#### 1.2 Update Machine-Side Function Names

**File**: `software/yeet_application/sfc_embedded/src/machine_side_tasks.cpp`

**Changes**: Same pattern as grid-side, but with `machine_` prefix instead of `grid_`.

**File**: `software/yeet_application/sfc_embedded/include/sfc_embedded/machine_side_tasks.hpp`

**Changes**: Same pattern as grid-side header.

---

### Phase 2: Create Unified Host Simulator

#### 2.1 Create Dual-Controller Main

**File**: `software/yeet_application/sfc_embedded/src/host_platform/host_main_dual.cpp`

```cpp
/**
 * @file host_main_dual.cpp
 * @brief Host simulator running BOTH grid and machine controllers
 */

#include "sfc_embedded/grid_side_tasks.hpp"
#include "sfc_embedded/machine_side_tasks.hpp"
#include "sfc_embedded/host_platform/host_tcp_server.hpp"
#include "sfc_embedded/host_platform/host_timing.hpp"
#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"

int main(int argc, char* argv[])
{
    using namespace yeet::application::sfc;
    
    // Parse arguments
    config cfg = parse_args(argc, argv);
    
    std::cout << "======================================" << std::endl;
    std::cout << "  SFC Dual-Controller Host Simulator" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Grid telemetry:    localhost:" << cfg.grid_port << std::endl;
    std::cout << "Machine telemetry: localhost:" << cfg.machine_port << std::endl;
    std::cout << "Control frequency: " << cfg.frequency_hz << " Hz" << std::endl;
    std::cout << "======================================" << std::endl;
    
    // Create TCP servers for each side
    host::host_tcp_server grid_telemetry_server;
    host::host_tcp_server machine_telemetry_server;
    
    if (!grid_telemetry_server.initialize(cfg.grid_port, "0.0.0.0")) {
        std::cerr << "[ERROR] Failed to initialize grid TCP server" << std::endl;
        return 1;
    }
    
    if (!machine_telemetry_server.initialize(cfg.machine_port, "0.0.0.0")) {
        std::cerr << "[ERROR] Failed to initialize machine TCP server" << std::endl;
        return 1;
    }
    
    // Create plant simulator (using existing plant_complete_sfc_closed_loop_simulation_builder)
    integration::plant_complete_sfc_closed_loop_simulation_builder plant_system;
    plant_system.initialize();
    
    // Initialize control systems
    grid_embedded_init();
    machine_embedded_init();
    
    // Create timing loop
    host::host_timing_loop control_loop(cfg.frequency_hz);
    control_loop.start();
    
    std::cout << "[READY] Dual-controller simulator running" << std::endl;
    std::cout << "[READY] Press Ctrl+C to stop" << std::endl;
    
    while (control_loop.is_running() && !shutdown_requested.load()) {
        control_loop.wait_for_next_cycle();
        
        std::uint32_t timestamp_ms = static_cast<std::uint32_t>(control_loop.get_elapsed_time_ms());
        
        // Process TCP servers
        grid_telemetry_server.process();
        machine_telemetry_server.process();
        
        // ===== PREPARE INPUTS FROM PLANT =====
        task_inputs_t grid_inputs = prepare_grid_inputs(plant_system);
        task_inputs_t machine_inputs = prepare_machine_inputs(plant_system);
        
        task_outputs_t grid_outputs{};
        task_outputs_t machine_outputs{};
        
        // ===== EXECUTE CONTROL TASKS (200µs rate) =====
        grid_task_200us(&grid_inputs, &grid_outputs);
        machine_task_200us(&machine_inputs, &machine_outputs);
        
        // Every 10 cycles: 1ms tasks
        if (control_loop.get_cycle_number() % 10 == 0) {
            // Exchange fiber-optic signals between controllers
            exchange_fiber_optic_data(grid_outputs, machine_outputs, grid_inputs, machine_inputs);
            
            grid_task_1000us(&grid_inputs, &grid_outputs);
            machine_task_1000us(&machine_inputs, &machine_outputs);
        }
        
        // Every 100 cycles: 10ms tasks
        if (control_loop.get_cycle_number() % 100 == 0) {
            grid_task_10000us(&grid_inputs, &grid_outputs);
            machine_task_10000us(&machine_inputs, &machine_outputs);
        }
        
        // ===== UPDATE PLANT WITH CONTROL OUTPUTS =====
        update_plant_from_controllers(plant_system, grid_outputs, machine_outputs);
        
        // ===== HANDLE NETWORK TELEMETRY =====
        handle_grid_network(grid_telemetry_server, grid_inputs, grid_outputs);
        handle_machine_network(machine_telemetry_server, machine_inputs, machine_outputs);
        
        control_loop.mark_cycle_end();
    }
    
    // Shutdown
    std::cout << "\n[SHUTDOWN] Stopping..." << std::endl;
    control_loop.stop();
    grid_telemetry_server.shutdown();
    machine_telemetry_server.shutdown();
    
    return 0;
}
```

---

### Phase 3: CMake Build Configuration

#### 3.1 Add Dual-Controller Build Option

**File**: `software/yeet_application/sfc_embedded/CMakeLists.txt`

**Add**:
```cmake
# Option for dual-controller build (host only)
option(YEET_BUILD_DUAL_CONTROLLER "Build dual-controller executable (host only)" OFF)

if(YEET_SFC_HOST_BUILD AND YEET_BUILD_DUAL_CONTROLLER)
    message(STATUS "yeet sfc_embedded: Building DUAL-CONTROLLER HOST SIMULATOR")
    
    # Create dual-controller executable
    add_executable(sfc_dual_host_simulator)
    
    target_sources(
        sfc_dual_host_simulator
        PRIVATE
            # Host platform layer
            src/host_platform/host_main_dual.cpp
            src/host_platform/host_tcp_server.cpp
            src/host_platform/host_timing.cpp
            src/host_platform/host_io_mocks.cpp
            # BOTH control sides
            src/grid_side_tasks.cpp
            src/machine_side_tasks.cpp
            src/fiber_optic_protocol.cpp
            src/grid_side_telemetry_builder.cpp
            src/machine_side_telemetry_builder.cpp
    )
    
    target_include_directories(
        sfc_dual_host_simulator
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    
    target_link_libraries(
        sfc_dual_host_simulator
        PRIVATE 
            yeet::yeet_control
            yeet::yeet_application_sfc_integration
    )
    
    target_compile_definitions(
        sfc_dual_host_simulator
        PRIVATE 
            HOST_BUILD
            YEET_BUILD_DUAL_CONTROLLER
            YEET_EMBEDDED_GRID_SIDE      # Include grid code
            YEET_EMBEDDED_MACHINE_SIDE   # Include machine code
    )
    
    if(WIN32)
        target_link_libraries(sfc_dual_host_simulator PRIVATE ws2_32)
    else()
        target_link_libraries(sfc_dual_host_simulator PRIVATE pthread)
    endif()
    
    message(STATUS "  - Dual-controller simulator: sfc_dual_host_simulator ✅")
endif()
```

#### 3.2 Add CMake Preset

**File**: `CMakePresets.json`

**Add**:
```json
{
  "name": "host-dual-controller",
  "displayName": "Host Dual-Controller Simulator",
  "binaryDir": "${sourceDir}/build-host",
  "cacheVariables": {
    "YEET_SFC_HOST_BUILD": "ON",
    "YEET_BUILD_DUAL_CONTROLLER": "ON",
    "CMAKE_BUILD_TYPE": "Debug"
  }
}
```

---

### Phase 4: VS Code Integration

#### 4.1 Add Launch Configuration

**File**: `.vscode/launch.json`

**Add**:
```json
{
  "name": "SFC Dual-Controller Simulator",
  "type": "cppdbg",
  "request": "launch",
  "program": "${workspaceFolder}/build-host/bin/debug/sfc_dual_host_simulator",
  "args": [
    "--grid-port", "7000",
    "--machine-port", "7001",
    "--frequency", "100",
    "--verbose"
  ],
  "stopAtEntry": false,
  "cwd": "${workspaceFolder}",
  "environment": [],
  "externalConsole": false,
  "MIMode": "gdb",
  "setupCommands": [
    {
      "description": "Enable pretty-printing for gdb",
      "text": "-enable-pretty-printing",
      "ignoreFailures": true
    }
  ],
  "preLaunchTask": "Build Dual-Controller Simulator"
}
```

#### 4.2 Add Build Task

**File**: `.vscode/tasks.json`

**Add**:
```json
{
  "label": "Build Dual-Controller Simulator",
  "type": "shell",
  "command": "cmake",
  "args": [
    "--build",
    "build-host",
    "--target",
    "sfc_dual_host_simulator",
    "--config",
    "Debug"
  ],
  "group": {
    "kind": "build",
    "isDefault": false
  },
  "problemMatcher": ["$gcc"],
  "detail": "Build dual-controller host simulator"
}
```

---

## File Changes Summary

### Files to Modify

1. **Grid-side task functions** → Add `grid_` prefix
   - `software/yeet_application/sfc_embedded/src/grid_side_tasks.cpp`
   - `software/yeet_application/sfc_embedded/include/sfc_embedded/grid_side_tasks.hpp`

2. **Machine-side task functions** → Add `machine_` prefix
   - `software/yeet_application/sfc_embedded/src/machine_side_tasks.cpp`
   - `software/yeet_application/sfc_embedded/include/sfc_embedded/machine_side_tasks.hpp`

3. **Build configuration**
   - `software/yeet_application/sfc_embedded/CMakeLists.txt`
   - `CMakePresets.json`

4. **VS Code integration**
   - `.vscode/launch.json`
   - `.vscode/tasks.json`

### Files to Create

1. **Dual-controller main**
   - `software/yeet_application/sfc_embedded/src/host_platform/host_main_dual.cpp`

2. **Helper functions**
   - `software/yeet_application/sfc_embedded/src/host_platform/plant_interface.cpp` (prepare inputs/outputs)

---

## Testing Strategy

### Unit Tests
- ✅ Verify grid functions callable with `grid_` prefix
- ✅ Verify machine functions callable with `machine_` prefix
- ✅ Verify backward compatibility wrappers work for single-side builds

### Integration Tests
- ✅ Build dual-controller executable successfully
- ✅ Both TCP servers accept connections
- ✅ Telemetry streams work independently
- ✅ Plant model responds to both controllers
- ✅ Fiber-optic signals exchange correctly

### Hardware Tests (Future)
- ✅ Grid-side firmware still works on STM32
- ✅ Machine-side firmware still works on STM32
- ✅ Physical fiber-optic communication verified

---

## Benefits of This Approach

### ✅ Advantages

1. **Single Executable**: Easier to develop, debug, and test
2. **Integrated Plant**: No inter-process communication overhead
3. **Simultaneous Debugging**: Can debug both controllers at once
4. **Code Reuse**: Same task code for embedded and host
5. **Backward Compatible**: Existing embedded builds unchanged

### ⚠️ Considerations

1. **Shared State**: Must be careful with global variables
2. **Timing**: Both controllers share same timing loop
3. **Memory**: More memory usage (both controllers loaded)

---

## Implementation Timeline

### Week 1: Function Renaming
- [ ] Rename grid-side functions with `grid_` prefix
- [ ] Rename machine-side functions with `machine_` prefix
- [ ] Add backward compatibility wrappers
- [ ] Test individual side builds still work

### Week 2: Dual-Controller Main
- [ ] Create `host_main_dual.cpp`
- [ ] Integrate with existing plant simulator
- [ ] Implement fiber-optic data exchange
- [ ] Add dual TCP server support

### Week 3: Build System
- [ ] Update CMakeLists.txt
- [ ] Add CMake presets
- [ ] Update VS Code tasks/launch configs
- [ ] Test on Windows and Linux

### Week 4: Testing & Documentation
- [ ] Comprehensive testing
- [ ] User documentation
- [ ] Example workflows
- [ ] Performance profiling

---

## Next Steps

1. **Review**: Confirm this approach meets requirements
2. **Prototype**: Quick rename of one function set to verify concept
3. **Implement**: Follow week-by-week plan
4. **Test**: Continuous testing throughout
5. **Deploy**: Roll out to team with documentation

---

**Document Version**: 2.0 (Single-Executable Approach)  
**Date**: November 1, 2025  
**Status**: Ready for Implementation
