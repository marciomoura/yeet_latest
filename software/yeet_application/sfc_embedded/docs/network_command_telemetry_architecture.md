# Network Command and Telemetry Architecture

## Overview

The network interface has been refactored into a clean, extensible architecture using two specialized classes:

1. **`command_parser`** - Stateless command parser for text-based network commands
2. **`telemetry_manager`** - Stateful telemetry streaming manager

This architecture follows SOLID principles and makes it easy to extend functionality without modifying existing code.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     yeet_network()                          │
│                   (Network Task Entry)                      │
└──────────────────────┬──────────────────┬───────────────────┘
                       │                  │
         ┌─────────────▼─────────┐  ┌────▼──────────────────┐
         │   command_parser      │  │  telemetry_manager    │
         │   (Stateless)         │  │  (Stateful)           │
         └─────────────┬─────────┘  └────┬──────────────────┘
                       │                  │
         ┌─────────────▼─────────┐  ┌────▼──────────────────┐
         │  parsed_command       │  │ telemetry_transmission│
         │  - type               │  │ - data                │
         │  - is_valid()         │  │ - size                │
         │  - is_telemetry()     │  │ - is_ready()          │
         └───────────────────────┘  └───────────────────────┘
```

## Class Responsibilities

### command_parser (Stateless)

**Purpose**: Parse text-based commands from network buffer

**Responsibilities**:
- Parse incoming byte buffers
- Identify command type (START, STOP, etc.)
- Return structured command result
- No internal state (can be used from any context)

**Interface**:
```cpp
class command_parser {
public:
    static parsed_command parse(const uint8_t* buffer, uint16_t length);
};

struct parsed_command {
    command_type type;
    bool is_valid() const;
    bool is_telemetry_command() const;
};

enum class command_type {
    none,
    start_telemetry,
    stop_telemetry,
    unknown
};
```

**Usage Example**:
```cpp
auto cmd = command_parser::parse(rx_buffer, rx_length);
if (cmd.is_valid()) {
    switch (cmd.type) {
        case command_type::start_telemetry:
            telemetry_mgr.start();
            break;
        case command_type::stop_telemetry:
            telemetry_mgr.stop();
            break;
    }
}
```

### telemetry_manager (Stateful)

**Purpose**: Manage telemetry streaming state and packet transmission

**Responsibilities**:
- Track streaming state (on/off)
- Handle rate decimation (e.g., 100 Hz from 1 kHz task)
- Build telemetry packets from control system
- Encode packets to binary format
- Manage internal timestamps and counters
- Return packet data when transmission is due

**Interface**:
```cpp
class telemetry_manager {
public:
    telemetry_manager(const sfc_grid_side_control& control, uint32_t decimation = 10);
    
    void start();
    void stop();
    bool is_streaming() const;
    
    telemetry_transmission update(bool previous_tx_pending);
    
    void set_decimation_factor(uint32_t factor);
    uint32_t get_decimation_factor() const;
};

struct telemetry_transmission {
    const uint8_t* data;
    size_t size;
    bool is_ready() const;
};
```

**Usage Example**:
```cpp
// Create manager (once, as static)
static telemetry_manager manager(control_system, 10);

// Control streaming
manager.start();   // Begin streaming
manager.stop();    // Stop streaming

// Update periodically (e.g., 1 kHz network task)
auto tx = manager.update(outputs->transmit_request);
if (tx.is_ready()) {
    memcpy(outputs->tx_buffer, tx.data, tx.size);
    outputs->tx_length = tx.size;
    outputs->transmit_request = true;
}
```

## Integration in grid_side_tasks.cpp

### Static Object Management

Both classes are managed as static objects with accessor functions:

```cpp
// Grid-side control system (already existed)
static inline sfc_grid_side_control& get_grid_side_control() {
    static auto per_unit_system = create_per_unit_system();
    static sfc_grid_side_control grid_side_control(per_unit_system);
    return grid_side_control;
}

// Telemetry manager (new)
static inline telemetry_manager& get_telemetry_manager() {
    static telemetry_manager manager(get_grid_side_control(), 10);
    return manager;
}
```

### Network Task Flow

```cpp
void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs) {
    // ==================== COMMAND PROCESSING ====================
    if (inputs && inputs->network_rx.data_ready) {
        // Parse command
        auto cmd = command_parser::parse(
            inputs->network_rx.rx_buffer, 
            inputs->network_rx.rx_length
        );
        
        // Dispatch to handler
        if (cmd.is_valid()) {
            switch (cmd.type) {
                case command_type::start_telemetry:
                    get_telemetry_manager().start();
                    break;
                case command_type::stop_telemetry:
                    get_telemetry_manager().stop();
                    break;
            }
        }
    }
    
    // ==================== TELEMETRY TRANSMISSION ====================
    if (outputs && inputs && inputs->network_rx.tcp_connected) {
        // Update telemetry (handles decimation, packet building)
        auto tx = get_telemetry_manager().update(
            outputs->network_tx.transmit_request
        );
        
        // Transmit if ready
        if (tx.is_ready()) {
            memcpy(outputs->network_tx.tx_buffer, tx.data, tx.size);
            outputs->network_tx.tx_length = tx.size;
            outputs->network_tx.transmit_request = true;
            outputs->network_tx.socket_id = inputs->network_rx.socket_id;
        }
    }
}
```

## Benefits of New Architecture

### 1. Separation of Concerns
- **Command parsing** is isolated from command execution
- **Telemetry logic** is separate from network interface
- Each class has a single, well-defined responsibility

### 2. Extensibility
Adding new commands is straightforward:

```cpp
// In network_command_parser.hpp
enum class command_type {
    none,
    start_telemetry,
    stop_telemetry,
    set_parameter,     // NEW
    get_status,        // NEW
    reset_controller,  // NEW
    unknown
};

// In network_command_parser.cpp
parsed_command command_parser::parse(...) {
    if (match_command(buffer, length, "SET:", 4)) {
        return {command_type::set_parameter};
    }
    if (match_command(buffer, length, "GET:", 4)) {
        return {command_type::get_status};
    }
    // ... existing commands
}

// In grid_side_tasks.cpp
switch (cmd.type) {
    case command_type::set_parameter:
        // Handle parameter setting
        break;
    case command_type::get_status:
        // Handle status query
        break;
    // ... existing handlers
}
```

### 3. Testability
Both classes can be easily unit tested:

```cpp
// Test command parser
TEST(CommandParser, ParseStartCommand) {
    const uint8_t buffer[] = "START\n";
    auto cmd = command_parser::parse(buffer, 6);
    EXPECT_EQ(cmd.type, command_type::start_telemetry);
    EXPECT_TRUE(cmd.is_valid());
}

// Test telemetry manager
TEST(TelemetryManager, StreamingControl) {
    MockControl control;
    telemetry_manager manager(control, 10);
    
    manager.start();
    EXPECT_TRUE(manager.is_streaming());
    
    manager.stop();
    EXPECT_FALSE(manager.is_streaming());
}
```

### 4. Maintainability
- Clear, self-documenting code
- No hard-coded parsing logic in network task
- Easy to understand control flow
- Type-safe command dispatch (no magic numbers)

### 5. Performance
- **Zero dynamic allocation**: All objects are static
- **Minimal overhead**: Parser is stateless (no cache thrashing)
- **Efficient packet buffer**: Manager owns internal buffer (no copies)
- **Predictable execution**: Clear control flow, no hidden side effects

## Memory Usage

### Static Memory (BSS segment)
```
sfc_grid_side_control:     ~8 KB  (existing)
telemetry_manager:         ~1 KB  (new - includes 1024-byte packet buffer)
command_parser:            0 bytes (stateless - no static data)
────────────────────────────────
Total:                     ~9 KB
```

### Stack Usage (per call)
```
yeet_network():            ~100 bytes
  - parsed_command:        4 bytes
  - telemetry_transmission: 16 bytes
  - local variables:       ~80 bytes
```

### Code Size (Flash)
```
command_parser:            ~300 bytes
telemetry_manager:         ~800 bytes
────────────────────────────────
Total increase:            ~1.1 KB
```

## Configuration Options

### Decimation Factor
Control transmission rate by adjusting decimation:

```cpp
// 50 Hz transmission (20 = transmit every 20th call)
static telemetry_manager manager(control, 20);

// 10 Hz transmission (100 = transmit every 100th call)
static telemetry_manager manager(control, 100);

// Runtime adjustment
get_telemetry_manager().set_decimation_factor(50);
```

### Signal Group Selection
The `telemetry_manager` can be extended to support selective group transmission:

```cpp
// Future enhancement in telemetry_manager.hpp
void set_active_groups(uint32_t flags);

// Usage
using namespace telemetry::telemetry_flags;
manager.set_active_groups(measurements | observer);  // Only measurements + observer
```

## Future Extensions

### 1. Parameter Setting Commands
```cpp
// Add to command_parser
enum class command_type {
    // ... existing
    set_current_limit,
    set_voltage_limit,
    set_firing_angle_offset
};

// Parse value from buffer
struct parsed_command {
    command_type type;
    float value;  // For parameter commands
};
```

### 2. Query Commands
```cpp
enum class command_type {
    // ... existing
    query_status,
    query_faults,
    query_configuration
};

// Respond with specific data packet
```

### 3. Binary Command Protocol
For efficiency, add binary command support:

```cpp
// Binary command format:
// [Command ID: 1 byte] [Length: 1 byte] [Payload: N bytes] [CRC: 2 bytes]

class binary_command_parser {
public:
    static parsed_command parse_binary(const uint8_t* buffer, uint16_t length);
};
```

### 4. Acknowledgment/Response
Add response mechanism for commands:

```cpp
struct command_response {
    bool success;
    const char* message;
};

command_response execute_command(const parsed_command& cmd);
```

## Troubleshooting

### Command not recognized
- **Check buffer content**: Verify exact command string
- **Check length**: Ensure minimum length requirement met
- **Case sensitivity**: Commands are case-sensitive ("START" not "start")

### Telemetry not transmitting
- **Check streaming state**: `manager.is_streaming()` should return true
- **Check TCP connection**: Verify `inputs->network_rx.tcp_connected`
- **Check decimation**: Wait sufficient time (100 ms for decimation=10 at 1 kHz)
- **Check previous transmission**: Ensure `transmit_request` flag is cleared

### High CPU load
- **Increase decimation**: Use higher factor (20, 50, 100)
- **Reduce signal groups**: Select only needed groups
- **Profile update()**: Check time spent in builder

## Testing Checklist

- [ ] Unit test command parser with all command types
- [ ] Unit test telemetry manager start/stop/update
- [ ] Integration test: Send START command, verify streaming begins
- [ ] Integration test: Send STOP command, verify streaming stops
- [ ] Performance test: Measure CPU time in update()
- [ ] Memory test: Verify static allocation size
- [ ] Stress test: Send commands rapidly, verify no crashes
- [ ] Stress test: Network disconnect during streaming, verify recovery

## References

- `network_command_parser.hpp` - Command parser interface
- `network_command_parser.cpp` - Command parser implementation
- `network_telemetry_manager.hpp` - Telemetry manager interface
- `network_telemetry_manager.cpp` - Telemetry manager implementation
- `network_telemetry_protocol.md` - Telemetry protocol specification
- `network_telemetry_integration.md` - Integration guide
- `grid_side_tasks.cpp` - Network task implementation
