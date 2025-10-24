# SFC Parameter Configuration System

## 📋 Overview

The parameter configuration system enables **real-time tuning** of SFC control parameters via a web interface, without recompiling or reflashing firmware.

## 🏗️ Architecture

```
┌────────────────────┐         ┌─────────────────────┐         ┌──────────────────┐
│   Web Browser      │         │   Python Backend    │         │   STM32 (C++)    │
│   HTML + JS        │◄───────►│   Flask Server      │◄───────►│   TCP Server     │
│                    │   HTTP  │   Parameter Client  │   TCP   │   model_registry │
│  - Component View  │         │   - Validation      │         │   - Datamodels   │
│  - Parameter Forms │         │   - Caching         │         │   - Parameters   │
│  - Live Updates    │         │   - REST API        │         │                  │
└────────────────────┘         └─────────────────────┘         └──────────────────┘
      localhost:5000                  Python 3.x                    192.168.2.100:7000
```

## 🚀 Quick Start

### 1. Start the Web Server

```bash
cd software/yeet/tools
python parameter_web_server.py --controller-host 192.168.2.100 --controller-port 7000
```

### 2. Open Web Interface

Navigate to: **http://localhost:5000/**

### 3. Configure Parameters

1. Parameters are **automatically discovered** from the controller
2. **Grouped by component** (grid_observer, current_controller, etc.)
3. **Set new values** with real-time validation
4. Changes take effect **immediately**

## 📦 Components

### Python Modules

#### `parameter_protocol.py`
Binary protocol encoder/decoder for parameter commands.

**Packet Types:**
- `PARAM_LIST_REQUEST` (0x44): Request full parameter catalog
- `PARAM_LIST_RESPONSE` (0x45): Parameter metadata (one per parameter)
- `PARAM_GET_REQUEST` (0x42): Query current value
- `PARAM_GET_RESPONSE` (0x43): Current value response
- `PARAM_SET_REQUEST` (0x40): Set new value
- `PARAM_SET_RESPONSE` (0x41): Confirmation/error

**Data Structures:**
```python
@dataclass
class ParameterMetadata:
    component: str              # e.g., "grid_observer"
    name: str                   # e.g., "kp"
    display_name: str          # e.g., "PLL Proportional Gain"
    unit: str                   # e.g., "Hz", "pu", "rad"
    type: ParameterValueType   # FLOAT32, INT32, BOOLEAN, ENUM_VALUE
    min_value: float
    max_value: float
    default_value: float
    current_value: float
    description: str
```

#### `parameter_network_client.py`
TCP client for parameter configuration.

**Usage:**
```python
from parameter_network_client import ParameterClient

client = ParameterClient('192.168.2.100', 7000)
client.connect()

# Discover all parameters
params = client.get_all_parameters()
print(f"Found {len(params)} parameters")

# Get current value
value = client.get_parameter('grid_observer', 'kp')
print(f"Current Kp: {value.value}")

# Set new value
success = client.set_parameter('grid_observer', 'kp', 0.5)

client.disconnect()
```

**Features:**
- ✅ Automatic parameter discovery
- ✅ Thread-safe caching
- ✅ Range validation
- ✅ Timeout handling
- ✅ Component filtering

#### `parameter_web_server.py`
Flask REST API and web interface.

**REST API Endpoints:**

| Method | Endpoint                        | Description                      |
| ------ | ------------------------------- | -------------------------------- |
| `GET`  | `/api/parameters`               | Get all parameters with metadata |
| `GET`  | `/api/parameters/<comp>/<name>` | Get specific parameter           |
| `PUT`  | `/api/parameters/<comp>/<name>` | Set parameter value              |
| `GET`  | `/api/components`               | List all components              |
| `GET`  | `/api/components/<comp>`        | Get component parameters         |
| `GET`  | `/`                             | Web interface                    |

**Example API Calls:**
```bash
# Get all parameters
curl http://localhost:5000/api/parameters

# Get specific parameter
curl http://localhost:5000/api/parameters/grid_observer/kp

# Set parameter
curl -X PUT -H "Content-Type: application/json" \
     -d '{"value": 0.5}' \
     http://localhost:5000/api/parameters/grid_observer/kp

# Get all grid observer parameters
curl http://localhost:5000/api/components/grid_observer
```

## 🎨 Web Interface Features

### Modern UI
- **Gradient header** with system branding
- **Collapsible component sections** (click to expand/collapse)
- **Responsive parameter cards** with hover effects
- **Real-time notifications** (success/error toasts)
- **Status indicator** (loading/ready/error)

### Parameter Cards
Each parameter displays:
- **Display name** (human-readable)
- **Unit badge** (Hz, pu, rad, etc.)
- **Description** (tooltip on hover)
- **Input field** with type validation
- **Range display** (min/max/default)
- **Set button** (apply changes)

### Workflow
1. **Automatic discovery**: Page loads and fetches all parameters
2. **Grouped display**: Parameters organized by component
3. **Edit values**: Change input field (validates against range)
4. **Apply changes**: Click "Set" button
5. **Instant feedback**: Success/error notification appears
6. **Refresh**: Click "🔄 Refresh All" to reload from controller

## 📊 Available Parameters (65 total)

### Grid-Side Components (28 parameters)

#### **grid_measurements** (13 params)
- Filtering: `voltage_lpf_cutoff_frequency`, `current_lpf_cutoff_frequency`
- Commutation detection: `commutation_start_threshold`, `commutation_end_threshold`
- ADC configuration: `voltage_adc_bits`, `voltage_adc_full_scale`, etc.
- Scaling: `voltage_transducer_gain`, `pt_ratio`, `ct_ratio`

#### **grid_observer** (5 params)
- PLL tuning: `kp`, `ti`
- Frequency: `nominal_frequency`, `initial_frequency`, `frequency_filter_cutoff`

#### **current_controller** (5 params)
- PI gains: `kp`, `ti`
- Limits: `min_firing_angle`, `max_firing_angle`, `current_reference_magnitude_limit`

#### **grid_pulse_generator** (3 params)
- Control: `enable`, `pulse_width`, `reference_type`

#### **grid_modulator** (2 params)
- Forced commutation: `force_commutation_angle`, `commutation_timeout`

### Machine-Side Components (19 parameters)
Similar structure for machine-side measurements, observer, pulse generator, modulator.

### High-Level Control (18 parameters)
Speed controller, torque controller, excitation controller, soft starter.

**See `PARAMETER_INVENTORY.md` for complete catalog.**

## 🔧 Command-Line Options

```bash
python parameter_web_server.py [OPTIONS]

Options:
  --controller-host HOST   Controller IP address (default: 192.168.2.100)
  --controller-port PORT   Controller TCP port (default: 7000)
  --web-host HOST          Web server host (default: 0.0.0.0)
  --web-port PORT          Web server port (default: 5000)
  --debug                  Enable Flask debug mode
  -h, --help              Show help message
```

**Examples:**
```bash
# Default settings (controller at 192.168.2.100:7000, web UI at 0.0.0.0:5000)
python parameter_web_server.py

# Custom controller address
python parameter_web_server.py --controller-host 192.168.1.100 --controller-port 8000

# Debug mode (auto-reload on code changes)
python parameter_web_server.py --debug
```

## 🧪 Testing

### Protocol Tests
```bash
cd software/yeet/tools
python test_parameter_protocol.py
```

Tests encoding/decoding of all packet types without requiring a live controller.

**Expected output:**
```
======================================================================
Parameter Protocol Test Suite
======================================================================

=== Test: PARAM_LIST_REQUEST Encoding ===
✓ PASS

=== Test: PARAM_GET_REQUEST Encoding ===
✓ PASS

=== Test: PARAM_SET_REQUEST Encoding ===
✓ PASS

=== Test: PARAM_LIST_RESPONSE Decoding ===
✓ PASS

=== Test: PARAM_GET_RESPONSE Decoding ===
✓ PASS

=== Test: PARAM_SET_RESPONSE Decoding ===
✓ PASS

======================================================================
✓ ALL TESTS PASSED
======================================================================
```

## 🔐 Security Considerations

### Current Implementation
- ⚠️ **No authentication** - Anyone with network access can change parameters
- ⚠️ **No encryption** - Commands sent in plaintext over TCP
- ⚠️ **No audit log** - Parameter changes not recorded

### Future Enhancements (TODO)
- [ ] HTTP Basic Auth or JWT tokens
- [ ] TLS/SSL encryption
- [ ] Parameter change logging with timestamps
- [ ] User roles (read-only vs. read-write)
- [ ] Rate limiting to prevent abuse

## 📈 Scalability

### Adding New Components (3 steps)

#### Step 1: C++ - Register in `model_registry`
```cpp
// In grid_side_tasks.cpp or machine_side_tasks.cpp
void initialize_parameters(model_registry& registry) {
    // Existing components...
    
    // NEW COMPONENT - just one line!
    model_registry_entry new_entry;
    new_entry.parameters = new_component_runner.get_parameters();
    registry.insert("new_component", std::move(new_entry));
}
```

#### Step 2: Python - Automatic Discovery
**No code changes needed!** Python automatically discovers the new component via `PARAM_LIST_REQUEST`.

#### Step 3: Web UI - Automatic Rendering
**No code changes needed!** Web interface auto-generates forms from discovered metadata.

### Result
Add 100 C++ parameters → They all appear in web UI automatically! 🎉

## 🎯 Best Practices

### Parameter Naming
- **Component names**: Lowercase with underscores (`grid_observer`, `current_controller`)
- **Parameter names**: Lowercase with underscores (`kp`, `voltage_lpf_cutoff_frequency`)
- **Display names**: Title case with spaces (`PLL Proportional Gain`)

### Range Selection
- Set **realistic ranges** to prevent dangerous values
- Use **min > 0** for parameters that can't be zero (frequencies, gains)
- Set **max < ∞** to prevent overflow

### Units
- Use **standard units**: `Hz`, `pu`, `rad`, `s`, `ms`, `A`, `V`
- Use **`-`** for dimensionless parameters
- Be **consistent** across similar parameters

### Descriptions
- Write **clear, concise** descriptions (< 100 chars)
- Mention **typical values** or use cases
- Warn about **critical parameters** that affect stability

## 🐛 Troubleshooting

### "Failed to connect to controller"
**Causes:**
- Controller not running or network unreachable
- Wrong IP address or port
- Firewall blocking TCP connection

**Solutions:**
```bash
# Check network connectivity
ping 192.168.2.100

# Test TCP port
telnet 192.168.2.100 7000

# Check firewall settings
```

### "Failed to load parameters"
**Causes:**
- Protocol version mismatch (Python vs. C++)
- Timeout waiting for response
- Invalid packet structure

**Solutions:**
- Verify Python `PROTOCOL_VERSION` matches C++ `protocol_version`
- Increase timeout: `ParameterClient(..., timeout=10.0)`
- Run `test_parameter_protocol.py` to verify protocol

### "Failed to set parameter"
**Common errors:**
- `OUT_OF_RANGE`: Value outside [min, max]
- `NOT_FOUND`: Component or parameter name doesn't exist
- `READ_ONLY`: Parameter cannot be changed at runtime

**Solutions:**
- Check parameter range in web UI
- Verify spelling of component/parameter names
- Some parameters may require controller restart

### Parameter changes have no effect
**Possible causes:**
- Parameter not connected to control logic
- Cached value not updated
- Controller needs restart for some parameters

**Debug steps:**
1. Verify SET response status is SUCCESS
2. Query parameter with GET to confirm value changed
3. Check controller logs for errors
4. Test with simulator before hardware

## 📝 Example Session

```bash
# Terminal 1: Start web server
$ python parameter_web_server.py
INFO - Connecting to controller at 192.168.2.100:7000...
INFO - Connected successfully!
INFO - Starting web server on 0.0.0.0:5000...
 * Running on http://0.0.0.0:5000/

# Terminal 2: Test API
$ curl http://localhost:5000/api/components
["grid_observer","current_controller","grid_pulse_generator",...]

$ curl http://localhost:5000/api/parameters/grid_observer/kp
{
  "component": "grid_observer",
  "name": "kp",
  "value": 0.4,
  "metadata": {
    "display_name": "PLL Proportional Gain",
    "unit": "-",
    "min": 0.01,
    "max": 10.0,
    "description": "Proportional gain for grid voltage PLL"
  }
}

$ curl -X PUT -H "Content-Type: application/json" \
       -d '{"value": 0.5}' \
       http://localhost:5000/api/parameters/grid_observer/kp
{
  "status": "success",
  "component": "grid_observer",
  "name": "kp",
  "value": 0.5
}
```

## 🔗 Related Documentation

- **Feature Plan**: `PARAMETER_CONFIGURATION_FEATURE_PLAN.md` - System architecture and implementation phases
- **Parameter Inventory**: `PARAMETER_INVENTORY.md` - Complete catalog of all 65 parameters
- **Telemetry System**: `software/yeet/tools/docs/telemetry_readme.md` - Real-time data streaming
- **Model Registry**: `software/yeet/include/yeet/model_registry.hpp` - C++ parameter registration

## 💡 Tips & Tricks

### Batch Parameter Updates
Use a Python script for bulk updates:
```python
from parameter_network_client import ParameterClient

client = ParameterClient('192.168.2.100', 7000)
client.connect()

# Conservative PLL tuning
client.set_parameter('grid_observer', 'kp', 0.3)
client.set_parameter('grid_observer', 'ti', 50.0)

# Aggressive current control
client.set_parameter('current_controller', 'kp', 1.0)
client.set_parameter('current_controller', 'ti', 50.0)

client.disconnect()
```

### Save/Load Presets
Export current configuration:
```python
params = client.get_all_parameters()
preset = {p.full_name: p.current_value for p in params}

import json
with open('conservative_tuning.json', 'w') as f:
    json.dump(preset, f, indent=2)

# Load preset
with open('conservative_tuning.json', 'r') as f:
    preset = json.load(f)

for full_name, value in preset.items():
    component, name = full_name.split('.')
    client.set_parameter(component, name, value)
```

### Monitor Parameter Changes
Combine with telemetry to see effect of parameter changes:
```bash
# Terminal 1: Monitor telemetry
python telemetry_monitor.py --host 192.168.2.100 --port 7000

# Terminal 2: Change parameters via web UI
# → Observe immediate effect on telemetry plots
```

---

**Version**: 1.0  
**Last Updated**: 2025-10-23  
**Author**: GitHub Copilot
