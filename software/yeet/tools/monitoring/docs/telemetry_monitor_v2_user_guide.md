# SFC Control Center - User Guide v2.0

## Overview

The SFC Control Center is a modernized, tab-based monitoring and configuration application for the Static Frequency Converter system. This version features a clean, contemporary UI with organized tabs for different operational aspects.

---

## 🎨 Modern UI Features

### Tab-Based Organization
The interface is now organized into **four main tabs**:

1. **📊 Telemetry Monitor** - Real-time digital dashboard with signal values
2. **📈 Oscilloscope** - Time-domain waveform visualization  
3. **⚙️ Parameters** - System configuration and tuning
4. **🎮 Commands** - Control operations and test procedures

### Visual Improvements
- **Modern Color Scheme**: Contemporary blue/gray palette with proper contrast
- **Rounded Corners**: Smooth, modern appearance throughout
- **Enhanced Typography**: Improved fonts and sizing for better readability
- **Better Spacing**: Organized layout with appropriate margins and padding
- **Hover Effects**: Interactive feedback on buttons and controls
- **Status Indicators**: Clear visual feedback with color-coded messages

---

## 📋 Tab Details

### 1. 📊 Telemetry Monitor Tab

**Purpose**: Real-time display of all system signals in digital format

**Features**:
- Organized signal groups in collapsible sections
- Color-coded status indicators
- Per-unit (pu) value display
- Enumerated state display (e.g., IDLE, OPERATION_ON)

**Signal Groups**:
- **Grid & DC Measurements**: Voltage, current magnitudes, commutation status
- **Observer**: Estimated voltage, frequency, phase angle
- **Modulator**: Firing angles and enable status
- **Controller**: Current references, errors, saturation
- **State Machine**: System state, breaker status, pulse releases

**Usage**:
1. Connect to controller
2. Click Start to stream telemetry
3. View real-time values updating in the dashboard
4. Values update every 50ms

---

### 2. 📈 Oscilloscope Tab

**Purpose**: Time-domain visualization of key waveforms

**Features**:
- Three synchronized plot areas
- Adjustable time window (0.1s to 60s)
- Clear data function
- Auto-scaling y-axis
- Legend for signal identification

**Plot Areas**:

#### Plot 1: Grid Measurements
- Grid voltage magnitude (V mag)
- Three-phase voltages (Va, Vb, Vc)
- Grid current magnitude (I mag)
- Three-phase currents (Ia, Ib, Ic)
- DC-link current (Idc)

#### Plot 2: Grid Observer
- Estimated voltage magnitude (V est)
- Estimated frequency (f est)

#### Plot 3: DC Link Current Controller
- Current reference (I ref)
- Measured current (I meas)
- Current error (Error)
- Controller output (Output)

**Controls**:
- **Window (s)**: Adjust visible time range
- **Clear Data**: Reset all plot buffers

**Usage**:
1. Connect and start telemetry
2. Switch to Oscilloscope tab
3. Adjust time window as needed
4. Observe waveforms in real-time
5. Click Stop to freeze display for analysis

---

### 3. ⚙️ Parameters Tab

**Purpose**: Configure controller parameters and tune system behavior

**Features**:
- **Search functionality**: Filter parameters by name
- **Grouped by component**: Organized by subsystem
- **Live editing**: Modify parameters in real-time
- **Range validation**: Min/max limits enforced
- **Non-blocking load**: UI stays responsive during parameter discovery
- **Status feedback**: Progress indicators and error messages

**Parameter Groups** (typical):
- Grid Observer (PLL gains, filter settings)
- Current Controller (Kp, Ki, limits)
- Modulator (firing angle limits)
- State Machine (timing parameters)
- Protection (threshold settings)

**Workflow**:
1. Connect to controller
2. Parameters load automatically in background (~2-5 seconds)
3. Use search box to find specific parameter
4. Edit value in input field
5. Click **Set** button to apply
6. Verify change in telemetry/oscilloscope

**Tips**:
- Search for keywords like "gain", "time", "limit"
- Parameters show [min, max] valid range
- Changes apply immediately
- Use Refresh to reload all parameters

---

### 4. 🎮 Commands Tab

**Purpose**: High-level control operations and test configurations

#### 4.1 Rectifier Operation Section

**Rectifier Enable**
- ☑️ **Enable Rectifier Operation**: Master enable for rectifier mode
- Status updates shown in command status area

**DC-Link Current Reference**
- **Range**: 0.0 to 2.0 pu
- **Step**: 0.05 pu
- **Apply Button**: Send reference to controller
- Used during current regulation testing

**Usage**:
```
1. Check "Enable Rectifier Operation"
2. Set DC-Link Current Reference (e.g., 0.5 pu)
3. Click "Apply" 
4. Monitor response in Oscilloscope tab
```

#### 4.2 Testing Procedure Section

**Test Configuration Dropdown**

Available test configurations:

1. **Thyristor Pulsation Test**
   - Objective: Verify thyristor firing and gate pulse generation
   - Tests single thyristor operation
   - Validates firing angle timing

2. **Current Regulation Test**
   - Objective: Validate DC-link current controller
   - Tests closed-loop current control
   - Verifies tracking and settling time

3. **Breaker Commands Test**
   - Objective: Test circuit breaker control
   - Validates breaker status feedback
   - Tests interlock logic

4. **Grid Observer Validation Test**
   - Objective: Verify voltage and frequency estimation
   - Compares estimated vs. measured values
   - Validates phase tracking

5. **Phase-Locked Loop Test**
   - Objective: Validate PLL performance
   - Tests lock time and phase error
   - Verifies frequency tracking

6. **Commutation Analysis Test**
   - Objective: Analyze commutation process
   - Monitors overlap angle
   - Detects commutation failures

7. **Full System Integration Test**
   - Objective: Comprehensive validation
   - Tests all subsystems together
   - Validates startup and shutdown sequences

**Test Workflow**:
```
1. Select test from dropdown
2. Read test description (appears below)
3. Click "Configure Test" button
4. System applies test-specific settings
5. Switch to Telemetry/Oscilloscope tabs to monitor
6. Follow test procedure in description
```

**Test Description Area**:
- Shows detailed test objectives
- Lists procedure steps
- Defines expected results
- Formatted with HTML for clarity

#### 4.3 Command Status Section

Real-time status display showing:
- Active commands
- Test configurations
- Applied references
- Error messages

**Status Colors**:
- 🔵 Blue background: Informational
- ✅ Success indicators for applied commands
- ⚠️ Warnings when needed

---

## 🎛️ Control Panel (Always Visible)

Located at the top of the window, accessible from any tab:

### Connection Controls
- **Host**: IP address (default: localhost)
- **Port**: TCP port (default: 7000)
- **Connect**: Establish connection
- **Disconnect**: Close connection

### Telemetry Controls
- **▶ Start**: Begin telemetry streaming (green button)
- **⏸ Stop**: Pause telemetry streaming (red button)

### View Controls
- **Window (s)**: Oscilloscope time window
- **Clear Data**: Reset oscilloscope buffers

### Statistics
- **Packets**: Total received
- **Loss**: Packet loss percentage

---

## 🚀 Quick Start Guide

### First-Time Setup
```
1. Launch application: python telemetry_monitor_with_parameters.py --host localhost --port 7000
2. Window opens at 1400x900 pixels
3. Default tab: Telemetry Monitor
```

### Basic Operation
```
1. Click "Connect" button
2. Wait for "Connected to host:port" status message
3. Parameters load automatically (UI stays responsive)
4. Click "▶ Start" to begin telemetry streaming
5. Switch between tabs to view different aspects
6. Click "⏸ Stop" to freeze display
7. Click "Disconnect" when done
```

### Parameter Tuning
```
1. Connect to controller
2. Click "Parameters" tab
3. Wait for parameters to load (status shown at bottom)
4. Search for desired parameter
5. Edit value and click "Set"
6. Switch to Oscilloscope to see effect
```

### Running a Test
```
1. Connect to controller
2. Click "Commands" tab
3. Select test from "Configure for test" dropdown
4. Read test description
5. Click "Configure Test" button
6. Follow procedure in description
7. Monitor results in other tabs
```

---

## 💡 Tips and Best Practices

### Performance Optimization
- **Adjust window size**: Smaller time windows = better performance
- **Clear data periodically**: Prevents memory buildup
- **Stop when not monitoring**: Reduces network traffic

### Effective Troubleshooting
- **Check connection first**: Verify host/port settings
- **Review status bar**: Always check for error messages
- **Use Stop before changes**: Pause telemetry when configuring
- **Monitor packet loss**: High loss indicates network issues

### Efficient Parameter Tuning
- **Stop before major changes**: Pause telemetry when changing multiple parameters
- **One parameter at a time**: Verify effects individually
- **Use search extensively**: Don't scroll through all parameters
- **Document changes**: Note parameter values that work well

### Test Execution
- **Read descriptions fully**: Understand objectives before starting
- **Prepare monitoring**: Set up oscilloscope view before test
- **Follow procedures**: Tests assume specific sequences
- **Record observations**: Note any unexpected behavior

---

## 🔧 Keyboard Shortcuts (Recommended for Future)

Currently implemented shortcuts:
- **Tab**: Cycle through input fields
- **Enter**: Apply parameter changes (when input focused)
- **Escape**: Clear search box

Suggested future shortcuts:
- **Ctrl+K**: Focus search box
- **Ctrl+R**: Refresh parameters
- **Space**: Toggle Start/Stop telemetry
- **Ctrl+L**: Clear oscilloscope data
- **Ctrl+1/2/3/4**: Switch tabs directly

---

## 📊 Status Indicators Reference

### Connection Status
- "Disconnected" (gray) - No connection
- "Connected to host:port" (normal) - Active connection
- "Connection Failed" (error) - Unable to connect

### Telemetry Status
- "Telemetry streaming started" - Data flowing
- "Telemetry streaming stopped" - Data paused
- "Packets: X" - Received count
- "Loss: X%" - Packet loss rate

### Parameter Status
- "No parameters loaded" - Initial state
- "Discovering parameters..." - Loading in progress
- "✓ Loaded X parameters from Y components" - Success
- "✗ Failed to connect to controller" - Error

### Command Status
- "Ready - No commands active" - Initial state
- "Rectifier operation enabled/disabled" - State change
- "DC-Link current reference set to X pu" - Applied reference
- "Test configured: Test Name" - Test applied

---

## 🎨 Color Scheme Reference

### Button Colors
- **Blue** (#3b82f6): Standard actions
- **Green** (#10b981): Start/Enable actions
- **Red** (#ef4444): Stop/Disable actions
- **Gray** (#9ca3af): Disabled state

### Background Colors
- **White** (#ffffff): Main content areas
- **Light Gray** (#f5f7fa): Window background
- **Light Blue** (#f0f9ff): Telemetry header
- **Light Yellow** (#fef3c7): Parameters header
- **Very Light Gray** (#f9fafb): Status areas

### Border Colors
- **Light Gray** (#e5e7eb): Standard borders
- **Medium Gray** (#d1d5db): Input borders
- **Blue** (#3b82f6): Active/focused borders
- **Tab Accent** (#3b82f6): Selected tab indicator

---

## 🔍 Troubleshooting Guide

### Problem: Application won't start
**Symptoms**: Python import error or window doesn't appear
**Solutions**:
1. Verify Python dependencies installed: `pip install pyqt5 pyqtgraph`
2. Check Python version (requires 3.7+)
3. Ensure all required modules present in tools directory

### Problem: Can't connect to controller
**Symptoms**: "Connection Failed" message box
**Solutions**:
1. Verify controller application is running
2. Check host IP address is correct
3. Verify port number (default 7000)
4. Test network connectivity: `ping <host>`
5. Check firewall settings

### Problem: Parameters not loading
**Symptoms**: "Failed to connect to controller" in parameter status
**Solutions**:
1. Verify connection established first
2. Click "Refresh Parameters" to retry
3. Check controller logs for errors
4. Ensure controller has parameters registered

### Problem: Oscilloscope not updating
**Symptoms**: Plots frozen, no new data
**Solutions**:
1. Verify telemetry is started (Stop button should be enabled)
2. Click "▶ Start" button
3. Check packet statistics (should be increasing)
4. Verify controller is sending telemetry

### Problem: UI feels sluggish
**Symptoms**: Slow response, lag in updates
**Solutions**:
1. Reduce oscilloscope window size (e.g., 5s instead of 30s)
2. Clear oscilloscope data: Click "Clear Data"
3. Stop telemetry when not needed
4. Close other resource-intensive applications

### Problem: Parameter Set fails
**Symptoms**: Error message when clicking "Set" button
**Solutions**:
1. Check value is within [min, max] range
2. Verify parameter is writable (not read-only)
3. Confirm connection is still active
4. Check controller logs for rejection reasons

### Problem: Test configuration has no effect
**Symptoms**: Clicking "Configure Test" doesn't change behavior
**Solutions**:
1. Verify connection is active
2. Check command status for confirmation
3. Monitor telemetry tab for state changes
4. Ensure test prerequisites are met (e.g., breakers closed)
5. Review controller logs for errors

---

## 📚 Related Documentation

- **Technical Details**: See `telemetry_monitor_improvements.md`
- **UI Layout Guide**: See `telemetry_monitor_ui_guide.md` (legacy, panel-based)
- **Parameter Protocol**: See parameter system documentation
- **Telemetry Protocol**: See telemetry client documentation

---

## 🆕 What's New in v2.0

### Major Changes
✅ **Tab-based interface** replacing three-panel layout
✅ **Commands tab** with rectifier control and test configurations
✅ **Modern UI styling** with contemporary colors and rounded corners
✅ **Improved typography** and spacing throughout
✅ **Better visual hierarchy** with color-coded section headers
✅ **Enhanced user feedback** with styled status messages

### Benefits
- **Cleaner organization**: Each functional area has its own space
- **More screen real estate**: Tabs maximize content area
- **Better focus**: Work on one aspect at a time
- **Easier navigation**: Clear tab labels with icons
- **Professional appearance**: Modern, polished look

### Breaking Changes
- Layout changed from split panels to tabs
- Control panel always visible at top (not tabbed)
- Parameter panel has new header styling
- Status messages have new formatting

---

**Version**: 2.0  
**Last Updated**: 2025-10-24  
**Author**: GitHub Copilot  
**License**: Same as parent project
