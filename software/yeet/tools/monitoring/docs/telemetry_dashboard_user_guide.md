# SFC Telemetry Dashboard - User Guide

## Quick Start

### Installation

1. **Install Python Dependencies**:
   ```bash
   pip install PyQt5
   ```

2. **Navigate to Tools Directory**:
   ```bash
   cd software/yeet/tools
   ```

3. **Run Dashboard**:
   ```bash
   python telemetry_dashboard.py --host 192.168.2.100 --port 7000
   ```

### First Connection

1. **Configure Connection**:
   - Enter controller IP address (default: 192.168.2.100)
   - Enter controller TCP port (default: 7000)
   - Click "Connect"

2. **Start Telemetry**:
   - Click "Start Streaming"
   - Signals will begin updating in real-time
   - All signal groups display current values

3. **Monitor Data**:
   - Watch digital displays update at 20 Hz
   - Check connection statistics for packet loss
   - Signals organized by functional group

4. **Stop Telemetry**:
   - Click "Stop Streaming" to pause
   - Click "Clear Data" to reset buffers
   - Click "Disconnect" to close connection

## User Interface Overview

### Window Layout

```
┌────────────────────────────────────────────────────────────┐
│ SFC Telemetry Dashboard                             [_][□][X]│
├────────────────────────────────────────────────────────────┤
│ ┌─ Connection ─────────────────────────────────────────┐  │
│ │ Host: [192.168.2.100] Port: [7000]                   │  │
│ │ [Connect]             [Disconnect]                    │  │
│ └──────────────────────────────────────────────────────┘  │
│ ┌─ Telemetry Control ──────────────────────────────────┐  │
│ │ [Start Streaming] [Stop Streaming] [Clear Data]      │  │
│ └──────────────────────────────────────────────────────┘  │
│ ┌─ Grid & DC Measurements ─────────────────────────────┐  │
│ │ Grid Voltage Mag.:   [█ 1.023 █] pu                  │  │
│ │ Grid Voltage Phase A:[█ 0.987 █] pu                  │  │
│ │ Grid Voltage Phase B:[█ 0.995 █] pu                  │  │
│ │ Grid Voltage Phase C:[█ 1.012 █] pu                  │  │
│ │ DC Link Current:     [█ 0.456 █] pu                  │  │
│ └──────────────────────────────────────────────────────┘  │
│ ┌─ Grid Observer ──────────────────────────────────────┐  │
│ │ Estimated Frequency: [█ 1.000 █] pu                  │  │
│ │ Estimated Angle:     [█ 2.456 █] rad                 │  │
│ └──────────────────────────────────────────────────────┘  │
│ ┌─ Grid-Side Modulator ────────────────────────────────┐  │
│ │ Firing Angle Ref.:   [█ 1.234 █] rad                 │  │
│ │ Firing Angle Actual: [█ 1.235 █] rad                 │  │
│ │ Force Commutation:   [█   0   █]                     │  │
│ └──────────────────────────────────────────────────────┘  │
│ ┌─ Current Controller ─────────────────────────────────┐  │
│ │ Current Error:       [█ 0.001 █] pu                  │  │
│ │ Controller Output:   [█ 1.234 █] pu                  │  │
│ │ Operating Mode:      [█   1   █]                     │  │
│ │ Controller Saturated:[█   0   █]                     │  │
│ │ Freeze Integrator:   [█   0   █]                     │  │
│ └──────────────────────────────────────────────────────┘  │
│ ┌─ State Machine ──────────────────────────────────────┐  │
│ │ Current State:       [█   3   █]                     │  │
│ │ Pulse Releases:      [█   1   █]                     │  │
│ └──────────────────────────────────────────────────────┘  │
│ ┌─ Connection Statistics ──────────────────────────────┐  │
│ │ Packets Received: 1234  Packets Lost: 0              │  │
│ │ Loss Rate: 0.00%  Bytes: 88848  Errors: 0            │  │
│ └──────────────────────────────────────────────────────┘  │
│ Status: Streaming telemetry...                             │
└────────────────────────────────────────────────────────────┘
```

### UI Components

#### Connection Group
- **Host Field**: IP address of embedded controller
- **Port Field**: TCP port number (typically 7000)
- **Connect Button**: Establish TCP connection
- **Disconnect Button**: Close TCP connection

#### Telemetry Control Group
- **Start Streaming Button**: Send START command to controller
- **Stop Streaming Button**: Send STOP command to controller
- **Clear Data Button**: Clear all buffered data and reset statistics

#### Signal Display Groups

Each group displays related signals in LCD-style digital displays:

**Grid & DC Measurements**:
- Grid voltage magnitude and per-phase values (pu)
- DC link current (pu)

**Grid Observer**:
- Estimated grid frequency (pu, 1.0 = 60 Hz)
- Estimated grid angle (radians, 0 to 2π)

**Grid-Side Modulator**:
- Commanded and actual firing angles (radians)
- Force commutation acknowledgment flag

**Current Controller**:
- Control error and PI output (pu)
- Operating mode (0=forced, 1=natural commutation)
- Status flags (saturated, freeze integrator)

**State Machine**:
- Current state enumeration value
- Pulse releases enabled flag

#### Connection Statistics Group
- **Packets Received**: Total telemetry packets decoded
- **Packets Lost**: Detected sequence number gaps
- **Loss Rate**: Percentage of packets lost
- **Bytes Received**: Total data received from controller
- **Decode Errors**: Invalid packets rejected

#### Status Bar
- Shows current connection and streaming status
- Displays last action or error message

## Operating Procedures

### Normal Operation Sequence

1. **Power On Controller**:
   - Ensure embedded controller is running
   - Wait for network interface to initialize
   - Controller should be reachable via ping

2. **Launch Dashboard**:
   ```bash
   python telemetry_dashboard.py --host <controller-ip>
   ```

3. **Establish Connection**:
   - Verify host/port settings
   - Click "Connect"
   - Status bar shows "Connected to <ip>:<port>"

4. **Begin Monitoring**:
   - Click "Start Streaming"
   - All signal displays activate
   - Values update at 20 Hz

5. **Monitor Operation**:
   - Watch signals for expected behavior
   - Check statistics for packet loss
   - All groups should show live data

6. **Stop Monitoring**:
   - Click "Stop Streaming" when done
   - Signals freeze at last values
   - Can restart streaming without reconnecting

7. **Disconnect**:
   - Click "Disconnect"
   - Dashboard ready for new connection

### Troubleshooting Common Issues

#### "Connection Failed" Error

**Symptoms**: Cannot connect to controller

**Possible Causes**:
1. Controller not powered or network interface down
2. Incorrect IP address or port number
3. Network cable disconnected
4. Firewall blocking connection

**Solutions**:
1. Verify controller is running: `ping <controller-ip>`
2. Check IP address configuration on controller
3. Verify port 7000 is open on both ends
4. Try connecting from another PC to isolate network issues

#### No Data After START Command

**Symptoms**: Connected but signals show "--"

**Possible Causes**:
1. Controller not in correct state to stream telemetry
2. Control loop not running
3. Network receive thread failed

**Solutions**:
1. Check controller logs for errors
2. Verify control loop is executing (check debug outputs)
3. Restart dashboard application
4. Power cycle controller

#### High Packet Loss

**Symptoms**: Loss rate > 1%, signals unstable

**Possible Causes**:
1. Network congestion or poor WiFi signal
2. Controller CPU overload
3. Network buffer overflow

**Solutions**:
1. Use wired Ethernet instead of WiFi
2. Reduce other network traffic
3. Check controller CPU usage
4. Increase TCP buffer sizes on controller

#### UI Freezing or Slow Response

**Symptoms**: Dashboard unresponsive, slow updates

**Possible Causes**:
1. Host PC CPU overload
2. Too many simultaneous applications
3. Display driver issues

**Solutions**:
1. Close other CPU-intensive applications
2. Reduce UI update rate in code if needed
3. Check host PC performance monitor

## Signal Interpretation

### Per-Unit Values

Most signals use per-unit (pu) scaling:
- **1.0 pu** = nominal/rated value
- **Grid voltage**: 1.0 pu = rated line-to-neutral voltage
- **DC current**: 1.0 pu = rated DC link current

### Angle Values

Angles in radians (0 to 2π or ±π depending on signal):
- **Grid angle**: 0 to 2π (0 = phase A zero crossing)
- **Firing angle**: Typically 0 to π (thyristor delay angle)

### Boolean Flags

Binary signals (0 or 1):
- **0**: False, inactive, disabled
- **1**: True, active, enabled

Examples:
- Pulse releases: 0=blocked, 1=enabled
- Saturated: 0=normal, 1=saturated
- Operating mode: 0=forced commutation, 1=natural commutation

### State Machine Values

State machine current state is an enumeration:
- **0**: IDLE / RESET
- **1**: INITIALIZATION
- **2**: READY
- **3**: STARTING
- **4**: RUNNING
- **5**: STOPPING
- **6**: FAULT

*(Exact mapping depends on embedded code state machine definition)*

## Expected Signal Behavior

### During Controller Startup

1. **Initial State** (0-2 seconds):
   - All measurements zero or near-zero
   - State machine in IDLE (0) or INIT (1)
   - Pulse releases disabled (0)

2. **Initialization** (2-5 seconds):
   - Grid observer acquiring frequency and angle
   - Measurements showing actual grid values
   - State transitions to READY (2)

3. **Ready State**:
   - All measurements stable
   - Observer locked to grid
   - Waiting for start command

### During Motor Starting Sequence

1. **Pre-Start**:
   - Firing angle at maximum (late delay)
   - DC current at zero
   - State machine transitions to STARTING (3)

2. **Current Ramp**:
   - DC current increases gradually
   - Firing angle decreases to regulate current
   - Controller output tracking current reference

3. **Running**:
   - DC current at commanded level
   - Firing angle stable
   - State machine in RUNNING (4)
   - Operating mode transitions to natural (1)

### Normal Operating Ranges

Expected signal ranges during typical operation:

| Signal              | Normal Range   | Alarm Threshold     |
| ------------------- | -------------- | ------------------- |
| Grid Voltage Mag.   | 0.9 - 1.1 pu   | < 0.85 or > 1.15 pu |
| DC Link Current     | 0.0 - 1.1 pu   | > 1.2 pu            |
| Estimated Frequency | 0.98 - 1.02 pu | < 0.95 or > 1.05 pu |
| Current Error       | -0.1 - 0.1 pu  | > 0.2 pu            |
| Firing Angle        | 0.5 - 2.5 rad  | < 0.3 or > 2.8 rad  |

## Command Line Options

### Basic Usage
```bash
python telemetry_dashboard.py [OPTIONS]
```

### Available Options

**`--host <ip-address>`**
- Controller IP address
- Default: `192.168.2.100`
- Example: `--host 192.168.1.50`

**`--port <port-number>`**
- Controller TCP port
- Default: `7000`
- Example: `--port 8000`

**`--buffer-size <samples>`**
- Signal history buffer size (samples per signal)
- Default: `10000`
- Larger values use more memory but support longer data retention
- Example: `--buffer-size 20000`

### Example Commands

**Connect to default controller**:
```bash
python telemetry_dashboard.py
```

**Connect to specific IP**:
```bash
python telemetry_dashboard.py --host 192.168.1.100
```

**Connect with custom port**:
```bash
python telemetry_dashboard.py --host 192.168.2.100 --port 8000
```

**Large buffer for extended recording**:
```bash
python telemetry_dashboard.py --buffer-size 50000
```

**Full custom configuration**:
```bash
python telemetry_dashboard.py --host 192.168.1.50 --port 7500 --buffer-size 20000
```

## Keyboard Shortcuts

*(Currently not implemented, planned for future versions)*

Planned shortcuts:
- **Ctrl+C**: Connect
- **Ctrl+D**: Disconnect
- **Ctrl+S**: Start streaming
- **Ctrl+T**: Stop streaming
- **Ctrl+R**: Clear data
- **Ctrl+Q**: Quit application

## Performance Tips

### For Best Performance

1. **Use Wired Ethernet**: Reduces latency and packet loss
2. **Close Unnecessary Programs**: Free up CPU for dashboard
3. **Minimize Window Movement**: Reduces rendering overhead during updates
4. **Disable Window Transparency**: If using compositing window manager

### For Limited Resources

1. **Reduce Buffer Size**: Use `--buffer-size 5000` to save memory
2. **Lower Update Rate**: Modify `UI_UPDATE_INTERVAL_MS` in code (e.g., 100 ms)
3. **Disable Unused Groups**: Comment out groups in `_create_signal_group_displays()`

## Log Files

The dashboard writes log messages to console (stdout):

**Log Levels**:
- **INFO**: Normal operations (connection, commands)
- **WARNING**: Recoverable issues (packet loss, decode errors)
- **ERROR**: Serious problems (connection failures)

**Redirect to File**:
```bash
python telemetry_dashboard.py > dashboard.log 2>&1
```

**Enable Debug Logging**:
Modify in `telemetry_dashboard.py`:
```python
logging.basicConfig(level=logging.DEBUG, ...)
```

## Support and Maintenance

### Reporting Issues

When reporting problems, include:
1. Dashboard version (check file header)
2. Python version: `python --version`
3. PyQt5 version: `pip show PyQt5`
4. Operating system and version
5. Controller firmware version
6. Full error message and log output
7. Steps to reproduce issue

### Updating Dashboard

1. **Check for Updates**:
   - New dashboard files in repository
   - Updated protocol definitions

2. **Apply Updates**:
   ```bash
   git pull origin main
   # Dashboard files updated automatically
   ```

3. **Verify Compatibility**:
   - Check protocol version matches embedded firmware
   - Test connection with updated dashboard

### Common Maintenance Tasks

**Reset Statistics**:
- Click "Clear Data" button
- Or disconnect and reconnect

**Change Connection**:
- Click "Disconnect"
- Update host/port fields
- Click "Connect"

**Restart Dashboard**:
- Close window or Ctrl+C in terminal
- Relaunch: `python telemetry_dashboard.py`

## FAQ

### Q: Why do signals show "--" when connected?
**A**: Telemetry streaming not started. Click "Start Streaming" button.

### Q: Why do I see packet loss?
**A**: Normal for <0.1%. Higher rates indicate network issues. Use wired connection.

### Q: Can I record data for later analysis?
**A**: Not yet implemented. Planned for Phase 3 (data logging feature).

### Q: How do I view signal trends over time?
**A**: Not yet implemented. Planned for Phase 2 (plotting feature).

### Q: Can I control the system from the dashboard?
**A**: Only START/STOP telemetry commands currently. Parameter control planned for Phase 4.

### Q: Why are some signals always zero?
**A**: Those signals may not be implemented in the embedded code yet (marked as reserved).

### Q: What's the maximum buffer size?
**A**: Limited by available RAM. 100,000 samples ≈ 40 MB. Use `--buffer-size` to adjust.

### Q: Can I run multiple dashboards simultaneously?
**A**: Yes, but each dashboard needs its own TCP connection. Only one START command will be active.

## Next Steps

After mastering basic operation:

1. **Phase 2 - Plotting**: Add real-time waveform plots (see architecture doc)
2. **Phase 3 - Logging**: Record telemetry to files for post-analysis
3. **Phase 4 - Control**: Send configuration and control commands

See `docs/telemetry_dashboard_architecture.md` for extension details.
