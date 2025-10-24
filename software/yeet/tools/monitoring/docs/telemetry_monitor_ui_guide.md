# Telemetry Monitor UI Guide

## Control Panel Layout

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              Control Panel                                          │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│  Host: [192.168.2.100 ] Port: [7000] [Connect] [Disconnect]  │                     │
│                                                                                     │
│  Telemetry: [▶ Start] [⏸ Stop]  │  Window (s): [5.0 ] [Clear Data]  │            │
│                                                                                     │
│  Packets: 1234    Loss: 0.5%                                                       │
│                                                                                     │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## Button States Flow

### Initial State (Disconnected)
```
[Connect]      - ENABLED  (green/default)
[Disconnect]   - DISABLED (gray)
[▶ Start]      - DISABLED (gray)
[⏸ Stop]       - DISABLED (gray)
```

### After Clicking Connect
```
[Connect]      - DISABLED (gray)
[Disconnect]   - ENABLED  (default)
[▶ Start]      - ENABLED  (green, bold)
[⏸ Stop]       - DISABLED (gray)

Status Bar: "Connected to 192.168.2.100:7000"
Parameters: Loading in background...
```

### After Clicking Start (Telemetry Active)
```
[Connect]      - DISABLED (gray)
[Disconnect]   - ENABLED  (default)
[▶ Start]      - DISABLED (gray)
[⏸ Stop]       - ENABLED  (red, bold)

Status Bar: "Telemetry streaming started"
Oscilloscope: Updating in real-time
```

### After Clicking Stop (Telemetry Paused)
```
[Connect]      - DISABLED (gray)
[Disconnect]   - ENABLED  (default)
[▶ Start]      - ENABLED  (green, bold)
[⏸ Stop]       - DISABLED (gray)

Status Bar: "Telemetry streaming stopped"
Oscilloscope: Frozen at last frame
```

---

## Main Window Layout

```
┌────────────────────────────────────────────────────────────────────────────────────┐
│                               SFC Monitor - Telemetry + Parameters                 │
├────────────────────────────────────────────────────────────────────────────────────┤
│                                Control Panel                                       │
│  [Connection Controls] │ [Telemetry Controls] │ [View Controls] │ [Statistics]    │
├───────────────────────────┬────────────────────────────────────────────────────────┤
│                           │                                                        │
│   Digital Dashboard       │              Oscilloscope Plots                       │
│   ┌──────────────────┐    │   ┌────────────────────────────────────────────────┐ │
│   │ Grid & DC Meas   │    │   │  Grid Measurements                             │ │
│   │ • V Mag: 1.00 pu │    │   │  [Real-time waveforms]                         │ │
│   │ • I Mag: 0.45 pu │    │   └────────────────────────────────────────────────┘ │
│   │ • I DC:  0.50 pu │    │                                                        │
│   └──────────────────┘    │   ┌────────────────────────────────────────────────┐ │
│                           │   │  Grid Observer                                 │ │
│   ┌──────────────────┐    │   │  [Real-time waveforms]                         │ │
│   │ Observer         │    │   └────────────────────────────────────────────────┘ │
│   │ • V Est: 1.00 pu │    │                                                        │
│   │ • f Est: 1.00 pu │    │   ┌────────────────────────────────────────────────┐ │
│   └──────────────────┘    │   │  DC Link Current Controller                    │ │
│                           │   │  [Real-time waveforms]                         │ │
│   ┌──────────────────┐    │   └────────────────────────────────────────────────┘ │
│   │ [More Groups]    │    │                                                        │
│   └──────────────────┘    │                                                        │
│                           │                                                        │
├───────────────────────────┴────────────────────────────────────────────────────────┤
│                           Parameter Configuration Panel                            │
│  Search: [____________]  [🔄 Refresh Parameters]                                  │
│  ┌──────────────────────────────────────────────────────────────────────────────┐ │
│  │  ╔═══════════════════════════════════════════════════════════════════════╗  │ │
│  │  ║ Grid Observer                                                         ║  │ │
│  │  ╠═══════════════════════════════════════════════════════════════════════╣  │ │
│  │  ║ PLL Proportional Gain:  [0.4000] - [0.01, 10.0]  [Set]             ║  │ │
│  │  ║ PLL Integral Time:      [50.00 ] s [1.0, 200.0]  [Set]             ║  │ │
│  │  ╚═══════════════════════════════════════════════════════════════════════╝  │ │
│  │                                                                              │ │
│  │  ╔═══════════════════════════════════════════════════════════════════════╗  │ │
│  │  ║ Current Controller                                                    ║  │ │
│  │  ╠═══════════════════════════════════════════════════════════════════════╣  │ │
│  │  ║ Controller Kp:          [0.8000] - [0.01, 5.0]   [Set]             ║  │ │
│  │  ║ Controller Ti:          [100.0 ] s [10.0, 500.0] [Set]             ║  │ │
│  │  ╚═══════════════════════════════════════════════════════════════════════╝  │ │
│  │                                                                              │ │
│  └──────────────────────────────────────────────────────────────────────────────┘ │
│  Status: ✓ Loaded 45 parameters from 8 components                                │
├────────────────────────────────────────────────────────────────────────────────────┤
│  Status Bar: Telemetry streaming started                                          │
└────────────────────────────────────────────────────────────────────────────────────┘
```

---

## Parameter Panel States

### Initial State
```
┌────────────────────────────────────────────────────────────┐
│  Search: [____________]  [🔄 Refresh Parameters]          │
│                                                            │
│  No parameters loaded. Click 'Refresh Parameters' to load.│
└────────────────────────────────────────────────────────────┘
```

### Loading State (Non-Blocking!)
```
┌────────────────────────────────────────────────────────────┐
│  Search: [____________]  [🔄 Refresh Parameters]          │
│                                                            │
│  Discovering parameters...                                │
│  (UI remains responsive - you can resize, click, etc.)    │
└────────────────────────────────────────────────────────────┘
```

### Loaded State
```
┌────────────────────────────────────────────────────────────┐
│  Search: [____________]  [🔄 Refresh Parameters]          │
│  ╔════════════════════════════════════════════════════════╗│
│  ║ Grid Observer                                          ║│
│  ║ PLL Proportional Gain: [0.4000] - [0.01, 10.0] [Set] ║│
│  ║ PLL Integral Time:     [50.00 ] s [1.0, 200.0] [Set] ║│
│  ╚════════════════════════════════════════════════════════╝│
│  ✓ Loaded 45 parameters from 8 components                 │
└────────────────────────────────────────────────────────────┘
```

### Error State
```
┌────────────────────────────────────────────────────────────┐
│  Search: [____________]  [🔄 Refresh Parameters]          │
│                                                            │
│  ✗ Failed to connect to controller                        │
│  (Retry by clicking Refresh Parameters)                   │
└────────────────────────────────────────────────────────────┘
```

---

## Usage Scenarios

### Scenario 1: Quick Look at Telemetry
```
1. [Connect] → Connection established
2. [▶ Start] → Telemetry streams, oscilloscope updates
3. Observe waveforms for a few seconds
4. [⏸ Stop] → Pause to analyze frozen waveform
5. [Disconnect] → Clean exit
```

### Scenario 2: Tune Controller Parameters
```
1. [Connect] → Connection established
2. Wait for parameters to load (status updates shown)
3. Search for "controller"
4. Change "Controller Kp" to 1.0
5. [Set] → New value applied
6. [▶ Start] → Start telemetry to see effect
7. Observe controller response
8. [⏸ Stop] → Pause to analyze
9. Adjust parameters again if needed
```

### Scenario 3: Multiple Start/Stop Cycles
```
1. [Connect] → Connection established
2. [▶ Start] → Stream telemetry
3. [⏸ Stop] → Pause to change parameters
4. Adjust multiple parameters
5. [▶ Start] → Resume to see changes
6. [⏸ Stop] → Pause to adjust more
7. Repeat as needed
8. [Disconnect] → When done
```

### Scenario 4: Search and Configure Specific Parameter
```
1. [Connect] → Connection established
2. Parameters load in background (UI stays responsive)
3. In search box, type "frequency"
4. Only frequency-related parameters shown
5. Edit desired parameter
6. [Set] → Apply change
7. Clear search to see all parameters again
```

---

## Keyboard Shortcuts (Future Enhancement)

Suggested shortcuts for power users:

```
Ctrl+K     - Focus search box
Ctrl+R     - Refresh parameters
Space      - Start/Stop telemetry (toggle)
Ctrl+L     - Clear oscilloscope data
Ctrl+Q     - Quit application
Enter      - Apply parameter change (when input focused)
Esc        - Cancel parameter edit
```

---

## Color Coding

### Button Colors
- **Green** (▶ Start): Safe action, begins streaming
- **Red** (⏸ Stop): Caution action, stops streaming
- **Blue** (default buttons): Standard actions
- **Gray** (disabled): Not available in current state

### Status Messages
- **Green text**: Success messages (✓)
- **Orange text**: In-progress/loading (⚠)
- **Red text**: Error messages (✗)
- **Gray text**: Neutral information

### Parameter Groups
- **Blue borders**: Default component group style
- **Hover**: Slight highlight for interactivity
- **Modified**: Background tint when parameter changed (not yet implemented)

---

## Performance Tips

### For Large Parameter Lists (100+)
- Use search box to filter parameters
- Only visible parameters are rendered
- Smooth scrolling maintained

### For High-Rate Telemetry
- Adjust window size to reduce points plotted
- Auto-downsampling prevents performance degradation
- 20 Hz update rate maintained even with long time windows

### For Slow Connections
- Parameter loading shows progress
- Timeout: 5 seconds per parameter
- Retry: 3 attempts per parameter
- Can continue using UI during load

---

## Troubleshooting

### Issue: Parameters Not Loading
**Symptoms**: "Failed to connect to controller" message
**Solutions**:
1. Verify controller is running and accessible
2. Check host/port settings
3. Ensure controller has parameters registered
4. Click "Refresh Parameters" to retry

### Issue: Telemetry Not Updating
**Symptoms**: Oscilloscope frozen, dashboard shows old values
**Solutions**:
1. Check if telemetry is started (green Start button should be disabled)
2. Click "▶ Start" button
3. Verify controller is sending telemetry packets
4. Check packet statistics (should be increasing)

### Issue: UI Feels Sluggish
**Symptoms**: Slow response to clicks, delayed rendering
**Solutions**:
1. Reduce oscilloscope window size (Window spin box)
2. Close other heavy applications
3. Clear oscilloscope data periodically
4. Stop telemetry when not needed

### Issue: Parameter Set Fails
**Symptoms**: "✗ Failed to set parameter" message
**Solutions**:
1. Check if value is within valid range [min, max]
2. Verify parameter is writable (not read-only)
3. Check connection to controller
4. Review controller logs for errors

---

## Best Practices

### 1. Connection Management
✅ **Do**: Click Disconnect when done to free resources
✅ **Do**: Use Start/Stop for pause/resume (don't disconnect/reconnect)
❌ **Don't**: Leave application connected when not using it

### 2. Parameter Configuration
✅ **Do**: Stop telemetry before changing multiple parameters
✅ **Do**: Verify parameter effects one at a time
✅ **Do**: Use search to find specific parameters quickly
❌ **Don't**: Change many parameters simultaneously without testing

### 3. Telemetry Monitoring
✅ **Do**: Use appropriate time window for your analysis
✅ **Do**: Clear data periodically to maintain performance
✅ **Do**: Pause telemetry to examine specific waveforms
❌ **Don't**: Keep very long time windows (>30s) unnecessarily

### 4. Workflow Organization
✅ **Do**: Connect → Load params → Configure → Start telemetry → Observe
✅ **Do**: Stop telemetry while tuning parameters
✅ **Do**: Refresh parameters after controller reset
❌ **Don't**: Try to configure parameters while telemetry streaming is intensive

---

**Version**: 2.0  
**Last Updated**: 2025-10-24
