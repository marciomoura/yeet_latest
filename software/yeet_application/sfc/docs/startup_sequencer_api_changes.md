# Startup Sequencer API Changes

> **Note**: This document describes API changes from v1.0 to v2.0. For complete documentation of the current implementation, see [Startup Sequencer Documentation](startup_sequencer.md).

## Summary of Changes

The startup sequencer has been updated with the following improvements:

### 1. Grouped Configuration Structures

**Before:**
```cpp
struct startup_sequencer_config {
    current_pu_t flux_buildup_current_setpoint;
    duration_t flux_buildup_duration;
    frequency_t virtual_frequency_initial;
    frequency_t virtual_frequency_final;
    duration_t virtual_frequency_duration;
    torque_pu_t starting_torque;
};

sequencer.configure(config);
```

**After:**
```cpp
// Three separate configuration structures
struct flux_buildup_config {
    current_pu_t field_current_setpoint;
    duration_t duration;
};

struct virtual_frequency_config {
    frequency_t initial;
    frequency_t final;
    duration_t duration;  // 0 = skip virtual flux phase
};

struct virtual_torque_config {
    torque_pu_t initial;
    torque_pu_t final;
    duration_t duration;  // 0 = immediate to final value
};

// Separate configuration methods
sequencer.configure_flux_buildup(flux_config);
sequencer.configure_virtual_frequency(freq_config);
sequencer.configure_virtual_torque(torque_config);
```

### 2. Torque Ramping

**New Feature:** Virtual torque reference now ramps linearly from initial to final value over a configurable duration.

- `initial`: Starting torque value (e.g., 0.2 pu)
- `final`: Ending torque value (e.g., 0.4 pu)
- `duration`: Ramp time in seconds
  - If `duration == 0`, immediately uses final value
  - If `initial == final`, maintains constant torque

**Ramp Formula:**
```
torque(t) = initial + (final - initial) * (t / duration)
```

### 3. Grouped Output Structures

**Before:**
```cpp
struct startup_sequencer_outputs {
    reference_frame_source_t reference_frame_source;
    bool enable_speed_controller;
    sfc_torque_controller::control_mode torque_control_mode;
    torque_pu_t manual_torque_reference;
    angle_t virtual_rotor_angle;
    frequency_t virtual_rotor_frequency;
    bool release_machine_side_pulses;
    startup_sequencer_state_t current_state;
    bool is_ready_for_operation;
};

// Access
auto outputs = sequencer.get_outputs();
torque_pu_t torque = outputs.manual_torque_reference;
```

**After:**
```cpp
// Three grouped output structures
struct flux_buildup_outputs {
    current_pu_t field_current_setpoint;
};

struct virtual_reference_outputs {
    angle_t rotor_angle;
    frequency_t rotor_frequency;
    torque_pu_t torque_reference;
};

struct controller_command_outputs {
    reference_frame_source_t reference_frame_source;
    bool enable_speed_controller;
    sfc_torque_controller::control_mode torque_control_mode;
    bool release_machine_side_pulses;
    startup_sequencer_state_t current_state;
    bool is_ready_for_operation;
};

struct startup_sequencer_outputs {
    flux_buildup_outputs flux_buildup;
    virtual_reference_outputs virtual_reference;
    controller_command_outputs controller_commands;
};

// Access with grouped outputs
auto outputs = sequencer.get_outputs();
torque_pu_t torque = outputs.virtual_reference.torque_reference;
bool enable_speed = outputs.controller_commands.enable_speed_controller;
current_pu_t field_setpoint = outputs.flux_buildup.field_current_setpoint;
```

## Migration Guide

### Updating Configuration Code

**Old Code:**
```cpp
startup_sequencer_config config{
    .flux_buildup_current_setpoint = current_pu_t{1.0},
    .flux_buildup_duration = duration_t{3.0},
    .virtual_frequency_initial = frequency_t{1.0},
    .virtual_frequency_final = frequency_t{10.0},
    .virtual_frequency_duration = duration_t{10.0},
    .starting_torque = torque_pu_t{0.3}
};

sequencer.configure(config);
```

**New Code:**
```cpp
flux_buildup_config flux_cfg{
    .field_current_setpoint = current_pu_t{1.0},
    .duration = duration_t{3.0}
};

virtual_frequency_config freq_cfg{
    .initial = frequency_t{1.0},
    .final = frequency_t{10.0},
    .duration = duration_t{10.0}
};

virtual_torque_config torque_cfg{
    .initial = torque_pu_t{0.3},
    .final = torque_pu_t{0.3},  // Constant torque
    .duration = duration_t{0.0}  // Immediate (no ramp)
};

sequencer.configure_flux_buildup(flux_cfg);
sequencer.configure_virtual_frequency(freq_cfg);
sequencer.configure_virtual_torque(torque_cfg);
```

### Updating Output Access Code

**Old Code:**
```cpp
auto outputs = sequencer.get_outputs();

// Apply to controllers
torque_controller.set_control_mode(outputs.torque_control_mode);
torque_controller.set_manual_torque(outputs.manual_torque_reference);
speed_controller.set_enable(outputs.enable_speed_controller);

// Select reference frame
if (outputs.reference_frame_source == reference_frame_source_t::VIRTUAL) {
    modulator.set_reference_angle(outputs.virtual_rotor_angle);
}

// Control pulse release
modulator.set_pulse_enable(outputs.release_machine_side_pulses);
```

**New Code:**
```cpp
auto outputs = sequencer.get_outputs();

// Apply to controllers
torque_controller.set_control_mode(outputs.controller_commands.torque_control_mode);
torque_controller.set_manual_torque(outputs.virtual_reference.torque_reference);
speed_controller.set_enable(outputs.controller_commands.enable_speed_controller);

// Select reference frame
if (outputs.controller_commands.reference_frame_source == reference_frame_source_t::VIRTUAL) {
    modulator.set_reference_angle(outputs.virtual_reference.rotor_angle);
}

// Control pulse release
modulator.set_pulse_enable(outputs.controller_commands.release_machine_side_pulses);

// Get field current setpoint
excitation_controller.set_current_reference(outputs.flux_buildup.field_current_setpoint);
```

## Benefits of the New API

1. **Logical Grouping**: Related parameters and outputs are grouped together
2. **Easier Configuration**: Configure each aspect independently
3. **Better Readability**: Output access is more explicit about what's being controlled
4. **Torque Ramping**: Smooth torque transitions for better machine dynamics
5. **Flexibility**: Configure only what needs to change without full reconfiguration

## Example Usage with Torque Ramping

```cpp
// Example: Ramp torque from 20% to 40% over 5 seconds
virtual_torque_config torque_cfg{
    .initial = torque_pu_t{0.2},
    .final = torque_pu_t{0.4},
    .duration = duration_t{5.0}
};
sequencer.configure_virtual_torque(torque_cfg);

// Example: Immediate jump to 30% torque (no ramp)
virtual_torque_config torque_cfg{
    .initial = torque_pu_t{0.3},
    .final = torque_pu_t{0.3},
    .duration = duration_t{0.0}
};
sequencer.configure_virtual_torque(torque_cfg);

// Example: Hold 25% constant torque for 10 seconds
virtual_torque_config torque_cfg{
    .initial = torque_pu_t{0.25},
    .final = torque_pu_t{0.25},
    .duration = duration_t{10.0}  // Any value, will stay constant
};
sequencer.configure_virtual_torque(torque_cfg);
```
