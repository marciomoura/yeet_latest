# Machine-Side Startup Sequencer

[← Back to Documentation Index](index.md)

## 1. Overview

The **Machine-Side Startup Sequencer** (`sfc_startup_sequencer`) is a critical component that coordinates the complete startup procedure for LCI-driven synchronous machines, from standstill to normal closed-loop operation. It operates as a sub-state machine within the main `OPERATION_ON` state and ensures safe, repeatable startup behavior.

### Purpose

Starting a large synchronous machine with an LCI requires careful sequencing:

1. **Flux Establishment**: The magnetic field must be built up before applying torque
2. **Low-Speed Starting**: At very low speeds, insufficient back-EMF prevents natural commutation
3. **Virtual Reference Frame**: Open-loop V/Hz control is used until sufficient speed is reached
4. **Smooth Transition**: Switching from virtual to estimated reference frame must be seamless
5. **Closed-Loop Control**: Final transition to speed-controlled operation

The startup sequencer automates this entire procedure, removing the need for manual intervention and ensuring consistency across all startups.

### Startup Sequence

The startup procedure consists of three phases:

1. **Flux Build-Up (Pre-Fluxing)**: Establish magnetic field in the machine rotor (no rotation)
2. **Starting Procedure (Virtual Reference Frame)**: Rotate machine using open-loop V/Hz control with ramping frequency and torque
3. **Closed-Loop Operation**: Automatic transition to observer-based speed-controlled operation

---

## 2. State Machine Architecture

The sequencer implements a 3-state machine:

```
                    ┌──────────┐
                    │   IDLE   │
                    └────┬─────┘
                         │ start_sequence()
                         ▼
                ┌─────────────────┐
                │  FLUX_BUILDUP   │  ← Phase 1: Build field
                └────────┬─────────┘
                         │ Timer + field established
                         ▼
            ┌─────────────────────────┐
            │  STARTING_PROCEDURE     │  ← Phase 2: Virtual flux + torque
            └────────┬─────────────────┘
                     │ Timer (automatic)
                     ▼
    ┌─────────────────────────────┐
    │ CLOSED_LOOP_OPERATION       │  ← Phase 3: Speed control
    └─────────────────────────────┘
```

### State Descriptions

#### IDLE
- **Purpose**: Safe state with all outputs disabled
- **Entry**: Power-up or after `stop_sequence()`
- **Outputs**:
  - `enable_speed_controller = false`
  - `torque_control_mode = SPEED_CONTROL`
  - `manual_torque_reference = 0.0 pu`
  - `release_machine_side_pulses = false`
  - `reference_frame_source = ESTIMATED`
- **Exit**: `start_sequence()` called → `FLUX_BUILDUP`

#### FLUX_BUILDUP (Phase 1)
- **Purpose**: Establish magnetic field in the synchronous machine
- **Configuration**:
  - `flux_buildup_current_setpoint`: Field current target (typically 1.0 pu)
  - `flux_buildup_duration`: Wait time for field establishment (typically 2-5 s)
- **Actions**:
  - Machine-side converter held safe (no pulses)
  - Excitation controller brings field current to setpoint
  - Timer counts to configured duration
- **Outputs**:
  - `enable_speed_controller = false`
  - `torque_control_mode = MANUAL_TORQUE`
  - `manual_torque_reference = 0.0 pu`
  - `release_machine_side_pulses = false`
  - `reference_frame_source = VIRTUAL`
- **Exit Conditions**:
  - Timer elapsed AND `is_field_established == true` → `STARTING_PROCEDURE`
  - `stop_sequence()` → `IDLE`

#### STARTING_PROCEDURE (Phase 2)
- **Purpose**: Rotate machine using virtual reference frame (open-loop V/Hz)
- **Configuration**:
  - `virtual_frequency_initial`: Starting frequency (e.g., 1 Hz)
  - `virtual_frequency_final`: Ending frequency (e.g., 10 Hz)
  - `virtual_frequency_duration`: Ramp time (e.g., 10 s)
    - If `duration == 0`, skips directly to `CLOSED_LOOP_OPERATION`
    - If `initial == final`, holds constant frequency for duration
  - `starting_torque`: Constant torque applied during starting (e.g., 0.3 pu)
- **Actions**:
  - Generate virtual frequency: Linear ramp from initial to final over duration
  - Integrate frequency to produce virtual rotor angle
  - Apply constant torque via torque controller
- **Frequency Profile**:
  ```
  f(t) = f_initial + (f_final - f_initial) * (t / duration)
  θ(t) = ∫ 2π * f(t) dt
  ```
- **Outputs**:
  - `enable_speed_controller = false`
  - `torque_control_mode = MANUAL_TORQUE`
  - `manual_torque_reference = starting_torque`
  - `release_machine_side_pulses = true`
  - `reference_frame_source = VIRTUAL`
  - `virtual_rotor_angle = θ(t)`
  - `virtual_rotor_frequency = f(t)`
- **Exit Conditions**:
  - Timer elapsed (`duration`) → `CLOSED_LOOP_OPERATION` (automatic)
  - `stop_sequence()` → `IDLE`

#### CLOSED_LOOP_OPERATION (Phase 3)
- **Purpose**: Normal operation with speed control
- **Entry**: Automatic after `STARTING_PROCEDURE` timer expires
- **Actions**:
  - Switch reference frame source from VIRTUAL to ESTIMATED
  - Enable speed controller
  - Switch torque controller to speed control mode
- **Angle Synchronization**:
  - Before transition (100ms window), virtual angle is synchronized to estimated angle
  - Prevents discontinuity when switching reference frames
- **Outputs**:
  - `enable_speed_controller = true`
  - `torque_control_mode = SPEED_CONTROL`
  - `manual_torque_reference = 0.0 pu` (not used)
  - `release_machine_side_pulses = true`
  - `reference_frame_source = ESTIMATED`
  - `is_ready_for_operation = true`

---

## 3. Configuration Parameters

### Configuration Structure

```cpp
struct startup_sequencer_config {
    // Flux build-up parameters
    current_pu_t flux_buildup_current_setpoint{1.0};  ///< [pu]
    duration_t flux_buildup_duration{3.0};            ///< [s]

    // Virtual frequency profile
    frequency_t virtual_frequency_initial{1.0};   ///< [Hz]
    frequency_t virtual_frequency_final{10.0};    ///< [Hz]
    duration_t virtual_frequency_duration{10.0};  ///< [s], 0 = skip

    // Starting procedure
    torque_pu_t starting_torque{0.3};  ///< [pu]
};
```

### Parameter Tuning Guide

| Parameter                       | Typical Range | Tuning Guidance                                                  |
| ------------------------------- | ------------- | ---------------------------------------------------------------- |
| `flux_buildup_current_setpoint` | 0.8 - 1.0 pu  | Nominal field current for machine                                |
| `flux_buildup_duration`         | 2 - 5 s       | 3-5× field winding time constant (τ_f)                           |
| `virtual_frequency_initial`     | 0.5 - 2 Hz    | Low enough for smooth start, high enough for commutation         |
| `virtual_frequency_final`       | 5 - 15 Hz     | Speed where observer is reliable (≈10-20% rated)                 |
| `virtual_frequency_duration`    | 5 - 20 s      | Acceleration time, limited by thermal and mechanical constraints |
| `starting_torque`               | 0.2 - 0.5 pu  | Just enough to overcome inertia and friction                     |

### Special Configurations

#### Skip Virtual Flux Phase
Set `virtual_frequency_duration = 0.0` to skip Phase 2 entirely:
- Use case: Machine already rotating, quick re-synchronization
- Sequence: `FLUX_BUILDUP` → `CLOSED_LOOP_OPERATION` (direct)

#### Constant Virtual Frequency
Set `virtual_frequency_initial == virtual_frequency_final`:
- Use case: Hold machine at constant low speed for testing
- Behavior: Frequency stays constant for `virtual_frequency_duration`, then transitions

---

## 4. Integration Example

```cpp
#include "sfc/sfc_startup_sequencer.hpp"

// Construction
sfc_startup_sequencer sequencer(duration_t{50e-6});  // 50µs sampling

// Configuration
startup_sequencer_config config;
config.flux_buildup_current_setpoint = current_pu_t{1.0};
config.flux_buildup_duration = duration_t{3.0};
config.virtual_frequency_initial = frequency_t{1.0};
config.virtual_frequency_final = frequency_t{10.0};
config.virtual_frequency_duration = duration_t{10.0};
config.starting_torque = torque_pu_t{0.3};
sequencer.configure(config);

// Start sequence when entering OPERATION_ON state
sequencer.start_sequence();

// In main control loop (50µs cycle)
while (running) {
    // Get inputs
    bool field_established = excitation_controller.is_field_established();
    angle_t estimated_angle = observer.get_rotor_angle();
    
    // Update sequencer
    sequencer.update(field_established, estimated_angle);
    
    // Get outputs
    const auto& outputs = sequencer.get_outputs();
    
    // Apply to subordinate controllers
    speed_controller.set_enable(outputs.enable_speed_controller);
    torque_controller.set_control_mode(outputs.torque_control_mode);
    torque_controller.set_manual_torque(outputs.manual_torque_reference);
    
    // Select reference frame for modulator
    if (outputs.reference_frame_source == reference_frame_source_t::VIRTUAL) {
        modulator.set_reference_angle(outputs.virtual_rotor_angle);
    } else {
        modulator.set_reference_angle(estimated_angle);
    }
    
    // Control pulse release
    modulator.set_pulse_enable(outputs.release_machine_side_pulses);
    
    // Get field current setpoint
    current_pu_t field_setpoint = sequencer.get_flux_setpoint();
    excitation_controller.set_current_reference(field_setpoint);
}

// Stop sequence when exiting OPERATION_ON
sequencer.stop_sequence();
```

---

## 5. Operational Timeline

### Typical 13-Second Startup

```
Time  State                    Actions
────────────────────────────────────────────────────────────────
0.0s  IDLE                    - System safe, all outputs disabled
      ↓ start_sequence()
      
0.0s  FLUX_BUILDUP            - Apply field current setpoint (1.0 pu)
                              - Machine stationary (no pulses)
                              - Wait for field establishment
3.0s  ↓ Timer + field ready

3.0s  STARTING_PROCEDURE      - Virtual frequency: 1 Hz → 10 Hz
                              - Apply starting torque: 0.3 pu
                              - Release machine pulses
                              - Machine accelerates
13.0s ↓ Timer (10s elapsed)

13.0s CLOSED_LOOP_OPERATION   - Switch to estimated reference frame
                              - Enable speed controller
                              - Normal operation begins
```

---

## 6. Virtual Reference Frame Generation

### Frequency Ramp Algorithm

```cpp
// Calculate ramp progress (0.0 to 1.0)
float progress = min(1.0, elapsed_time / virtual_frequency_duration);

// Linear interpolation
float current_freq = initial_freq + (final_freq - initial_freq) * progress;

// Convert to angular velocity
float omega = 2 * π * current_freq;  // rad/s

// Integrate to get angle
angle += omega * sampling_time;

// Wrap to [0, 2π]
while (angle >= 2*π) angle -= 2*π;
while (angle < 0) angle += 2*π;
```

### Synchronization Before Transition

To prevent angle discontinuity when switching from virtual to estimated frame:

```cpp
// 100ms before end of STARTING_PROCEDURE
if (time_remaining < 0.1s) {
    // Synchronize virtual angle to estimated
    virtual_angle = estimated_angle;
    integrator.reset(estimated_angle);
}
```

This ensures smooth handoff to the observer-based angle.

---

## 7. Testing and Commissioning

### Initial Testing Sequence

1. **Test Flux Build-Up Only**
   - Set `virtual_frequency_duration = 0.0` (skip Phase 2)
   - Verify field current reaches setpoint
   - Verify `is_field_established` flag
   - Measure actual field time constant

2. **Test Constant Virtual Frequency**
   - Set `virtual_frequency_initial = virtual_frequency_final = 2.0 Hz`
   - Set `virtual_frequency_duration = 5.0 s`
   - Verify machine rotates at constant speed
   - Check commutation stability

3. **Test Frequency Ramp**
   - Set `virtual_frequency_initial = 1.0 Hz`
   - Set `virtual_frequency_final = 10.0 Hz`
   - Set `virtual_frequency_duration = 10.0 s`
   - Verify smooth acceleration
   - Monitor for commutation failures

4. **Test Full Sequence**
   - Configure all parameters
   - Execute complete startup
   - Verify smooth transition to closed-loop
   - Check speed controller takes over cleanly

### Monitoring Points

During commissioning, monitor these signals:
- `current_state`: Verify state progression
- `virtual_rotor_frequency`: Check frequency ramp profile
- `virtual_rotor_angle`: Verify integration (should increase monotonically)
- `is_field_established`: Confirm field readiness
- `time_in_current_state`: Track state durations
- Machine speed (from observer): Compare to virtual frequency

---

## 8. Fault Handling

### Automatic Transitions to IDLE

The sequencer automatically returns to IDLE state when:
- `stop_sequence()` is called (from any state)
- Main state machine exits OPERATION_ON state

### Recommended External Protection

The application should monitor for:
- **Flux Build-Up Timeout**: If `is_field_established` never becomes true
- **Excessive Acceleration Time**: If machine doesn't reach expected speed
- **Commutation Failures**: During STARTING_PROCEDURE
- **Observer Loss**: If estimated angle becomes unreliable

When a fault is detected, call `stop_sequence()` to safely shut down.

---

## 9. API Reference

### Public Methods

#### Configuration
```cpp
void configure(const startup_sequencer_config& config);
```
Set all tunable parameters. Call during initialization.

#### Control
```cpp
bool start_sequence();
```
Start the startup sequence from IDLE. Returns false if not in IDLE state.

```cpp
void stop_sequence();
```
Immediately return to IDLE state. Safe to call from any state.

```cpp
void update(bool is_field_established, angle_t estimated_rotor_angle);
```
Main update function. Call every control cycle (e.g., 50µs).

```cpp
void reset();
```
Reset all internal states, timers, and return to IDLE. For initialization or fault recovery.

#### Outputs
```cpp
const startup_sequencer_outputs& get_outputs() const noexcept;
```
Get complete output structure containing all control signals.

```cpp
current_pu_t get_flux_setpoint() const noexcept;
```
Get the configured field current setpoint for the excitation controller.

#### Status
```cpp
startup_sequencer_state_t get_current_state() const noexcept;
```
Get current state (IDLE, FLUX_BUILDUP, STARTING_PROCEDURE, CLOSED_LOOP_OPERATION).

```cpp
bool is_startup_complete() const noexcept;
```
Returns true when in CLOSED_LOOP_OPERATION state.

```cpp
frequency_t get_virtual_frequency() const noexcept;
```
Get current virtual frequency (for monitoring/diagnostics).

```cpp
duration_t get_time_in_current_state() const noexcept;
```
Get elapsed time in current state (for monitoring/debugging).

---

## 10. Related Documentation

- [Flux Build-Up Procedure](flux_buildup_procedure.md) - Detailed theory on pre-fluxing
- [Virtual Reference Frame](virtual_reference_frame.md) - V/Hz starting method
- [SFC State Machine](state_machine.md) - Main operational states
- [Torque Controller](torque_controller.md) - Control mode switching
- [Speed Controller](speed_controller.md) - Closed-loop operation

---

## 11. References

- IEEE Std 421.5-2016: Recommended Practice for Excitation System Models
- "Load-Commutated Inverter Drives for Large Synchronous Motors", ABB Technical Paper
- "Starting Methods for Synchronous Motors", Siemens Application Note
