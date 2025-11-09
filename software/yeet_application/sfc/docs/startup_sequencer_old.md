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

---

## 2. State Machine Architecture

The sequencer implements a 5-state machine:

```
                    ┌──────────┐
                    │   IDLE   │  ← Initial state, system safe
                    └────┬─────┘
                         │ start_sequence()
                         ▼
                ┌─────────────────┐
                │  FLUX_BUILDUP   │  ← Build magnetic field (timer-based)
                └────────┬─────────┘
                         │ Field established
                         ▼
            ┌─────────────────────────┐
            │  STARTING_PROCEDURE     │  ← Virtual frame, constant torque
            └────────┬─────────────────┘
                     │ Speed ≥ threshold
                     ▼
        ┌──────────────────────────────┐
        │ TRANSITION_TO_ESTIMATED      │  ← Switch reference frames
        └────────┬──────────────────────┘
                 │ Synchronization complete
                 ▼
    ┌─────────────────────────────┐
    │ CLOSED_LOOP_OPERATION       │  ← Normal speed control
    └─────────────────────────────┘
```

### State Descriptions

#### **IDLE**
- **Entry**: Default state on power-up or after `stop_sequence()`
- **Actions**: All outputs safe, sequencer inactive
- **Exit**: `start_sequence()` called → Transition to `FLUX_BUILDUP`

#### **FLUX_BUILDUP**
- **Entry**: From `IDLE` via `start_sequence()`
- **Actions**:
  - Hold machine-side modulator in safe state (no pulses)
  - Monitor field establishment via excitation controller
  - Wait for timer expiration
- **Duration**: Configurable (typical: 2-5 seconds)
- **Outputs**:
  - `enable_speed_controller = false`
  - `torque_control_mode = MANUAL_TORQUE`
  - `manual_torque_reference = 0.0 pu`
  - `release_machine_side_pulses = false`
- **Exit Conditions**:
  - Timer expires AND `is_field_established == true` → `STARTING_PROCEDURE`
  - Fault detected → `IDLE`

#### **STARTING_PROCEDURE**
- **Entry**: From `FLUX_BUILDUP` when field is established
- **Actions**:
  - Generate virtual reference frame (open-loop V/Hz)
  - Apply constant starting torque
  - Ramp virtual frequency from initial value
  - Monitor machine speed
- **Duration**: Until speed threshold reached (typical: 5-20 seconds)
- **Outputs**:
  - `reference_frame_source = VIRTUAL`
  - `enable_speed_controller = false`
  - `torque_control_mode = MANUAL_TORQUE`
  - `manual_torque_reference = starting_torque` (e.g., 0.3 pu)
  - `release_machine_side_pulses = true`
  - `virtual_rotor_angle` = integrated angle
- **Exit Conditions**:
  - `machine_speed >= transition_speed_threshold` → `TRANSITION_TO_ESTIMATED`
  - Timeout (starting failed) → `IDLE`
  - `machine_speed > maximum_starting_speed` (overspeed protection) → `IDLE`

#### **TRANSITION_TO_ESTIMATED**
- **Entry**: From `STARTING_PROCEDURE` when speed threshold reached
- **Actions**:
  - Switch reference frame source from `VIRTUAL` to `ESTIMATED`
  - Synchronize virtual angle integrator to estimated angle (prevents discontinuity)
  - Maintain constant torque during transition
- **Duration**: Very short (1-2 control cycles)
- **Outputs**:
  - `reference_frame_source = ESTIMATED`
  - `enable_speed_controller = false`
  - `torque_control_mode = MANUAL_TORQUE`
  - `manual_torque_reference = starting_torque`
  - `release_machine_side_pulses = true`
- **Exit Conditions**:
  - Synchronization complete (immediate in current implementation) → `CLOSED_LOOP_OPERATION`

#### **CLOSED_LOOP_OPERATION**
- **Entry**: From `TRANSITION_TO_ESTIMATED` when synchronized
- **Actions**:
  - Enable speed controller
  - Switch to speed control mode
  - Machine follows external speed reference
- **Duration**: Indefinite (until shutdown requested)
- **Outputs**:
  - `reference_frame_source = ESTIMATED`
  - `enable_speed_controller = true`
  - `torque_control_mode = SPEED_CONTROL`
  - `release_machine_side_pulses = true`
  - `is_ready_for_operation = true`
- **Exit Conditions**:
  - Main state machine exits `OPERATION_ON` → `IDLE`
  - `stop_sequence()` called → `IDLE`

---

## 3. Configuration Parameters

All sequencer parameters are set via the `startup_sequencer_config` structure:

### Flux Build-Up Parameters

| Parameter               | Type         | Default | Units | Description                                   |
| ----------------------- | ------------ | ------- | ----- | --------------------------------------------- |
| `flux_buildup_duration` | `duration_t` | 3.0     | s     | Time to establish field winding magnetic flux |

**Tuning Guidance:**
- Minimum value: 3× field winding L/R time constant
- Typical range: 2-5 seconds
- Longer duration = safer but slower startup
- Can be measured by monitoring field current rise time

### Starting Procedure Parameters

| Parameter                      | Type          | Default | Units | Description                                       |
| ------------------------------ | ------------- | ------- | ----- | ------------------------------------------------- |
| `starting_torque`              | `torque_pu_t` | 0.3     | pu    | Constant torque applied during starting procedure |
| `starting_frequency_ramp_rate` | `frequency_t` | 2.0     | Hz/s  | Rate of virtual frequency increase                |
| `transition_speed_threshold`   | `speed_pu_t`  | 0.15    | pu    | Speed at which to switch to estimated frame       |
| `starting_timeout`             | `duration_t`  | 30.0    | s     | Maximum time allowed for starting procedure       |

**Tuning Guidance:**

**`starting_torque`:**
- Must overcome static friction and windage
- Typical range: 0.2-0.4 pu
- Too low: Slow or failed acceleration
- Too high: Excessive mechanical stress, potential overshoot

**`starting_frequency_ramp_rate`:**
- Controls virtual V/Hz ramp slope
- Typical range: 1-5 Hz/s
- Too fast: Potential commutation issues, instability
- Too slow: Extended startup time

**`transition_speed_threshold`:**
- Speed at which back-EMF is sufficient for natural commutation
- Typical range: 0.10-0.20 pu (10-20% rated speed)
- Too low: Insufficient back-EMF, commutation failures
- Too high: Unnecessary time in open-loop mode

**`starting_timeout`:**
- Safety limit to prevent indefinite starting attempts
- Typical range: 20-60 seconds
- Should be significantly longer than expected startup time
- Triggers abort to `IDLE` state if exceeded

### Virtual Reference Frame Generation

| Parameter                   | Type          | Default | Units | Description                      |
| --------------------------- | ------------- | ------- | ----- | -------------------------------- |
| `virtual_frequency_initial` | `frequency_t` | 0.5     | Hz    | Starting frequency for V/Hz ramp |

**Tuning Guidance:**
- Minimum torque production frequency
- Typical range: 0.3-1.0 Hz
- Too low: Very slow acceleration, potential stall
- Too high: Torque pulsations, commutation issues

### Safety Limits

| Parameter                | Type         | Default | Units | Description                            |
| ------------------------ | ------------ | ------- | ----- | -------------------------------------- |
| `maximum_starting_speed` | `speed_pu_t` | 0.25    | pu    | Maximum speed allowed in starting mode |

**Tuning Guidance:**
- Safety limit to prevent runaway in virtual frame mode
- Should be slightly above `transition_speed_threshold`
- Typical margin: 1.5× transition threshold
- Triggers abort to `IDLE` if exceeded

---

## 4. Integration with Main Control System

### A. Main State Machine Integration

The sequencer is instantiated and controlled by the `operation_on_state`:

```cpp
class operation_on_state : public sfc_state_type {
public:
    // ... constructor ...

    sfc_state_type* update(const sfc_state_inputs& inputs) noexcept override {
        // Detect entry to OPERATION_ON state
        if (_just_entered) {
            _startup_sequencer.start_sequence();
            _just_entered = false;
        }

        // Update sequencer every cycle
        _startup_sequencer.update(
            excitation_controller.get_is_field_established(),
            measured_speed_pu,
            observer.get_estimated_angle()
        );

        // Use sequencer outputs ...
        auto outputs = _startup_sequencer.get_outputs();
        
        // ... state transition logic ...
    }

private:
    sfc_startup_sequencer _startup_sequencer;
    bool _just_entered{true};
};
```

### B. Controller Reset Integration

On entry to `OPERATION_ON`, the main state machine asserts `reset_controllers`:

```cpp
// In application integration layer
if (state_machine.get_outputs().reset_controllers) {
    // Reset all controllers to clean initial state
    speed_controller.reset(speed_pu_t{0.0});
    torque_controller.reset();
    current_controller.reset();
    grid_observer.reset();
    machine_observer.reset();
}
```

This ensures all controllers start from a known, consistent state.

### C. Torque Controller Integration

The sequencer controls the torque controller's operating mode:

```cpp
// Set control mode from sequencer
torque_controller.set_control_mode(sequencer.get_outputs().torque_control_mode);

// If in manual mode, provide manual torque reference
if (sequencer.get_outputs().torque_control_mode == control_mode::MANUAL_TORQUE) {
    torque_controller.set_manual_torque_reference(
        sequencer.get_outputs().manual_torque_reference
    );
}
```

### D. Speed Controller Integration

The sequencer controls when the speed controller is active:

```cpp
// Freeze integrator when speed controller is disabled
speed_controller.freeze_integrator(!sequencer.get_outputs().enable_speed_controller);
```

When `enable_speed_controller = false`, the integrator is frozen to prevent wind-up.

### E. Machine-Side Modulator Integration

The modulator must accept the reference frame source selection:

```cpp
// Modulator update with reference frame selection
void sfc_machine_side_modulator::update(
    bool enable,
    reference_frame_source_t frame_source,
    angle_t virtual_rotor_angle,
    angle_t estimated_rotor_angle,
    // ... other parameters
) {
    // Select angle based on frame source
    angle_t selected_angle = (frame_source == VIRTUAL) 
        ? virtual_rotor_angle 
        : estimated_rotor_angle;
    
    // Use selected_angle for sector calculation and firing angle
    // ...
}
```

---

## 5. Operational Sequence Timeline

### Example Startup Timeline

```
Time    State                  Actions
──────────────────────────────────────────────────────────────────
0.0s    IDLE                   Waiting for start command
        ↓ start_sequence()
        
0.0s    FLUX_BUILDUP           Field current ramping up
                               Virtual angle = 0°
                               Torque = 0 pu
                               No pulses released

3.0s    ↓                      Timer expired, field established
        
3.0s    STARTING_PROCEDURE     Virtual frequency = 0.5 Hz
                               Torque = 0.3 pu
                               Pulses released
                               Machine begins rotating

3.5s    STARTING_PROCEDURE     Virtual frequency = 1.5 Hz (ramping)
                               Speed = 3% (measured)

5.0s    STARTING_PROCEDURE     Virtual frequency = 3.5 Hz
                               Speed = 8%

7.0s    STARTING_PROCEDURE     Virtual frequency = 5.5 Hz
                               Speed = 14%

8.0s    ↓                      Speed reached 15% threshold
        
8.0s    TRANSITION_TO_EST.     Reference frame switched
                               Virtual angle synced to estimated
                               Torque maintained at 0.3 pu

8.01s   ↓                      Synchronization complete
        
8.01s   CLOSED_LOOP_OP.        Speed controller enabled
                               Torque mode = SPEED_CONTROL
                               System ready for operation
                               Following external speed reference
```

---

## 6. Commissioning Procedure

### Step 1: Initial Conservative Settings

Start with very conservative parameters for first startup:

```cpp
startup_sequencer_config config;
config.flux_buildup_duration = 5.0;              // Extra margin
config.starting_torque = 0.2;                    // Low initial torque
config.starting_frequency_ramp_rate = 1.0;       // Slow ramp
config.virtual_frequency_initial = 0.5;
config.transition_speed_threshold = 0.15;
config.starting_timeout = 60.0;                  // Long timeout
config.maximum_starting_speed = 0.25;

sequencer.configure(config);
```

### Step 2: Baseline Test

Execute a complete startup sequence and monitor:
- Field current rise time → Determine minimum `flux_buildup_duration`
- Acceleration profile → Assess if `starting_torque` is adequate
- Time to reach threshold → Verify `starting_timeout` is appropriate
- Transition smoothness → Check for discontinuities during frame switch

### Step 3: Optimize Starting Torque

Gradually increase `starting_torque` while monitoring:
- Shaft torque (if instrumented)
- Acceleration rate
- Mechanical vibrations
- Current consumption

Find the minimum torque that provides smooth, timely acceleration without excessive stress.

### Step 4: Optimize Ramp Rate

Increase `starting_frequency_ramp_rate` to reduce startup time:
- Monitor commutation stability
- Check for torque pulsations
- Verify no overshoot at transition point
- Confirm smooth transition to estimated frame

### Step 5: Fine-Tune Transition Threshold

Adjust `transition_speed_threshold` based on:
- Back-EMF magnitude measurements
- Commutation margin during transition
- Observer tracking accuracy at different speeds
- Stability after frame switch

Lower threshold = Faster transition (but requires sufficient back-EMF)
Higher threshold = Safer transition (but longer open-loop time)

### Step 6: Verify Safety Limits

Test safety mechanisms:
- Inject timeout condition (prevent acceleration) → Verify abort to `IDLE`
- Exceed `maximum_starting_speed` → Verify abort to `IDLE`
- Remove field excitation → Verify abort during `FLUX_BUILDUP`

### Step 7: Endurance Testing

- Multiple consecutive startups (cold and warm)
- Startups at different initial conditions
- Verify repeatability and consistency
- Monitor for thermal accumulation in power electronics

---

## 7. Fault Handling and Recovery

### Fault Conditions and Responses

| Fault Condition                       | Detection Method                    | Response      |
| ------------------------------------- | ----------------------------------- | ------------- |
| Field not established                 | Timer + `is_field_established` flag | Abort to IDLE |
| Starting timeout                      | `starting_timeout_timer` expires    | Abort to IDLE |
| Overspeed during starting             | `speed > maximum_starting_speed`    | Abort to IDLE |
| Main state machine exits OPERATION_ON | External signal                     | Abort to IDLE |

### Recovery Procedure

After a fault abort to `IDLE`:

1. **Fault Diagnosis**: Determine root cause (e.g., insufficient starting torque, mechanical binding)
2. **Correct Issue**: Adjust parameters or resolve mechanical/electrical problem
3. **Reset Sequence**: Call `sequencer.reset()` if needed
4. **Retry Startup**: Main state machine re-enters `OPERATION_ON`

---

## 8. Testing and Validation

### Unit Tests

The sequencer should have comprehensive unit tests covering:

- **State Transitions**: Verify all valid transitions occur correctly
- **Timer Behavior**: Confirm timers trigger transitions at correct times
- **Virtual Frame Generation**: Validate angle integration and wrapping
- **Output Generation**: Check outputs are correct for each state
- **Fault Handling**: Verify aborts occur under fault conditions
- **Manual Transitions**: Test `force_transition_to_estimated()` and `force_transition_to_virtual()`

### Integration Tests

- **Full Sequence**: IDLE → FLUX_BUILDUP → STARTING_PROCEDURE → TRANSITION → CLOSED_LOOP
- **Controller Integration**: Verify torque, speed controllers respond correctly to sequencer outputs
- **Modulator Integration**: Confirm correct reference frame selection and angle usage
- **Reset Functionality**: Verify controllers reset on entry to `OPERATION_ON`

### Hardware-in-Loop Tests

- **Actual Motor Starting**: From standstill to closed-loop operation
- **Transition Smoothness**: Measure torque and speed during frame switch
- **Commutation Stability**: Monitor commutation failures during startup
- **Mechanical Stress**: Verify no excessive vibration or shaft shock

---

## 9. Advanced Topics

### Future Enhancements

1. **Dynamic Frequency Ramping**: Adjust virtual frequency based on actual speed feedback
2. **Adaptive Torque Control**: Vary starting torque based on acceleration rate
3. **Multi-Stage Starting**: Different torque levels for different speed ranges
4. **Load-Dependent Profiles**: Adjust parameters based on load inertia
5. **Sensorless Starting**: Estimate rotor position without encoder during startup

### Synchronization Algorithm Improvements

The current implementation immediately transitions from virtual to estimated frame. Potential enhancements:

- **Phase Lock Loop (PLL)**: Gradually synchronize virtual frequency to estimated frequency
- **Angle Tracking Verification**: Verify angle tracking stability before enabling speed control
- **Smooth Handoff**: Blend virtual and estimated angles over several cycles

---

## 10. Related Documentation

- [Flux Build-Up Procedure](flux_buildup_procedure.md) - Detailed theory of pre-fluxing
- [Virtual Reference Frame Starting](virtual_reference_frame.md) - Open-loop V/Hz control method
- [Reference Frame Switching](reference_frame_switching.md) - Transition strategy and synchronization
- [State Machine](state_machine.md) - Main operational state machine
- [Excitation Control](excitation_control.md) - Field current management

---

## 11. Conclusion

The Machine-Side Startup Sequencer provides a robust, automated solution for starting large synchronous machines with LCI drives. By carefully managing flux build-up, virtual reference frame generation, and smooth transitions, it ensures safe and repeatable operation across all startup scenarios.

Proper commissioning and tuning are essential to achieve optimal performance while maintaining adequate safety margins. The sequencer's structured state machine architecture and comprehensive configuration options allow adaptation to a wide range of machine types and application requirements.

---

**Document Version:** 1.0  
**Last Updated:** November 8, 2025  
**Author:** SFC Control System Documentation
