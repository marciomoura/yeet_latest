# Machine-Side Startup Sequencer# Machine-Side Startup Sequencer



[← Back to Documentation Index](index.md)[← Back to Documentation Index](index.md)



## 1. Overview## 1. Overview



The **Machine-Side Startup Sequencer** (`sfc_startup_sequencer`) is a critical component that coordinates the complete startup procedure for LCI-driven synchronous machines, from standstill to normal closed-loop operation. It operates as a sub-state machine within the main `OPERATION_ON` state and ensures safe, repeatable startup behavior.The **Machine-Side Startup Sequencer** (`sfc_startup_sequencer`) is a critical component that coordinates the complete startup procedure for LCI-driven synchronous machines, from standstill to normal closed-loop operation. It operates as a sub-state machine within the main `OPERATION_ON` state and ensures safe, repeatable startup behavior.



### Purpose### Purpose



Starting a large synchronous machine with an LCI requires careful sequencing:Starting a large synchronous machine with an LCI requires careful sequencing:



1. **Flux Establishment**: The magnetic field must be built up before applying torque1. **Flux Establishment**: The magnetic field must be built up before applying torque

2. **Low-Speed Starting**: At very low speeds, insufficient back-EMF prevents natural commutation2. **Low-Speed Starting**: At very low speeds, insufficient back-EMF prevents natural commutation

3. **Virtual Reference Frame**: Open-loop V/Hz control is used until sufficient speed is reached3. **Virtual Reference Frame**: Open-loop V/Hz control is used until sufficient speed is reached

4. **Smooth Transition**: Switching from virtual to estimated reference frame must be seamless4. **Smooth Transition**: Switching from virtual to estimated reference frame must be seamless

5. **Closed-Loop Control**: Final transition to speed-controlled operation5. **Closed-Loop Control**: Final transition to speed-controlled operation



The startup sequencer automates this entire procedure, removing the need for manual intervention and ensuring consistency across all startups.The startup sequencer automates this entire procedure, removing the need for manual intervention and ensuring consistency across all startups.



### Startup Sequence### Startup Sequence



The startup procedure consists of three phases:The startup procedure consists of three phases:



1. **Flux Build-Up (Pre-Fluxing)**: Establish magnetic field in the machine rotor (no rotation)1. **Flux Build-Up (Pre-Fluxing)**: Establish magnetic field in the machine rotor (no rotation)

2. **Starting Procedure (Virtual Reference Frame)**: Rotate machine using open-loop V/Hz control with ramping frequency and torque2. **Starting Procedure (Virtual Reference Frame)**: Rotate machine using open-loop V/Hz control with ramping frequency and torque

3. **Closed-Loop Operation**: Automatic transition to observer-based speed-controlled operation3. **Closed-Loop Operation**: Automatic transition to observer-based speed-controlled operation



------



## 2. State Machine Architecture## 2. State Machine Architecture



The sequencer implements a **4-state machine** that progresses through the complete startup procedure:The sequencer implements a 3-state machine:



``````

┌──────────┐                    ┌──────────┐

│   IDLE   │  ← Power-up state, all outputs safe                    │   IDLE   │

└─────┬────┘                    └────┬─────┘

      │ start_sequence()                         │ start_sequence()

      ↓                         ▼

┌──────────────┐                ┌─────────────────┐

│ FLUX_BUILDUP │  ← Establish rotor magnetic field (no rotation)                │  FLUX_BUILDUP   │  ← Phase 1: Build field

└──────┬───────┘                └────────┬─────────┘

       │ is_field_established == true                         │ Timer + field established

       ↓                         ▼

┌────────────────────┐            ┌─────────────────────────┐

│ STARTING_PROCEDURE │  ← Open-loop V/Hz control with ramping            │  STARTING_PROCEDURE     │  ← Phase 2: Virtual flux + torque

└──────┬─────────────┘            └────────┬─────────────────┘

       │ virtual_frequency_ramp.is_finished() AND                     │ Timer (automatic)

       │ virtual_torque_ramp.is_finished()                     ▼

       ↓    ┌─────────────────────────────┐

┌──────────────────────┐    │ CLOSED_LOOP_OPERATION       │  ← Phase 3: Speed control

│ CLOSED_LOOP_OPERATION│  ← Normal speed control    └─────────────────────────────┘

└──────────────────────┘```

       │ stop_sequence()

       ↓### State Descriptions

   (back to IDLE)

```#### IDLE

- **Purpose**: Safe state with all outputs disabled

### State Descriptions- **Entry**: Power-up or after `stop_sequence()`

- **Outputs**:

#### IDLE  - `enable_speed_controller = false`

- **Purpose**: Safe state with all outputs disabled  - `torque_control_mode = SPEED_CONTROL`

- **Entry Conditions**:   - `manual_torque_reference = 0.0 pu`

  - System power-up  - `release_machine_side_pulses = false`

  - After `stop_sequence()` or `reset()`  - `reference_frame_source = ESTIMATED`

  - Fault/abort conditions- **Exit**: `start_sequence()` called → `FLUX_BUILDUP`

- **Actions**:

  - All control outputs set to safe defaults#### FLUX_BUILDUP (Phase 1)

  - Virtual reference frame reset to zero- **Purpose**: Establish magnetic field in the synchronous machine

  - All timers and ramps reset- **Configuration**:

- **Outputs**:  - `flux_buildup_current_setpoint`: Field current target (typically 1.0 pu)

  - `release_speed_controller = false`  - `flux_buildup_duration`: Wait time for field establishment (typically 2-5 s)

  - `release_torque_controller = false`- **Actions**:

  - `release_machine_side_pulses = false`  - Machine-side converter held safe (no pulses)

  - `release_excitation = false`  - Excitation controller brings field current to setpoint

  - `release_machine_observer = false`  - Timer counts to configured duration

  - `reference_frame_source = ESTIMATED`- **Outputs**:

  - `torque_control_mode = SPEED_CONTROL`  - `enable_speed_controller = false`

- **Exit**: `start_sequence()` called → `FLUX_BUILDUP`  - `torque_control_mode = MANUAL_TORQUE`

  - `manual_torque_reference = 0.0 pu`

---  - `release_machine_side_pulses = false`

  - `reference_frame_source = VIRTUAL`

#### FLUX_BUILDUP (Phase 1: Pre-Fluxing)- **Exit Conditions**:

  - Timer elapsed AND `is_field_established == true` → `STARTING_PROCEDURE`

**Purpose**: Establish the rotor magnetic field before attempting rotation  - `stop_sequence()` → `IDLE`



**Physical Background**:#### STARTING_PROCEDURE (Phase 2)

Synchronous machines require a DC magnetic field in the rotor winding to produce electromagnetic torque. This field must be established before any armature currents are applied. The field winding is an inductor with time constant τ_f = L_f / R_f, causing the field current to rise exponentially when voltage is applied.- **Purpose**: Rotate machine using virtual reference frame (open-loop V/Hz)

- **Configuration**:

**What Happens**:  - `virtual_frequency_initial`: Starting frequency (e.g., 1 Hz)

- The **excitation controller** applies DC voltage to the rotor field winding  - `virtual_frequency_final`: Ending frequency (e.g., 10 Hz)

- Field current rises exponentially: `I_f(t) = I_final × (1 - e^(-t/τ_f))`  - `virtual_frequency_duration`: Ramp time (e.g., 10 s)

- Magnetic flux (Φ_f) builds up proportionally to field current    - If `duration == 0`, skips directly to `CLOSED_LOOP_OPERATION`

- Machine remains stationary (no firing pulses to machine-side converter)    - If `initial == final`, holds constant frequency for duration

  - `starting_torque`: Constant torque applied during starting (e.g., 0.3 pu)

**Control Dependencies**:- **Actions**:

The startup sequencer **does not directly control** the excitation system. Instead:  - Generate virtual frequency: Linear ramp from initial to final over duration

1. It **releases** the excitation controller via `release_excitation = true`  - Integrate frequency to produce virtual rotor angle

2. The **excitation controller** independently manages:  - Apply constant torque via torque controller

   - Field voltage setpoint (from its own lookup table)- **Frequency Profile**:

   - Field current regulation  ```

   - Field establishment detection  f(t) = f_initial + (f_final - f_initial) * (t / duration)

3. The sequencer **monitors** the `is_field_established` boolean flag from the excitation controller  θ(t) = ∫ 2π * f(t) dt

  ```

**Input Signal**:- **Outputs**:

- `is_field_established` (bool): Provided by excitation controller when field current has reached setpoint and stabilized  - `enable_speed_controller = false`

  - `torque_control_mode = MANUAL_TORQUE`

**Timing**:  - `manual_torque_reference = starting_torque`

- **No internal timer** in the startup sequencer for flux buildup  - `release_machine_side_pulses = true`

- Transition occurs when excitation controller confirms field is ready  - `reference_frame_source = VIRTUAL`

- Typical duration: 2-5 seconds (depends on machine field time constant)  - `virtual_rotor_angle = θ(t)`

  - `virtual_rotor_frequency = f(t)`

**Outputs During This State**:- **Exit Conditions**:

- `release_excitation = true` → Enables excitation controller  - Timer elapsed (`duration`) → `CLOSED_LOOP_OPERATION` (automatic)

- `release_machine_side_pulses = false` → Machine converter disabled  - `stop_sequence()` → `IDLE`

- `release_machine_observer = false` → Observer not needed yet

- `release_speed_controller = false` → No speed control yet#### CLOSED_LOOP_OPERATION (Phase 3)

- `release_torque_controller = false` → No torque control yet- **Purpose**: Normal operation with speed control

- `reference_frame_source = VIRTUAL` → Prepare for virtual frame starting- **Entry**: Automatic after `STARTING_PROCEDURE` timer expires

- **Actions**:

**Exit Conditions**:  - Switch reference frame source from VIRTUAL to ESTIMATED

- **Success**: `is_field_established == true` → Transition to `STARTING_PROCEDURE`  - Enable speed controller

- **Abort**: `stop_sequence()` called → Return to `IDLE`  - Switch torque controller to speed control mode

- **Angle Synchronization**:

**Typical Field Time Constants**:  - Before transition (100ms window), virtual angle is synchronized to estimated angle

  - Prevents discontinuity when switching reference frames

| Machine Rating | Typical τ_f | 95% Settlement (3τ) | 99% Settlement (5τ) |- **Outputs**:

|----------------|-------------|---------------------|---------------------|  - `enable_speed_controller = true`

| 1-5 MW         | 0.5-1.0 s   | 1.5-3.0 s           | 2.5-5.0 s           |  - `torque_control_mode = SPEED_CONTROL`

| 5-20 MW        | 1.0-2.0 s   | 3.0-6.0 s           | 5.0-10.0 s          |  - `manual_torque_reference = 0.0 pu` (not used)

| >20 MW         | 2.0-4.0 s   | 6.0-12.0 s          | 10.0-20.0 s         |  - `release_machine_side_pulses = true`

  - `reference_frame_source = ESTIMATED`

---  - `is_ready_for_operation = true`



#### STARTING_PROCEDURE (Phase 2: Virtual Reference Frame)---



**Purpose**: Rotate the machine from standstill using open-loop V/Hz control## 3. Configuration Parameters



**Physical Background**:### Configuration Structure

At very low speeds (0-15% typically), the machine's back-EMF is too small for natural (load) commutation. The LCI thyristors cannot reliably turn off based on the machine voltage alone. Solution: Generate a **virtual reference frame** (artificial rotor angle) that increases at a controlled rate, allowing the modulator to fire thyristors in sequence and produce rotating torque.

```cpp

**What is V/Hz Control?**struct startup_sequencer_config {

Maintain constant Volts-per-Hertz ratio to preserve magnetic flux:    // Flux build-up parameters

```    current_pu_t flux_buildup_current_setpoint{1.0};  ///< [pu]

V_applied / f_applied = constant    duration_t flux_buildup_duration{3.0};            ///< [s]

```

In an LCI, this is achieved by:    // Virtual frequency profile

- Controlling DC link current (determines voltage magnitude)    frequency_t virtual_frequency_initial{1.0};   ///< [Hz]

- Controlling virtual frequency (determines electrical frequency)    frequency_t virtual_frequency_final{10.0};    ///< [Hz]

    duration_t virtual_frequency_duration{10.0};  ///< [s], 0 = skip

**Configuration Parameters** (via `configure_virtual_frequency()` and `configure_virtual_torque()`):

    // Starting procedure

**Virtual Frequency Profile**:    torque_pu_t starting_torque{0.3};  ///< [pu]

```cpp};

struct virtual_frequency_config {```

    frequency_t initial;   // Starting frequency (e.g., 1.0 Hz)

    frequency_t final;     // Target frequency (e.g., 10.0 Hz)### Parameter Tuning Guide

    duration_t duration;   // Ramp time (e.g., 10.0 s)

};| Parameter                       | Typical Range | Tuning Guidance                                                  |

```| ------------------------------- | ------------- | ---------------------------------------------------------------- |

| `flux_buildup_current_setpoint` | 0.8 - 1.0 pu  | Nominal field current for machine                                |

**Virtual Torque Profile**:| `flux_buildup_duration`         | 2 - 5 s       | 3-5× field winding time constant (τ_f)                           |

```cpp| `virtual_frequency_initial`     | 0.5 - 2 Hz    | Low enough for smooth start, high enough for commutation         |

struct virtual_torque_config {| `virtual_frequency_final`       | 5 - 15 Hz     | Speed where observer is reliable (≈10-20% rated)                 |

    torque_pu_t initial;   // Starting torque (e.g., 0.2 pu)| `virtual_frequency_duration`    | 5 - 20 s      | Acceleration time, limited by thermal and mechanical constraints |

    torque_pu_t final;     // Target torque (e.g., 0.4 pu)| `starting_torque`               | 0.2 - 0.5 pu  | Just enough to overcome inertia and friction                     |

    duration_t duration;   // Ramp time (e.g., 5.0 s), 0 = instant

};### Special Configurations

```

#### Skip Virtual Flux Phase

**Actions During This State**:Set `virtual_frequency_duration = 0.0` to skip Phase 2 entirely:

1. **Virtual Frequency Generation**:- Use case: Machine already rotating, quick re-synchronization

   - Uses `linear_ramp<frequency_t>` to ramp from initial to final frequency- Sequence: `FLUX_BUILDUP` → `CLOSED_LOOP_OPERATION` (direct)

   - Formula: `f(t) = f_initial + (f_final - f_initial) × min(1.0, t/duration)`

   - Frequency increases linearly over configured duration#### Constant Virtual Frequency

Set `virtual_frequency_initial == virtual_frequency_final`:

2. **Virtual Torque Generation**:- Use case: Hold machine at constant low speed for testing

   - Uses `linear_ramp<torque_pu_t>` to ramp from initial to final torque- Behavior: Frequency stays constant for `virtual_frequency_duration`, then transitions

   - Allows smooth torque buildup or constant torque (if initial == final)

   - Independent timing from frequency ramp---



3. **Virtual Angle Integration**:## 4. Integration Example

   - Integrates virtual frequency to produce reference angle

   - Uses `integrator<angle_wrapped>` for automatic angle wrapping [0, 2π]```cpp

   - Formula: `θ_virtual(t) = ∫ (2π × f_virtual(t)) dt`#include "sfc/sfc_startup_sequencer.hpp"



4. **Angle Synchronization** (near end of ramp):// Construction

   - Virtual angle is synchronized to estimated angle from observersfc_startup_sequencer sequencer(duration_t{50e-6});  // 50µs sampling

   - Prevents discontinuity when switching reference frames

   - Smooth handoff to closed-loop control// Configuration

startup_sequencer_config config;

**Control Flow**:config.flux_buildup_current_setpoint = current_pu_t{1.0};

```config.flux_buildup_duration = duration_t{3.0};

Virtual Frequency Ramp → Virtual Frequency → ω = 2πf → Integrate → Virtual Angleconfig.virtual_frequency_initial = frequency_t{1.0};

Virtual Torque Ramp → Virtual Torque → Torque Controller → DC Current Referenceconfig.virtual_frequency_final = frequency_t{10.0};

```config.virtual_frequency_duration = duration_t{10.0};

config.starting_torque = torque_pu_t{0.3};

**Outputs During This State**:sequencer.configure(config);

- `release_machine_side_pulses = true` → Enable firing pulses

- `release_excitation = true` → Maintain field excitation// Start sequence when entering OPERATION_ON state

- `release_machine_observer = true` → Enable observer for angle estimationsequencer.start_sequence();

- `release_speed_controller = false` → No speed feedback yet

- `release_torque_controller = true` → Enable torque control// In main control loop (50µs cycle)

- `reference_frame_source = VIRTUAL` → Use generated anglewhile (running) {

- `torque_control_mode = MANUAL_TORQUE` → Direct torque command    // Get inputs

- `virtual_rotor_angle` → Generated angle for modulator    bool field_established = excitation_controller.is_field_established();

- `virtual_rotor_frequency` → Current frequency    angle_t estimated_angle = observer.get_rotor_angle();

- `virtual_torque_reference` → Current torque setpoint    

    // Update sequencer

**Exit Conditions**:    sequencer.update(field_established, estimated_angle);

- **Success**: Both frequency AND torque ramps complete → `CLOSED_LOOP_OPERATION`    

- **Abort**: `stop_sequence()` called → `IDLE`    // Get outputs

    const auto& outputs = sequencer.get_outputs();

**Typical Acceleration Profile**:    

```    // Apply to subordinate controllers

Time    Frequency    Torque       Machine Speed    speed_controller.set_enable(outputs.enable_speed_controller);

0s      1.0 Hz       0.2 pu       0 RPM (breakaway)    torque_controller.set_control_mode(outputs.torque_control_mode);

2s      2.8 Hz       0.28 pu      ~80 RPM (accelerating)    torque_controller.set_manual_torque(outputs.manual_torque_reference);

5s      5.5 Hz       0.35 pu      ~160 RPM (accelerating)    

10s     10.0 Hz      0.4 pu       ~290 RPM (approaching transition)    // Select reference frame for modulator

```    if (outputs.reference_frame_source == reference_frame_source_t::VIRTUAL) {

        modulator.set_reference_angle(outputs.virtual_rotor_angle);

**Tuning Guidelines**:    } else {

        modulator.set_reference_angle(estimated_angle);

| Parameter | Typical Range | Too Low | Too High |    }

|-----------|---------------|---------|----------|    

| `virtual_frequency_initial` | 0.5-2.0 Hz | Very slow start, potential stall | Initial torque pulse, mechanical shock |    // Control pulse release

| `virtual_frequency_final` | 5-15 Hz | Early transition risk | Extended open-loop time |    modulator.set_pulse_enable(outputs.release_machine_side_pulses);

| `virtual_frequency_duration` | 5-20 s | Machine can't track | Unnecessarily slow startup |    

| `virtual_torque_initial` | 0.2-0.5 pu | Failed acceleration | Excessive mechanical stress |    // Get field current setpoint

    current_pu_t field_setpoint = sequencer.get_flux_setpoint();

**Special Configurations**:    excitation_controller.set_current_reference(field_setpoint);

}

1. **Skip Virtual Phase Entirely**:

   - Set `virtual_frequency_duration = 0.0`// Stop sequence when exiting OPERATION_ON

   - Use when machine is already rotating (quick resync)sequencer.stop_sequence();

```

2. **Constant Frequency Hold**:

   - Set `initial == final` frequency---

   - Hold machine at specific speed for testing

## 5. Operational Timeline

3. **Instant Torque Application**:

   - Set `virtual_torque_duration = 0.0`### Typical 13-Second Startup

   - Immediate jump to final torque value

```

4. **Constant Torque**:Time  State                    Actions

   - Set `initial == final` torque────────────────────────────────────────────────────────────────

   - Maintain steady torque throughout ramp0.0s  IDLE                    - System safe, all outputs disabled

      ↓ start_sequence()

---      

0.0s  FLUX_BUILDUP            - Apply field current setpoint (1.0 pu)

#### CLOSED_LOOP_OPERATION (Phase 3: Normal Operation)                              - Machine stationary (no pulses)

                              - Wait for field establishment

**Purpose**: Transition to observer-based closed-loop speed control3.0s  ↓ Timer + field ready



**Entry**: Automatic when both virtual frequency and torque ramps complete3.0s  STARTING_PROCEDURE      - Virtual frequency: 1 Hz → 10 Hz

                              - Apply starting torque: 0.3 pu

**Reference Frame Transition**:                              - Release machine pulses

The critical step is switching from the virtual (generated) reference frame to the estimated (observer-based) reference frame:                              - Machine accelerates

13.0s ↓ Timer (10s elapsed)

```

Before: Modulator uses θ_virtual (generated by integrator)13.0s CLOSED_LOOP_OPERATION   - Switch to estimated reference frame

After:  Modulator uses θ_estimated (from observer)                              - Enable speed controller

```                              - Normal operation begins

```

**Synchronization Strategy**:

To prevent angle discontinuity (which would cause current/torque spikes):---

1. Near the end of STARTING_PROCEDURE, the virtual angle integrator is continuously synchronized to the estimated angle

2. Method: `_virtual_angle_integrator.reset(angle_wrapped::from_radians(estimated_angle))`## 6. Virtual Reference Frame Generation

3. Result: At the moment of switching, θ_virtual ≈ θ_estimated (smooth handoff)

### Frequency Ramp Algorithm

**Actions During Transition**:

1. Switch `reference_frame_source` from `VIRTUAL` to `ESTIMATED````cpp

2. Switch `torque_control_mode` from `MANUAL_TORQUE` to `SPEED_CONTROL`// Calculate ramp progress (0.0 to 1.0)

3. Enable speed controller: `release_speed_controller = true`float progress = min(1.0, elapsed_time / virtual_frequency_duration);

4. Speed controller takes over, following external speed reference

// Linear interpolation

**Outputs During This State**:float current_freq = initial_freq + (final_freq - initial_freq) * progress;

- `release_speed_controller = true` → Speed feedback loop active

- `release_torque_controller = true` → Torque control continues// Convert to angular velocity

- `release_machine_side_pulses = true` → Firing pulses enabledfloat omega = 2 * π * current_freq;  // rad/s

- `release_excitation = true` → Field excitation maintained

- `release_machine_observer = true` → Observer provides angle// Integrate to get angle

- `reference_frame_source = ESTIMATED` → Use observer angleangle += omega * sampling_time;

- `torque_control_mode = SPEED_CONTROL` → Speed loop controls torque

- `is_startup_complete = true` → Startup finished// Wrap to [0, 2π]

while (angle >= 2*π) angle -= 2*π;

**Speed Control Handoff**:while (angle < 0) angle += 2*π;

``````

Speed Generator → Speed Reference → Speed Controller → Torque Reference → Torque Controller → Current Reference

```### Synchronization Before Transition



**Exit Conditions**:To prevent angle discontinuity when switching from virtual to estimated frame:

- `stop_sequence()` called → Return to `IDLE`

- Fault/trip condition → Return to `IDLE````cpp

- Normal operation continues indefinitely// 100ms before end of STARTING_PROCEDURE

if (time_remaining < 0.1s) {

---    // Synchronize virtual angle to estimated

    virtual_angle = estimated_angle;

## 3. Configuration API    integrator.reset(estimated_angle);

}

The startup sequencer uses **grouped configuration structures** for clarity and independent parameterization:```



### Virtual Frequency ConfigurationThis ensures smooth handoff to the observer-based angle.



```cpp---

virtual_frequency_config freq_cfg{

    .initial = frequency_t{1.0},    // Starting frequency [Hz]## 7. Testing and Commissioning

    .final = frequency_t{10.0},     // Target frequency [Hz]

    .duration = duration_t{10.0}    // Ramp time [s]### Initial Testing Sequence

};

sequencer.configure_virtual_frequency(freq_cfg);1. **Test Flux Build-Up Only**

```   - Set `virtual_frequency_duration = 0.0` (skip Phase 2)

   - Verify field current reaches setpoint

**Parameter Details**:   - Verify `is_field_established` flag

   - Measure actual field time constant

| Field | Type | Description | Typical Values |

|-------|------|-------------|----------------|2. **Test Constant Virtual Frequency**

| `initial` | `frequency_t` | Starting frequency for V/Hz ramp | 0.5-2.0 Hz |   - Set `virtual_frequency_initial = virtual_frequency_final = 2.0 Hz`

| `final` | `frequency_t` | Target frequency at end of ramp | 5-15 Hz |   - Set `virtual_frequency_duration = 5.0 s`

| `duration` | `duration_t` | Time to complete ramp (0 = skip) | 5-20 s |   - Verify machine rotates at constant speed

   - Check commutation stability

### Virtual Torque Configuration

3. **Test Frequency Ramp**

```cpp   - Set `virtual_frequency_initial = 1.0 Hz`

virtual_torque_config torque_cfg{   - Set `virtual_frequency_final = 10.0 Hz`

    .initial = torque_pu_t{0.2},    // Starting torque [pu]   - Set `virtual_frequency_duration = 10.0 s`

    .final = torque_pu_t{0.4},      // Target torque [pu]   - Verify smooth acceleration

    .duration = duration_t{5.0}     // Ramp time [s], 0 = instant   - Monitor for commutation failures

};

sequencer.configure_virtual_torque(torque_cfg);4. **Test Full Sequence**

```   - Configure all parameters

   - Execute complete startup

**Parameter Details**:   - Verify smooth transition to closed-loop

   - Check speed controller takes over cleanly

| Field | Type | Description | Typical Values |

|-------|------|-------------|----------------|### Monitoring Points

| `initial` | `torque_pu_t` | Starting torque reference | 0.2-0.5 pu |

| `final` | `torque_pu_t` | Target torque reference | 0.2-0.5 pu |During commissioning, monitor these signals:

| `duration` | `duration_t` | Ramp time (0 = instant) | 0-10 s |- `current_state`: Verify state progression

- `virtual_rotor_frequency`: Check frequency ramp profile

**Benefits of Torque Ramping**:- `virtual_rotor_angle`: Verify integration (should increase monotonically)

- Smooth acceleration (reduced mechanical stress)- `is_field_established`: Confirm field readiness

- Gradual load application- `time_in_current_state`: Track state durations

- Better control of inrush currents- Machine speed (from observer): Compare to virtual frequency

- Adjustable torque profile for different load conditions

---

### Configuration Examples

## 8. Fault Handling

**Example 1: Standard Startup**

```cpp### Automatic Transitions to IDLE

// Configure frequency ramp: 1 Hz → 10 Hz over 10 seconds

virtual_frequency_config freq_cfg{The sequencer automatically returns to IDLE state when:

    .initial = frequency_t{1.0},- `stop_sequence()` is called (from any state)

    .final = frequency_t{10.0},- Main state machine exits OPERATION_ON state

    .duration = duration_t{10.0}

};### Recommended External Protection



// Configure torque ramp: 0.2 pu → 0.4 pu over 5 secondsThe application should monitor for:

virtual_torque_config torque_cfg{- **Flux Build-Up Timeout**: If `is_field_established` never becomes true

    .initial = torque_pu_t{0.2},- **Excessive Acceleration Time**: If machine doesn't reach expected speed

    .final = torque_pu_t{0.4},- **Commutation Failures**: During STARTING_PROCEDURE

    .duration = duration_t{5.0}- **Observer Loss**: If estimated angle becomes unreliable

};

When a fault is detected, call `stop_sequence()` to safely shut down.

sequencer.configure_virtual_frequency(freq_cfg);

sequencer.configure_virtual_torque(torque_cfg);---

```

## 9. API Reference

**Example 2: Constant Torque Startup**

```cpp### Public Methods

// Ramp frequency as normal

virtual_frequency_config freq_cfg{#### Configuration

    .initial = frequency_t{1.0},```cpp

    .final = frequency_t{10.0},void configure(const startup_sequencer_config& config);

    .duration = duration_t{10.0}```

};Set all tunable parameters. Call during initialization.



// Hold constant torque (initial == final)#### Control

virtual_torque_config torque_cfg{```cpp

    .initial = torque_pu_t{0.3},bool start_sequence();

    .final = torque_pu_t{0.3},     // Same value = constant```

    .duration = duration_t{0.0}    // Duration doesn't matterStart the startup sequence from IDLE. Returns false if not in IDLE state.

};

```cpp

sequencer.configure_virtual_frequency(freq_cfg);void stop_sequence();

sequencer.configure_virtual_torque(torque_cfg);```

```Immediately return to IDLE state. Safe to call from any state.



**Example 3: Quick Resynchronization**```cpp

```cppvoid update(bool is_field_established, angle_t estimated_rotor_angle);

// Skip virtual phase entirely (machine already rotating)```

virtual_frequency_config freq_cfg{Main update function. Call every control cycle (e.g., 50µs).

    .initial = frequency_t{10.0},

    .final = frequency_t{10.0},```cpp

    .duration = duration_t{0.0}    // Skip rampvoid reset();

};```

Reset all internal states, timers, and return to IDLE. For initialization or fault recovery.

// Instant torque application

virtual_torque_config torque_cfg{#### Outputs

    .initial = torque_pu_t{0.3},```cpp

    .final = torque_pu_t{0.3},const startup_sequencer_outputs& get_outputs() const noexcept;

    .duration = duration_t{0.0}    // Instant```

};Get complete output structure containing all control signals.



sequencer.configure_virtual_frequency(freq_cfg);```cpp

sequencer.configure_virtual_torque(torque_cfg);current_pu_t get_flux_setpoint() const noexcept;

``````

Get the configured field current setpoint for the excitation controller.

---

#### Status

## 4. Grouped Output Structures```cpp

startup_sequencer_state_t get_current_state() const noexcept;

The sequencer provides **three grouped output structures** for clarity:```

Get current state (IDLE, FLUX_BUILDUP, STARTING_PROCEDURE, CLOSED_LOOP_OPERATION).

### Output Structure Overview

```cpp

```cppbool is_startup_complete() const noexcept;

struct startup_sequencer_outputs {```

    flux_buildup_outputs flux_buildup;Returns true when in CLOSED_LOOP_OPERATION state.

    virtual_reference_outputs virtual_reference;

    controller_command_outputs controller_commands;```cpp

};frequency_t get_virtual_frequency() const noexcept;

``````

Get current virtual frequency (for monitoring/diagnostics).

### Flux Build-Up Outputs

```cpp

**Currently empty** (excitation controller manages its own setpoints independently)duration_t get_time_in_current_state() const noexcept;

```

```cppGet elapsed time in current state (for monitoring/debugging).

struct flux_buildup_outputs {

    // Reserved for future use---

};

```## 10. Related Documentation



### Virtual Reference Outputs- [Flux Build-Up Procedure](flux_buildup_procedure.md) - Detailed theory on pre-fluxing

- [Virtual Reference Frame](virtual_reference_frame.md) - V/Hz starting method

```cpp- [SFC State Machine](state_machine.md) - Main operational states

struct virtual_reference_outputs {- [Torque Controller](torque_controller.md) - Control mode switching

    angle_t rotor_angle;           // Virtual rotor angle [rad]- [Speed Controller](speed_controller.md) - Closed-loop operation

    frequency_t rotor_frequency;   // Virtual rotor frequency [Hz]

    torque_pu_t torque_reference;  // Virtual torque reference [pu]---

};

```## 11. References



**Usage**:- IEEE Std 421.5-2016: Recommended Practice for Excitation System Models

- `rotor_angle` → Machine-side modulator (when reference_frame_source == VIRTUAL)- "Load-Commutated Inverter Drives for Large Synchronous Motors", ABB Technical Paper

- `rotor_frequency` → Monitoring/diagnostics- "Starting Methods for Synchronous Motors", Siemens Application Note

- `torque_reference` → Torque controller (when torque_control_mode == MANUAL_TORQUE)

### Controller Command Outputs

```cpp
struct controller_command_outputs {
    reference_frame_source_t reference_frame_source;
    sfc_torque_controller::control_mode torque_control_mode;
    bool release_machine_side_pulses;
    bool release_excitation;
    bool release_machine_observer;
    bool release_speed_controller;
    bool release_torque_controller;
    startup_sequencer_state_t current_state;
};
```

**Usage**:
- `reference_frame_source` → Modulator (select VIRTUAL or ESTIMATED angle)
- `torque_control_mode` → Torque controller (MANUAL_TORQUE or SPEED_CONTROL)
- `release_*` → Enable signals for various components
- `current_state` → State machine status

### Accessing Outputs

```cpp
auto outputs = sequencer.get_outputs();

// Access virtual reference outputs
angle_t angle = outputs.virtual_reference.rotor_angle;
frequency_t freq = outputs.virtual_reference.rotor_frequency;
torque_pu_t torque = outputs.virtual_reference.torque_reference;

// Access controller commands
bool enable_speed = outputs.controller_commands.release_speed_controller;
reference_frame_source_t ref_source = outputs.controller_commands.reference_frame_source;
startup_sequencer_state_t state = outputs.controller_commands.current_state;
```

---

## 5. Integration Example

### Complete Integration

```cpp
#include "sfc/sfc_startup_sequencer.hpp"

// Construction (1ms sampling time)
sfc_startup_sequencer sequencer(duration_t{0.001});

// Configuration
virtual_frequency_config freq_cfg{
    .initial = frequency_t{1.0},
    .final = frequency_t{10.0},
    .duration = duration_t{10.0}
};

virtual_torque_config torque_cfg{
    .initial = torque_pu_t{0.2},
    .final = torque_pu_t{0.4},
    .duration = duration_t{5.0}
};

sequencer.configure_virtual_frequency(freq_cfg);
sequencer.configure_virtual_torque(torque_cfg);

// Start sequence when entering OPERATION_ON state
sequencer.start_sequence();

// In main control loop (1ms cycle)
while (running) {
    // Get inputs
    bool is_field_established = excitation_controller.is_field_established();
    angle_wrapped estimated_angle = observer.get_estimated_angle();
    
    // Update sequencer
    sequencer.update(is_field_established, estimated_angle);
    
    // Get outputs
    auto outputs = sequencer.get_outputs();
    
    // Apply controller commands
    speed_controller.set_enable(outputs.controller_commands.release_speed_controller);
    torque_controller.set_enable(outputs.controller_commands.release_torque_controller);
    torque_controller.set_control_mode(outputs.controller_commands.torque_control_mode);
    
    // Apply reference frame selection
    if (outputs.controller_commands.reference_frame_source == reference_frame_source_t::VIRTUAL) {
        modulator.set_reference_angle(outputs.virtual_reference.rotor_angle);
        torque_controller.set_manual_torque(outputs.virtual_reference.torque_reference);
    } else {
        modulator.set_reference_angle(observer.get_estimated_angle());
    }
    
    // Monitor state
    if (outputs.controller_commands.current_state == startup_sequencer_state_t::CLOSED_LOOP_OPERATION) {
        // Startup complete, normal operation
    }
}

// Stop sequence when exiting OPERATION_ON
sequencer.stop_sequence();
```

---

## 6. Operational Timeline

### Typical 13-Second Startup Example

```
Time   State                   Actions                              Outputs
─────────────────────────────────────────────────────────────────────────────
0.0s   IDLE                    - System safe                        - All releases false
       ↓ start_sequence()      - All outputs disabled               - reference_source = ESTIMATED
       
0.0s   FLUX_BUILDUP            - release_excitation = true          - Virtual reference zeroed
                               - Excitation controller activated    - Observer disabled
                               - Field current rising               - Pulses disabled
                               - Wait for is_field_established      
3.0s   ↓ is_field_established

3.0s   STARTING_PROCEDURE      - Virtual frequency: 1 Hz → 10 Hz    - release_pulses = true
                               - Virtual torque: 0.2 → 0.4 pu       - release_observer = true
                               - Virtual angle integrating          - release_torque_ctrl = true
                               - Angle synchronization active       - reference_source = VIRTUAL
                               - Machine accelerating               - torque_mode = MANUAL
13.0s  ↓ Ramps complete

13.0s  CLOSED_LOOP_OPERATION   - Switch to estimated frame          - release_speed_ctrl = true
                               - Enable speed controller            - reference_source = ESTIMATED
                               - Speed control takes over           - torque_mode = SPEED_CONTROL
                               - Normal operation                   - is_startup_complete = true
```

---

## 7. Testing and Commissioning

### Initial Testing Sequence

**Step 1: Test Flux Build-Up Only**
```cpp
// Skip virtual phase entirely
virtual_frequency_config freq_cfg{
    .initial = frequency_t{1.0},
    .final = frequency_t{1.0},
    .duration = duration_t{0.0}    // Skip
};

sequencer.configure_virtual_frequency(freq_cfg);
sequencer.start_sequence();
```
- Verify excitation controller activates
- Measure actual field current rise time
- Confirm is_field_established flag timing
- Validate no machine rotation occurs

**Step 2: Test Constant Virtual Frequency**
```cpp
// Hold at 2 Hz for 5 seconds
virtual_frequency_config freq_cfg{
    .initial = frequency_t{2.0},
    .final = frequency_t{2.0},
    .duration = duration_t{5.0}
};

virtual_torque_config torque_cfg{
    .initial = torque_pu_t{0.3},
    .final = torque_pu_t{0.3},
    .duration = duration_t{0.0}
};

sequencer.configure_virtual_frequency(freq_cfg);
sequencer.configure_virtual_torque(torque_cfg);
sequencer.start_sequence();
```
- Monitor firing pulse timing
- Check commutation stability
- Verify constant speed maintenance

**Step 3: Test Frequency Ramp**
```cpp
// Full ramp test
virtual_frequency_config freq_cfg{
    .initial = frequency_t{1.0},
    .final = frequency_t{10.0},
    .duration = duration_t{10.0}
};

virtual_torque_config torque_cfg{
    .initial = torque_pu_t{0.3},
    .final = torque_pu_t{0.3},
    .duration = duration_t{0.0}
};

sequencer.configure_virtual_frequency(freq_cfg);
sequencer.configure_virtual_torque(torque_cfg);
sequencer.start_sequence();
```
- Monitor acceleration profile
- Check for commutation failures
- Verify ramp linearity
- Measure actual vs. commanded frequency

**Step 4: Test Torque Ramping**
```cpp
// Test torque ramp independently
virtual_frequency_config freq_cfg{
    .initial = frequency_t{5.0},
    .final = frequency_t{5.0},
    .duration = duration_t{10.0}
};

virtual_torque_config torque_cfg{
    .initial = torque_pu_t{0.1},
    .final = torque_pu_t{0.5},
    .duration = duration_t{8.0}    // Ramp over 8 seconds
};

sequencer.configure_virtual_frequency(freq_cfg);
sequencer.configure_virtual_torque(torque_cfg);
sequencer.start_sequence();
```
- Verify smooth torque increase
- Monitor DC link current tracking
- Check mechanical response

**Step 5: Test Full Sequence**
```cpp
// Complete startup with all features
virtual_frequency_config freq_cfg{
    .initial = frequency_t{1.0},
    .final = frequency_t{10.0},
    .duration = duration_t{10.0}
};

virtual_torque_config torque_cfg{
    .initial = torque_pu_t{0.2},
    .final = torque_pu_t{0.4},
    .duration = duration_t{5.0}
};

sequencer.configure_virtual_frequency(freq_cfg);
sequencer.configure_virtual_torque(torque_cfg);
sequencer.start_sequence();
```
- Run complete startup sequence
- Verify all state transitions
- Check speed controller takeover
- Validate smooth handoff to closed-loop

### Tuning Procedure

1. **Determine Field Time Constant**:
   - Apply step voltage to field winding
   - Measure time to 63.2% of final current
   - τ_f = measured time

2. **Optimize Virtual Frequency Ramp**:
   - Start conservative (slow ramp rate)
   - Gradually increase final frequency
   - Monitor commutation margin
   - Find minimum safe final frequency

3. **Tune Virtual Torque Profile**:
   - Start with low constant torque
   - Test ramped vs. constant torque
   - Adjust initial/final values for smooth acceleration
   - Balance startup time vs. mechanical stress

4. **Validate Transition**:
   - Monitor angle discontinuity at switchover
   - Check current/torque spikes
   - Verify observer lock quality
   - Adjust final frequency if needed

---

## 8. Troubleshooting

### Issue: Sequencer Stuck in FLUX_BUILDUP

**Symptoms**: State never transitions to STARTING_PROCEDURE

**Possible Causes**:
1. Excitation controller not releasing `is_field_established` flag
2. Excitation system fault
3. Field winding open circuit

**Diagnosis**:
```cpp
// Check sequencer state
if (sequencer.get_current_state() == startup_sequencer_state_t::FLUX_BUILDUP) {
    // Check excitation controller status
    bool field_established = excitation_controller.is_field_established();
    // Check field current feedback (if available)
    current_t field_current = excitation_controller.get_field_current();
}
```

**Solution**:
- Verify excitation controller is properly configured
- Check field current sensor calibration
- Inspect field circuit for faults

### Issue: Commutation Failures During STARTING_PROCEDURE

**Symptoms**: Firing pulses irregular, DC link current oscillating

**Possible Causes**:
1. Virtual frequency ramping too fast
2. Insufficient torque (machine can't track frequency)
3. Initial frequency too low

**Solution**:
- Reduce `virtual_frequency_duration` (slower ramp)
- Increase `virtual_torque_initial` (more breakaway torque)
- Raise `virtual_frequency_initial` (start at higher frequency)

### Issue: Angle Discontinuity at Transition

**Symptoms**: Current spike when switching to CLOSED_LOOP_OPERATION

**Possible Causes**:
1. Observer not converged
2. Virtual-to-estimated angle mismatch
3. Synchronization not working

**Diagnosis**:
```cpp
// Monitor angle difference before transition
angle_t virtual_angle = outputs.virtual_reference.rotor_angle;
angle_t estimated_angle = observer.get_estimated_angle();
angle_t error = estimated_angle - virtual_angle;
```

**Solution**:
- Increase `virtual_frequency_final` (allow more observer settling time)
- Verify observer is enabled during STARTING_PROCEDURE
- Check synchronization logic in sequencer

### Issue: Machine Doesn't Accelerate

**Symptoms**: Virtual frequency ramping but speed stays near zero

**Possible Causes**:
1. Insufficient torque
2. Mechanical brake engaged
3. Load too high for configured torque

**Solution**:
- Increase `virtual_torque_initial` and `virtual_torque_final`
- Check mechanical systems (brakes, couplings)
- Verify DC link current is tracking torque reference

---

## 9. API Changes Summary

### Removed Features

❌ **Flux Buildup Configuration** (now handled by excitation controller):
- `flux_buildup_duration` parameter removed
- `configure_flux_buildup()` method removed
- Internal flux buildup timer removed

### New Features

✅ **Grouped Configuration Structures**:
- `virtual_frequency_config` - separate configuration for frequency ramp
- `virtual_torque_config` - separate configuration for torque ramp
- `configure_virtual_frequency()` method
- `configure_virtual_torque()` method

✅ **Torque Ramping**:
- Independent torque ramp with adjustable duration
- Constant torque mode (initial == final)
- Instant torque application (duration == 0)

✅ **Grouped Output Structures**:
- `flux_buildup_outputs` - reserved for future use
- `virtual_reference_outputs` - angle, frequency, torque
- `controller_command_outputs` - enables, modes, state

✅ **Improved Angle Wrapping**:
- Uses `integrator<angle_wrapped>` for automatic wrapping
- Zero-cost wrapping via uint32 overflow
- Eliminated manual angle normalization loops

---

## 10. Related Documentation

- [Dictionary](dictionary.md) - Terminology and naming conventions
- [Per-Unit System](per_unit_system.md) - Normalization framework
- [Machine-Side Observer](machine_side_observer.md) - Angle estimation
- [Excitation Controller](excitation_controller.md) - Field control
- [Torque Controller](torque_controller.md) - Torque regulation
- [Speed Controller](speed_controller.md) - Speed loop design

---

**Document Version:** 2.0  
**Last Updated:** November 9, 2025  
**Author:** SFC Control System Documentation  
**Changes**: Complete rewrite with API v2.0 updates, removed flux buildup timer, added torque ramping, grouped outputs
