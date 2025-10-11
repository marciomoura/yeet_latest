# Excitation Control

## 1. Objective and Core Principle

**Primary Objective:** Generate a time-varying excitation voltage setpoint reference that is sent to the external excitation controller to maintain optimal magnetic flux in the synchronous machine during LCI-driven acceleration.

**Core Principle:** The algorithm is designed to establish and maintain a constant **Volts-per-Hertz (V/Hz)** ratio in the synchronous machine's air gap. This ensures the magnetic flux is held at an optimal, constant level during the acceleration phase up to a certain speed. Constant flux allows the machine to produce maximum torque for a given stator current, which is critical for efficiently accelerating large gas turbines.

The application does not calculate V/Hz directly. Instead, it uses a pre-commissioned lookup table that directly maps **rotor speed** to the **required excitation voltage setpoint** (expressed in per-unit) to achieve the target V/Hz characteristic. This table is empirically derived from the generator's design data (its open-circuit saturation curve and exciter characteristics).

---

## 2. Component Architecture

### Class: `sfc_excitation_controller`

This component follows the standard `yeet_application_sfc` architectural pattern with stateless control logic and explicit enable/disable methods.

#### **Real-Time Inputs (passed to `update()`):**

All real-time data must be passed as arguments to the `update()` method:

1.  **`machine_rotor_speed_pu`** (`speed_pu_t`)
    *   **Source:** Machine-side observer or encoder feedback
    *   **Format:** Per-unit speed (0.0 to 1.0+ normalized to machine base frequency)
    *   **Purpose:** Primary independent variable for the lookup table

2.  **`exciter_ready`** (`bool`)
    *   **Source:** Digital input or fieldbus signal from external excitation controller
    *   **Format:** Boolean flag indicating exciter health and readiness
    *   **Purpose:** Critical permissive for field energization

#### **Configuration Parameters (set via `configure_*()` methods):**

1.  **Excitation Voltage Lookup Table:**
    *   **Method:** `configure_excitation_voltage_lookup_table()`
    *   **Parameters:** Arrays of speed breakpoints and corresponding excitation voltage setpoint values (in per-unit)
    *   **Default:** 6-point table covering 0% to 100% speed range

2.  **Pre-Fluxing Parameters:**
    *   **Method:** `configure_prefluxing_duration()`
    *   **Parameter:** `duration_t prefluxing_time` - Time to establish magnetic field before torque application
    *   **Default:** 2.0 seconds
    *   **Purpose:** Allows the highly inductive field winding to fully energize

3.  **Setpoint Tolerance (when no feedback available):**
    *   **Method:** `configure_field_established_timeout()`
    *   **Parameter:** `duration_t timeout` - Maximum time to wait for field establishment
    *   **Default:** 5.0 seconds
    *   **Purpose:** Assumes field is established if timeout expires without fault

#### **Control Outputs (retrieved via `get_*()` methods):**

1.  **`get_excitation_voltage_setpoint_pu()`** → `voltage_pu_t`
    *   **Destination:** Analog output or fieldbus interface to excitation controller
    *   **Format:** Per-unit voltage setpoint (normalized to exciter rated voltage)
    *   **Range:** 0.0 pu to 1.0+ pu

2.  **`get_enable_excitation()`** → `bool`
    *   **Destination:** Digital output or fieldbus command to excitation controller
    *   **Format:** Boolean command (true = enable, false = disable)
    *   **Purpose:** Master enable for external exciter regulation loop

3.  **`get_is_field_established()`** → `bool`
    *   **Status:** Indicates whether pre-fluxing sequence is complete
    *   **Logic:** True after pre-fluxing timer expires without fault
    *   **Purpose:** Permissive for torque production

---

## 3. Control Methods and Operational Sequence

### Starting the Excitation Sequence

#### **Method:** `start_excitation()`

**Preconditions:**
- Exciter must be ready (`exciter_ready == true`)
- Component must not already be running
- No active faults

**Actions:**
1. Enables excitation output (`enable_excitation = true`)
2. Sets excitation voltage setpoint to initial value (0% speed point from lookup table)
3. Starts pre-fluxing timer
4. Sets internal running flag

**Returns:** `bool` - Success if preconditions met, false otherwise

**Example Usage:**
```cpp
if (excitation_controller.start_excitation()) {
    // Pre-fluxing sequence initiated successfully
    // System will wait for field to establish
}
```

---

### Stopping the Excitation Sequence

#### **Method:** `stop_excitation()`

**Actions:**
1. Disables excitation output (`enable_excitation = false`)
2. Ramps excitation voltage setpoint to zero
3. Resets pre-fluxing timer
4. Clears internal running flag

**Returns:** `void`

**Example Usage:**
```cpp
excitation_controller.stop_excitation();
// Field will be safely de-energized
```

---

### Main Update Cycle

#### **Method:** `update(speed_pu_t machine_rotor_speed_pu, bool exciter_ready)`

**Behavior:**

1. **If not running:** Outputs remain disabled (excitation voltage = 0.0 pu, enable = false)

2. **If running and in pre-fluxing phase:**
   - Maintains excitation voltage at initial value (0% speed point)
   - Monitors pre-fluxing timer
   - If timer expires: marks field as established, transitions to normal operation
   - If exciter fault detected: calls `stop_excitation()` internally

3. **If running and field established:**
   - Evaluates lookup table with current `machine_rotor_speed_pu`
   - Updates excitation voltage setpoint via linear interpolation
   - Maintains `enable_excitation = true`
   - Continuously monitors `exciter_ready` for faults

**Example Usage:**
```cpp
// Called every control cycle (e.g., 1 kHz)
excitation_controller.update(
    machine_observer.get_estimated_speed_pu(),
    digital_inputs.get_exciter_ready_status()
);

// Retrieve outputs
voltage_pu_t excitation_voltage_setpoint = excitation_controller.get_excitation_voltage_setpoint_pu();
bool enable_cmd = excitation_controller.get_enable_excitation();
```

---

## 4. Excitation Voltage Lookup Table

### Table Structure

The lookup table is the core data structure defining the V/Hz profile. It uses **linear interpolation** between configured points to generate smooth excitation voltage setpoint references for any rotor speed.

#### **Default 6-Point Table Configuration:**

| Speed (pu) | Excitation Voltage (pu) | Description                                                                                                   |
| :--------- | :---------------------- | :------------------------------------------------------------------------------------------------------------ |
| 0.00       | 0.30                    | **Pre-Fluxing Voltage:** Establishes magnetic field at standstill before torque application                   |
| 0.05       | 0.30                    | **Initial Hold:** Maintains constant flux during initial slow-speed rotation for maximum breakaway torque     |
| 0.80       | 1.00                    | **V/Hz Corner Point:** Linear ramp from 5% to 80% maintains constant V/Hz ratio                               |
| 0.95       | 0.80                    | **Field Weakening:** Above 80%, flux is reduced to allow further acceleration at voltage limit                |
| 1.00       | 0.73                    | **Nominal Operating Point:** Excitation voltage at rated speed (handoff to turbine self-sustaining operation) |
| 1.05       | 0.70                    | **Overspeed Margin:** Continued field weakening for slight overspeed conditions                               |

*Note: Per-unit values are normalized to the exciter's rated output voltage. Actual per-unit values must be commissioned based on the specific synchronous machine's saturation curve and open-circuit characteristics.*

### Configuration Example

```cpp
// Speed breakpoints in per-unit (normalized to machine base frequency)
std::array<real_t, 6> speed_axis_pu = {
    0.00, 0.05, 0.80, 0.95, 1.00, 1.05
};

// Corresponding excitation voltage setpoints in per-unit (normalized to exciter rated voltage)
std::array<real_t, 6> excitation_voltage_axis_pu = {
    0.30, 0.30, 1.00, 0.80, 0.73, 0.70
};

excitation_controller.configure_excitation_voltage_lookup_table(
    speed_axis_pu,
    excitation_voltage_axis_pu
);
```

### Interpolation Algorithm

For any given speed between table points, the excitation voltage setpoint is calculated using linear interpolation:

$$
v_{\text{excitation,ref}} = v_{\text{excitation},n} + \frac{\omega_{\text{rotor}} - \omega_n}{\omega_{n+1} - \omega_n} \cdot (v_{\text{excitation},n+1} - v_{\text{excitation},n})
$$

Where:
- $\omega_{\text{rotor}}$ is the current rotor speed (pu)
- $\omega_n$, $\omega_{n+1}$ are adjacent speed breakpoints
- $v_{\text{excitation},n}$, $v_{\text{excitation},n+1}$ are corresponding excitation voltage setpoints (pu)

---

## 5. Timer-Based Field Establishment Detection

When direct field current feedback is unavailable, the controller uses timer-based logic to determine when the field has been established.

### Pre-Fluxing Timer Logic

```cpp
// In update() method, during pre-fluxing phase:
if (is_running && !is_field_established) {
    prefluxing_timer.update(sampling_time);
    
    if (prefluxing_timer.has_elapsed()) {
        is_field_established = true;
        // Ready for torque production
    }
}
```

### Configuration

```cpp
// Set pre-fluxing duration based on field winding time constant
// Typical value: 3-5 times the field winding L/R time constant
excitation_controller.configure_prefluxing_duration(duration_t{2.0}); // 2 seconds

// Set maximum timeout for safety
excitation_controller.configure_field_established_timeout(duration_t{5.0}); // 5 seconds
```

### Alternative: Feedback-Based Detection

If excitation voltage or field current feedback is available from the exciter:

```cpp
// Future enhancement: Configure tolerance for setpoint achievement
// excitation_controller.configure_excitation_voltage_tolerance(voltage_pu_t{0.05}); // ±5% pu

// The controller can then verify actual excitation voltage matches reference
// bool field_established = excitation_controller.get_is_field_at_setpoint();
```

---

## 6. Integration with LCI Control System

### Typical Operational Sequence

```cpp
// 1. System initialization
excitation_controller.configure_prefluxing_duration(duration_t{2.0});
excitation_controller.configure_excitation_voltage_lookup_table(speed_axis, voltage_axis);

// 2. Start sequence - called when LCI receives start command
if (state_machine.is_in_state(BREAKERS_ON) && exciter_is_healthy) {
    if (excitation_controller.start_excitation()) {
        // Pre-fluxing initiated
        state_machine.transition_to(PREFLUXING);
    }
}

// 3. Main control loop - called every cycle
excitation_controller.update(
    machine_observer.get_estimated_speed_pu(),
    digital_inputs.get_exciter_ready()
);

// 4. Check for field establishment
if (excitation_controller.get_is_field_established() &&
    state_machine.is_in_state(PREFLUXING)) {
    // Field is ready, can start torque production
    state_machine.transition_to(RAMPING);
    speed_controller.enable();
}

// 5. Output to hardware
analog_outputs.set_excitation_voltage_setpoint(
    excitation_controller.get_excitation_voltage_setpoint_pu()
);
digital_outputs.set_exciter_enable(
    excitation_controller.get_enable_excitation()
);

// 6. Stop sequence - called on shutdown or fault
if (stop_command_received || fault_detected) {
    excitation_controller.stop_excitation();
    state_machine.transition_to(SHUTDOWN);
}
```

---

## 7. Safety Considerations

### Fault Detection and Response

1. **Exciter Not Ready:**
   - If `exciter_ready` becomes false during operation, automatically call `stop_excitation()`
   - Log fault condition and transition to safe state

2. **Pre-Fluxing Timeout:**
   - If field is not established within configured timeout, raise alarm
   - Allows manual intervention or automatic retry logic

3. **Speed Sensor Loss:**
   - If speed measurement becomes invalid, freeze excitation voltage at last known good value
   - Do not ramp to zero abruptly (can cause voltage transients)

### Controlled Shutdown

```cpp
void shutdown_with_demagnetization() {
    // Ramp excitation voltage down gradually over configured time
    excitation_controller.configure_excitation_ramp_down_time(duration_t{1.0}); // 1 second
    excitation_controller.stop_excitation();
    
    // Controller will automatically ramp to zero over 1 second
}
```

---

## 8. Variable Naming Conventions

Following the `yeet` platform dictionary standard of `location_quantity_unit`:

| Programming Variable             | LaTeX Symbol                | Description                                           |
| :------------------------------- | :-------------------------- | :---------------------------------------------------- |
| `machine_rotor_speed_pu`         | $\omega_{\text{rotor}}$     | Machine rotor speed in per-unit                       |
| `excitation_voltage_setpoint_pu` | $v_{\text{excitation,ref}}$ | Commanded excitation voltage setpoint to exciter (pu) |
| `field_winding_time_constant`    | $\tau_{\text{field}}$       | Field winding L/R time constant                       |
| `prefluxing_duration`            | $t_{\text{preflux}}$        | Pre-fluxing timer duration                            |
| `exciter_ready`                  | -                           | Digital status from external exciter                  |
| `enable_excitation`              | -                           | Digital command to external exciter                   |
| `is_field_established`           | -                           | Internal status flag                                  |

---

## 9. Summary

The `sfc_excitation_controller` provides a clean, deterministic interface for managing excitation voltage setpoints in LCI-driven synchronous machines:

- **No internal state machine** - controlled via explicit `start_excitation()` and `stop_excitation()` methods
- **Timer-based field establishment** - works without field current feedback
- **Lookup table architecture** - flexible configuration for different machine characteristics
- **Per-unit voltage setpoints** - scalable across different exciter ratings
- **Standard `yeet_application_sfc` pattern** - consistent with other SFC components
- **Real-time safe** - no dynamic allocation, deterministic execution

The controller acts as the master conductor, commanding the external excitation controller to deliver precise excitation voltage that maintains optimal flux throughout the acceleration sequence.