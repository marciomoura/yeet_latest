# Commutation Notch Masking for PLL Stability

## 1. Rationale and Purpose

In line-commutated converter systems, the process of commutation—transferring current from one thyristor to another—causes a temporary short circuit between two phases. This results in a characteristic "notch" in the measured AC voltage waveforms. These notches are significant distortions that can severely impact the performance and stability of a Phase-Locked Loop (PLL), which relies on a clean sinusoidal waveform to accurately track the grid's phase and frequency.

The `sfc_commutation_mask` module is designed to solve this problem. It acts as a filter, providing the PLL with a clean, synthesized sinusoidal waveform during the commutation period. By "masking" the notch, it ensures the PLL remains locked and stable, which is critical for the correct operation of the overall control system.

## 2. Algorithm Description

The algorithm operates as a state machine, switching between two primary modes: **PASS-THROUGH** and **MASKING**. The transition between these modes is controlled by the `is_commuting` boolean signal provided by the `sfc_commutation_monitor`.

### State 1: PASS-THROUGH (Not Commuting)

When `is_commuting` is `false`, the module is in its normal pass-through mode.

1.  **Output:** The measured three-phase voltages are passed directly to the output without modification.
2.  **State Update:** The module continuously prepares for a potential commutation event. It converts the measured `abc` voltages into the rotating `dq` reference frame using the PLL's current angle (`θ_pll`). The d-component of this vector, which represents the magnitude of the voltage, is stored as `_last_good_magnitude`.

    `V_dq = Park(V_abc, θ_pll)`

    `_last_good_magnitude = V_d`

### State 2: MASKING (Commutation Detected)

When `is_commuting` becomes `true`, the module transitions to masking mode.

1.  **Transition (First Cycle):** On the rising edge of `is_commuting`, the module latches the current PLL angle (`θ_pll`) into an internal state variable, `_internal_angle`. The `_last_good_magnitude` captured in the previous cycle is now frozen for the duration of the notch.

2.  **Waveform Synthesis (During Commutation):** For the entire period that `is_commuting` remains `true`, the module ignores the measured voltages. Instead, it generates a synthetic waveform:
    *   The internal angle is propagated forward by integrating the estimated PLL frequency (`ω_pll`) over the time step (`dt`):

        `_internal_angle(t) = _internal_angle(t-1) + ω_pll * dt`

    *   A synthetic `dq` voltage vector is created using the frozen magnitude. The q-component is set to zero, representing a perfectly aligned, purely active voltage vector.

        `V_dq_synthetic = {d: _last_good_magnitude, q: 0}`

    *   This synthetic `dq` vector is transformed back to the `abc` frame using the inverse Park transformation with the propagated `_internal_angle`.

        `V_abc_output = InversePark(V_dq_synthetic, _internal_angle)`

3.  **Return to Pass-Through:** When `is_commuting` returns to `false`, the module seamlessly switches back to the PASS-THROUGH state on the next cycle, once again using the live measured voltages.

This process ensures that the PLL receives a smooth, continuous sinusoidal signal, effectively bridging the gap caused by the commutation notch.

## 3. Implementation Interface

### Class: `sfc_commutation_mask`

**Description:**
A stateful class that masks commutation notches from a measured three-phase voltage waveform to ensure a clean input for a PLL.

**Function Prototype:**

```cpp
abc<voltage_pu_t> sfc_commutation_mask::update(
    const abc<voltage_pu_t>& measured_voltages,
    const angle_wrapped& pll_angle,
    const frequency_t& pll_frequency,
    bool is_commuting,
    real_t dt);
```

**Inputs:**

*   `measured_voltages` (`const abc<voltage_pu_t>&`): The instantaneous measured three-phase voltages in per-unit.
*   `pll_angle` (`const angle_wrapped&`): The current angle output from the system's PLL.
*   `pll_frequency` (`const frequency_t&`): The current frequency output from the system's PLL.
*   `is_commuting` (`bool`): The boolean signal from `sfc_commutation_monitor`, which is `true` during a commutation notch.
*   `dt` (`real_t`): The execution time step (delta-time) in seconds, used for integration.

**Output:**

*   (`abc<voltage_pu_t>`): The processed three-phase voltage waveform. This is either the original measurement or a synthetically generated waveform, depending on the `is_commuting` state.

**Internal State Variables:**

*   `_last_good_magnitude` (`voltage_pu_t`): Stores the voltage vector magnitude (d-component) captured just before a commutation event begins.
*   `_internal_angle` (`angle_wrapped`): The internal angle used for reconstructing the waveform, propagated forward during the commutation period.
*   `_is_masking` (`bool`): An internal flag to track if the module is currently in the masking state.

## 4. Mathematical Foundation

### Park and Inverse Park Transformations

The `sfc_commutation_mask` relies heavily on coordinate transformations between the stationary `abc` frame and the rotating `dq` frame. The mathematical relationships are:

**Park Transformation (abc → dq):**
```
[V_d]   [cos(θ)    cos(θ-2π/3)   cos(θ+2π/3)] [V_a]
[V_q] = [-sin(θ)  -sin(θ-2π/3)  -sin(θ+2π/3)] [V_b]
                                               [V_c]
```

**Inverse Park Transformation (dq → abc):**
```
[V_a]   [cos(θ)    -sin(θ)  ] [V_d]
[V_b] = [cos(θ-2π/3) -sin(θ-2π/3)] [V_q]
[V_c]   [cos(θ+2π/3) -sin(θ+2π/3)]
```

Where `θ` is the angle of the rotating reference frame.

### Frequency Integration

During the masking period, the internal angle is propagated using simple Euler integration:

```
θ_internal(k+1) = θ_internal(k) + ω_pll * Δt
```

Where:
- `θ_internal(k)` is the internal angle at time step k
- `ω_pll` is the angular frequency from the PLL (rad/s)
- `Δt` is the control loop time step

## 5. Performance Considerations

### Computational Complexity

The `sfc_commutation_mask` performs the following operations each control cycle:

**During Pass-Through Mode:**
- One `abc` to `dq` transformation (involves trigonometric calculations)
- Simple assignment operations

**During Masking Mode:**
- One angle integration (addition operation)
- One `dq` to `abc` transformation (involves trigonometric calculations)

The computational load is dominated by the trigonometric functions (`sin` and `cos`) used in the coordinate transformations. Modern control processors typically handle these efficiently.

### Memory Requirements

The module maintains minimal state:
- One `voltage_pu_t` value for magnitude storage
- One `angle_wrapped` for internal angle
- One `bool` for state tracking

Total memory footprint is typically less than 16 bytes.

## 6. Integration Guidelines

### Typical Usage Pattern

```cpp
// Initialization
sfc_commutation_mask mask;
sfc_commutation_monitor monitor{{.commutation_end_threshold = 0.05F}};

// In the control loop
bool is_commuting = monitor.update(measured_voltages, firing_pulses);
abc<voltage_pu_t> clean_voltages = mask.update(
    measured_voltages, 
    pll_angle, 
    pll_frequency, 
    is_commuting, 
    control_dt
);

// Feed clean voltages to PLL instead of raw measurements
pll.update(clean_voltages);
```

### Configuration Considerations

- **Time Step (`dt`):** Should match the control loop execution period
- **PLL Frequency Accuracy:** The quality of the synthesized waveform depends on the accuracy of the PLL frequency estimate
- **Commutation Detection:** Requires proper tuning of the `sfc_commutation_monitor` thresholds

### Limitations

1. **Frequency Accuracy:** The synthesized waveform quality depends on the PLL frequency estimate. Significant frequency errors will cause phase drift during long commutation periods.

2. **Magnitude Freeze:** The magnitude is frozen during commutation, so any genuine voltage sags or swells during this period will not be tracked.

3. **Single-Cycle Delay:** The magnitude capture occurs one cycle before commutation starts, introducing a single-cycle delay in tracking voltage changes.

## 7. Testing and Validation

The module includes comprehensive unit tests that verify:

- **Pass-through behavior:** Unchanged output when not commuting
- **Masking transition:** Proper switching to synthetic waveform generation
- **Waveform continuity:** Smooth transition between real and synthetic signals
- **State management:** Correct internal state transitions

Tests validate both the mathematical correctness of the transformations and the behavioral correctness of the state machine.