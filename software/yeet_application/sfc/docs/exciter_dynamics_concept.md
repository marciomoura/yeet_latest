# Realistic Exciter Dynamics for Synchronous Machines

## Overview

The current "perfect exciter" implementation provides instantaneous voltage tracking, which is unrealistic. Real exciters have dynamics dominated by the field winding time constant. This document describes how to reuse the existing `speed_voltage_relationship` component to model realistic exciter behavior.

## Physical Basis

### Dominant Time Constants

For a synchronous machine excitation system, the dominant dynamics are:

1. **Field Winding Time Constant (τ_d0')** - Dominant, typically 0.5 - 8.0 seconds
   - Represents the L/R time constant of the field winding
   - Varies significantly with machine size and design
   - Limits the rate of flux change in the rotor

2. **Thyristor Bridge Dynamics** - Very fast, typically < 0.010 seconds
   - Switching frequency limited by line frequency (50/60 Hz)
   - Usually neglected in electromechanical simulations

3. **Current Controller Bandwidth** - Fast, typically 0.010 - 0.050 seconds
   - Exciter's own field current controller
   - Often combined with field winding dynamics

4. **Communication Delays** - Application dependent, typically 0.001 - 0.010 seconds
   - Digital communication between controllers
   - Usually negligible compared to field winding

### Transfer Function Model

The exciter can be approximated as a first-order system:

```
         K_exc
H(s) = --------
       1 + τ_exc·s
```

Where:
- `K_exc` = Steady-state gain (typically 1.0 for per-unit systems)
- `τ_exc` = Equivalent exciter time constant [seconds]

For a field-winding dominated system:
```
τ_exc ≈ τ_d0' = L_field / R_field
```

## Typical Parameters by Machine Type

### Small Synchronous Machines (100 kW - 1 MW)

| Parameter                                | Typical Value | Range       | Notes                                    |
| ---------------------------------------- | ------------- | ----------- | ---------------------------------------- |
| Rated Power                              | 100 kW - 1 MW | -           | Auxiliary power, small industrial        |
| Field Winding Time Constant (τ_d0')      | 0.5 - 2.0 s   | 0.3 - 3.0 s | Smaller field windings, lower inductance |
| Equivalent Exciter Time Constant (τ_exc) | 0.3 - 1.5 s   | 0.2 - 2.0 s | Includes some controller dynamics        |
| Field Voltage Response                   | Fast          | -           | Quick flux buildup                       |

**Recommended Configuration:**
```cpp
// Small machine (e.g., 500 kW synchronous motor)
plant.configure_speed_voltage_relationship(
    1.0,    // coefficient (unity gain for external control)
    0.0,    // offset (no bias)
    0.8     // time constant [s] - typical for small machines
);
```

### Medium Synchronous Machines (1 MW - 10 MW)

| Parameter                                | Typical Value   | Range           | Notes                               |
| ---------------------------------------- | --------------- | --------------- | ----------------------------------- |
| Rated Power                              | 1 MW - 10 MW    | -               | Industrial drives, small generators |
| Field Winding Time Constant (τ_d0')      | 1.5 - 4.0 s     | 1.0 - 5.0 s     | Medium field windings               |
| Equivalent Exciter Time Constant (τ_exc) | 1.0 - 3.0 s     | 0.8 - 4.0 s     | Field winding dominant              |
| Field Voltage Response                   | Moderate        | -               | Noticeable delay in flux response   |
| **Typical for LCI Gas Turbine Starters** | **2.0 - 3.0 s** | **1.5 - 4.0 s** | **This is your application range!** |

**Recommended Configuration (LCI Application):**
```cpp
// Medium machine (e.g., 2.95 MW synchronous motor for gas turbine starting)
plant.configure_speed_voltage_relationship(
    1.0,    // coefficient (unity gain for external control)
    0.0,    // offset (no bias)
    2.5     // time constant [s] - typical for LCI applications
);
```

### Large Synchronous Machines (10 MW - 100+ MW)

| Parameter                                | Typical Value   | Range        | Notes                                 |
| ---------------------------------------- | --------------- | ------------ | ------------------------------------- |
| Rated Power                              | 10 MW - 100+ MW | -            | Large generators, motor drives        |
| Field Winding Time Constant (τ_d0')      | 3.0 - 8.0 s     | 2.0 - 12.0 s | Large field windings, high inductance |
| Equivalent Exciter Time Constant (τ_exc) | 2.0 - 6.0 s     | 1.5 - 8.0 s  | Very slow flux response               |
| Field Voltage Response                   | Slow            | -            | Significant delay, impacts control    |

**Recommended Configuration:**
```cpp
// Large machine (e.g., 50 MW synchronous motor)
plant.configure_speed_voltage_relationship(
    1.0,    // coefficient (unity gain for external control)
    0.0,    // offset (no bias)
    5.0     // time constant [s] - typical for large machines
);
```

### Salient Pole vs. Round Rotor Machines

| Machine Type                     | Typical τ_d0' | Application                                     | Notes                                                |
| -------------------------------- | ------------- | ----------------------------------------------- | ---------------------------------------------------- |
| **Salient Pole**                 | 1.5 - 5.0 s   | Hydro generators, low-speed motors              | Concentrated field windings, moderate time constants |
| **Round Rotor (Turbogenerator)** | 4.0 - 10.0 s  | Thermal/nuclear generators, high-speed machines | Distributed field windings, longer time constants    |
| **Wound Rotor Motor (LCI)**      | 2.0 - 4.0 s   | Variable-speed drives, gas turbine starting     | Medium time constants, typical for your application  |

## Implementation Strategy

### Option 1: Reuse speed_voltage_relationship (Recommended)

**Advantages:**
- ✅ No new code required
- ✅ Already has first-order dynamics support
- ✅ Configurable time constant
- ✅ Well-tested component

**How it works:**

The `speed_voltage_relationship` component already supports dynamic response via its time constant parameter. In `external_control` mode, we can configure it to:

1. **Input**: External excitation voltage setpoint (instead of speed)
2. **Output**: Filtered voltage (with first-order dynamics)
3. **Transfer Function**: Acts as low-pass filter with τ_exc

**Current Usage (Perfect Exciter):**
```cpp
// In resolve_machine_voltage() for external_control mode:
return voltage_t(_external_excitation_voltage_setpoint_pu.value() * _nominal_machine_voltage_rms.value());
// ❌ Instantaneous - no dynamics!
```

**Modified Usage (Realistic Exciter):**
```cpp
// Configure speed_voltage_relationship to act as exciter filter
// Instead of using speed as input, use external voltage setpoint
void step_voltage_control() {
    if (_voltage_mode == external_control) {
        // Feed external setpoint to speed_voltage_relationship as "reference"
        // The component's first-order dynamics will filter it
        _speed_voltage_relationship.set_speed(_external_excitation_voltage_setpoint_pu.value());
        _speed_voltage_relationship.update();
        // Output has realistic dynamics based on configured time constant
    }
    else if (_voltage_mode == speed_dependent) {
        // Normal V/Hz control using actual machine speed
        _speed_voltage_relationship.set_speed(_mechanical_load.get_speed_pu().value());
        _speed_voltage_relationship.update();
    }
}

voltage_t resolve_machine_voltage() const noexcept {
    if (_voltage_mode == external_control || _voltage_mode == speed_dependent) {
        // Both modes now use filtered output from speed_voltage_relationship
        return voltage_t(_speed_voltage_relationship.get_voltage_pu() * _nominal_machine_voltage_rms.value());
    }
    else if (_voltage_mode == fixed_voltage) {
        return voltage_t(_machine_voltage_pu.value() * _nominal_machine_voltage_rms.value());
    }
    return _nominal_machine_voltage_rms;
}
```

### Option 2: Add Dedicated Exciter Model (Future Enhancement)

For more complex exciter modeling (IEEE standards: AC1A, ST1A, etc.), create a dedicated component:

```cpp
class exciter_model {
public:
    void configure_parameters(real_t time_constant, real_t gain, real_t saturation_limit);
    void set_voltage_reference(voltage_pu_t v_ref);
    void update(duration_t dt);
    voltage_pu_t get_field_voltage() const;
private:
    real_t _tau;
    voltage_pu_t _v_field_filtered;
    // Could add: saturation, rate limits, ceiling voltage
};
```

**When to use:**
- IEEE standard exciter models required
- Modeling saturation effects
- Modeling exciter ceiling voltage
- Modeling under-excitation limits

## Configuration Examples

### Example 1: Conservative (Slow Exciter)

Useful for worst-case stability analysis:

```cpp
// Configure for slow exciter response (conservative design validation)
plant.configure_speed_voltage_relationship(
    1.0,    // Unity gain (no V/Hz slope in external mode)
    0.0,    // No offset
    4.0     // 4-second time constant (slow)
);

// Set to external control mode
plant.set_voltage_control_mode(plant_model_voltage_control_mode::external_control);
```

**Result:** Voltage buildup takes approximately 3-4 time constants = 12-16 seconds to reach 95% of setpoint.

### Example 2: Typical LCI Application

Realistic parameters for 3 MW synchronous motor:

```cpp
// Configure for typical LCI synchronous motor exciter
plant.configure_speed_voltage_relationship(
    1.0,    // Unity gain
    0.0,    // No offset
    2.5     // 2.5-second time constant (typical for medium machines)
);

plant.set_voltage_control_mode(plant_model_voltage_control_mode::external_control);
```

**Result:** Voltage buildup takes approximately 7.5-10 seconds to reach 95% of setpoint.

### Example 3: Fast Exciter (Best Case)

Small machine or modern static exciter:

```cpp
// Configure for fast static exciter response
plant.configure_speed_voltage_relationship(
    1.0,    // Unity gain
    0.0,    // No offset
    0.5     // 0.5-second time constant (fast)
);

plant.set_voltage_control_mode(plant_model_voltage_control_mode::external_control);
```

**Result:** Voltage buildup takes approximately 1.5-2 seconds to reach 95% of setpoint.

### Example 4: Perfect Exciter (Current Behavior)

Zero time constant for algorithm validation:

```cpp
// Configure for instantaneous response (perfect exciter)
plant.configure_speed_voltage_relationship(
    1.0,    // Unity gain
    0.0,    // No offset
    0.0     // Zero time constant (instantaneous)
);

plant.set_voltage_control_mode(plant_model_voltage_control_mode::external_control);
```

**Result:** Voltage tracks setpoint instantaneously (algebraic, no dynamics).

## Impact on Control Performance

### Pre-Fluxing Phase

**With Perfect Exciter (τ_exc = 0 s):**
- Voltage reaches 0.3 pu instantly
- Field establishes in ~2 seconds (pre-fluxing duration)
- No voltage lag

**With Realistic Exciter (τ_exc = 2.5 s):**
- Voltage ramps to 0.3 pu with exponential response
- Reaches 95% of 0.3 pu in ~7.5 seconds
- Pre-fluxing duration must be increased to account for dynamics
- Recommended: Set pre-fluxing to 3-4 time constants (7.5-10 seconds)

### Acceleration Phase

**Impact on Speed Control Loop:**
- Slower voltage response affects machine torque
- Speed controller must account for exciter dynamics
- May require detuning of speed controller gains
- Exciter dynamics can introduce additional phase lag

**V/Hz Tracking:**
- Voltage lags behind speed during acceleration
- Controller must lead the reference to compensate
- Faster ramps require more aggressive compensation

### Stability Considerations

**Positive Effects:**
- Filters high-frequency noise in voltage setpoint
- Reduces voltage overshoot
- Smoother flux transitions

**Negative Effects:**
- Introduces phase lag in excitation loop
- Can limit bandwidth of outer control loops
- May cause instability if not properly accounted for

## Testing Recommendations

### Test 1: Step Response

Verify exciter dynamics by applying a voltage step:

```cpp
// Apply step from 0.5 pu to 1.0 pu
plant.set_excitation_voltage_setpoint_pu(voltage_pu_t{0.5});
simulator.simulate_for(10.0);  // Settle at 0.5 pu

plant.set_excitation_voltage_setpoint_pu(voltage_pu_t{1.0});
simulator.simulate_for(15.0);  // Observe rise time

// Expected behavior with τ_exc = 2.5 s:
// - Rise time (10% to 90%): ~5.5 seconds
// - Settling time (to 98%): ~10 seconds
```

### Test 2: Ramp Response

Verify tracking during acceleration:

```cpp
// Slow ramp: 0 to 1.0 pu over 50 seconds
// With τ_exc = 2.5 s, voltage will lag speed by approximately τ_exc
// Steady-state lag = 0.05 pu (for 0.02 pu/s ramp rate)
```

### Test 3: Pre-Fluxing Duration

Determine required pre-fluxing time:

```cpp
// For τ_exc = 2.5 s:
// - 1 time constant: 63% of final value
// - 2 time constants: 86% of final value
// - 3 time constants: 95% of final value (typical target)
// - 4 time constants: 98% of final value (conservative)

// Recommended pre-fluxing duration = 3-4 × τ_exc
// For τ_exc = 2.5 s: 7.5 - 10 seconds
```

## Recommended Values for Your Application

Based on the 2.95 MW LCI system for gas turbine starting:

### Conservative Design (Robust Startup)
```cpp
plant.configure_speed_voltage_relationship(1.0, 0.0, 3.0);  // τ_exc = 3.0 s
excitation_controller.get_parameters().prefluxing_duration = duration_t{10.0};  // 3.3 time constants
```

### Typical Design (Balanced)
```cpp
plant.configure_speed_voltage_relationship(1.0, 0.0, 2.5);  // τ_exc = 2.5 s
excitation_controller.get_parameters().prefluxing_duration = duration_t{8.0};  // 3.2 time constants
```

### Aggressive Design (Fast Startup)
```cpp
plant.configure_speed_voltage_relationship(1.0, 0.0, 1.5);  // τ_exc = 1.5 s
excitation_controller.get_parameters().prefluxing_duration = duration_t{5.0};  // 3.3 time constants
```

### Development/Testing (Perfect Exciter)
```cpp
plant.configure_speed_voltage_relationship(1.0, 0.0, 0.0);  // τ_exc = 0 s (instantaneous)
excitation_controller.get_parameters().prefluxing_duration = duration_t{2.0};  // Original value
```

## References

1. IEEE Std 421.5-2016: "IEEE Recommended Practice for Excitation System Models for Power System Stability Studies"
2. Kundur, P., "Power System Stability and Control", McGraw-Hill, 1994
3. IEEE Std 1110-2002: "IEEE Guide for Synchronous Generator Modeling Practices and Applications in Power System Stability Analyses"

## Implementation Notes

### Backward Compatibility

The proposed changes maintain backward compatibility:
- Default time constant = 0.0 (perfect exciter, current behavior)
- Users can enable dynamics by setting time constant > 0
- No changes required to existing tests unless dynamics are desired

### Future Enhancements

1. Add exciter saturation effects
2. Implement IEEE standard models (AC1A, ST1A, etc.)
3. Add under-excitation limiter
4. Model exciter ceiling voltage
5. Add field current measurement and feedback
