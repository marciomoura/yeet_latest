# PI Controller

## Overview

The PI controller implements a discrete-time Proportional-Integral controller with advanced features including anti-windup logic and integrator control. It supports both scalar types and coordinate frame types (abc, alphabeta, dq).

## Features

### Basic PI Control
- Proportional gain (Kp) configuration
- Integral gain configuration via continuous-time Ki or integral time constant Ti
- Discrete-time implementation with configurable sampling time

### Anti-Windup Logic
- Back-calculation anti-windup method
- Integrator clamping (conditional integration) anti-windup method
- Configurable anti-windup gain (Kc)
- Prevents integral windup during output saturation
- Works with all supported coordinate frame types

### Integrator Control
- Ability to temporarily disable the integrator
- Useful for transitioning between control modes
- Maintains integral state when disabled

### Output Saturation
- Configurable output limits (min/max)
- Saturation detection and reporting
- Component-wise saturation for coordinate frame types

### Coordinate Frame Support
- Scalar types (double, float)
- Three-phase abc coordinates
- Two-phase alphabeta coordinates  
- Rotating dq coordinates
- Automatic component-wise processing for vector types

## Usage

### Basic Configuration

```cpp
// Scalar PI controller
pi_controller<double> controller;
controller.configure(kp, ki_continuous, sampling_time);

// dq frame PI controller
pi_controller<dq<double>> dq_controller;
dq_controller.configure_with_ti(kp, ti, sampling_time);
```

### Anti-Windup Configuration

```cpp
// Set output limits and anti-windup gain for back-calculation method
controller.set_output_limits(-10.0, 10.0);
controller.set_antiwindup_gain(1.0);  // Typically 1/Ti

// Alternatively, enable integrator clamping method
controller.set_output_limits(-10.0, 10.0);
controller.enable_integrator_clamping(true);
```

### Integrator Control

```cpp
// Disable integrator temporarily
controller.enable_integrator(false);

// Re-enable integrator
controller.enable_integrator(true);
```

### Control Loop

```cpp
double error = setpoint - measurement;
double output = controller.update(error);

// Check for saturation
if (controller.is_output_saturated()) {
    // Handle saturation condition
    auto saturation_error = controller.get_saturation_error();
}
```

## Anti-Windup Algorithms

The controller implements two anti-windup methods:

### Back-Calculation Anti-Windup

1. Compute proportional term: `proportional = Kp * error`
2. Compute unsaturated output: `unsaturated_output = proportional + integral`
3. Apply saturation: `saturated_output = clamp(unsaturated_output, min, max)`
4. Compute saturation error: `sat_error = saturated_output - unsaturated_output`
5. Update integral with anti-windup: `integral += (Ki * error + Kc * sat_error) * Ts`

When saturation occurs, the saturation error modifies the integral term to prevent windup.

### Integrator Clamping (Conditional Integration)

1. Compute proportional term: `proportional = Kp * error`
2. Compute unsaturated output: `unsaturated_output = proportional + integral`
3. Apply saturation: `saturated_output = clamp(unsaturated_output, min, max)`
4. If output is saturated: freeze integral (no update)
5. If output is not saturated: update integral normally: `integral += Ki * error * Ts`

This method simply "freezes" the integrator when the controller output is saturated.

## Coordinate Frame Handling

For coordinate frame types (abc, alphabeta, dq), the controller:
- Applies gains to each component independently
- Performs component-wise saturation
- Detects saturation if any component is saturated
- Computes component-wise saturation error for anti-windup

## Tuning Guidelines

### Anti-Windup Gain (Kc)
- Typical value: `Kc = 1/Ti` where Ti is the integral time constant
- Higher values provide faster anti-windup response
- Lower values provide gentler anti-windup action
- Only applies to back-calculation method

### Anti-Windup Method Selection
- Back-calculation: Use `set_antiwindup_gain()` with appropriate gain
- Integrator clamping: Use `enable_integrator_clamping(true)`
- Both methods require output limits to be set

### Output Limits
- Set based on actuator physical limits
- Consider safety margins
- Ensure limits are symmetric when possible

### Integrator Control
- Disable during initialization or fault conditions
- Re-enable once system is in normal operating state
- Monitor integral value when re-enabling

## API Reference

### Configuration Methods
- `configure(kp, ki_continuous, ts)` - Direct Ki specification
- `configure_with_ti(kp, ti, ts)` - Time constant specification
- `set_output_limits(min, max)` - Output saturation limits
- `set_antiwindup_gain(kc)` - Anti-windup gain for back-calculation method
- `enable_integrator_clamping(enable)` - Enable integrator clamping anti-windup
- `enable_integrator(enable)` - Integrator on/off control
- `enable_output_limits(enable)` - Output limits on/off control

### Control Methods
- `update(error)` - Main control update
- `reset()` - Reset integral and saturation state

### Status Methods
- `is_output_saturated()` - Check saturation status
- `get_saturation_error()` - Get saturation error
- `is_integrator_enabled()` - Check integrator status
- `has_output_limits()` - Check output limits status

### Getter Methods
- `get_kp()`, `get_ki()` - Get controller gains
- `get_sampling_time()`, `get_integral()` - Get timing and state
- `get_antiwindup_gain()` - Get anti-windup gain