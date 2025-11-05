# Integrator

## Overview

The `integrator` class implements a discrete-time numerical integrator using the rectangular (forward Euler) integration method. It is designed for real-time control applications where continuous integration of signals is required, such as in PI controllers, phase-locked loops, and position estimation.

## Mathematical Foundation

### Continuous-Time Integration

In continuous time, integration is defined as:

```
y(t) = y(t₀) + ∫[t₀ to t] x(τ) dτ
```

### Discrete-Time Approximation

The rectangular (forward Euler) method approximates the integral using:

```
y[k] = y[k-1] + x[k] × Ts
```

where:
- `y[k]` is the current integrator output (accumulated value)
- `y[k-1]` is the previous integrator output
- `x[k]` is the current input sample
- `Ts` is the sampling time (integration step size)

This is the simplest and most computationally efficient numerical integration method, suitable for real-time applications with sufficiently small time steps.

## Features

- **Fixed Sampling Time**: Configured once for consistent integration
- **Template-Based**: Works with scalars and coordinate frame types
- **Resettable State**: Initialize or reset to any value
- **Implicit Conversion**: Can be used directly as its output type
- **Zero Overhead**: Minimal computational cost per update

## Construction and Configuration

### Basic Setup

```cpp
#include "yeet_control/integrator.hpp"
using namespace yeet::control;

// Construction with sampling time (in seconds)
double sampling_time = 100e-6;  // 100 µs
integrator<double> angle_integrator(sampling_time);

// Or default construct and configure later
integrator<double> position_integrator;
position_integrator.configure(sampling_time);
```

### Resetting State

```cpp
// Reset to zero (default)
angle_integrator.reset();

// Reset to specific value
double initial_position = 1.5;
position_integrator.reset(initial_position);
```

## Usage

### Basic Integration

```cpp
// Integrate angular velocity to get angle
double angular_velocity = 100.0;  // rad/s
double angle = angle_integrator.update(angular_velocity);

// After Ts seconds, angle increases by (angular_velocity × Ts)
```

### Accessing Output

```cpp
// Update and get result
double result = angle_integrator.update(input_value);

// Or update and access separately
angle_integrator.update(input_value);
double output = angle_integrator.get_output();

// Implicit conversion
double value = angle_integrator;  // Calls operator T()
```

### Coordinate Frame Integration

The integrator works with coordinate frame types:

```cpp
#include "yeet_control/reference_frame.hpp"

// Integrate dq currents
integrator<dq<current_pu_t>> flux_integrator(100e-6);

// Input: voltage in dq frame
dq<voltage_pu_t> voltage_dq{voltage_pu_t{1.0}, voltage_pu_t{0.5}};

// Output: flux linkage in dq frame
dq<current_pu_t> flux_dq = flux_integrator.update(voltage_dq);
```

## Practical Applications

### Angular Position from Velocity

```cpp
class PositionEstimator {
public:
    PositionEstimator(double sampling_time) 
        : _position_integrator(sampling_time)
    {
        _position_integrator.reset(0.0);
    }
    
    angle_wrapped estimate_position(double angular_velocity) {
        // Integrate velocity to get position
        double position_rad = _position_integrator.update(angular_velocity);
        return angle_wrapped::from_radians(angle_t{position_rad});
    }
    
    void reset_position(double initial_angle) {
        _position_integrator.reset(initial_angle);
    }
    
private:
    integrator<double> _position_integrator;
};
```

### PI Controller Integral Term

```cpp
class PIController {
public:
    PIController(double kp, double ki, double ts) 
        : _kp(kp), _ki(ki), _integral(ts)
    {
    }
    
    double update(double error) {
        // Proportional term
        double p_term = _kp * error;
        
        // Integral term (integrate error with Ki gain)
        double i_term = _integral.update(_ki * error);
        
        return p_term + i_term;
    }
    
    void reset() {
        _integral.reset();
    }
    
private:
    double _kp, _ki;
    integrator<double> _integral;
};
```

### Phase-Locked Loop

```cpp
class SimplePLL {
public:
    SimplePLL(double sampling_time) 
        : _angle_integrator(sampling_time),
          _ts(sampling_time)
    {
    }
    
    void update(double phase_error) {
        // PI controller on phase error
        double freq_estimate = _nominal_freq + 
                               _kp * phase_error + 
                               _ki_integral.update(phase_error);
        
        // Integrate frequency to get angle
        double angle = _angle_integrator.update(freq_estimate);
        _estimated_angle = angle_wrapped::from_radians(angle_t{angle});
    }
    
    angle_wrapped get_angle() const { return _estimated_angle; }
    
private:
    integrator<double> _angle_integrator;
    integrator<double> _ki_integral;
    angle_wrapped _estimated_angle{0.0};
    double _nominal_freq{2.0 * pi * 50.0};  // 50 Hz
    double _kp{100.0}, _ki{10000.0};
    double _ts;
};
```

### Velocity Integration to Position

```cpp
class VelocityController {
public:
    VelocityController(double ts) 
        : _position_integrator(ts)
    {
    }
    
    double get_position(double measured_velocity) {
        // Integrate velocity to track position
        return _position_integrator.update(measured_velocity);
    }
    
    void calibrate_position(double actual_position) {
        // Reset integrator to calibrated position
        _position_integrator.reset(actual_position);
    }
    
private:
    integrator<double> _position_integrator;
};
```

### Flux Estimation in Motor Control

```cpp
class FluxEstimator {
public:
    FluxEstimator(double ts) 
        : _flux_integrator(ts)
    {
    }
    
    dq<current_pu_t> estimate_flux(const dq<voltage_pu_t>& voltage, 
                                     const dq<current_pu_t>& current) {
        // Back-EMF = Voltage - R×Current
        double R = 0.01;  // Resistance
        dq<voltage_pu_t> back_emf{
            voltage.d() - R * current.d(),
            voltage.q() - R * current.q()
        };
        
        // Integrate back-EMF to get flux linkage
        return _flux_integrator.update(back_emf);
    }
    
private:
    integrator<dq<current_pu_t>> _flux_integrator;
};
```

## Accuracy Considerations

### Integration Error

The rectangular (forward Euler) method has first-order accuracy:

```
Global error ∝ O(Ts)
```

**Implications:**
- Halving the sampling time (Ts) approximately halves the integration error
- For accurate integration, use small time steps relative to signal dynamics
- Error accumulates over time for long-duration integrations

### Improving Accuracy

For applications requiring higher accuracy, consider:

1. **Smaller Sampling Time**: Reduce Ts for better approximation
2. **Trapezoidal Method**: Average of current and previous input
3. **Higher-Order Methods**: Runge-Kutta methods (available in `range_kutta_derivative.hpp`)

Example with trapezoidal method (manual implementation):

```cpp
// Store previous input
double prev_input = 0.0;

// Trapezoidal integration
double output = integrator.update((current_input + prev_input) * 0.5);
prev_input = current_input;
```

### Dealing with Integration Drift

Long-term integration can accumulate errors:

```cpp
// Periodic re-calibration
if (calibration_signal_available) {
    double actual_value = get_calibrated_measurement();
    integrator.reset(actual_value);
}

// Or use complementary filtering
double integrated = integrator.update(velocity);
double hybrid = 0.99 * integrated + 0.01 * direct_measurement;
integrator.reset(hybrid);  // Correct drift
```

## Design Guidelines

### Choosing Sampling Time

The sampling time should be:

1. **Small relative to signal dynamics**: Typically Ts < 1/10 of the fastest time constant
2. **Consistent with control loop rate**: Match the control update frequency
3. **Considered for numerical stability**: Smaller is more accurate but may increase noise sensitivity

### Anti-Windup Protection

When using integrators in control loops, implement anti-windup to prevent saturation:

```cpp
class PIControllerWithAntiWindup {
public:
    PIControllerWithAntiWindup(double kp, double ki, double ts) 
        : _kp(kp), _ki(ki), _integral(ts)
    {
    }
    
    double update(double error, double min_limit, double max_limit) {
        double p_term = _kp * error;
        double i_term = _integral.get_output();
        double output_raw = p_term + i_term;
        
        // Apply saturation
        double output = std::clamp(output_raw, min_limit, max_limit);
        
        // Only integrate if not saturated (anti-windup)
        if (output == output_raw) {
            _integral.update(_ki * error);
        }
        
        return output;
    }
    
private:
    double _kp, _ki;
    integrator<double> _integral;
};
```

### Initial Conditions

Set appropriate initial conditions to avoid transients:

```cpp
// Bad: Starting from zero may cause large initial error
integrator<double> bad_integrator(dt);
bad_integrator.reset(0.0);

// Good: Initialize to steady-state value
integrator<double> good_integrator(dt);
double steady_state_value = 1.0;
good_integrator.reset(steady_state_value);
```

## Performance Characteristics

### Computational Cost

Per `update()` call:
- 1 multiplication (input × Ts)
- 1 addition (accumulation)
- **Total**: 2 floating-point operations

### Memory Footprint

- Sampling time: 1 double (8 bytes)
- Accumulated value: sizeof(T) (e.g., 8 bytes for double)
- **Total**: ~16 bytes for scalar types

### Execution Time

Typical execution times:
- ARM Cortex-M4 @ 168 MHz: ~10 ns
- ARM Cortex-M7 @ 400 MHz: ~5 ns
- x86-64 @ 3.0 GHz: ~2 ns

## Comparison with Other Integration Methods

| Method | Order | Error | Cost | Use Case |
|--------|-------|-------|------|----------|
| Rectangular (Forward Euler) | O(Ts) | Higher | Lowest | Fast control loops |
| Trapezoidal | O(Ts²) | Medium | Low | General purpose |
| Simpson's Rule | O(Ts⁴) | Lower | Medium | High accuracy |
| Runge-Kutta | O(Ts⁴) | Lowest | Highest | Simulation |

## Debugging and Validation

### Validate Configuration

```cpp
// Ensure sampling time is positive
assert(integrator.get_sampling_time() > 0.0);

// Check initial state
integrator.reset(1.0);
assert(integrator.get_output() == 1.0);
```

### Unit Test Example

```cpp
void test_integrator_constant_input() {
    double ts = 0.01;  // 10 ms
    integrator<double> integ(ts);
    integ.reset(0.0);
    
    // Integrate constant input
    double input = 10.0;
    for (int i = 0; i < 100; ++i) {
        integ.update(input);
    }
    
    // Expected: 10.0 × 0.01 × 100 = 10.0
    double output = integ.get_output();
    assert(std::abs(output - 10.0) < 0.001);
}
```

### Visualize Integration

```cpp
// Generate data for plotting
std::vector<double> time, input, output;
integrator<double> integ(0.001);

for (double t = 0.0; t < 1.0; t += 0.001) {
    double input_value = std::sin(2 * pi * t);  // 1 Hz sine
    double output_value = integ.update(input_value);
    
    time.push_back(t);
    input.push_back(input_value);
    output.push_back(output_value);
}
// Plot to verify integration of sin(t) gives -cos(t)
```

## Common Pitfalls

1. **Forgetting to Reset**: Integrators accumulate indefinitely; reset when needed
2. **Wrong Sampling Time**: Using incorrect Ts leads to scaling errors
3. **No Anti-Windup**: Integrators in control loops can wind up during saturation
4. **Overflow**: Long-term integration with large inputs can overflow
5. **Noise Amplification**: Integration amplifies low-frequency noise and DC offsets

## Thread Safety

The integrator is **not thread-safe**. If accessed from multiple threads:

```cpp
// Protect with mutex
std::mutex integrator_mutex;
integrator<double> shared_integrator(dt);

// In thread
{
    std::lock_guard<std::mutex> lock(integrator_mutex);
    result = shared_integrator.update(input);
}
```

## See Also

- [pi_controller.md](pi_controller.md) - Uses integrator for integral term
- **derivative** - Numerical differentiation (component available in `yeet_control/derivative.hpp`)
- [first_order_low_pass_filter.md](first_order_low_pass_filter.md) - Filtering integrated signals
- [three_phase_pll.md](three_phase_pll.md) - PLL using frequency integration
