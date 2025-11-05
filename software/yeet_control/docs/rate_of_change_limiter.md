# Rate of Change Limiter

## Overview

The `rate_of_change_limiter` (also known as a slew rate limiter) constrains the rate at which a signal can change over time. It ensures smooth transitions by limiting both rising and falling edges to a specified maximum rate, preventing instantaneous jumps that could cause instability in control systems or excessive mechanical stress.

## Mathematical Foundation

The rate limiter implements:

```
Δmax = rate_limit × Ts

if (input - output) > Δmax:
    output = output + Δmax
elif (input - output) < -Δmax:
    output = output - Δmax
else:
    output = input
```

where:
- `rate_limit` is the maximum rate of change in units/second
- `Ts` is the sampling time
- `Δmax` is the maximum allowed change per sample
- `input` is the target value
- `output` is the current rate-limited value

## Features

- **Symmetric Rate Limiting**: Same limit for rising and falling edges
- **Enable/Disable Control**: Can be bypassed for pass-through mode
- **Configurable Rate**: Adjust rate limit during operation
- **Smooth Ramping**: Guarantees smooth transitions
- **Real-Time Safe**: No dynamic allocation, deterministic execution

## Construction and Configuration

### Basic Setup

```cpp
#include "yeet_control/rate_of_change_limiter.hpp"
using namespace yeet::control;

// Construction with sampling time only
double sampling_time = 100e-6;  // 100 µs
rate_of_change_limiter limiter(sampling_time);

// Construction with rate limit, sampling time, and initial value
real_t rate_limit = 1000.0;  // 1000 units/s
real_t initial_value = 0.0;
rate_of_change_limiter configured_limiter(rate_limit, sampling_time, initial_value);
```

### Configure Rate Limit

```cpp
// Set maximum rate of change (units per second)
real_t max_rate = 500.0;  // 500 units/s
limiter.configure(max_rate);

// The actual change per sample is: max_rate × sampling_time
// For 500 units/s and 100 µs: 0.05 units per sample
```

### Enable/Disable

```cpp
// Enable rate limiting (default)
limiter.configure_enable(true);

// Disable (pass-through mode)
limiter.configure_enable(false);

// Check enable state
bool is_active = limiter.is_enabled();
```

## Usage

### Basic Operation

```cpp
// In control loop
real_t target_value = 10.0;
limiter.update(target_value);

// Get rate-limited output
real_t limited_output = limiter.get_output();

// The output will ramp smoothly towards target_value
// at the configured rate
```

### Reset State

```cpp
// Reset to zero
limiter.reset();

// Reset to specific value
real_t initial = 5.0;
limiter.reset(initial);
```

## Practical Applications

### Reference Command Smoothing

```cpp
class SmoothReferenceGenerator {
public:
    SmoothReferenceGenerator(double ts) 
        : _limiter(1000.0, ts, 0.0)  // 1000 units/s ramp rate
    {
    }
    
    real_t generate(real_t commanded_reference) {
        // Smooth step changes in reference
        _limiter.update(commanded_reference);
        return _limiter.get_output();
    }
    
    void set_ramp_rate(real_t rate) {
        _limiter.configure(rate);
    }
    
private:
    rate_of_change_limiter _limiter;
};

// Usage
SmoothReferenceGenerator generator(100e-6);
real_t smooth_ref = generator.generate(100.0);  // Ramps to 100
```

### Motor Speed Acceleration Control

```cpp
class SpeedController {
public:
    SpeedController(double ts) 
        : _acceleration_limiter(100.0, ts, 0.0)  // 100 rad/s² acceleration
    {
    }
    
    real_t limit_acceleration(real_t target_speed) {
        // Limit motor acceleration/deceleration
        _acceleration_limiter.update(target_speed);
        return _acceleration_limiter.get_output();
    }
    
    void set_max_acceleration(real_t max_accel) {
        _acceleration_limiter.configure(max_accel);
    }
    
    void emergency_stop() {
        // Disable limiter for immediate stop
        _acceleration_limiter.configure_enable(false);
        _acceleration_limiter.update(0.0);
        _acceleration_limiter.configure_enable(true);
    }
    
private:
    rate_of_change_limiter _acceleration_limiter;
};
```

### Voltage Ramp Control

```cpp
class VoltageRampController {
public:
    VoltageRampController(double ts) 
        : _voltage_limiter(50.0, ts, 0.0)  // 50 V/s ramp
    {
    }
    
    voltage_pu_t ramp_voltage(voltage_pu_t target) {
        real_t target_value = target.value();
        _voltage_limiter.update(target_value);
        return voltage_pu_t{_voltage_limiter.get_output()};
    }
    
    void soft_start() {
        // Slow ramp during startup
        _voltage_limiter.configure(10.0);  // 10 V/s
    }
    
    void normal_operation() {
        // Faster ramp during normal operation
        _voltage_limiter.configure(50.0);  // 50 V/s
    }
    
private:
    rate_of_change_limiter _voltage_limiter;
};
```

### Current Reference Limiting

```cpp
class CurrentLimiter {
public:
    CurrentLimiter(double ts) 
        : _current_limiter(500.0, ts, 0.0)  // 500 A/s slew rate
    {
    }
    
    current_pu_t limit_current(current_pu_t target_current) {
        real_t target = target_current.value();
        _current_limiter.update(target);
        return current_pu_t{_current_limiter.get_output()};
    }
    
    // Different rates for charging vs discharging
    void set_charge_rate(real_t rate) {
        _current_limiter.configure(rate);
    }
    
private:
    rate_of_change_limiter _current_limiter;
};
```

### Torque Reference Smoothing

```cpp
class TorqueController {
public:
    TorqueController(double ts) 
        : _torque_limiter(200.0, ts, 0.0)  // 200 Nm/s
    {
    }
    
    real_t smooth_torque(real_t commanded_torque) {
        _torque_limiter.update(commanded_torque);
        return _torque_limiter.get_output();
    }
    
    void set_jerk_limit(real_t max_torque_rate) {
        // Limit jerk (rate of torque change)
        _torque_limiter.configure(max_torque_rate);
    }
    
    void reset_to_current(real_t current_torque) {
        // Bumpless transfer when switching control modes
        _torque_limiter.reset(current_torque);
    }
    
private:
    rate_of_change_limiter _torque_limiter;
};
```

## Design Guidelines

### Choosing Rate Limit

The rate limit should balance responsiveness and smoothness:

**High Rate Limits (Fast)**
- **Pros**: Quick response to commands, minimal lag
- **Cons**: May cause mechanical stress, electrical transients
- **Use**: Lightweight systems, fast actuators, measurement filtering

**Low Rate Limits (Slow)**
- **Pros**: Smooth operation, reduced stress, stable control
- **Cons**: Slower response, increased settling time
- **Use**: Heavy mechanical systems, safety-critical applications

**Calculation Example:**

```cpp
// Target: Ramp from 0 to 100 units in 1 second
real_t target = 100.0;
real_t ramp_time = 1.0;  // seconds
real_t required_rate = target / ramp_time;  // 100 units/s

rate_of_change_limiter limiter(required_rate, sampling_time, 0.0);
```

### Ramp Time Calculation

Given a rate limit and target change:

```
ramp_time = Δvalue / rate_limit
```

```cpp
// Calculate time to reach target
real_t current = limiter.get_output();
real_t target = 50.0;
real_t rate = 100.0;  // units/s

real_t delta = std::abs(target - current);
real_t time_to_target = delta / rate;  // seconds
```

### Combining with Other Limiters

```cpp
// Cascade rate limiter with amplitude limiter
class SafeController {
public:
    SafeController(double ts) 
        : _rate_limiter(100.0, ts, 0.0)
    {
    }
    
    real_t safe_output(real_t command) {
        // First limit rate of change
        _rate_limiter.update(command);
        real_t rate_limited = _rate_limiter.get_output();
        
        // Then apply amplitude limits
        return std::clamp(rate_limited, -50.0, 50.0);
    }
    
private:
    rate_of_change_limiter _rate_limiter;
};
```

## Advanced Features

### Asymmetric Rate Limiting

For different rising and falling rates, use two limiters:

```cpp
class AsymmetricRateLimiter {
public:
    AsymmetricRateLimiter(double ts) 
        : _rising_limiter(500.0, ts, 0.0),
          _falling_limiter(1000.0, ts, 0.0)
    {
    }
    
    real_t update(real_t input) {
        real_t current = _rising_limiter.get_output();
        
        if (input > current) {
            // Use slow rate for rising
            _rising_limiter.update(input);
            real_t output = _rising_limiter.get_output();
            _falling_limiter.reset(output);
            return output;
        } else {
            // Use fast rate for falling
            _falling_limiter.update(input);
            real_t output = _falling_limiter.get_output();
            _rising_limiter.reset(output);
            return output;
        }
    }
    
private:
    rate_of_change_limiter _rising_limiter;
    rate_of_change_limiter _falling_limiter;
};
```

### Adaptive Rate Limiting

Adjust rate based on operating conditions:

```cpp
class AdaptiveRateLimiter {
public:
    AdaptiveRateLimiter(double ts) 
        : _limiter(100.0, ts, 0.0)
    {
    }
    
    real_t update(real_t input, real_t system_state) {
        // Slow down near limits
        real_t base_rate = 100.0;
        real_t adaptive_rate = base_rate * (1.0 - std::abs(system_state) / 100.0);
        adaptive_rate = std::max(adaptive_rate, 10.0);  // Minimum rate
        
        _limiter.configure(adaptive_rate);
        _limiter.update(input);
        return _limiter.get_output();
    }
    
private:
    rate_of_change_limiter _limiter;
};
```

## Performance Characteristics

### Computational Cost

Per `update()` call:
- 1 subtraction (error calculation)
- 2 comparisons (limit checking)
- 1 addition (output update)
- **Total**: ~4 operations (O(1) complexity)

### Memory Footprint

- Sampling time: 8 bytes (double)
- Max change per sample: 4 bytes (real_t)
- Current output: 4 bytes (real_t)
- Enable flag: 1 byte (bool)
- **Total**: ~17 bytes

### Execution Time

Typical execution times:
- ARM Cortex-M4 @ 168 MHz: ~15 ns
- ARM Cortex-M7 @ 400 MHz: ~8 ns
- x86-64 @ 3.0 GHz: ~3 ns

## Debugging and Validation

### Validate Configuration

```cpp
// Check rate limit is properly configured
assert(limiter.is_enabled());

// Verify output never exceeds rate
real_t prev = limiter.get_output();
limiter.update(1000.0);  // Large step
real_t curr = limiter.get_output();
real_t actual_rate = (curr - prev) / sampling_time;
assert(actual_rate <= configured_rate * 1.01);  // Small tolerance
```

### Unit Test Example

```cpp
void test_rate_limiter_ramp() {
    double ts = 0.01;  // 10 ms
    real_t rate = 10.0;  // 10 units/s
    rate_of_change_limiter limiter(rate, ts, 0.0);
    
    // Step input
    real_t target = 1.0;
    for (int i = 0; i < 100; ++i) {
        limiter.update(target);
    }
    
    // Should reach target in 0.1 seconds (10 samples)
    real_t output = limiter.get_output();
    assert(std::abs(output - target) < 0.01);
}
```

### Visualize Ramp Response

```cpp
// Generate step response data
std::vector<double> time, input, output;
rate_of_change_limiter limiter(100.0, 0.001, 0.0);

for (double t = 0.0; t < 1.0; t += 0.001) {
    double step = (t < 0.5) ? 0.0 : 50.0;
    limiter.update(step);
    
    time.push_back(t);
    input.push_back(step);
    output.push_back(limiter.get_output());
}
// Plot to verify smooth ramp
```

## Common Pitfalls

1. **Rate Too Low**: System responds too slowly to commands
2. **Rate Too High**: Defeats the purpose of limiting
3. **Wrong Units**: Rate must match signal units (e.g., rad/s for angles)
4. **Forgetting to Reset**: Initial state may not match actual system state
5. **Sampling Time Mismatch**: Using wrong Ts leads to incorrect rate

## Comparison with Other Smoothing Methods

| Method | Response Type | Overshoot | Phase Lag | Use Case |
|--------|---------------|-----------|-----------|----------|
| Rate Limiter | Linear ramp | None | Proportional to rate | Reference commands |
| Low-Pass Filter | Exponential | None | Yes | Noise rejection |
| Moving Average | Delayed step | None | Yes | Signal smoothing |
| Median Filter | Step-like | None | Minimal | Spike removal |

## Integration with Control Systems

### With PI Controller

```cpp
// Limit reference to PI controller
rate_of_change_limiter ref_limiter(100.0, dt, 0.0);
pi_controller<double> controller;

// In control loop
real_t raw_reference = get_command();
ref_limiter.update(raw_reference);
real_t smooth_reference = ref_limiter.get_output();

double error = smooth_reference - measurement;
double control_output = controller.update(error);
```

### With Feed-Forward Control

```cpp
// Rate limiter in feed-forward path
rate_of_change_limiter ff_limiter(50.0, dt, 0.0);

// Feed-forward term based on reference derivative
real_t reference = get_reference();
ff_limiter.update(reference);
real_t ref_rate = (ff_limiter.get_output() - prev_ref) / dt;
real_t feedforward = Kff * ref_rate;
prev_ref = ff_limiter.get_output();
```

## See Also

- **hysteresis_limiter** - Amplitude limiting with hysteresis (component available in `yeet_control/hysteresis_limiter.hpp`)
- [first_order_low_pass_filter.md](first_order_low_pass_filter.md) - Alternative smoothing method
- [integrator.md](integrator.md) - Integration of rate-limited signals
- [pi_controller.md](pi_controller.md) - Control with rate-limited references
