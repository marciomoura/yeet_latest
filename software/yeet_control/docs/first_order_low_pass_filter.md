# First-Order Low-Pass Filter

## Overview

The `first_order_low_pass_filter` implements a discrete-time first-order low-pass filter using the bilinear transform (Tustin's method) with frequency pre-warping. This filter is optimized for real-time embedded applications with features including numerical stability, frequency response analysis, and settling time calculations.

## Mathematical Foundation

### Continuous-Time Transfer Function

The continuous-time transfer function of a first-order low-pass filter is:

```
H(s) = ωc / (s + ωc)
```

where `ωc` is the cutoff frequency in radians/second.

At the cutoff frequency, the magnitude response is -3 dB (approximately 0.707 or 1/√2 of the DC gain).

### Discrete-Time Implementation

The bilinear transform (Tustin's method) converts the continuous transfer function to discrete-time:

```
s = (2/Ts) × (z - 1)/(z + 1)
```

This yields the discrete-time transfer function:

```
H(z) = (b₀z + b₁) / (z + a₁)
```

### Difference Equation

The filter implements the following recursive equation:

```
y[n] = -a₁ × y[n-1] + b₀ × x[n] + b₁ × x[n-1]
```

where:
- `y[n]` is the current output
- `y[n-1]` is the previous output
- `x[n]` is the current input
- `x[n-1]` is the previous input
- `a₁`, `b₀`, `b₁` are the filter coefficients

### Frequency Pre-Warping

To compensate for frequency warping inherent in the bilinear transform, the cutoff frequency is pre-warped:

```
ωw = (2/Ts) × tan(ωc × Ts/2)
```

This ensures the discrete filter matches the continuous filter's response at the cutoff frequency.

## Features

- **Bilinear Transform with Pre-Warping**: Accurate frequency response matching
- **Double-Precision Internal Arithmetic**: Enhanced numerical stability
- **Template-Based Design**: Works with scalars and coordinate frame types
- **Frequency Response Analysis**: Built-in magnitude and phase response calculation
- **Settling Time Calculation**: Helper methods for system design
- **Real-Time Safe**: No dynamic allocation, deterministic execution

## Construction and Configuration

### Basic Setup

```cpp
#include "yeet_control/first_order_low_pass_filter.hpp"
using namespace yeet::control;

// Construction with sampling period (in seconds)
double sampling_period = 100e-6;  // 100 µs
first_order_low_pass_filter<double> filter(sampling_period);

// Configure cutoff frequency (in Hz)
double cutoff_frequency = 10.0;  // 10 Hz
filter.configure(cutoff_frequency);
```

### Configuration from Time Constant

The relationship between cutoff frequency and time constant is:

```
fc = 1/(2πτ)
```

where `τ` is the time constant in seconds.

```cpp
// Calculate cutoff from time constant
double time_constant = 0.01;  // 10 ms
double cutoff = first_order_low_pass_filter<double>::calculate_cutoff_frequency(time_constant);
filter.configure(cutoff);
```

### Settling Time Calculation

Calculate time required to reach a specified percentage of the final value:

```cpp
// Time to reach 95% of final value
double settling_time = first_order_low_pass_filter<double>::calculate_settling_time(
    cutoff_frequency, 
    95.0  // 95% of final value
);

// Common settling times for first-order systems:
// 63.2% : t = τ
// 86.5% : t = 2τ
// 95.0% : t = 3τ
// 98.2% : t = 4τ
// 99.3% : t = 5τ
```

## Usage

### Real-Time Filtering

```cpp
// In control loop
double noisy_measurement = adc_read();
double filtered_value = filter.update(noisy_measurement);

// Or access output separately
filter.update(noisy_measurement);
double output = filter.get_output();
```

### Reset Filter State

```cpp
// Reset to zero
filter.reset();

// Reset to specific value (useful for initialization)
double initial_value = 1.0;
filter.reset(initial_value);
```

### Coordinate Frame Filtering

The filter works seamlessly with coordinate frame types:

```cpp
#include "yeet_control/reference_frame.hpp"

// Create filter for dq frame
first_order_low_pass_filter<dq<voltage_pu_t>> dq_filter(100e-6);
dq_filter.configure(50.0);  // 50 Hz cutoff

// Filter dq voltages
dq<voltage_pu_t> measured = measure_voltages_dq();
dq<voltage_pu_t> filtered = dq_filter.update(measured);

// Each component (d and q) is filtered independently
```

## Frequency Response Analysis

### Magnitude Response

Calculate the magnitude response at any frequency:

```cpp
double test_frequency = 5.0;  // Hz

// Linear magnitude (gain factor)
double magnitude = filter.get_magnitude_response(test_frequency);

// Magnitude in decibels
double magnitude_db = filter.get_magnitude_response_db(test_frequency);

// At cutoff frequency, magnitude should be -3 dB (≈0.707)
double mag_at_fc = filter.get_magnitude_response(cutoff_frequency);
// mag_at_fc ≈ 0.707
```

### Phase Response

Calculate the phase response at any frequency:

```cpp
// Phase in radians
double phase_rad = filter.get_phase_response(test_frequency);

// Phase in degrees
double phase_deg = filter.get_phase_response_degrees(test_frequency);

// At cutoff frequency, phase is -45 degrees
double phase_at_fc = filter.get_phase_response_degrees(cutoff_frequency);
// phase_at_fc ≈ -45.0
```

### Bode Plot Generation

```cpp
// Generate frequency response data for plotting
std::vector<double> frequencies = {0.1, 1.0, 5.0, 10.0, 20.0, 50.0, 100.0};
std::vector<double> magnitudes_db;
std::vector<double> phases_deg;

for (double freq : frequencies) {
    magnitudes_db.push_back(filter.get_magnitude_response_db(freq));
    phases_deg.push_back(filter.get_phase_response_degrees(freq));
}
```

## Practical Applications

### Sensor Measurement Filtering

```cpp
class TemperatureSensor {
public:
    TemperatureSensor(double sampling_period) 
        : _filter(sampling_period)
    {
        // 1 Hz cutoff for slow temperature dynamics
        _filter.configure(1.0);
    }
    
    double read_temperature() {
        double raw_reading = read_adc();
        return _filter.update(raw_reading);
    }
    
private:
    first_order_low_pass_filter<double> _filter;
};
```

### Current Measurement Filtering

```cpp
class CurrentController {
public:
    CurrentController(double ts) 
        : _current_filter(ts)
    {
        // Higher cutoff for faster current dynamics
        // Filter harmonics while preserving control bandwidth
        _current_filter.configure(200.0);  // 200 Hz
    }
    
    void update(double measured_current, double reference) {
        // Filter measurement
        double filtered_current = _current_filter.update(measured_current);
        
        // Control based on filtered value
        double error = reference - filtered_current;
        // ... PI control ...
    }
    
private:
    first_order_low_pass_filter<double> _current_filter;
};
```

### DC Bus Voltage Smoothing

```cpp
// Filter DC bus voltage for slow variations
first_order_low_pass_filter<voltage_pu_t> vdc_filter(100e-6);
vdc_filter.configure(10.0);  // 10 Hz cutoff

// In control loop
voltage_pu_t measured_vdc = measure_dc_voltage();
voltage_pu_t filtered_vdc = vdc_filter.update(measured_vdc);

// Use filtered value for feed-forward compensation
```

### Speed Estimation Filtering

```cpp
class SpeedEstimator {
public:
    SpeedEstimator(double ts) : _speed_filter(ts)
    {
        // Low cutoff for mechanical speed (slow dynamics)
        _speed_filter.configure(5.0);  // 5 Hz
    }
    
    double estimate_speed(angle_wrapped current_angle) {
        // Calculate instantaneous speed
        angle_wrapped delta = current_angle - _previous_angle;
        double speed = delta.get_radians() / _dt;
        
        // Filter to remove noise
        double filtered_speed = _speed_filter.update(speed);
        
        _previous_angle = current_angle;
        return filtered_speed;
    }
    
private:
    first_order_low_pass_filter<double> _speed_filter;
    angle_wrapped _previous_angle{0.0};
    double _dt{100e-6};
};
```

## Design Guidelines

### Choosing Cutoff Frequency

The cutoff frequency determines the balance between noise rejection and response speed:

**Low Cutoff (< 1 Hz)**
- **Pros**: Excellent noise rejection, smooth output
- **Cons**: Slow response to changes, large settling time
- **Use**: Temperature, pressure, slow mechanical quantities

**Medium Cutoff (1-100 Hz)**
- **Pros**: Good compromise between filtering and response
- **Cons**: Some phase lag in control loops
- **Use**: Speed estimation, voltage measurements, general purpose

**High Cutoff (> 100 Hz)**
- **Pros**: Fast response, minimal phase lag
- **Cons**: Limited noise rejection
- **Use**: Current measurements, inner control loops, fast signals

### Nyquist Criterion

The filter enforces the Nyquist criterion: the cutoff frequency must be less than half the sampling frequency:

```
fc < fs/2
```

```cpp
double sampling_freq = 10000.0;  // 10 kHz sampling
double sampling_period = 1.0 / sampling_freq;

first_order_low_pass_filter<double> filter(sampling_period);

// This is valid (fc < 5000 Hz)
filter.configure(1000.0);  // OK

// This will assert fail (fc >= 5000 Hz)
filter.configure(6000.0);  // ERROR: Violates Nyquist criterion
```

### Cascading Filters

Multiple first-order filters can be cascaded for steeper roll-off:

```cpp
// Two cascaded first-order filters ≈ second-order filter
first_order_low_pass_filter<double> filter1(sampling_period);
first_order_low_pass_filter<double> filter2(sampling_period);

filter1.configure(cutoff);
filter2.configure(cutoff);

// Cascade in series
double filtered = filter2.update(filter1.update(noisy_signal));

// Total roll-off: -40 dB/decade (vs -20 dB/decade for single filter)
```

## Advanced Features

### Accessing Filter Coefficients

```cpp
// Retrieve filter coefficients for analysis
double a1 = filter.get_feedback_coeff_a1();
double b0 = filter.get_feedforward_coeff_b0();
double b1 = filter.get_feedforward_coeff_b1();
double ts = filter.get_sampling_time();

// Use for custom implementations or analysis
```

### Numerical Stability

The implementation uses double-precision arithmetic internally for all calculations, even when the template type T is float. This prevents numerical instability with small coefficient values:

```cpp
// Even with float template, internal math uses double
first_order_low_pass_filter<float> filter(100e-6);
filter.configure(0.01f);  // Very low cutoff

// Coefficients computed in double precision
// Results converted back to float only at output
```

## Performance Characteristics

### Computational Cost

Per `update()` call:
- 2 multiplications (b0 × input, b1 × prev_input)
- 1 multiplication (a1 × prev_output)
- 2 additions
- **Total**: ~5 floating-point operations

### Memory Footprint

- Coefficients: 3 doubles (24 bytes)
- State: 2 doubles (16 bytes)
- Sampling time: 1 double (8 bytes)
- **Total**: ~48 bytes per filter instance

### Execution Time

Typical execution times (single filter, float type):
- ARM Cortex-M4 @ 168 MHz: ~50 ns
- ARM Cortex-M7 @ 400 MHz: ~20 ns
- x86-64 @ 3.0 GHz: ~5 ns

## Comparison with Other Filters

| Filter Type | Roll-off | Phase Lag | Complexity | Use Case |
|-------------|----------|-----------|------------|----------|
| First-order LPF | -20 dB/dec | Moderate | Low | General filtering |
| Moving average | -20 dB/dec | High | Low | Simple smoothing |
| Second-order LPF | -40 dB/dec | Higher | Medium | Sharper cutoff |
| Notch filter | Band-specific | Frequency-dependent | Medium | Harmonic rejection |

## Debugging and Validation

### Validate Configuration

```cpp
// Check filter is properly configured
assert(filter.get_sampling_time() > 0.0);
assert(filter.get_feedforward_coeff_b0() > 0.0);

// Verify cutoff frequency is reasonable
double fc = 10.0;
filter.configure(fc);
double mag_at_fc = filter.get_magnitude_response(fc);
assert(std::abs(mag_at_fc - 0.707) < 0.01);  // Should be ~0.707 at fc
```

### Step Response Test

```cpp
// Test step response
filter.reset(0.0);
double step_input = 1.0;

// Apply step input and measure response
std::vector<double> response;
for (int i = 0; i < 1000; ++i) {
    double output = filter.update(step_input);
    response.push_back(output);
}

// Final value should approach input
assert(std::abs(response.back() - step_input) < 0.01);
```

## See Also

- **second_order_filter** - Biquad filters for sharper roll-off
- [pi_controller.md](pi_controller.md) - Controllers with similar structure
- [reference_frame.md](reference_frame.md) - Coordinate frame types for filtering
- [units.md](units.md) - Type-safe dimensional quantities
