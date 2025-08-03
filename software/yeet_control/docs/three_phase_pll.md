# Three-Phase Phase Locked Loop (PLL)

## Overview

The `three_phase_pll` class implements a synchronous reference frame (SRF) based phase locked loop for estimating the angular frequency and phase of a three-phase sinusoidal signal. This implementation is specifically designed for power system applications where accurate frequency and phase tracking of grid voltages is critical.

## Features

### Core Functionality
- **Frequency Estimation**: Accurate tracking of fundamental frequency with configurable limits.
- **Phase Estimation**: Continuous phase angle estimation using integration.
- **Fast Convergence**: Rapid lock-on to nominal frequency with minimal settling time.

### Robustness Features
- **Voltage Unbalance Tolerance**: Maintains accuracy even with unbalanced three-phase inputs.
- **Frequency Limits**: Configurable minimum and maximum frequency bounds.
- **Lock Detection**: Automatic detection of PLL lock status.
- **Anti-Windup Protection**: PI controller with integrator clamping to prevent windup.

### Signal Processing Chain
1. **Clarke Transformation**: Converts three-phase abc signals to two-phase αβ coordinates.
2. **Park Transformation**: Transforms αβ signals to rotating dq frame using estimated angle.
3. **Error Extraction**: Uses q-axis component as frequency error signal.
4. **PI Control**: Processes error through tuned PI controller for frequency estimation.
5. **Integration**: Integrates frequency to obtain phase angle.

## Algorithm Description

### Mathematical Foundation

The PLL operates on the principle that in a balanced three-phase system, when the reference frame rotates at the same frequency as the input signals, the q-axis component of the Park transform becomes zero.

**Transformation Chain:**
`abc → αβ → dq`

**Control Loop:**
`ω_est = ω_nominal + PI_controller(-q_error)`
`θ_est = ∫ ω_est dt`

### Control Structure
```
Three-Phase    Clarke     Park      PI         Integrator    Estimated
Input     →   Transform → Transform → Controller → (ω→θ)   →   Angle
   ↑              ↓         ↓          ↑                        ↓
   abc          αβ        dq       q_error                     θ_est
                                     ↑___________________________|
                                          Feedback Loop
```

## Configuration Parameters

The PLL is configured through dedicated methods:

- `configure_pi_controller(kp, ki)`: Sets the gains for the frequency PI controller.
- `configure_frequency_limits(min_freq, max_freq)`: Sets the operational frequency range.
- `configure_nominal_frequency(nominal_freq)`: Sets the feed-forward nominal frequency.

### Tuning Guidelines

#### PI Controller Gains
- **Proportional Gain (Kp)**: Higher values provide faster response but may cause overshoot.
- **Integral Gain (Ki)**: Eliminates steady-state error.

#### Frequency Limits
- Set based on the expected operating range of the system (e.g., ±5 Hz around nominal for grid applications).

## Usage Examples

### Basic Setup
```cpp
#include "yeet_control/three_phase_pll.hpp"

using namespace yeet::control;

// 1. Instantiate PLL with sampling time
three_phase_pll<double> pll(100e-6); // 100 µs sampling time

// 2. Configure PLL parameters
pll.configure_nominal_frequency(50.0);      // 50 Hz nominal
pll.configure_pi_controller(100.0, 10000.0); // Kp, Ki gains
pll.configure_frequency_limits(45.0, 55.0); // Min/max frequency
```

### Real-Time Processing
```cpp
// In real-time loop
abc<double> measured_voltages = get_voltage_measurements();

// Update the PLL with the latest measurements
pll.update(measured_voltages);

// Use estimated values
double estimated_frequency = pll.get_estimated_frequency();
angle_wrapped estimated_angle = pll.get_estimated_angle();
bool is_locked = pll.is_locked(0.05); // 5% threshold

// Monitor lock status
if (is_locked) {
    // PLL is locked, estimates are reliable
    use_frequency_estimate(estimated_frequency);
} else {
    // PLL is acquiring lock, use with caution
    handle_unlocked_condition();
}
```

### Advanced Monitoring
```cpp
// Detailed monitoring after update
std::cout << "Frequency: " << pll.get_estimated_frequency() << " Hz" << std::endl;
std::cout << "Phase: " << pll.get_estimated_angle().get_degrees() << " degrees" << std::endl;
std::cout << "Q-axis error: " << pll.get_q_error() << std::endl;

const auto& alphabeta = pll.get_voltage_alphabeta();
std::cout << "Alpha-Beta: [" << alphabeta.alpha() << ", " << alphabeta.beta() << "]" << std::endl;

const auto& dq = pll.get_voltage_dq();
std::cout << "DQ: [" << dq.d() << ", " << dq.q() << "]" << std::endl;

// Check controller status
const auto& controller = pll.get_frequency_controller();
if (controller.is_output_saturated()) {
    std::cout << "Warning: Frequency controller saturated" << std::endl;
}
```

## API Reference

See the header file `three_phase_pll.hpp` for complete API documentation with detailed parameter descriptions and usage examples.
