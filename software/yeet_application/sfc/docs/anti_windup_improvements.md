# Anti-Windup Improvements for SFC Controllers

This document explains the anti-windup strategy implemented in the SFC (Static Frequency Converter) control system to prevent integrator windup in cascaded control loops.

## Problem Description

In cascaded control systems like the SFC drive, integrator windup can occur at multiple levels:

1. **Current Controller Internal Windup**: When the firing angle hits its physical limits (0° to 90°)
2. **Speed-to-Current Controller Windup**: When the current controller saturates, but the speed controller continues to integrate error

Without proper anti-windup measures, these issues lead to:
- Large overshoots when saturation is removed
- Poor transient response
- Potential system instability

## Implementation

### 1. Current Controller Internal Anti-Windup

The [`sfc_current_controller`](software/yeet_application/sfc/include/sfc/sfc_current_controller.hpp ) now implements proper internal anti-windup:

#### Key Improvements:
- **PI Controller Limits**: The internal PI controller's output limits are set to match the physical firing angle limits
- **Automatic Anti-Windup**: Integrator clamping is enabled to prevent windup when limits are reached
- **Type Consistency**: Changed from `double` to `real_t` for consistency

#### Code Example:
```cpp
// In recompute_internal_limits():
_cos_alpha_max = std::cos(_grid_firing_angle_min_radians.value());
_cos_alpha_min = std::cos(_grid_firing_angle_max_radians.value());

// Configure PI controller anti-windup with these limits
_pi_controller.set_output_limits(_cos_alpha_min, _cos_alpha_max);
_pi_controller.enable_integrator_clamping(true);
```

### 2. Back-Calculation Anti-Windup Between Controllers

The [`sfc_speed_controller`](software/yeet_application/sfc/include/sfc/sfc_speed_controller.hpp ) now provides a method to handle downstream saturation:

#### New Method:
```cpp
void handle_downstream_saturation(bool downstream_saturated);
```

#### Usage Pattern:
```cpp
// Update current controller first
current_controller.update(current_ref, current_measured, grid_voltage_abc, back_emf);

// Handle anti-windup: freeze speed controller if current controller saturates
speed_controller.handle_downstream_saturation(current_controller.is_saturated());

// Update speed controller
speed_controller.update(speed_ref, speed_measured);

// Get outputs
torque_pu_t torque_ref = speed_controller.get_torque_ref_pu();
current_pu_t current_ref = convert_torque_to_current(torque_ref);
```

### 3. Fixed Slew Rate Limiting Bug

The current controller now correctly applies the slew rate limit:

#### Before (Bug):
```cpp
_rate_of_change_limiter.update(saturated_current_reference_pu.value());
_current_reference_limited_pu = saturated_current_reference_pu; // BUG: Not using limiter output
```

#### After (Fixed):
```cpp
_rate_of_change_limiter.update(saturated_current_reference_pu.value());
_current_reference_limited_pu = current_pu_t{_rate_of_change_limiter.get_output()}; // FIXED
```

## Benefits

### 1. **Improved Transient Response**
- No more large overshoots after periods of saturation
- Faster settling times during load changes
- More predictable behavior during acceleration/deceleration

### 2. **Enhanced Stability**
- Prevents integrator windup from destabilizing the system
- Better behavior during fault conditions or extreme operating points
- More robust operation across the full speed range

### 3. **Better Integration**
- Clear interface for coordinating multiple control loops
- Standardized anti-windup approach across all controllers
- Easy to extend to additional control layers

## Integration Guidelines

### For Application Developers:

1. **Always check saturation status** after updating the current controller
2. **Call `handle_downstream_saturation()`** before updating the speed controller
3. **Monitor saturation status** for diagnostic purposes

### For System Integrators:

1. **Configure appropriate limits** for all controllers during commissioning
2. **Test anti-windup behavior** under various operating conditions
3. **Log saturation events** for performance analysis

## Example Integration

```cpp
class sfc_drive_controller {
public:
    void update(speed_pu_t speed_ref, speed_pu_t speed_measured, 
                current_pu_t current_measured, voltage_pu_t grid_voltage_abc, 
                voltage_pu_t back_emf) {
        
        // Get current reference from speed controller (previous cycle)
        current_pu_t current_ref = _torque_to_current_converter.convert(_speed_controller.get_torque_ref_pu());
        
        // Update current controller
        _current_controller.update(current_ref, current_measured, grid_voltage_abc, back_emf);
        
        // Handle anti-windup coordination
        _speed_controller.handle_downstream_saturation(_current_controller.is_saturated());
        
        // Update speed controller for next cycle
        _speed_controller.update(speed_ref, speed_measured);
        
        // Log saturation for diagnostics
        _diagnostics.current_controller_saturated = _current_controller.is_saturated();
        _diagnostics.speed_controller_saturated = _speed_controller.is_saturated();
    }
    
private:
    sfc_speed_controller _speed_controller;
    sfc_current_controller _current_controller;
    // ... other components
};
```

This implementation provides a robust foundation for anti-windup in LCI control systems, ensuring stable and predictable operation across all operating conditions.
