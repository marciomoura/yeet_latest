# Excitation Control Integration with Plant Model

## Overview

This document describes how to integrate the `sfc_excitation_controller` with the `sfc_plant_model_augmented_6p_6p` to create a closed-loop simulation with **perfect exciter tracking**. In this configuration, the excitation controller's voltage setpoint is directly applied to the plant model's machine-side voltage source, simulating an ideal excitation system.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        LCI Control System                            │
│                                                                      │
│  ┌──────────────┐       ┌──────────────┐      ┌─────────────────┐  │
│  │   Machine    │──────▶│  Excitation  │─────▶│  External       │  │
│  │   Observer   │ speed │  Controller  │ v_ex │  Exciter        │  │
│  │              │  ω_pu │              │  pu  │  (Perfect)      │  │
│  └──────────────┘       └──────────────┘      └─────────────────┘  │
│                                                         │            │
└─────────────────────────────────────────────────────────┼────────────┘
                                                          │
                                                          │ v_ex_pu
                                                          ▼
                 ┌────────────────────────────────────────────────┐
                 │       Augmented Plant Model                    │
                 │                                                │
                 │  Mode: external_control                       │
                 │  Machine Voltage = v_ex_pu × V_nominal        │
                 │                                                │
                 │  ┌──────────────┐      ┌─────────────────┐   │
                 │  │  Mechanical  │      │  Synchronous    │   │
                 │  │  Dynamics    │◀────▶│  Machine        │   │
                 │  │  (ω, T_em)   │      │  (Back-EMF)     │   │
                 │  └──────────────┘      └─────────────────┘   │
                 │                                                │
                 └────────────────────────────────────────────────┘
```

## Configuration Steps

### 1. Configure the Plant Model

Set the plant model to use **external voltage control mode**:

```cpp
// Create augmented plant model
sfc_plant_model_augmented_6p_6p plant;

// Enable variable speed mode (mechanical dynamics)
plant.set_operation_mode(plant_model_machine_operation_mode::variable_speed);

// Enable external voltage control (excitation controller drives voltage)
plant.set_voltage_control_mode(plant_model_voltage_control_mode::external_control);

// Initialize
plant.initialize();
```

### 2. Create and Configure Excitation Controller

```cpp
// Create excitation controller
sfc_excitation_controller excitation_controller(duration_t{0.001}); // 1 kHz

// Configure pre-fluxing parameters
excitation_controller.configure_prefluxing_duration(duration_t{2.0}); // 2 seconds

// Configure V/Hz lookup table (speed → voltage)
std::array<real_t, 6> speed_axis_pu = {0.0, 0.05, 0.80, 0.95, 1.00, 1.05};
std::array<real_t, 6> voltage_axis_pu = {0.30, 0.30, 1.00, 0.80, 0.73, 0.70};
excitation_controller.configure_excitation_voltage_lookup_table(speed_axis_pu, voltage_axis_pu);

// Start excitation sequence
excitation_controller.start_excitation();
```

### 3. Main Control Loop

```cpp
// Main simulation loop
for (int cycle = 0; cycle < num_cycles; ++cycle) {
    // 1. Get current machine speed from plant model
    speed_pu_t machine_speed = plant.get_mechanical_speed_pu();
    
    // 2. Update excitation controller with current speed
    bool exciter_ready = true; // Assume exciter is always ready in simulation
    excitation_controller.update(machine_speed, exciter_ready);
    
    // 3. Get excitation voltage setpoint from controller
    voltage_pu_t excitation_voltage_setpoint = excitation_controller.get_excitation_voltage_setpoint_pu();
    bool enable_excitation = excitation_controller.get_enable_excitation();
    
    // 4. Apply excitation setpoint to plant model (perfect tracking)
    if (enable_excitation) {
        plant.set_excitation_voltage_setpoint_pu(excitation_voltage_setpoint);
    } else {
        plant.set_excitation_voltage_setpoint_pu(voltage_pu_t{0.0});
    }
    
    // 5. Update plant model (includes mechanical dynamics)
    plant.update();
    
    // 6. Log data if needed
    log_data(cycle, machine_speed, excitation_voltage_setpoint);
}
```

## Complete Integration Example

Here's a complete example showing the full closed-loop setup:

```cpp
#include "sfc/sfc_plant_model_augmented_6p_6p.hpp"
#include "sfc/sfc_excitation_controller.hpp"
#include <iostream>

int main() {
    using namespace yeet::application::sfc;
    
    // ========================================================================
    // 1. Create and configure plant model
    // ========================================================================
    sfc_plant_model_augmented_6p_6p plant;
    
    // Configure for variable speed with external excitation control
    plant.set_operation_mode(plant_model_machine_operation_mode::variable_speed);
    plant.set_voltage_control_mode(plant_model_voltage_control_mode::external_control);
    
    // Set initial conditions
    plant.set_initial_mechanical_speed_pu(speed_pu_t{0.0});
    plant.set_load_torque_pu(torque_pu_t{0.5}); // 50% load torque
    
    // Initialize plant
    plant.initialize();
    
    // ========================================================================
    // 2. Create and configure excitation controller
    // ========================================================================
    sfc_excitation_controller excitation_controller(duration_t{0.001}); // 1 ms
    
    // Configure timing
    excitation_controller.configure_prefluxing_duration(duration_t{2.0});
    excitation_controller.configure_excitation_ramp_down_time(duration_t{1.0});
    
    // Configure V/Hz characteristic
    std::array<real_t, 6> speed_axis = {0.0, 0.05, 0.80, 0.95, 1.00, 1.05};
    std::array<real_t, 6> voltage_axis = {0.30, 0.30, 1.00, 0.80, 0.73, 0.70};
    excitation_controller.configure_excitation_voltage_lookup_table(speed_axis, voltage_axis);
    
    // Start excitation
    if (excitation_controller.start_excitation()) {
        std::cout << "Excitation started successfully" << std::endl;
    }
    
    // ========================================================================
    // 3. Simulation loop
    // ========================================================================
    const int simulation_cycles = 10000; // 10 seconds at 1 kHz
    
    for (int cycle = 0; cycle < simulation_cycles; ++cycle) {
        // Get current machine state
        speed_pu_t machine_speed = plant.get_mechanical_speed_pu();
        torque_pu_t em_torque = plant.get_electromagnetic_torque_pu();
        
        // Update excitation controller
        excitation_controller.update(machine_speed, true);
        
        // Get excitation setpoint
        voltage_pu_t v_ex = excitation_controller.get_excitation_voltage_setpoint_pu();
        bool enable = excitation_controller.get_enable_excitation();
        
        // Apply to plant (perfect exciter)
        plant.set_excitation_voltage_setpoint_pu(enable ? v_ex : voltage_pu_t{0.0});
        
        // Update plant dynamics
        plant.update();
        
        // Print status every second
        if (cycle % 1000 == 0) {
            std::cout << "Time: " << cycle * 0.001 << "s, "
                      << "Speed: " << machine_speed.value() << " pu, "
                      << "V_exc: " << v_ex.value() << " pu, "
                      << "T_em: " << em_torque.value() << " pu, "
                      << "Field Est: " << excitation_controller.get_is_field_established()
                      << std::endl;
        }
    }
    
    // Shutdown sequence
    excitation_controller.stop_excitation();
    
    // Run ramp-down
    for (int i = 0; i < 2000; ++i) {
        speed_pu_t machine_speed = plant.get_mechanical_speed_pu();
        excitation_controller.update(machine_speed, true);
        voltage_pu_t v_ex = excitation_controller.get_excitation_voltage_setpoint_pu();
        plant.set_excitation_voltage_setpoint_pu(v_ex);
        plant.update();
    }
    
    std::cout << "Simulation complete" << std::endl;
    
    return 0;
}
```

## Voltage Control Modes Comparison

The augmented plant model now supports **three voltage control modes**:

| Mode                   | Description                                        | Use Case                                           |
| ---------------------- | -------------------------------------------------- | -------------------------------------------------- |
| `fixed_voltage`        | Machine voltage is constant (1.0 pu)               | Simple testing, no excitation control              |
| `speed_dependent`      | Voltage calculated from internal V/Hz relationship | Autonomous plant behavior, open-loop testing       |
| **`external_control`** | **Voltage set by external excitation controller**  | **Closed-loop testing with excitation controller** |

## Key Features of External Control Mode

### Perfect Tracking
The external control mode simulates an **ideal exciter** with:
- **Zero delay**: Voltage changes instantly to match setpoint
- **Perfect accuracy**: No tracking error
- **No saturation**: Can achieve any voltage within configured limits

### Benefits for Testing
1. **Isolates Controller Performance**: Test excitation controller logic without real exciter dynamics
2. **Repeatability**: Deterministic behavior for regression testing
3. **Fast Simulation**: No need to model complex exciter hardware
4. **Clear Causality**: Direct connection shows controller effectiveness

### Comparison with Real System
In a real LCI system:
- Excitation controller outputs a **current setpoint** (not voltage)
- External exciter has **dynamics** (thyristor bridges, field winding L/R)
- Field current creates **flux** which produces **back-EMF**
- There are **delays and nonlinearities**

The `external_control` mode bypasses all this complexity and directly sets the voltage, which is perfect for:
- Algorithm development
- Controller tuning
- System-level testing
- Educational demonstrations

## Integration with SFC State Machine

For a complete LCI startup sequence with excitation control:

```cpp
// During PREFLUXING state
if (state_machine.is_in_state(sfc_state_id::PREFLUXING)) {
    if (!excitation_controller.get_is_running()) {
        excitation_controller.start_excitation();
    }
    
    // Wait for field establishment
    if (excitation_controller.get_is_field_established()) {
        state_machine.transition_to(sfc_state_id::RAMPING);
    }
}

// During RAMPING state
if (state_machine.is_in_state(sfc_state_id::RAMPING)) {
    // Excitation controller automatically adjusts voltage with speed
    // Machine accelerates under torque control
}

// During SHUTDOWN
if (state_machine.is_in_state(sfc_state_id::SHUTDOWN)) {
    excitation_controller.stop_excitation();
    // Voltage ramps down safely
}
```

## Troubleshooting

### Issue: Voltage not changing
**Check:**
- Voltage control mode is set to `external_control`
- `set_excitation_voltage_setpoint_pu()` is called every cycle
- Excitation controller `enable_excitation` is true

### Issue: Machine not accelerating
**Check:**
- Operation mode is set to `variable_speed`
- Load torque is reasonable (< electromagnetic torque)
- Mechanical inertia is configured properly

### Issue: Unstable simulation
**Check:**
- Excitation voltage not changing too rapidly
- Mechanical time constants are reasonable
- Sampling time is appropriate (typically 1 kHz for excitation, 100 kHz for plant)

## Summary

The **external voltage control mode** provides a clean interface for integrating the excitation controller with the plant model:

1. Set `plant.set_voltage_control_mode(external_control)`
2. Call `plant.set_excitation_voltage_setpoint_pu(v_ex)` each cycle
3. Plant model uses this voltage directly for the machine-side voltage source

This creates a **perfect exciter** simulation ideal for testing and validating the excitation control algorithms without the complexity of real exciter hardware dynamics.
