# Enhanced Voltage Model for Low-Speed Operation

## Physical Motivation

At standstill and very low speeds, the traditional back-EMF model `E = Φ × ω` predicts zero voltage, which is physically incorrect. In reality, voltage is required to:
- Overcome stator resistance: `V_R = R_s × I_s`
- Establish magnetic flux through the field winding
- Induce current in the stator even when the rotor is stationary

## Implementation

The enhanced model blends between two operating regions:

### 1. Low-Speed Region (Resistive-Dominant)
At standstill and low speeds (ω < 0.15 pu), voltage is primarily determined by resistive voltage drops:
```
V_resistive = V_offset × Φ
```
where:
- `V_offset = 0.1 pu` (default, configurable via `voltage_control_params.voltage_offset`)
- `Φ` is the per-unit flux (proportional to field current)

### 2. High-Speed Region (EMF-Dominant)
At higher speeds (ω > 0.15 pu), voltage is dominated by back-EMF:
```
V_emf = Φ × ω
```

### 3. Smooth Transition
The total voltage uses a smooth blending function:
```
blend = min(|ω| / ω_transition, 1.0)
V_total = V_resistive × (1 - blend) + V_emf × blend
```
where `ω_transition = 0.15 pu` is the transition speed.

## Voltage Behavior vs. Speed

| Speed (pu) | Blend Factor | Dominant Component | Voltage Expression |
| ---------- | ------------ | ------------------ | ------------------ |
| 0.00       | 0.00         | Resistive 100%     | `V ≈ 0.1 × Φ`      |
| 0.05       | 0.33         | Resistive 67%      | Blended            |
| 0.10       | 0.67         | Resistive 33%      | Blended            |
| 0.15       | 1.00         | EMF 100%           | `V = Φ × ω`        |
| 1.00       | 1.00         | EMF 100%           | `V = Φ × 1.0 = Φ`  |

## Example Calculation

Given:
- Field current: `i_f = 1.0 pu` → Flux: `Φ = 1.0 pu`
- Speed: `ω = 0.0 pu` (standstill)
- Voltage offset: `V_offset = 0.1 pu`

**Traditional Model:**
```
V = Φ × ω = 1.0 × 0.0 = 0.0 pu  ← Incorrect!
```

**Enhanced Model:**
```
blend = min(0.0 / 0.15, 1.0) = 0.0
V_resistive = 0.1 × 1.0 = 0.1 pu
V_emf = 1.0 × 0.0 = 0.0 pu
V_total = 0.1 × (1 - 0.0) + 0.0 × 0.0 = 0.1 pu  ← Correct!
```

At rated speed with rated flux:
```
blend = min(1.0 / 0.15, 1.0) = 1.0
V_resistive = 0.1 × 1.0 = 0.1 pu
V_emf = 1.0 × 1.0 = 1.0 pu
V_total = 0.1 × (1 - 1.0) + 1.0 × 1.0 = 1.0 pu  ← Pure EMF
```

## Configuration

The voltage offset can be adjusted via the `voltage_control_parameters` structure:

```cpp
sfc_plant_model_voltage_control_parameters params;
params.voltage_offset = 0.15f;  // Increase to 15% for higher resistive component
plant.configure_voltage_control_parameters(params);
```

Typical values:
- Small machines (low R_s): `V_offset = 0.05 - 0.10 pu`
- Large machines (higher R_s): `V_offset = 0.10 - 0.15 pu`
- Very large LCI drives: `V_offset = 0.15 - 0.20 pu`

## Physical Justification

This model aligns with fundamental machine physics:

1. **At Standstill (ω = 0):**
   - No mechanical rotation → No back-EMF
   - Voltage needed to establish flux: `V = R × I + L × di/dt`
   - Steady-state: `V ≈ R × I` (resistive component)

2. **At Low Speed (0 < ω < ω_transition):**
   - Small back-EMF contribution
   - Resistive voltage still significant
   - Both components present

3. **At High Speed (ω > ω_transition):**
   - Back-EMF dominates: `E >> R × I`
   - Voltage follows classical synchronous machine model
   - Resistive component negligible

## Code Location

Implementation: `software/yeet_application/sfc/src/sfc_plant_model_voltage_behind_reactance.cpp`
- Method: `step_voltage_control()`
- Lines: ~395-425
- Default offset: `kDefaultVoltageOffset = 0.1f`
- Transition speed: `transition_speed = 0.15f`
