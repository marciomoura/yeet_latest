# Dimension and Units Library

## 1. Overview

This library provides a type-safe, zero-overhead C++17 framework for dimensional analysis in scientific and engineering computations. Its primary goal is to leverage the C++ type system to catch unit-related errors at compile time, preventing a common class of bugs in numerical code.

By encoding physical dimensions (like Length, Mass, Time) and unit systems (like SI and Per-Unit) into types, the library ensures that operations are physically coherent. This eliminates logical errors such as adding a voltage to a current or comparing a value in SI units to a per-unit value without explicit conversion. The design guarantees that these compile-time checks add no performance penalty to runtime calculations.

## 2. Core Concepts

### Dimensional Representation
Physical dimensions are represented using a template-based system that encodes the seven base SI units as integer exponents:

```cpp
template <int L, int M, int T, int I, int Th, int N, int J>
struct dimension {};
```

Where:
- **L**: Length exponent
- **M**: Mass exponent  
- **T**: Time exponent
- **I**: Current exponent
- **Th**: Temperature exponent
- **N**: Amount of substance exponent
- **J**: Luminous intensity exponent

For example, a `Velocity` (Length/Time) has dimension `<1, 0, -1, 0, 0, 0, 0>`, while a `Force` (Mass × Length / Time²) has dimension `<1, 1, -2, 0, 0, 0, 0>`.

### The Quantity
The central component of the library is the **quantity** template class:

```cpp
template <typename Dimension, typename UnitSystem, typename Representation>
class quantity {
    // Implementation details...
};
```

A `quantity` bundles:
- A numerical value (using any arithmetic type like `float`, `double`, `long double`)
- Its physical dimension (encoded as a type)
- Its unit system (`si` or `per_unit`)

### Unit System Differentiation
The library provides first-class support for two unit systems through tag types:

```cpp
struct si {};        // International System of Units
struct per_unit {};  // Per-Unit system for power systems
```

## 3. Key Features

### Compile-Time Safety
The library enforces correctness before the program runs:

#### Dimensional Correctness
Operations between incompatible dimensions result in compile-time errors:

```cpp
auto length_val = length<double>(10.0);    // 10 meters
auto time_val = duration_type<double>(2.0); // 2 seconds
auto velocity_val = length_val / time_val;   // OK: Length/Time = Velocity
// auto invalid = length_val + time_val;     // Compile error!
```

#### Unit System Integrity
Direct mixing of SI and Per-Unit quantities is prohibited:

```cpp
auto v_si = voltage<double>(120.0);        // 120 V (SI)
auto v_pu = voltage_pu<double>(0.95);      // 0.95 pu
// auto invalid = v_si + v_pu;             // Compile error!
```

### Comprehensive Arithmetic Operations

#### Addition & Subtraction
Only permitted between quantities of identical dimension and unit system:

```cpp
auto v1 = voltage<double>(120.0);
auto v2 = voltage<double>(240.0);
auto v_total = v1 + v2;  // Result: 360 V
```

#### Multiplication & Division
Automatically calculates resulting dimensions based on dimensional analysis:

```cpp
auto v = voltage<double>(120.0);      // Volts
auto i = current<double>(5.0);        // Amperes
auto p = v * i;                       // Result: power<double> (Watts)

auto f = force<double>(100.0);        // Newtons
auto a = length<double>(10.0);        // Meters (area)
auto pressure = f / a;                // Result: pressure (force_dim/area_dim)
```

#### Scalar Operations
Quantities can be multiplied or divided by plain numbers:

```cpp
auto base_voltage = voltage<double>(120.0);
auto doubled = base_voltage * 2.0;  // 240 V
auto halved = base_voltage / 2.0;   // 60 V
```

### Predefined Unit Types

#### Base SI Units
```cpp
template <typename Rep = double> using length = quantity<length_dim, si, Rep>;
template <typename Rep = double> using mass = quantity<mass_dim, si, Rep>;
template <typename Rep = double> using duration_type = quantity<time_dim, si, Rep>;
template <typename Rep = double> using current = quantity<current_dim, si, Rep>;
```

#### Mechanical Units
```cpp
template <typename Rep = double> using velocity = quantity<velocity_dim, si, Rep>;
template <typename Rep = double> using acceleration = quantity<acceleration_dim, si, Rep>;
template <typename Rep = double> using force = quantity<force_dim, si, Rep>;
template <typename Rep = double> using torque = quantity<torque_dim, si, Rep>;
template <typename Rep = double> using moment_of_inertia = quantity<moment_of_inertia_dim, si, Rep>;
```

#### Electrical Units
```cpp
template <typename Rep = double> using voltage = quantity<voltage_dim, si, Rep>;
template <typename Rep = double> using power = quantity<power_dim, si, Rep>;
template <typename Rep = double> using resistance = quantity<resistance_dim, si, Rep>;
template <typename Rep = double> using impedance = quantity<resistance_dim, si, Rep>;
template <typename Rep = double> using capacitance = quantity<capacitance_dim, si, Rep>;
template <typename Rep = double> using inductance = quantity<inductance_dim, si, Rep>;
template <typename Rep = double> using frequency = quantity<frequency_dim, si, Rep>;
template <typename Rep = double> using magnetic_flux = quantity<magnetic_flux_dim, si, Rep>;
template <typename Rep = double> using magnetic_field_strength = quantity<mag_field_strength_dim, si, Rep>;
template <typename Rep = double> using magnetic_flux_density = quantity<flux_density_dim, si, Rep>;
```

#### Per-Unit Equivalents
```cpp
template <typename Rep = double> using voltage_pu = quantity<voltage_dim, per_unit, Rep>;
template <typename Rep = double> using current_pu = quantity<current_dim, per_unit, Rep>;
template <typename Rep = double> using power_pu = quantity<power_dim, per_unit, Rep>;
template <typename Rep = double> using frequency_pu = quantity<frequency_dim, per_unit, Rep>;
template <typename Rep = double> using torque_pu = quantity<torque_dim, per_unit, Rep>;
// ... and more
```

### Safe System Conversion
Explicit conversion functions ensure safe transitions between unit systems:

```cpp
// Convert from Per-Unit to SI (requires base quantity)
auto v_pu = voltage_pu<double>(1.05);      // 1.05 pu
auto v_base = voltage<double>(400.0);      // 400 V base
auto v_si = to_si(v_pu, v_base);          // Result: 420 V

// Convert from SI to Per-Unit
auto v_actual = voltage<double>(380.0);    // 380 V actual
auto v_pu_result = to_pu(v_actual, v_base); // Result: 0.95 pu
```

### Comparison Operations
Standard comparison operators work with quantities of the same dimension and system:

```cpp
auto v1 = voltage<double>(120.0);
auto v2 = voltage<double>(240.0);

bool is_greater = (v2 > v1);    // true
bool is_equal = (v1 == v2);     // false
bool is_less_eq = (v1 <= v2);   // true
```

### Human-Readable Output
Quantities automatically format with appropriate unit symbols:

```cpp
auto v = voltage<double>(120.0);
auto i = current<double>(5.0);
auto p_pu = power_pu<double>(0.85);

std::cout << v << std::endl;     // Output: "120 V"
std::cout << i << std::endl;     // Output: "5 A"
std::cout << p_pu << std::endl;  // Output: "0.85 pu"
```

## 4. Usage Examples

### Basic Electrical Calculations

```cpp
// Ohm's Law: V = I × R
auto current_val = current<double>(10.0);         // 10 A
auto resistance_val = resistance<double>(12.0);   // 12 Ω
auto voltage_val = current_val * resistance_val;  // 120 V

// Power calculation: P = V × I
auto power_val = voltage_val * current_val;       // 1200 W
```

### Three-Phase Power Systems

```cpp
// Line voltage and current
auto v_line = voltage<double>(400.0);    // 400 V line voltage
auto i_line = current<double>(50.0);     // 50 A line current

// Three-phase power (simplified, without √3 factor in this example)
auto power_3ph = v_line * i_line * 3.0;  // Approximate 3-phase power
```

### Per-Unit System Usage

```cpp
// Base quantities for per-unit system
auto v_base = voltage<double>(400.0);    // 400 V base
auto p_base = power<double>(100000.0);   // 100 kW base

// Per-unit quantities
auto v_pu = voltage_pu<double>(1.05);    // 1.05 pu voltage
auto p_pu = power_pu<double>(0.80);      // 0.80 pu power

// Convert to SI for actual calculations
auto v_actual = to_si(v_pu, v_base);     // 420 V
auto p_actual = to_si(p_pu, p_base);     // 80 kW
```

### Mechanical Calculations

```cpp
// Rotational mechanics
auto inertia_val = moment_of_inertia<double>(2.5);  // 2.5 kg⋅m²
auto torque_val = torque<double>(100.0);            // 100 N⋅m

// Note: Angular acceleration would require additional dimension arithmetic
// This demonstrates the type safety of the system
```

## 5. Integration with Power Electronics Applications

### LCI (Load-Commutated Inverter) Applications
The units library is particularly valuable for LCI and thyristor-based power conversion:

```cpp
// DC link calculations
auto v_dc = voltage<double>(600.0);      // DC voltage
auto i_dc = current<double>(100.0);      // DC current
auto p_dc = v_dc * i_dc;                 // DC power

// Per-unit conversion for control systems
auto v_dc_base = voltage<double>(650.0);
auto v_dc_pu = to_pu(v_dc, v_dc_base);   // 0.923 pu
```

### Soft-Starter Applications
```cpp
// Motor nameplate data
auto v_rated = voltage<double>(400.0);    // Rated voltage
auto p_rated = power<double>(200000.0);   // Rated power (200 kW)
auto f_rated = frequency<double>(50.0);   // Rated frequency

// Soft-start calculations with reduced voltage
auto v_start = v_rated * 0.6;            // 60% voltage start
auto v_start_pu = to_pu(v_start, v_rated); // 0.6 pu
```

## 6. Performance Characteristics

### Zero Runtime Overhead
The dimensional analysis is performed entirely at compile time. Runtime operations are equivalent to operations on raw numeric types:

```cpp
// This quantity operation...
auto result = voltage_val * current_val;

// ...compiles to the same assembly as:
// double result = voltage_raw * current_raw;
```

### Memory Efficiency
A `quantity` object has the same memory footprint as its underlying numeric type:

```cpp
static_assert(sizeof(voltage<double>) == sizeof(double));    // Always true
static_assert(sizeof(power_pu<float>) == sizeof(float));    // Always true
```

### Compile-Time Validation
All dimensional checking happens during compilation, ensuring no runtime performance penalty for type safety.

## 7. Advanced Features

### Template Metaprogramming
The library uses sophisticated template metaprogramming for dimension arithmetic:

```cpp
// Automatic dimension calculation for complex expressions
auto energy = force<double>(100.0) * length<double>(50.0);  // 5000 J (energy_dim)
auto power_from_energy = energy / duration_type<double>(10.0); // 500 W (power_dim)
```

### Custom Representation Types
Users can specify different numeric types for precision control:

```cpp
auto precise_voltage = voltage<long double>(120.0L);  // High precision
auto fast_current = current<float>(5.0f);            // Fast computation
```

### Extensibility
New unit types can be easily added by defining new dimension combinations:

```cpp
// Example: Angular velocity (1/T dimension, same as frequency)
template <typename Rep = double>
using angular_velocity = quantity<frequency_dim, si, Rep>;
```

## 8. Error Prevention Examples

The library prevents common engineering mistakes at compile time:

```cpp
// These will cause compilation errors:
// auto invalid1 = voltage_val + current_val;        // Different dimensions
// auto invalid2 = voltage_val + voltage_pu_val;     // Different unit systems
// auto invalid3 = voltage_val == power_val;         // Incompatible comparison
```

## 9. Best Practices

### Consistent Unit System Usage
Always use consistent unit systems within calculation blocks:

```cpp
// Good: All SI units
auto v_si = voltage<double>(400.0);
auto i_si = current<double>(50.0);
auto p_si = v_si * i_si;

// Good: All per-unit
auto v_pu = voltage_pu<double>(1.0);
auto i_pu = current_pu<double>(1.0);
auto p_pu = v_pu * i_pu;
```

### Explicit Conversions
Always use explicit conversion functions when changing unit systems:

```cpp
auto v_base = voltage<double>(400.0);
auto v_pu = voltage_pu<double>(1.05);
auto v_si = to_si(v_pu, v_base);  // Explicit and safe
```

### Type Aliases for Domain-Specific Applications
Create meaningful type aliases for your specific application domain:

```cpp
using grid_voltage = voltage<double>;
using motor_current = current<double>;
using thyristor_firing_angle = angle<double>;  // If angle type exists
```

## 10. Compliance and Dependencies

- **C++17 Standard**: Full compliance with C++17 features including `constexpr`, `if constexpr`, and structured bindings support
- **No External Dependencies**: Uses only standard library components (`<iostream>`, `<string>`, `<type_traits>`)
- **Header-Only**: Complete implementation in a single header file for easy integration
- **Cross-Platform**: Works on all major compilers (GCC, Clang, MSVC) and platforms

## 11. Integration with YeeT Control Library

The units library integrates seamlessly with other YeeT control components:

### PI Controller Integration
```cpp
// PI controller with typed quantities
pi_controller<voltage<double>> voltage_controller;
auto voltage_error = voltage_ref - voltage_measured;
auto voltage_output = voltage_controller.update(voltage_error);
```

### Reference Frame Transformations
```cpp
// Three-phase voltage measurements with units
struct abc_voltages {
    voltage<double> va, vb, vc;
};

// Clarke transform with dimensional safety
auto alphabeta_result = clarke_transform(abc_voltages{v_a, v_b, v_c});
```

### LCI Pulse Generator
```cpp
// Firing angle control with type safety
auto firing_angle = angle<double>(15.0 * pi / 180.0);  // 15 degrees
auto commutation_voltage = voltage<double>(500.0);
// Pulse generator configuration...
```

This units library provides the foundation for type-safe, efficient engineering calculations throughout the YeeT control platform, particularly valuable for power electronics and motor control applications where unit errors can have significant safety and performance implications.