# YEET SFC Documentation Index

Welcome to the **YEET Static Frequency Converter (SFC)** documentation hub. This documentation provides comprehensive technical references for Load-Commutated Inverter (LCI) drive systems used in high-power synchronous machine applications.

## 📚 Documentation Overview

This documentation suite covers the theoretical foundations, mathematical models, and implementation details of LCI drive systems designed for gas turbine starting and helper motor applications.

### Core Documentation Components

#### 🔤 [Dictionary of LCI System Terms](dictionary.md)
**Essential terminology and variable naming conventions**

A comprehensive glossary that establishes the standardized naming convention of **`location_quantity_unit`** or **`location_component_attribute`** used throughout the YEET platform. This dictionary covers:

- **Grid-Side Components**: Source-side converter elements including grid voltage, current, firing angles, and commutating inductance
- **DC Link Components**: Intermediate stage elements including DC inductor, current, voltage references, and resistance
- **Machine-Side Components**: Load-side converter elements including machine voltage, current, firing angles, and extinction angles

*Key Features:*
- Programming variable names mapped to LaTeX symbols
- Detailed descriptions of each component's role in the system
- Clear distinction between rectifier and inverter operation modes

#### ⚖️ [Per-Unit System Definitions](per_unit_system.md)
**Normalized analysis framework for multi-power systems**

Comprehensive per-unit system implementation that normalizes variables across different voltage and power ratings. This system provides:

- **Base Quantity Definitions**: AC and DC side base values derived from synchronous machine ratings
- **Power Invariance**: Ensures consistent power calculations between AC and DC domains
- **Simplified Analysis**: Eliminates constants like π, √2, and √3 from equations
- **System Comparison**: Enables direct comparison between different power ratings

#### 🧮 [Steady-State Equations](steady_state_equations.md)
**Complete mathematical model for LCI operation**

Detailed mathematical framework covering all steady-state operating conditions for 6-pulse converter configurations:

- **DC Link Equations**: Voltage and current relationships between source and load
- **Commutation Analysis**: Overlap angles and timing constraints
- **Power Flow Calculations**: Active and reactive power in both directions
- **Operating Limits**: Physical constraints and safety margins

#### ⛓️ [Filter Chain and Observers](filter_chain.md)
**Signal processing and observer design for control stability**

A guide to the digital filtering chains and state observers required for stable and high-performance LCI operation. This section details:

- **Filtering strategies** for grid voltages, machine voltages, and DC link current
- **Implementation of Phase-Locked Loops (PLLs)** for grid synchronization
- **Rotor position and speed estimation techniques** for sensor and sensorless control
- **Specialized solutions** for Gas Turbine Starter (GTS) startup and wide-speed operation

#### 📈 [Average-Value Model](average_model.md)
**Dynamic plant model for controller design and tuning**

A guide to developing an average-value model for the LCI drive. This model is essential for simulating the system's dynamic behavior and tuning the PI controllers for both the inner current loop and the outer speed loop.

#### ⚡ [Current Controller](current_control.md)
**Inner loop for direct torque control**

Detailed guide on the high-speed current controller responsible for regulating DC link current and motor torque. Covers PI tuning, feedforward control, and linearization.

#### 🏃 [Speed Controller](speed_control.md)
**Outer loop for soft-starter applications**

Comprehensive explanation of the speed control loop, including its role in the cascaded architecture, PI controller design, anti-windup mechanisms, and speed profile generation for soft-starters.

## 🎯 Quick Navigation

### For New Users
1. **Start with** → [Dictionary](dictionary.md) to understand terminology
2. **Learn fundamentals** → [Per-Unit System](per_unit_system.md) for analysis framework
3. **Apply mathematics** → [Steady-State Equations](steady_state_equations.md) for calculations
4. **Understand signal processing** → [Filter Chain and Observers](filter_chain.md) for control implementation

### For Experienced Engineers
- **Variable Reference** → [Dictionary](dictionary.md) for programming variable names
- **System Design** → [Per-Unit System](per_unit_system.md) for scaling and comparison
- **Analysis Tools** → [Steady-State Equations](steady_state_equations.md) for performance evaluation
- **Control Implementation** → [Filter Chain and Observers](filter_chain.md) for advanced control topics

### For Software Developers
- **Naming Conventions** → [Dictionary](dictionary.md) for consistent variable naming
- **Normalization** → [Per-Unit System](per_unit_system.md) for scalable implementations
- **Mathematical Models** → [Steady-State Equations](steady_state_equations.md) for algorithm development
- **Control Algorithms** → [Filter Chain and Observers](filter_chain.md) for real-time processing

## 🔧 Implementation Context

This documentation supports the YEET platform's implementation of:

- **Control Systems**: Inner current loops and outer speed/torque control
- **Protection Systems**: Overcurrent, overvoltage, and extinction angle monitoring
- **Simulation Models**: High-fidelity dynamic and steady-state models
- **Testing Frameworks**: Closed-loop validation and commissioning tools

## 📊 System Applications

The documented LCI systems are designed for:

- **Gas Turbine Starting**: Variable frequency drive for turbine startup sequences
- **Helper Motor Operation**: Continuous torque assistance during operation
- **Large Synchronous Machines**: High-power motor and generator control
- **Industrial Processes**: Precise speed and torque control applications

## 🚀 Getting Started

1. **Review** the main [YEET README](../../../../README.md) for platform setup
2. **Understand** the system terminology using the [Dictionary](dictionary.md)
3. **Learn** the analysis framework with [Per-Unit System](per_unit_system.md)
4. **Apply** the mathematical models from [Steady-State Equations](steady_state_equations.md)
5. **Implement** control logic with guidance from the [Filter Chain and Observers](filter_chain.md)

---

## 📝 Documentation Standards

All documentation follows consistent conventions:
- **Variable Names**: Programming variables use `snake_case`
- **Mathematical Symbols**: LaTeX notation for equations
- **Units**: Clearly specified for all physical quantities
- **Cross-References**: Linked between related concepts

For questions, contributions, or suggestions regarding this documentation, please refer to the main YEET platform repository.

---

*Last Updated: January 2025*  
*Documentation Version: 1.0*  
*Compatible with: YEET Platform v1.x*
