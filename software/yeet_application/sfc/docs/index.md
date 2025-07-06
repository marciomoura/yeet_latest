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

*Mathematical Foundation:*
- Base power: $S_{base} = S_{rated,M}$
- Base voltage: $V_{base} = V_{rated,LL,M}$
- Base DC voltage: $V_{d,base} = \frac{3\sqrt{2}}{\pi} V_{base}$

#### 🧮 [Steady-State Equations](steady_state_equations.md)
**Complete mathematical model for LCI operation**

Detailed mathematical framework covering all steady-state operating conditions for 6-pulse converter configurations:

- **DC Link Equations**: Voltage and current relationships between source and load
- **Commutation Analysis**: Overlap angles and timing constraints
- **Power Flow Calculations**: Active and reactive power in both directions
- **Operating Limits**: Physical constraints and safety margins

*Core Equations Include:*
- Rectifier voltage: $V_d = \frac{3\sqrt{2}}{\pi}V_{LL,S} \cos(\alpha_R) - \frac{3 \omega_S L_s}{\pi}I_d$
- Inverter voltage: $V_{di} = \frac{3\sqrt{2}}{\pi}V_{LL,M} \cos(\beta) + \frac{3 \omega_M L_m}{\pi}I_d$
- DC current: $I_d = \frac{V_d - V_{di}}{R_d}$

## 🎯 Quick Navigation

### For New Users
1. **Start with** → [Dictionary](dictionary.md) to understand terminology
2. **Learn fundamentals** → [Per-Unit System](per_unit_system.md) for analysis framework
3. **Apply mathematics** → [Steady-State Equations](steady_state_equations.md) for calculations

### For Experienced Engineers
- **Variable Reference** → [Dictionary](dictionary.md) for programming variable names
- **System Design** → [Per-Unit System](per_unit_system.md) for scaling and comparison
- **Analysis Tools** → [Steady-State Equations](steady_state_equations.md) for performance evaluation

### For Software Developers
- **Naming Conventions** → [Dictionary](dictionary.md) for consistent variable naming
- **Normalization** → [Per-Unit System](per_unit_system.md) for scalable implementations
- **Mathematical Models** → [Steady-State Equations](steady_state_equations.md) for algorithm development

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
