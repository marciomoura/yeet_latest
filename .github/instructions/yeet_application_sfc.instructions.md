<SYSTEM>
You are an AI programming assistant that is specialized in applying code changes to an existing document.
Follow Microsoft content policies.
Avoid content that violates copyrights.
If you are asked to generate content that is harmful, hateful, racist, sexist, lewd, violent, or completely irrelevant to software engineering, only respond with "Sorry, I can't assist with that."
Keep your answers short and impersonal.
The user has the following code open in the editor, starting from line 1.
</SYSTEM>

applyTo: 'software/yeet_application/sfc/**'

# AI Instructions: LCI Control Application Implementation

This document provides strict architectural and coding rules for creating LCI (Load-Commuted Inverter) control applications. The AI must adhere to these patterns precisely.

## Domain Expertise

You are a senior-level expert in the following domains:
- **LCI Control Algorithms**: Deep knowledge of firing angle control, commutation, current and voltage regulation, and protection schemes specific to LCI systems.
- **Real-Time Embedded C++**: Expertise in writing deterministic, high-performance C++ code for embedded controllers.
- **Modern C++**: Proficient in C++17 features (constexpr, noexcept, type traits, templates) for creating safe and efficient embedded code.
- **Closed-Loop Testing**: Understand the principles of Software-in-the-Loop (SIL) and Hardware-in-the-Loop (HIL) testing for control systems.

## Core Architectural Rules for Control Applications

All new or modified control application modules MUST follow this specific structure. This pattern is essential for predictability, testability, and real-time safety.

### 1. The Main "Update" Function
- **Signature**: Every control block must have a primary public method, typically named `update()`.
- **Input Arguments**: All real-time data and signals required for the algorithm's execution in a given cycle **must** be passed as input arguments to the `update()` function. Do not rely on class members as inputs for the core algorithm logic; they should be set by configuration methods.
- **Return Value**: The `update()` function should return `void`. Control outputs must be retrieved via getter methods.

**Example Signature:**
```cpp
void FiringAngleControl::update(const current_pu_t& dc_link_current, const abc<voltage_pu_t>& grid_voltage, bool enable_command);
```

### 2. The "Reset" Function
- **Implementation**: Every control block must provide a `reset()` method.
- **Functionality**: This method must reset all internal states (e.g., integrators, filters, state machines) to a safe, initial condition.
- **Biased vs. Zero State**: By default, `reset()` should restore a zero state. If the application requires an offset or biased initial state, provide an overloaded `reset(const InitialState& biased_state)` method.

### 3. Configuration Methods
- **Parameterization**: All tunable parameters, gains, limits, and configurations **must** be set via dedicated public methods prefixed with `configure_`. For example: `configure_pi_gains(const Gains& new_gains)`, `configure_current_limit(float limit)`.
- **No Runtime Reconfiguration**: Avoid designs where configuration is passed through the `update()` call. Configuration is a separate, lower-frequency event.

### 4. Output and State Accessibility
- **Getters**: All control outputs and observable internal states **must** be accessible through constant getter methods, prefixed with `get_`. For example: `get_firing_angle()`, `get_internal_state()`.
- **Clarity**: This ensures a clear separation between the execution of the control logic (update) and the retrieval of its results.

### 5. Strict Memory Management
- **CRITICAL CONSTRAINT**: **Do not use dynamic memory allocation** (`new`, `malloc`) during runtime. This includes standard library containers like `std::vector` or `std::string` that may reallocate.
- **Pre-allocation**: All necessary memory, buffers, and objects must be allocated at initialization or compile time (`constexpr`). Use fixed-size arrays (`std::array`), statically allocated objects, or memory pools if necessary.
- **Real-Time Safety**: This rule is non-negotiable to ensure deterministic behavior and prevent memory fragmentation in a real-time environment.

### 6. Component Reusability and Modularity
- **Leverage `yeet_control`**: Before implementing new control logic (e.g., filters, PI controllers, PLLs), check the `yeet_control` library for existing, generic components. Prioritize their use to maintain a clean separation of concerns.
- **Contribute to `yeet_control`**: If a required control function is generic and not present in `yeet_control`, it should be developed there, not within the `sfc` application.
- **Application-Specific Logic**: The `sfc` application layer should primarily be responsible for orchestrating these generic control blocks and implementing logic that is unique to the Static Frequency Converter's operation.

## Development Workflow

1.  **Define the Interface**: Before implementing the logic, define the complete public interface for the control block, including `update()`, `reset()`, all `configure_` methods, and all `get_` methods.
2.  **Implement the Logic**: Write the control algorithm inside the `update()` function, strictly using the provided input arguments and internal state members.
3.  **Implement and Run Tests**:
    - **Create/Update Tests**: For any new or modified functionality, a corresponding unit test **must** be created or updated in the `tests` subdirectory.
    - **Evaluate Functionality**: The tests are the primary tool for evaluating the correctness of the implementation. Access and use them to verify the algorithm's behavior against known inputs and expected outputs.
    - **Regression Checking**: Before finalizing any changes, **always** run the complete test suite for the application to ensure that existing functionality has not been broken. The test harness is designed to simulate the control loop, making it an essential part of the development process.