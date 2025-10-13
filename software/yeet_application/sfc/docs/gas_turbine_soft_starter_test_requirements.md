# Gas Turbine Soft-Starter Test Requirements

## Document Overview

This document defines the comprehensive test requirements for validating the Static Frequency Converter (SFC) soft-starter application for gas turbine starting. The requirements are divided into two main categories:

1. **Functional Requirements**: System behavior and operational sequence validation
2. **Performance Requirements**: Control loop performance and dynamic response metrics

These requirements serve as the foundation for test case development and system validation.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Functional Requirements](#functional-requirements)
3. [Performance Requirements](#performance-requirements)
4. [Test Methodology](#test-methodology)
5. [Acceptance Criteria](#acceptance-criteria)

---

## System Overview

### System Architecture

The SFC soft-starter system consists of:

- **Speed Generator**: Profile-based speed reference generation with four configurable acceleration profiles
- **Speed Controller**: Outer control loop (10ms sampling) regulating motor speed
- **Torque Controller**: Middle control loop (10ms sampling) with dynamic torque limiting and breakaway boost
- **Current Controller**: Inner control loop (1ms sampling) regulating DC link current
- **Machine-Side Controller**: Inverter firing angle control for torque production
- **Grid-Side Controller**: Rectifier firing angle control for DC link voltage/current regulation
- **State Machine**: System sequencing and interlock management
- **Excitation Controller**: Synchronous motor field excitation regulation

### Control Hierarchy

```
State Machine (System Sequencing)
    ↓
Speed Generator (Profile Execution)
    ↓
Speed Controller (Outer Loop - 10ms)
    ↓
Torque Controller (Torque Limiting & Boost - 10ms)
    ↓
Current Controller (Inner Loop - 1ms)
    ↓
Firing Angle Controllers (200µs)
```

### Gas Turbine Starting Sequence

A typical gas turbine starting sequence consists of:

1. **Creep Speed** (~5% rated): Initial rotation for inspection, oil circulation
2. **Purge Acceleration** (~20% rated): Chamber ventilation to clear residual fuel vapors
3. **Ignition Acceleration** (~55% rated): Fuel ignition and flame stabilization
4. **Assist Acceleration** (~95% rated): Acceleration to self-sustaining speed

---

## Functional Requirements

### FR-1: Speed Generator Profile Configuration

**Requirement**: The speed generator shall support four independently configurable acceleration profiles.

**Sub-requirements**:
- **FR-1.1**: Each profile shall have configurable target speed (0-120% pu)
- **FR-1.2**: Each profile shall have configurable acceleration time (time to reach target from 0%)
- **FR-1.3**: Each profile shall have independent torque limits (max motoring, min braking)
- **FR-1.4**: Each profile shall support configurable breakaway boost torque and duration
- **FR-1.5**: Profiles can be individually enabled/disabled without requiring recompilation

**Test Strategy**: 
- Configure profiles with factory methods (`make_creep_speed_profile`, `make_purge_accel_profile`, etc.)
- Verify each profile reaches its target speed within specified time ±5%
- Validate torque limits are correctly applied during profile execution
- Confirm breakaway boost activates at profile start and deactivates after specified duration

**Acceptance Criteria**:
- Target speed reached: ±2% of configured value
- Acceleration time: ±5% of configured value
- Torque limits respected: No violations during steady-state operation
- Boost timing: Activates within 1 cycle (10ms), deactivates within ±50ms of configured duration

---

### FR-2: Profile Execution Commands

**Requirement**: The system shall respond correctly to all speed generator commands.

**Command Types**:
- `IDLE`: System inactive, zero torque/speed outputs
- `HOLD_SPEED`: Maintain current speed as setpoint
- `DECELERATE`: Controlled ramp to zero speed
- `RUN_PROFILE`: Execute selected acceleration profile

**Sub-requirements**:
- **FR-2.1**: Only one command shall be active at any time (no conflicting commands)
- **FR-2.2**: Command transitions shall be smooth without torque discontinuities
- **FR-2.3**: `IDLE` command shall immediately set torque reference to zero
- **FR-2.4**: `HOLD_SPEED` shall capture current speed and maintain it within ±3% error
- **FR-2.5**: `DECELERATE` shall ramp to zero at configured deceleration rate
- **FR-2.6**: `RUN_PROFILE` shall execute only if profile is enabled and configured

**Test Strategy**:
- Test each command individually in isolation
- Test command sequences with transitions (e.g., IDLE → RUN_PROFILE → HOLD_SPEED → DECELERATE → IDLE)
- Verify rejection of conflicting simultaneous commands
- Measure torque discontinuities at command transitions (shall be < 10% pu step)

**Acceptance Criteria**:
- Command response time: ≤ 20ms (2 cycles at 10ms sampling)
- Speed hold accuracy: ±3% of setpoint during steady-state
- Deceleration rate: ±10% of configured rate
- Torque discontinuity at transitions: < 10% pu instantaneous step

---

### FR-3: Profile Sequencing

**Requirement**: The system shall execute complete gas turbine starting sequences from standstill to self-sustaining operation.

**Standard Sequence**:
1. **Initial State**: Motor at rest (< 1% speed)
2. **Creep Profile**: Accelerate to ~5% for initial rotation
3. **Purge Profile**: Accelerate to ~20% for chamber purge (typically 2-5 minutes hold)
4. **Ignition Profile**: Accelerate to ~55% for fuel ignition
5. **Assist Profile**: Accelerate to ~95% for self-sustaining operation
6. **Handoff**: Transfer control to turbine (synchronous speed operation)

**Sub-requirements**:
- **FR-3.1**: Profiles shall execute sequentially without manual intervention
- **FR-3.2**: System shall detect profile completion (target speed reached and stable)
- **FR-3.3**: Hold periods between profiles shall maintain speed within ±3%
- **FR-3.4**: Profile transitions shall be bump-less (no torque or speed discontinuities)
- **FR-3.5**: Sequence shall be abortable at any point via `DECELERATE` or `IDLE` command

**Test Strategy**:
- Execute full 4-profile sequence in simulation
- Measure time to complete each profile
- Validate profile completion detection accuracy
- Verify speed stability during hold periods
- Test emergency stop (`DECELERATE`) from each profile

**Acceptance Criteria**:
- Total sequence time: Within ±10% of sum of profile acceleration times
- Profile completion detection: < 1 second delay after reaching target speed
- Speed stability during holds: ±3% of target speed
- Emergency stop response: Deceleration begins within 20ms of command

---

### FR-4: State Machine Integration

**Requirement**: The speed generator and control loops shall integrate correctly with the SFC state machine.

**State Machine States** (relevant to soft-start):
- `IDLE`: System inactive, no control outputs
- `AUXILIARIES_ON`: Pre-energization checks
- `BREAKERS_ON`: Main power available, ready for operation
- `OPERATION_ON`: Active control enabled, pulse releases active
- `SHUTDOWN_REQUESTED`: Controlled shutdown in progress
- `OPERATION_OFF`: Waiting for current extinguishment

**Sub-requirements**:
- **FR-4.1**: Speed generator shall only accept commands when state machine is in `OPERATION_ON`
- **FR-4.2**: Pulse releases shall be active throughout profile execution
- **FR-4.3**: Transition to `SHUTDOWN_REQUESTED` shall trigger automatic `DECELERATE` command
- **FR-4.4**: `OPERATION_OFF` state shall force `IDLE` command regardless of external inputs
- **FR-4.5**: State machine faults shall immediately disable all control outputs

**Test Strategy**:
- Start profile execution in each state machine state
- Verify pulse releases remain active during profiles
- Test shutdown request during active acceleration
- Simulate fault conditions and verify safe response

**Acceptance Criteria**:
- Commands rejected outside `OPERATION_ON`: 100% rejection rate
- Pulse releases active: Continuous during all profile execution phases
- Shutdown response time: < 20ms to start deceleration
- Fault response time: < 1ms to disable outputs (safety-critical)

---

### FR-5: Breakaway Boost Management

**Requirement**: The system shall apply breakaway torque boost to overcome static friction at low speeds.

**Sub-requirements**:
- **FR-5.1**: Boost shall activate at the start of acceleration profiles (0-5% speed region)
- **FR-5.2**: Boost magnitude shall be configurable per profile (0-100% pu additional torque)
- **FR-5.3**: Boost duration shall be configurable per profile (typically 0.5-3 seconds)
- **FR-5.4**: Boost shall deactivate smoothly to prevent torque shock
- **FR-5.5**: Boost shall not activate during `HOLD_SPEED` or `DECELERATE` commands

**Test Strategy**:
- Configure Profile 1 (Creep) with high boost (~50% pu), longer duration (~2s)
- Configure Profile 4 (Assist) with low boost (~10% pu), shorter duration (~0.5s)
- Measure torque reference during boost activation and deactivation
- Verify boost timing accuracy with elapsed timer

**Acceptance Criteria**:
- Boost activation delay: < 10ms after profile start
- Boost magnitude accuracy: ±5% of configured value
- Boost duration accuracy: ±50ms of configured duration
- Torque rate of change at boost end: < 50% pu/second (smooth transition)

---

### FR-6: Dynamic Torque Limiting

**Requirement**: Torque limits shall adjust dynamically based on active profile and operating conditions.

**Sub-requirements**:
- **FR-6.1**: Each profile shall enforce independent maximum motoring torque limit
- **FR-6.2**: Each profile shall enforce independent minimum braking torque limit (for deceleration)
- **FR-6.3**: Torque limits shall update within 1 control cycle (10ms) of profile change
- **FR-6.4**: Torque controller shall respect limits without overshoot
- **FR-6.5**: Absolute system limits shall override profile limits (safety bounds)

**Test Strategy**:
- Configure profiles with distinct torque limits (e.g., Profile 1: ±0.8 pu, Profile 4: ±1.2 pu)
- Apply large speed error to force torque saturation
- Verify torque reference stays within configured limits
- Test limit transitions during profile changes

**Acceptance Criteria**:
- Limit update latency: ≤ 10ms (1 control cycle)
- Limit overshoot: 0% (hard constraint, no violations)
- Limit transition smoothness: < 20% pu/second rate of change

---

### FR-7: Deceleration and Emergency Stop

**Requirement**: The system shall support controlled deceleration and emergency stop scenarios.

**Sub-requirements**:
- **FR-7.1**: `DECELERATE` command shall ramp speed to zero at configured deceleration rate
- **FR-7.2**: Deceleration rate shall be configurable (typically 2-10 seconds to full stop from 100% speed)
- **FR-7.3**: Emergency stop (state machine fault) shall trigger fastest safe deceleration
- **FR-7.4**: Deceleration shall use braking torque limits from active profile
- **FR-7.5**: System shall detect zero-speed arrival and transition to `IDLE` automatically

**Test Strategy**:
- Issue `DECELERATE` command from multiple speed points (20%, 50%, 95%)
- Measure deceleration time and linearity
- Simulate emergency stop and measure response time
- Verify automatic idle transition at zero speed

**Acceptance Criteria**:
- Deceleration linearity: R² > 0.95 (linear ramp)
- Deceleration time accuracy: ±10% of configured time
- Emergency stop response: < 10ms to maximum braking torque
- Zero-speed detection: < 100ms after crossing 0.5% speed threshold

---

## Performance Requirements

### PR-1: Speed Controller Performance

**Requirement**: The speed controller shall maintain stable, accurate speed regulation across all operating points.

**Metrics**:
- **Settling Time**: Time to reach ±2% of final value after step reference change
- **Overshoot**: Maximum overshoot beyond reference during transient
- **Steady-State Error**: Error magnitude after settling, under constant load
- **Disturbance Rejection**: Speed deviation from load torque step

**Sub-requirements**:
- **PR-1.1**: Step response settling time: < 3 seconds (for 10% speed step at 50% speed operating point)
- **PR-1.2**: Step response overshoot: < 10% of step magnitude
- **PR-1.3**: Steady-state error with constant load: < 1% of reference speed
- **PR-1.4**: Load torque step rejection (20% torque step): Speed deviation < 5%, recovery < 2 seconds

**Test Strategy**:
- Apply step speed reference changes at various operating points (10%, 30%, 50%, 80% speed)
- Apply load torque steps (10%, 20%, 30% pu) at constant speed
- Measure settling time using ±2% band criterion
- Measure overshoot as percentage of step magnitude
- Measure steady-state error after 5-second settling period

**Acceptance Criteria**:
- Settling time: < 3s for 10% speed steps
- Overshoot: < 10% at all operating points
- Steady-state error: < 1% pu absolute
- Load rejection: Speed deviation < 5%, recovery < 2s

**Test Conditions**:
- Load inertia: 1.0 pu (standard gas turbine inertia)
- Load torque: Variable 0-50% pu
- Operating speed range: 5-100% rated speed

---

### PR-2: Torque Controller Performance

**Requirement**: The torque controller shall enforce dynamic torque limits and apply breakaway boost correctly.

**Metrics**:
- **Limit Enforcement Accuracy**: Deviation of torque reference from configured limits
- **Boost Application Speed**: Latency from profile start to boost active
- **Boost Magnitude Accuracy**: Difference between configured and applied boost
- **Limit Transition Smoothness**: Rate of change during limit updates

**Sub-requirements**:
- **PR-2.1**: Torque limit violations: 0 occurrences (hard constraint)
- **PR-2.2**: Boost activation latency: < 10ms (1 control cycle at 10ms)
- **PR-2.3**: Boost magnitude accuracy: ±5% of configured value
- **PR-2.4**: Limit transition rate: < 100% pu/second (prevents actuator shock)

**Test Strategy**:
- Configure profiles with different torque limits and boost values
- Apply large speed errors to saturate torque controller
- Log all torque references and verify no limit violations
- Measure boost activation timing and magnitude
- Analyze limit transition derivatives

**Acceptance Criteria**:
- Zero limit violations throughout all test scenarios
- Boost activation within 10ms in 100% of profile starts
- Boost magnitude within ±5% tolerance
- Limit transition rate never exceeds 100% pu/s

---

### PR-3: Current Controller Performance

**Requirement**: The current controller (inner loop) shall track torque references accurately and rapidly.

**Metrics**:
- **Bandwidth**: Frequency at which gain drops to -3dB
- **Step Response Time**: Time to reach 90% of step reference
- **Tracking Error**: Steady-state deviation from reference
- **Stability Margins**: Gain margin and phase margin

**Sub-requirements**:
- **PR-3.1**: Closed-loop bandwidth: > 50 Hz (significantly faster than 10ms speed loop)
- **PR-3.2**: Step response 90% rise time: < 10ms
- **PR-3.3**: Steady-state tracking error: < 2% of reference current
- **PR-3.4**: Gain margin: > 6 dB, Phase margin: > 30°

**Test Strategy**:
- Apply current reference steps at various operating points
- Measure rise time (10-90% of final value)
- Measure steady-state error after settling
- Perform frequency response analysis (Bode plot) to extract bandwidth and margins

**Acceptance Criteria**:
- Bandwidth > 50 Hz at 50% DC link voltage operating point
- Rise time < 10ms for 20% current steps
- Tracking error < 2% pu
- Stability margins meet minimum thresholds (GM > 6dB, PM > 30°)

**Test Conditions**:
- DC link voltage range: 30-100% rated
- Current reference range: 0-100% rated current
- Grid voltage: Balanced, 100% rated

---

### PR-4: Speed Tracking During Acceleration Profiles

**Requirement**: The system shall accurately track profile-generated speed references during acceleration.

**Metrics**:
- **Tracking Error**: Deviation between reference and actual speed during ramp
- **Lag Time**: Phase delay between reference and actual speed
- **Tracking Accuracy**: RMS error over entire profile duration
- **Terminal Accuracy**: Error at profile completion (target speed reached)

**Sub-requirements**:
- **PR-4.1**: Maximum tracking error during ramp: < 7% of current reference speed
- **PR-4.2**: RMS tracking error over profile: < 3% of target speed
- **PR-4.3**: Terminal accuracy (at target speed): < 2% of target speed
- **PR-4.4**: Lag time during constant acceleration: < 0.5 seconds

**Test Strategy**:
- Execute each profile (Creep, Purge, Ignition, Assist) with nominal load
- Log reference speed and actual speed at 100 Hz
- Calculate instantaneous tracking error throughout profile
- Calculate RMS error over entire acceleration phase
- Measure lag time using cross-correlation

**Acceptance Criteria**:
- Tracking error peak < 7% in all profiles
- RMS error < 3% pu for each profile
- Terminal accuracy < 2% pu at profile end
- Lag time < 0.5s during linear acceleration

**Test Conditions**:
- Load torque: 20-40% pu (representative gas turbine load curve)
- Acceleration times: Per profile defaults (Creep: 60s, Purge: 120s, Ignition: 90s, Assist: 60s)
- Initial speed: < 1% pu

---

### PR-5: Low-Speed (Pulse Mode) Operation

**Requirement**: The system shall maintain stable operation in the forced commutation (pulse mode) region at very low speeds.

**Operating Region**: 0-20% rated speed (where natural commutation is unreliable)

**Metrics**:
- **Speed Ripple**: Peak-to-peak speed variation at steady state
- **Torque Pulsations**: Peak-to-peak torque variation at steady state
- **Minimum Stable Speed**: Lowest speed at which closed-loop control is effective
- **Stability**: Time-domain stability over extended low-speed operation

**Sub-requirements**:
- **PR-5.1**: Speed ripple at 5% speed: < 20% peak-to-peak of reference
- **PR-5.2**: Torque pulsations at 5% speed: < 40% peak-to-peak of average torque
- **PR-5.3**: Minimum stable speed: < 2% rated speed
- **PR-5.4**: Continuous low-speed operation: > 60 seconds without instability

**Test Strategy**:
- Hold constant speed references at 2%, 5%, 10%, 15% rated speed
- Apply constant load torque (10-20% pu)
- Measure speed and torque ripple using peak-to-peak and RMS metrics
- Gradually reduce speed reference to find minimum stable operating point
- Run extended low-speed tests (60+ seconds) to verify stability

**Acceptance Criteria**:
- Speed ripple < 20% p-p at 5% speed
- Torque pulsations < 40% p-p at 5% speed
- Minimum stable speed < 2% pu
- No growing oscillations or instability over 60 seconds at 5% speed

**Test Conditions**:
- Pulse mode active (forced commutation logic enabled)
- Load torque: 10-20% pu constant
- Grid voltage: Balanced, 100% rated

---

### PR-6: Multi-Profile Sequence Performance

**Requirement**: The system shall execute complete 4-profile sequences with consistent performance throughout.

**Metrics**:
- **Total Sequence Time**: Time from start to reaching final target speed (95% pu)
- **Inter-Profile Transition Smoothness**: Torque and speed discontinuities at profile changes
- **Cumulative Tracking Error**: Integrated absolute error over entire sequence
- **Energy Efficiency**: Total energy consumed vs. theoretical minimum

**Sub-requirements**:
- **PR-6.1**: Total sequence time: Within ±10% of sum of profile times
- **PR-6.2**: Speed discontinuity at profile transitions: < 3% pu step
- **PR-6.3**: Torque discontinuity at profile transitions: < 10% pu step
- **PR-6.4**: Cumulative absolute error: < 5% of total distance traveled

**Test Strategy**:
- Execute complete sequence: IDLE → Creep → Purge → Ignition → Assist → HOLD
- Include hold periods between profiles (e.g., 30s purge hold at 20% speed)
- Measure total elapsed time
- Detect and quantify discontinuities at transitions
- Integrate absolute tracking error over entire sequence

**Acceptance Criteria**:
- Sequence time within ±10% of expected (nominal ~330s with holds)
- Speed discontinuities < 3% pu at all 4 transitions
- Torque discontinuities < 10% pu at all 4 transitions
- Cumulative error < 5% of total speed integral

---

### PR-7: Load Disturbance Rejection

**Requirement**: The control system shall maintain speed regulation during realistic load variations representative of gas turbine operation.

**Disturbance Scenarios**:
1. **Step Load Change**: Sudden torque change (e.g., 20% step increase)
2. **Ramp Load Change**: Gradual torque ramp (e.g., 0→30% over 10 seconds)
3. **Oscillating Load**: Periodic torque variation (e.g., ±10% at 1 Hz)

**Sub-requirements**:
- **PR-7.1**: Step load (20% pu): Speed deviation < 5%, recovery time < 2s
- **PR-7.2**: Ramp load (0→30% over 10s): Speed tracking error < 3% throughout ramp
- **PR-7.3**: Oscillating load (±10% at 1 Hz): Speed ripple < 2% pu RMS

**Test Strategy**:
- Apply load disturbances at multiple operating speeds (20%, 50%, 80% rated)
- Hold speed reference constant during disturbance application
- Measure speed deviation magnitude and recovery time
- Use FFT analysis for oscillating load to quantify speed ripple

**Acceptance Criteria**:
- Step load recovery: Speed returns to ±2% of setpoint within 2 seconds
- Ramp load tracking: Speed error < 3% throughout 10-second ramp
- Oscillating load attenuation: Speed ripple < 2% pu RMS (significant attenuation)

**Test Conditions**:
- Speed setpoints: 20%, 50%, 80% rated
- Load torque baseline: 20% pu
- Disturbance magnitudes: Steps: ±20% pu, Ramps: 0-30% pu, Oscillations: ±10% pu

---

## Test Methodology

### Simulation Environment

All tests shall be conducted in the closed-loop SFC simulation environment with:
- **Plant Model**: Full electrical and mechanical dynamics of motor and turbine
- **Sampling Rates**: 
  - Control loops: 10ms (speed, torque), 1ms (current), 200µs (firing angles)
  - Simulation step: Variable (solver adaptive), data logging: 100µs
- **Numerical Integration**: Runge-Kutta 4th order or equivalent
- **Realistic Parasitics**: Commutation overlap, thyristor delays, measurement noise

### Test Execution

Each test case shall:
1. **Initialize**: Configure system, plant, and profiles
2. **Pre-condition**: Execute state machine startup sequence
3. **Execute**: Run test scenario with defined inputs
4. **Measure**: Log all relevant signals at appropriate sampling rates
5. **Analyze**: Calculate metrics and compare to acceptance criteria
6. **Validate**: Pass/fail determination based on requirements

### Data Collection

Minimum data set for all tests:
- Speed reference and actual speed (100 Hz logging)
- Torque reference and actual torque (100 Hz logging)
- Current reference and DC link current (1 kHz logging)
- Active profile and command state (on change)
- Torque limit values (on change)
- Breakaway boost status (on change)
- State machine state (on change)

### Pass/Fail Criteria

- **Functional Requirements**: Binary pass/fail (requirement met or not met)
- **Performance Requirements**: Statistical evaluation with tolerances
  - Primary criteria: Must meet specification
  - Secondary criteria: Should meet specification (warning if not met)
  - Tertiary criteria: Informational only

---

## Acceptance Criteria Summary

### Functional Tests (Must Pass 100%)

| Requirement           | Metric                                 | Target    | Critical |
| --------------------- | -------------------------------------- | --------- | -------- |
| FR-1: Profile Config  | Target speed accuracy                  | ±2% pu    | Yes      |
| FR-2: Commands        | Response time                          | ≤20ms     | Yes      |
| FR-3: Sequencing      | Completion detection                   | <1s delay | No       |
| FR-4: State Machine   | Command rejection outside OPERATION_ON | 100%      | Yes      |
| FR-5: Breakaway Boost | Activation latency                     | <10ms     | No       |
| FR-6: Torque Limiting | Limit violations                       | 0         | Yes      |
| FR-7: Deceleration    | Emergency stop response                | <10ms     | Yes      |

### Performance Tests (Statistical Validation)

| Requirement           | Metric             | Target   | Tolerance |
| --------------------- | ------------------ | -------- | --------- |
| PR-1: Speed Control   | Settling time      | <3s      | ±10%      |
| PR-1: Speed Control   | Overshoot          | <10%     | ±2%       |
| PR-1: Speed Control   | Steady-state error | <1% pu   | ±0.2%     |
| PR-2: Torque Control  | Limit violations   | 0        | None      |
| PR-3: Current Control | Bandwidth          | >50Hz    | -5Hz      |
| PR-4: Speed Tracking  | RMS error          | <3% pu   | ±0.5%     |
| PR-5: Low-Speed       | Speed ripple       | <20% p-p | ±5%       |
| PR-6: Multi-Profile   | Sequence time      | ±10%     | ±2%       |
| PR-7: Disturbance     | Recovery time      | <2s      | ±0.5s     |

---

## Test Implementation Notes

### Test File Organization

```
tests/
├── functional/
│   ├── profile_configuration_test.cpp
│   ├── command_execution_test.cpp
│   ├── profile_sequencing_test.cpp
│   ├── state_machine_integration_test.cpp
│   ├── breakaway_boost_test.cpp
│   ├── torque_limiting_test.cpp
│   └── deceleration_test.cpp
├── performance/
│   ├── speed_controller_performance_test.cpp
│   ├── torque_controller_performance_test.cpp
│   ├── current_controller_performance_test.cpp
│   ├── speed_tracking_test.cpp
│   ├── low_speed_operation_test.cpp
│   ├── multi_profile_sequence_test.cpp
│   └── disturbance_rejection_test.cpp
└── integration/
    └── gas_turbine_soft_starter_full_sequence_test.cpp
```

### Naming Conventions

- **Test Class**: `GasTurbineSoftStarterTest` (base fixture)
- **Test Cases**: `TEST_F(GasTurbineSoftStarterTest, <RequirementID>_<TestName>)`
- **Helper Functions**: Use descriptive names from test fixture
- **Logging**: Follow `sfc_loggers.hpp` conventions

### Checkers and Validators

Utilize `yeet_testing` framework:
- `make_range_checker`: For continuous value range validation
- `make_threshold_checker`: For event detection
- `checker_activation_always`: For always-active monitoring
- Custom checkers: For complex multi-condition validation

---

## References

- [Soft-Starter Sequencing Documentation](soft_starter_sequencing.md)
- [Speed Control Theory](speed_control.md)
- [State Machine Definition](state_machine.md)
- [Current Control Documentation](current_control.md)
- [Per-Unit System Definition](per_unit_system.md)

---

## Revision History

| Version | Date       | Author  | Changes                                          |
| ------- | ---------- | ------- | ------------------------------------------------ |
| 1.0     | 2025-10-12 | Initial | Created comprehensive test requirements document |

---

## Conclusion

This requirements document provides a complete specification for validating the SFC soft-starter application. Implementation of tests covering all functional and performance requirements will ensure the system meets gas turbine starting demands with high reliability, safety, and performance.

The requirements are designed to be:
- **Measurable**: All metrics have quantifiable targets
- **Testable**: All requirements can be validated in simulation
- **Comprehensive**: Cover functional behavior and dynamic performance
- **Realistic**: Based on actual gas turbine starting procedures and LCI control theory

Test cases developed from these requirements will serve as the foundation for system validation, commissioning support, and ongoing regression testing.
