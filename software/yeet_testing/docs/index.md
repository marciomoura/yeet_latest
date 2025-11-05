# YEET Testing Framework Documentation

## Overview

The YEET Testing Framework provides comprehensive tools for closed-loop testing and validation of real-time control systems. It enables Software-in-the-Loop (SIL) and Hardware-in-the-Loop (HIL) testing with data sampling, automated checking, and post-processing capabilities.

## Key Components

### Simulation Infrastructure

#### Simulator Core
- **Simulator**: Base class for closed-loop simulation environments
- **Simulator Runner**: Execution engine for automated test scenarios
- **Simulator Application**: Integration with YEET application data models

**Key Files:**
- `yeet_testing/simulator.hpp` - Simulator base class
- `yeet_testing/simulator_runner.hpp` - Test execution engine
- `yeet_testing/simulator_runner_application.hpp` - Application integration
- `yeet_testing/simulator_runner_application_interface.hpp` - Interface definitions

### Data Sampling and Logging

#### Data Sampler
- **Data Sampler Entry**: Individual signal or parameter sampling
- **Data Sampler**: Collection of sampled data with time synchronization
- **Sampler Runner**: Automated execution with data collection

**Key Files:**
- `yeet_testing/data_sampler_entry.hpp` - Signal sampling entry
- `yeet_testing/data_sampler.hpp` - Data collection manager
- `yeet_testing/simulator_sampler_runner.hpp` - Sampler execution

#### Post-Processing
- **JSON Export**: Machine-readable test results
- **Terminal Output**: Human-readable summaries
- **Custom Processing**: Extensible post-processing interface

**Key Files:**
- `yeet_testing/data_sample_json_post_processing.hpp` - JSON export
- `yeet_testing/data_sample_terminal_post_processing.hpp` - Terminal output
- `yeet_testing/data_sample_post_processing_interface.hpp` - Interface

### Automated Verification

#### Checker System
- **Simulator Checker**: Automated validation against expected behavior
- **Checker Runner**: Execution with pass/fail criteria
- **Failure Processing**: Detailed failure reporting and analysis

**Key Files:**
- `yeet_testing/simulator_checker.hpp` - Checker implementation
- `yeet_testing/simulator_checker_runner.hpp` - Checker execution
- `yeet_testing/checker_failures_post_processing_interface.hpp` - Failure reporting

### Parameter Management

#### Parameter Collection
- **Parameter Sweeps**: Automated testing across parameter ranges
- **Configuration Management**: Test scenario configuration
- **Batch Testing**: Multiple test configurations

**Key Files:**
- `yeet_testing/parameter_collection.hpp` - Parameter management

### Google Test Integration

#### GTest Support
- **Test Fixtures**: Integrated with Google Test framework
- **Automated Assertions**: Checker results as GTest assertions
- **CI/CD Ready**: Compatible with continuous integration pipelines

**Key Files:**
- `yeet_testing/simulator_gtest.hpp` - Google Test integration

## Testing Workflow

### 1. Define Test Scenario

```cpp
#include <yeet_testing/simulator_runner_application.hpp>

// Create simulator with application data model
auto simulator = yeet_testing::SimulatorRunnerApplication<MyAppDataModel>();

// Configure test scenario
simulator.set_simulation_time(10.0);  // 10 second test
simulator.set_timestep(100e-6);       // 100 µs timestep
```

### 2. Configure Data Sampling

```cpp
#include <yeet_testing/data_sampler.hpp>

// Create data sampler
auto sampler = yeet_testing::DataSampler();

// Add signals to sample
sampler.add_entry("motor_speed", &data_model.motor_speed);
sampler.add_entry("motor_torque", &data_model.motor_torque);
```

### 3. Run Simulation

```cpp
// Run with sampling
auto runner = yeet_testing::SimulatorSamplerRunner(simulator, sampler);
runner.run();

// Export results
auto json_processor = yeet_testing::DataSampleJsonPostProcessing("results.json");
json_processor.process(sampler.get_samples());
```

### 4. Automated Verification

```cpp
#include <yeet_testing/simulator_checker.hpp>

// Define checker
auto checker = yeet_testing::SimulatorChecker();
checker.add_check("motor_speed", [](float value) {
    return std::abs(value - 100.0f) < 1.0f;  // Within 1 rad/s
});

// Run with checking
auto checker_runner = yeet_testing::SimulatorCheckerRunner(simulator, checker);
bool passed = checker_runner.run();
```

## Integration with Google Test

```cpp
#include <yeet_testing/simulator_gtest.hpp>
#include <gtest/gtest.h>

TEST(MotorControlTest, SpeedTracking) {
    // Setup simulator
    auto simulator = create_test_simulator();
    
    // Run test
    auto runner = yeet_testing::SimulatorGTest(simulator);
    EXPECT_TRUE(runner.run());
    
    // Verify results
    EXPECT_NEAR(simulator.get_final_speed(), 100.0f, 1.0f);
}
```

## Testing Best Practices

### Test Organization
- **Unit Tests**: Individual component validation
- **Integration Tests**: Component interaction testing
- **System Tests**: Full application behavior validation
- **Regression Tests**: Automated verification of bug fixes

### Data Collection
- **Selective Sampling**: Sample only necessary signals to reduce overhead
- **Time Synchronization**: Ensure consistent time base across samples
- **Post-Processing**: Perform analysis offline to minimize real-time impact

### Verification Strategy
- **Tolerances**: Define appropriate tolerances for numerical comparisons
- **Transient Behavior**: Account for initialization and settling time
- **Edge Cases**: Test boundary conditions and error scenarios

## Performance Considerations

### Real-Time Constraints
- **Minimal Overhead**: Sampling designed for minimal execution time impact
- **Asynchronous Logging**: Data buffering for non-blocking operation
- **Deterministic Execution**: Predictable test execution timing

### Memory Management
- **Pre-Allocation**: All buffers allocated before test execution
- **Bounded Storage**: Configurable limits on data retention
- **Efficient Export**: Streaming export for large datasets

## Integration with Other Layers

### YEET Core
- Uses application data models and signal definitions
- Integrates with control execution policies
- Leverages parameter management system

### YEET Control
- Tests control algorithm implementations
- Validates numerical accuracy
- Verifies real-time performance

### YEET Application
- End-to-end application testing
- Configuration validation
- System-level verification

## Building and Running Tests

```bash
# Configure with testing enabled
cmake --preset host-gcc

# Build tests
cmake --build --preset host-gcc-debug

# Run all tests
ctest --test-dir build-host-gcc -C Debug

# Run specific test suite
ctest --test-dir build-host-gcc -C Debug -R yeet_testing
```

## Example Test Applications

The framework includes example test applications demonstrating:
- Basic simulator setup and execution
- Data sampling and export
- Automated checking and verification
- Parameter sweep testing
- Integration with Google Test

## See Also

- [YEET Core Library](../../yeet/docs/index.md)
- [YEET Control Library](../../yeet_control/docs/index.md)
- [SFC Application Testing](../../yeet_application/sfc/docs/index.md)
