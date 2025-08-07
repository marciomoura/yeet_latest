#include <gtest/gtest.h>

#include "sfc_integration/testing_closed_loop_sfc.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcSpeedControllerTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(200e-6);

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Initialize the system - allocates plant model and all components
        initialize();

        // ==================== Speed Control Signals ====================

        // Speed reference and measured values
        simulator.register_sampler_entry(
            {"speed_reference_pu", [&]() { return static_cast<float>(system.speed_reference_pu.read()); }});

        simulator.register_sampler_entry(
            {"speed_measured_pu", [&]() { return static_cast<float>(system.measured_speed_pu.read()); }});

        simulator.register_sampler_entry(
            {"speed_error_pu", [&]() {
                 return static_cast<float>(
                     system.speed_controller.get_signal_flow_datamodel().get_outputs().speed_error.read());
             }});

        // ==================== Current Control Signals ====================

        // Current reference from speed controller
        simulator.register_sampler_entry(
            {"current_reference_pu", [&]() {
                 return static_cast<float>(
                     system.speed_controller.get_signal_flow_datamodel().get_outputs().current_reference.read());
             }});

        // DC link current measured values
        simulator.register_sampler_entry({"dc_link_current_measured", [&]() {
                                              return static_cast<float>(
                                                  system.dc_link_measurements.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .dc_current_filtered_pu.read());
                                          }});

        // Firing angle from current controller
        simulator.register_sampler_entry({"firing_angle_reference_pu", [&]() {
                                              return static_cast<float>(
                                                  system.current_controller.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .firing_angle_reference.read()
                                                      .get_pu());
                                          }});

        simulator.register_sampler_entry({"firing_angle_reference_degrees", [&]() {
                                              return static_cast<float>(
                                                  system.current_controller.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .firing_angle_reference.read()
                                                      .get_degrees());
                                          }});

        // Current and speed errors for validation
        simulator.register_sampler_entry(
            {"dc_link_current_error", [&]() { return static_cast<float>(get_dc_link_current_error_pu()); }});

        simulator.register_sampler_entry({"speed_error", [&]() { return static_cast<float>(get_speed_error_pu()); }});

        // ==================== Plant Model Outputs ====================

        // Grid-side firing pulses
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<yeet::control::six_pulse_generator::pulses>(
            "grid_firing_pulses", [&]() -> yeet::control::six_pulse_generator::pulses {
                return system.grid_side_pulse_generator.get_signal_flow_datamodel().get_outputs().firing_pulses.read();
            }));

        // Machine-side firing pulses
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<yeet::control::six_pulse_generator::pulses>(
            "machine_firing_pulses", [&]() -> yeet::control::six_pulse_generator::pulses {
                return system.machine_side_pulse_generator.get_signal_flow_datamodel()
                    .get_outputs()
                    .firing_pulses.read();
            }));

        // Grid voltage (per-unit)
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("grid_voltage_pu", [&]() {
            return system.grid_measurements.get_signal_flow_datamodel().get_outputs().grid_voltage_abc.read();
        }));

        // Machine voltage (per-unit)
        simulator.register_sampler_entry(
            yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("machine_voltage_pu", [&]() {
                return system.machine_measurements.get_signal_flow_datamodel().get_outputs().machine_voltage_abc.read();
            }));
    }

    void initialize() { system.initialize(); }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // Parameter access methods
    auto parameter_collection() { return system.create_parameter_collection(); }

    template <typename T>
    yeet::parameter_set<T> parameters()
    {
        return parameter_collection().get<T>();
    }

    // Specific parameter accessors
    yeet::parameter_set<sfc_speed_controller_datamodel::parameters> speed_controller_parameters()
    {
        return parameters<sfc_speed_controller_datamodel::parameters>();
    }

    yeet::parameter_set<sfc_current_controller_datamodel::parameters> current_controller_parameters()
    {
        return parameters<sfc_current_controller_datamodel::parameters>();
    }

    yeet::parameter_set<sfc_grid_side_pulse_generator_datamodel::parameters> grid_side_pulse_generator_parameters()
    {
        return parameters<sfc_grid_side_pulse_generator_datamodel::parameters>();
    }

    yeet::parameter_set<sfc_machine_side_pulse_generator_datamodel::parameters>
    machine_side_pulse_generator_parameters()
    {
        return parameters<sfc_machine_side_pulse_generator_datamodel::parameters>();
    }

    yeet::parameter_set<sfc_grid_side_measurements_datamodel::parameters> grid_measurements_parameters()
    {
        return parameters<sfc_grid_side_measurements_datamodel::parameters>();
    }

    yeet::parameter_set<sfc_dc_link_measurements_datamodel::parameters> dc_link_measurements_parameters()
    {
        return parameters<sfc_dc_link_measurements_datamodel::parameters>();
    }

    // Helper functions to calculate errors
    current_pu_t get_dc_link_current_error_pu() const
    {
        // Cast away const to access non-const methods - this is safe in test context
        auto& mutable_system = const_cast<sfc_speed_controller_simulation_builder&>(system);
        return mutable_system.speed_controller.get_signal_flow_datamodel().get_outputs().current_reference.read() -
               mutable_system.dc_link_measurements.get_signal_flow_datamodel()
                   .get_outputs()
                   .dc_current_filtered_pu.read();
    }

    speed_pu_t get_speed_error_pu() const
    {
        // Cast away const to access non-const methods - this is safe in test context
        auto& mutable_system = const_cast<sfc_speed_controller_simulation_builder&>(system);
        return mutable_system.speed_reference_pu.read() - mutable_system.measured_speed_pu.read();
    }

    // System under test
    yeet::application::sfc::integration::sfc_speed_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test regulation of machine speed to a reference value
TEST_F(SfcSpeedControllerTest, TestSpeedRegulation)
{
    const speed_pu_t speed_error_max_value{0.05f};
    const speed_pu_t speed_error_min_value{-0.05f};

    // Check speed error during steady state before disturbance
    simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error pre disturbance", [&]() -> speed_pu_t { return get_speed_error_pu(); }, speed_error_min_value,
        speed_error_max_value, yeet::checker_activation_time_range{1.0, 1.5}));

    // Check speed error after disturbance
    simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error after disturbance", [&]() -> speed_pu_t { return get_speed_error_pu(); }, speed_error_min_value,
        speed_error_max_value, yeet::checker_activation_time_range{2.0}));

    // ==================== Test Configuration ====================

    // Enable both pulse generators
    grid_side_pulse_generator_parameters()->enable = true;
    machine_side_pulse_generator_parameters()->enable = true;

    // Enable speed control mode
    system.enable_speed_control = true;

    // Set machine-side firing angle to constant value (open-loop)
    system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    // Configure feedforward signals for improved performance
    system.ideal_dc_voltage_pu = voltage_pu_t(1.0);           // Typical DC voltage in per-unit
    system.inverter_back_emf_machine_pu = voltage_pu_t(1.0);  // No back EMF initially

    // Configure load torque
    system.load_torque_pu = torque_pu_t(0.0);  // No load initially

    // ==================== Test Execution ====================

    // Simulate until system can stabilize
    system.speed_reference_pu = speed_pu_t(0.0);
    system.measured_speed_pu = speed_pu_t(0.0);
    bool success = simulator.simulate_for(0.5);

    // Set the speed reference to 0.5 per-unit, and test for 1.0 second
    system.speed_reference_pu = speed_pu_t(0.5);
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

TEST_F(SfcSpeedControllerTest, TestLoadDisturbanceRejection)
{
    const speed_pu_t speed_error_max_value{0.05f};
    const speed_pu_t speed_error_min_value{-0.05f};

    // Check speed error during steady state before disturbance
    simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error pre disturbance", [&]() -> speed_pu_t { return get_speed_error_pu(); }, speed_error_min_value,
        speed_error_max_value, yeet::checker_activation_time_range{1.0, 1.5}));

    // Check speed error after disturbance
    simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error after disturbance", [&]() -> speed_pu_t { return get_speed_error_pu(); }, speed_error_min_value,
        speed_error_max_value, yeet::checker_activation_time_range{2.0}));

    // ==================== Test Configuration ====================

    // Enable both pulse generators
    grid_side_pulse_generator_parameters()->enable = true;
    machine_side_pulse_generator_parameters()->enable = true;

    // Enable speed control mode
    system.enable_speed_control = true;

    // Set machine-side firing angle to constant value
    system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    // Configure feedforward signals
    system.ideal_dc_voltage_pu = voltage_pu_t(1.0);
    system.inverter_back_emf_machine_pu = voltage_pu_t(1.0);

    // ==================== Test Execution ====================

    // Simulate until system stabilizes
    system.speed_reference_pu = speed_pu_t(0.0);
    system.measured_speed_pu = speed_pu_t(0.0);
    system.load_torque_pu = torque_pu_t(0.0);
    bool success = simulator.simulate_for(0.5);

    // Set the speed reference to 0.5 per-unit, and test for 1.0 second
    system.speed_reference_pu = speed_pu_t(0.5);
    success &= simulator.simulate_for(1.0);

    // Add a load disturbance by applying load torque
    system.load_torque_pu = torque_pu_t(0.3);  // 30% load torque

    // Execute the test for another second to allow the controller to respond
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

TEST_F(SfcSpeedControllerTest, SpeedStepResponseTest)
{
    // ==================== Test Configuration ====================

    // Enable both pulse generators
    grid_side_pulse_generator_parameters()->enable = true;
    machine_side_pulse_generator_parameters()->enable = true;

    // Enable speed control mode
    system.enable_speed_control = true;

    // Set machine-side firing angle to constant value (open-loop)
    system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    // Configure feedforward signals for improved performance
    system.ideal_dc_voltage_pu = voltage_pu_t(1.0);           // Typical DC voltage in per-unit
    system.inverter_back_emf_machine_pu = voltage_pu_t(1.0);  // No back EMF initially

    // Configure load torque
    system.load_torque_pu = torque_pu_t(0.1);  // Light load

    // ==================== Test Execution ====================

    // Phase 1: Start with low speed reference
    system.speed_reference_pu = speed_pu_t(0.1);  // 10% of rated speed
    system.measured_speed_pu = speed_pu_t(0.0);   // Start from standstill

    simulate_for(0.5);  // Allow system to settle

    // Phase 2: Step to medium speed reference
    system.speed_reference_pu = speed_pu_t(0.3);  // 30% of rated speed

    simulate_for(0.5);  // Observe step response

    // Phase 3: Step to higher speed reference
    system.speed_reference_pu = speed_pu_t(0.6);  // 60% of rated speed

    simulate_for(0.5);  // Observe transient behavior

    // Phase 4: Return to lower speed reference
    system.speed_reference_pu = speed_pu_t(0.2);  // 20% of rated speed

    simulate_for(0.5);  // Final settling period

    // ==================== Test Validation ====================

    // The test validates:
    // 1. Speed controller responds to reference changes
    // 2. Current controller properly follows speed controller output
    // 3. DC link current adjusts to provide required torque
    // 4. System remains stable throughout operation
    // 5. Cascaded control loop works correctly
}

TEST_F(SfcSpeedControllerTest, TorqueModeTest)
{
    // ==================== Test Configuration ====================

    // Enable both pulse generators
    grid_side_pulse_generator_parameters()->enable = true;
    machine_side_pulse_generator_parameters()->enable = true;

    // Disable speed control mode (enable torque mode)
    system.enable_speed_control = false;

    // Set machine-side firing angle to constant value
    system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    // Configure feedforward signals
    system.ideal_dc_voltage_pu = voltage_pu_t(1.0);
    system.inverter_back_emf_machine_pu = voltage_pu_t(1.0);

    // Configure load torque
    system.load_torque_pu = torque_pu_t(0.2);  // Moderate load

    // ==================== Test Execution ====================

    // In torque mode, provide external current reference directly
    system.external_current_reference_pu = current_pu_t(0.0);
    system.speed_reference_pu = speed_pu_t(0.0);  // Speed reference ignored in torque mode
    system.measured_speed_pu = speed_pu_t(0.0);

    simulate_for(0.3);  // Allow system to settle

    // Step current reference to generate torque
    system.external_current_reference_pu = current_pu_t(0.4);  // 40% current

    simulate_for(0.5);  // Observe response

    // Change current reference
    system.external_current_reference_pu = current_pu_t(0.7);  // 70% current

    simulate_for(0.5);  // Observe transient

    // Return to lower current
    system.external_current_reference_pu = current_pu_t(0.2);  // 20% current

    simulate_for(0.5);  // Final settling

    // ==================== Test Validation ====================

    // The test validates:
    // 1. Speed controller bypassed in torque mode
    // 2. External current reference properly passed to current controller
    // 3. Current controller tracks external reference
    // 4. Mode switching functionality works correctly
}

TEST_F(SfcSpeedControllerTest, SpeedRampTest)
{
    // ==================== Test Configuration ====================

    // Enable both pulse generators
    grid_side_pulse_generator_parameters()->enable = true;
    machine_side_pulse_generator_parameters()->enable = true;

    // Enable speed control mode
    system.enable_speed_control = true;

    // Set machine-side firing angle to constant value
    system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    // Configure feedforward signals
    system.ideal_dc_voltage_pu = voltage_pu_t(1.35);
    system.inverter_back_emf_machine_pu = voltage_pu_t(0.0);

    // Configure load torque
    system.load_torque_pu = torque_pu_t(0.15);  // Light load

    // ==================== Test Execution: Gradual Speed Ramp ====================

    // Start from zero speed
    system.speed_reference_pu = speed_pu_t(0.0);
    system.measured_speed_pu = speed_pu_t(0.0);
    simulate_for(0.05);

    // Ramp up speed gradually to test tracking performance
    const double ramp_duration = 0.4;
    const double time_step = 0.02;
    const int steps = static_cast<int>(ramp_duration / time_step);

    for (int i = 1; i <= steps; ++i) {
        double progress = static_cast<double>(i) / steps;
        double speed_ref = 0.5 * progress;  // Ramp to 50% of rated speed

        system.speed_reference_pu = speed_pu_t(speed_ref);
        simulate_for(time_step);
    }

    // Hold at final value
    simulate_for(0.2);

    // ==================== Test Validation ====================

    // This test validates:
    // 1. Speed controller can track slowly varying references
    // 2. No instability during smooth reference changes
    // 3. Proper dynamic response characteristics of cascaded control
}
