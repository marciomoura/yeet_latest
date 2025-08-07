#include <gtest/gtest.h>

#include "sfc_integration/testing_closed_loop_sfc.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcCurrentControllerTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(200e-6);

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Initialize the system - allocates plant model and all components
        initialize();

        // ==================== Current Control Signals ====================

        // DC link current reference and measured values
        simulator.register_sampler_entry(
            {"dc_link_current_reference", [&]() { return static_cast<float>(system.current_reference_pu.read()); }});

        simulator.register_sampler_entry({"dc_link_current_measured", [&]() {
                                              return static_cast<float>(
                                                  system.dc_link_measurements.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .dc_current_filtered_pu.read());
                                          }});

        simulator.register_sampler_entry({"firing angle", [&]() {
                                              return static_cast<float>(
                                                  system.current_controller.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .firing_angle_reference.read()
                                                      .get_pu());
                                          }});

        // simulator.register_sampler_entry(
        //     {"dc_link_current_raw", [&]() {
        //          return static_cast<float>(
        //              system.plant.get_signal_flow_datamodel().get_outputs().dc_link_current.read());
        //      }});

        // // Current controller output (firing angle)
        // simulator.register_sampler_entry({"firing_angle_reference", [&]() {
        //                                       return static_cast<float>(
        //                                           system.current_controller.get_signal_flow_datamodel()
        //                                               .get_outputs()
        //                                               .firing_angle_reference.read()
        //                                               .get_degrees());
        //                                   }});

        // // ==================== Plant Model Outputs ====================

        // // Grid-side firing pulses
        // simulator.register_sampler_entry(yeet::make_lambda_array_sampler<yeet::control::six_pulse_generator::pulses>(
        //     "grid_firing_pulses", [&]() -> yeet::control::six_pulse_generator::pulses {
        //         return
        //         system.grid_side_pulse_generator.get_signal_flow_datamodel().get_outputs().firing_pulses.read();
        //     }));

        // // Machine-side firing pulses
        // simulator.register_sampler_entry(yeet::make_lambda_array_sampler<yeet::control::six_pulse_generator::pulses>(
        //     "machine_firing_pulses", [&]() -> yeet::control::six_pulse_generator::pulses {
        //         return system.machine_side_pulse_generator.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .firing_pulses.read();
        //     }));

        // // Grid voltage (per-unit)
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("grid_voltage_pu", [&]() {
            return system.grid_measurements.get_signal_flow_datamodel().get_outputs().grid_voltage_abc.read();
        }));

        simulator.register_sampler_entry(
            yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("grid_voltage_unmasked_pu", [&]() {
                return system.grid_measurements.get_signal_flow_datamodel()
                    .get_outputs()
                    .grid_voltage_abc_unmasked.read();
            }));

        // // Machine voltage (per-unit)
        simulator.register_sampler_entry(
            yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("machine_voltage_pu", [&]() {
                return system.machine_measurements.get_signal_flow_datamodel().get_outputs().machine_voltage_abc.read();
            }));

        simulator.register_sampler_entry(
            yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("machine_voltage_unmasked_pu", [&]() {
                return system.machine_measurements.get_signal_flow_datamodel()
                    .get_outputs()
                    .machine_voltage_abc_unmasked.read();
            }));

        // // DC link voltage
        // simulator.register_sampler_entry(
        //     {"dc_link_voltage", [&]() {
        //          return static_cast<float>(
        //              system.plant.get_signal_flow_datamodel().get_outputs().dc_link_voltage.read());
        //      }});

        // // ==================== Measurement Chain Validation ====================

        // // Grid measurements - filtered values
        // simulator.register_sampler_entry(
        //     yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("grid_voltage_filtered_pu", [&]() {
        //         return
        //         system.grid_measurements.get_signal_flow_datamodel().get_outputs().grid_voltage_abc_pu.read();
        //     }));

        // // DC link measurements - filtered values
        // simulator.register_sampler_entry({"dc_voltage_filtered", [&]() {
        //                                       return static_cast<float>(
        //                                           system.dc_link_measurements.get_signal_flow_datamodel()
        //                                               .get_outputs()
        //                                               .dc_voltage_filtered_pu.read());
        //                                   }});
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

    // System under test
    yeet::application::sfc::integration::sfc_current_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

TEST_F(SfcCurrentControllerTest, CurrentStepResponseTest)
{
    // ==================== Test Configuration ====================

    // Enable both pulse generators
    grid_side_pulse_generator_parameters()->enable = true;
    machine_side_pulse_generator_parameters()->enable = true;

    // Configure current controller for stable operation
    // current_controller_parameters()->pi_gains.kp = 10.0;  // Proportional gain
    // current_controller_parameters()->pi_gains.ki = 100.0; // Integral gain
    // current_controller_parameters()->output_limits.max = angle_wrapped::from_degrees(150.0);
    // current_controller_parameters()->output_limits.min = angle_wrapped::from_degrees(5.0);

    // Set machine-side firing angle to constant value (open-loop)
    system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    // Configure feedforward signals for improved performance
    system.ideal_dc_voltage_pu = voltage_pu_t(1.0);           // Typical DC voltage in per-unit
    system.inverter_back_emf_machine_pu = voltage_pu_t(1.0);  // No back EMF initially

    // ==================== Test Execution ====================

    // Phase 1: Start with low current reference
    system.current_reference_pu = current_pu_t(1.0);  // 20% of rated current

    simulate_for(1.0);  // Allow system to settle

    // // Phase 2: Step to medium current reference
    system.current_reference_pu = current_pu_t(0.5);  // 50% of rated current

    simulate_for(0.5);  // Observe step response

    // Phase 3: Step to higher current reference
    system.current_reference_pu = current_pu_t(0.8);  // 80% of rated current

    simulate_for(0.15);  // Observe transient behavior

    // Phase 4: Return to lower current reference
    system.current_reference_pu = current_pu_t(0.3);  // 30% of rated current

    simulate_for(0.1);  // Final settling period

    // ==================== Test Validation ====================

    // The test validates:
    // 1. Current controller responds to reference changes
    // 2. Firing angle adjusts appropriately
    // 3. DC link current follows the reference
    // 4. System remains stable throughout operation
    // 5. Measurement chain provides proper feedback

    // Note: Detailed assertions would be added based on expected performance criteria
    // For now, this test ensures the control loop executes without errors
}

// TEST_F(SfcCurrentControllerTest, CurrentRampTest)
// {
//     // ==================== Test Configuration ====================

//     // Enable both pulse generators
//     grid_side_pulse_generator_parameters()->enable = true;
//     machine_side_pulse_generator_parameters()->enable = true;

//     // Set machine-side firing angle to constant value
//     system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

//     // Configure feedforward signals
//     system.ideal_dc_voltage_pu = voltage_pu_t(1.35);
//     system.inverter_back_emf_machine_pu = voltage_pu_t(0.0);

//     // ==================== Test Execution: Gradual Current Ramp ====================

//     // Start from zero current
//     system.current_reference_pu = current_pu_t(0.0);
//     simulate_for(0.05);

//     // Ramp up current gradually to test tracking performance
//     const double ramp_duration = 0.3;
//     const double time_step = 0.02;
//     const int steps = static_cast<int>(ramp_duration / time_step);

//     for (int i = 1; i <= steps; ++i) {
//         double progress = static_cast<double>(i) / steps;
//         double current_ref = 0.7 * progress;  // Ramp to 70% of rated current

//         system.current_reference_pu = current_pu_t(current_ref);
//         simulate_for(time_step);
//     }

//     // Hold at final value
//     simulate_for(0.1);

//     // ==================== Test Validation ====================

//     // This test validates:
//     // 1. Current controller can track slowly varying references
//     // 2. No instability during smooth reference changes
//     // 3. Proper dynamic response characteristics
// }

// TEST_F(SfcCurrentControllerTest, MeasurementFilteringTest)
// {
//     // ==================== Test Configuration ====================

//     // Enable pulse generators
//     grid_side_pulse_generator_parameters()->enable = true;
//     machine_side_pulse_generator_parameters()->enable = true;

//     // Set constant operating point
//     system.current_reference_pu = current_pu_t(0.4);  // 40% of rated current
//     system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);
//     system.ideal_dc_voltage_pu = voltage_pu_t(1.35);
//     system.inverter_back_emf_machine_pu = voltage_pu_t(0.0);

//     // ==================== Test Execution ====================

//     // Run for sufficient time to validate measurement filtering
//     simulate_for(0.4);

//     // ==================== Test Validation ====================

//     // This test validates:
//     // 1. Measurement filters are working correctly
//     // 2. Filtered signals are smoother than raw signals
//     // 3. No excessive noise in the control loop
//     // 4. Proper per-unit scaling in measurements
// }
