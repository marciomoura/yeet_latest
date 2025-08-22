#include <gtest/gtest.h>

#include "sfc_integration/plant_sensorless_current_control_closed_loop.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcSensorlessCurrentControllerTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(200e-6);

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Initialize the system - allocates plant model and all components
        initialize();

        system.machine_side_controller.get_configuration_datamodel()
            .get_parameters()
            .machine_side_commutation_reactance_pu =
            to_pu(system.plant.get_plant_model().get_electrical_parameters().machine_side_inductance,
                system.plant.get_plant_model().get_per_unit_system().get_machine_bases().base_inductance);

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

        simulator.register_sampler_entry({"grid_side_firing_angle_pu", [&]() {
                                              return static_cast<float>(
                                                  system.current_controller.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .firing_angle_reference.read()
                                                      .get_pu());
                                          }});

        simulator.register_sampler_entry({"machine_side_firing_angle_pu", [&]() {
                                              return static_cast<float>(
                                                  system.machine_side_controller.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .machine_side_firing_angle.read()
                                                      .get_pu());
                                          }});

        // Current error for validation
        simulator.register_sampler_entry(
            {"dc_link_current_error", [&]() { return static_cast<float>(get_dc_link_current_error_pu()); }});

        // simulator.register_sampler_entry(
        //     yeet::make_lambda_sampler<frequency_pu_t>("machine_estimated_frequency", [&]() {
        //         return system.machine_side_observer.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .estimated_frequency_pu.read();
        //     }));

        // // ==================== Plant Model Outputs ====================
        // simulator.register_sampler_entry(
        //     yeet::make_lambda_sampler<voltage_pu_t>("grid_voltage_magnitude", [&]() -> voltage_pu_t {
        //         return system.grid_measurements.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .grid_voltage_magnitude_filtered.read();
        //     }));

        // simulator.register_sampler_entry(
        //     yeet::make_lambda_sampler<voltage_pu_t>("machine_voltage_magnitude", [&]() -> voltage_pu_t {
        //         return system.machine_measurements.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .machine_voltage_magnitude_filtered.read();
        //     }));

        // // Grid voltage (per-unit)
        // simulator.register_sampler_entry(
        //     yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("observer - grid_voltage_pu", [&]() {
        //         return system.grid_side_observer.get_signal_flow_datamodel().get_inputs().grid_voltage_pu.read();
        //     }));

        // simulator.register_sampler_entry(
        //     yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>("observer - machine_voltage_pu", [&]() {
        //         return system.machine_side_observer.get_signal_flow_datamodel()
        //             .get_inputs()
        //             .machine_voltage_pu.read()
        //             .to_alphabeta();
        //     }));

        // simulator.register_sampler_entry(
        //     yeet::make_lambda_sampler<real_t>("observer - estimated_voltage_magnitude", [&]() {
        //         return system.machine_side_observer.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .estimated_voltage_magnitude.read();
        //     }));

        // simulator.register_sampler_entry(
        //     yeet::make_lambda_sampler<real_t>("observer - estimated_frequency", [&]() {
        //         return
        //         system.machine_side_observer.get_signal_flow_datamodel().get_outputs().estimated_frequency.read();
        //     }));

        // simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("observer - estimated_frequency_pu", [&]()
        // {
        //     return
        //     system.machine_side_observer.get_signal_flow_datamodel().get_outputs().estimated_frequency_pu.read();
        // }));

        // simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("observer - estimated_position", [&]() {
        //     return system.machine_side_observer.get_signal_flow_datamodel()
        //         .get_outputs()
        //         .estimated_position.read()
        //         .get_pu();
        // }));

        // simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("observer - plant position", [&]() {
        //     return
        //     system.plant.get_signal_flow_datamodel().get_outputs().machine_voltage_source_angle.read().get_pu();
        // }));

        // simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("observer - angle error", [&]() {
        //     return -0.01 +
        //            (angle_wrapped(0.01) +
        //                system.machine_side_observer.get_signal_flow_datamodel()
        //                    .get_outputs()
        //                    .estimated_position.read() -
        //                system.plant.get_signal_flow_datamodel().get_outputs().machine_voltage_source_angle.read())
        //                .get_pu();
        // }));

        // simulator.register_sampler_entry(
        //     yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>("observer - filtered_voltage_alphabeta", [&]() {
        //         return system.machine_side_observer.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .filtered_voltage_alphabeta.read();
        //     }));

        // simulator.register_sampler_entry(
        //     yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("grid_voltage_unmasked_pu", [&]() {
        //         return system.grid_measurements.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .grid_voltage_abc_unmasked.read();
        //     }));

        // // Machine voltage (per-unit)
        // simulator.register_sampler_entry(
        //     yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("machine_voltage_pu", [&]() {
        //         return
        //         system.machine_measurements.get_signal_flow_datamodel().get_outputs().machine_voltage_abc.read();
        //     }));

        // simulator.register_sampler_entry(
        //     yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("machine_voltage_unmasked_pu", [&]() {
        //         return system.machine_measurements.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .machine_voltage_abc_unmasked.read();
        //     }));

        // ==================== Test Configuration ====================
        // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
        // The pulse generators are now controlled through signal flow, not parameters
        grid_side_pulse_generator_parameters()->enable = false;
        machine_side_pulse_generator_parameters()->enable = false;
        simulator.simulate_for(1.0);

        // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
        // The pulse generators are now controlled through signal flow, not parameters
        grid_side_pulse_generator_parameters()->enable = true;
        machine_side_pulse_generator_parameters()->enable = true;
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

    // Helper function to calculate current error
    current_pu_t get_dc_link_current_error_pu() const
    {
        // Cast away const to access non-const methods - this is safe in test context
        auto& mutable_system = const_cast<sfc_sensorless_current_controller_simulation_builder&>(system);
        return mutable_system.current_reference_pu.read() -
               mutable_system.dc_link_measurements.get_signal_flow_datamodel()
                   .get_outputs()
                   .dc_current_filtered_pu.read();
    }

    // System under test
    yeet::application::sfc::integration::sfc_sensorless_current_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test regulation of DC-link current to a reference value
TEST_F(SfcSensorlessCurrentControllerTest, TestCurrentRegulation)
{
    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.16f};
    current_pu_t current_error_min_value{-0.16f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{2.0, 2.9}));
    bool success = simulator.simulate_for(2.0);

    system.current_reference_pu = current_pu_t(1.0);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{3.5, 4.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

// Test regulation of DC-link current to a reference value
TEST_F(SfcSensorlessCurrentControllerTest, MachineFrequencyStep)
{
    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.15f};
    current_pu_t current_error_min_value{-0.15f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{2.0, 2.9}));
    bool success = simulator.simulate_for(2.0);

    system.plant.get_plant_model().set_machine_parameters(frequency_t{40.0}, voltage_t{1980.0});
    // simulator.register_checker(yeet::make_range_checker<current_pu_t>(
    //     "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
    //     current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{3.2, 4.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

// Test regulation of DC-link current to a reference value
TEST_F(SfcSensorlessCurrentControllerTest, MachineFrequencyStepLowFrequency)
{
    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.15f};
    current_pu_t current_error_min_value{-0.15f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{2.0, 2.9}));
    bool success = simulator.simulate_for(2.0);

    system.plant.get_plant_model().set_machine_parameters(frequency_t{30.0}, voltage_t{1980.0});
    // simulator.register_checker(yeet::make_range_checker<current_pu_t>(
    //     "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
    //     current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{3.2, 4.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

// Test regulation of DC-link current to a reference value
TEST_F(SfcSensorlessCurrentControllerTest, MachineFrequencyStepHighFrequency)
{
    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.15f};
    current_pu_t current_error_min_value{-0.15f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{2.0, 2.9}));
    bool success = simulator.simulate_for(2.0);

    system.plant.get_plant_model().set_machine_parameters(frequency_t{60.0}, voltage_t{1980.0});
    // simulator.register_checker(yeet::make_range_checker<current_pu_t>(
    //     "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
    //     current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{3.2, 4.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}

// 1 - Checkers return the instance, so you can just run in between periods of time; user can just deactivate when it
// wants 2 - State-machine setting the booleans 3 -