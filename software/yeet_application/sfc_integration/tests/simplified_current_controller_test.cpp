#include <gtest/gtest.h>

#include "sfc_integration/testing_closed_loop_sfc.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
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

        simulator.register_sampler_entry({"firing_angle_reference_pu", [&]() {
                                              return static_cast<float>(
                                                  system.current_controller.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .firing_angle_reference.read()
                                                      .get_pu());
                                          }});

        // Current error for validation
        simulator.register_sampler_entry(
            {"dc_link_current_error", [&]() { return static_cast<float>(get_dc_link_current_error_pu()); }});

        // ==================== Plant Model Outputs ====================

        // Grid voltage (per-unit)
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
        auto& mutable_system = const_cast<sfc_current_controller_simulation_builder&>(system);
        return mutable_system.current_reference_pu.read() -
               mutable_system.dc_link_measurements.get_signal_flow_datamodel()
                   .get_outputs()
                   .dc_current_filtered_pu.read();
    }

    // System under test
    yeet::application::sfc::integration::sfc_current_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test regulation of DC-link current to a reference value
TEST_F(SfcCurrentControllerTest, TestCurrentRegulation)
{
    // ==================== Test Configuration ====================

    // Enable both pulse generators
    grid_side_pulse_generator_parameters()->enable = true;
    machine_side_pulse_generator_parameters()->enable = true;

    // Set machine-side firing angle to constant value (open-loop)
    system.machine_side_firing_angle = yeet::angle_wrapped::from_degrees(180.0 - 26.84);

    // Configure feedforward signals for improved performance
    system.ideal_dc_voltage_pu = voltage_pu_t(1.0);           // Typical DC voltage in per-unit
    system.inverter_back_emf_machine_pu = voltage_pu_t(1.0);  // No back EMF initially

    // ==================== Test Execution ====================
    current_pu_t current_error_max_value{0.06f};
    current_pu_t current_error_min_value{-0.06f};

    system.current_reference_pu = current_pu_t(0.5);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{0.6, 0.9}));
    bool success = simulator.simulate_for(1.0);

    system.current_reference_pu = current_pu_t(1.0);
    simulator.register_checker(yeet::make_range_checker<current_pu_t>(
        "current error pre disturbance", [&]() -> current_pu_t { return get_dc_link_current_error_pu(); },
        current_error_min_value, current_error_max_value, yeet::checker_activation_time_range{1.2, 2.0}));
    success &= simulator.simulate_for(1.0);

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}
