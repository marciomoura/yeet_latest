#include <gtest/gtest.h>

#include "sfc_integration/plant_sensorless_speed_control_closed_loop.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class SfcSensorlessSpeedControllerTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(ms(0.5));

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Initialize the system - allocates plant model and all components
        initialize();

        system.machine_side_controller.get_configuration_datamodel()
            .get_parameters()
            .machine_side_commutation_reactance_pu =
            to_pu(system.plant.get_plant_model().get_electrical_parameters().machine_side_inductance,
                system.plant.get_plant_model().get_per_unit_system().get_machine_bases().base_inductance);

        // ==================== Speed Control Signals ====================
        // ==================== Current Control Signals ====================

        // Current reference from speed controller
        simulator.register_sampler_entry(
            {"dc_link_current_reference", [&]() {
                 return static_cast<float>(
                     system.speed_controller.get_signal_flow_datamodel().get_outputs().current_reference.read());
             }});

        simulator.register_sampler_entry(
            {"speed reference", [&]() {
                 return static_cast<float>(
                     system.speed_controller.get_signal_flow_datamodel().get_inputs().speed_reference_pu.read());
             }});

        simulator.register_sampler_entry(
            {"speed measured", [&]() {
                 return static_cast<float>(
                     system.speed_controller.get_signal_flow_datamodel().get_inputs().measured_speed_pu.read());
             }});

        simulator.register_sampler_entry(
            {"speed error", [&]() {
                 return static_cast<float>(
                     system.speed_controller.get_signal_flow_datamodel().get_outputs().speed_error.read());
             }});

        simulator.register_sampler_entry(
            {"load torque", [&]() {
                 return static_cast<float>(system.plant.get_signal_flow_datamodel().get_inputs().load_torque_pu.read());
             }});

        simulator.register_sampler_entry({"actual machine firing angle", [&]() {
                                              return static_cast<float>(
                                                  system.machine_side_pulse_generator.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .actual_firing_angle.read()
                                                      .get_pu());
                                          }});

        simulator.register_sampler_entry({"actual grid firing angle", [&]() {
                                              return static_cast<float>(
                                                  system.grid_side_pulse_generator.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .actual_firing_angle.read()
                                                      .get_pu());
                                          }});

        // simulator.register_sampler_entry({"machine voltage magnitude", [&]() {
        //                                       return static_cast<float>(system.plant.get_signal_flow_datamodel()
        //                                               .get_outputs()
        //                                               .machine_voltage.read()
        //                                               .to_alphabeta()
        //                                               .magnitude());
        //                                   }});

        simulator.register_sampler_entry({"dc_link_current_measured", [&]() {
                                              return static_cast<float>(
                                                  system.dc_link_measurements.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .dc_current_filtered_pu.read());
                                          }});

        // Firing angle from current controller
        // simulator.register_sampler_entry({"grid_side_firing_angle", [&]() {
        //                                       return static_cast<float>(
        //                                           system.current_controller.get_signal_flow_datamodel()
        //                                               .get_outputs()
        //                                               .firing_angle_reference.read()
        //                                               .get_pu());
        //                                   }});

        // simulator.register_sampler_entry({"machine_side_firing_angle", [&]() {
        //                                       return static_cast<float>(
        //                                           system.machine_side_controller.get_signal_flow_datamodel()
        //                                               .get_outputs()
        //                                               .machine_side_firing_angle.read()
        //                                               .get_pu());
        //                                   }});

        simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>("machine_speed_pu", [&]() {
            return system.machine_side_observer.get_signal_flow_datamodel().get_outputs().estimated_speed.read();
        }));

        // Current and speed errors for validation
        // simulator.register_sampler_entry(
        //     {"dc_link_current_error", [&]() { return static_cast<float>(get_dc_link_current_error_pu()); }});

        // simulator.register_sampler_entry({"speed_error", [&]() { return static_cast<float>(get_speed_error_pu());
        // }});

        // ==================== Plant Model Outputs ====================

        // Grid-side firing pulses
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

        // Grid voltage (per-unit)
        // simulator.register_sampler_entry(
        //     yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>("grid_filtered_voltage_alphabeta", [&]() {
        //         return system.grid_side_observer.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .filtered_voltage_alphabeta.read();
        //     }));

        // // Machine voltage (per-unit)
        // simulator.register_sampler_entry(
        //     yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>("machine_filtered_voltage_alphabeta", [&]() {
        //         return system.machine_side_observer.get_signal_flow_datamodel()
        //             .get_outputs()
        //             .filtered_voltage_alphabeta.read();
        //     }));

        simulator.register_sampler_entry(
            yeet::make_lambda_sampler<voltage_pu_t>("grid_voltage_magnitude", [&]() -> voltage_pu_t {
                return system.grid_measurements.get_signal_flow_datamodel()
                    .get_outputs()
                    .grid_voltage_magnitude_filtered.read();
            }));

        simulator.register_sampler_entry(
            yeet::make_lambda_sampler<voltage_pu_t>("machine_voltage_magnitude", [&]() -> voltage_pu_t {
                return system.machine_measurements.get_signal_flow_datamodel()
                    .get_outputs()
                    .machine_voltage_magnitude_filtered.read();
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
        auto& mutable_system = const_cast<sfc_sensorless_speed_controller_simulation_builder&>(system);
        return mutable_system.speed_controller.get_signal_flow_datamodel().get_outputs().current_reference.read() -
               mutable_system.dc_link_measurements.get_signal_flow_datamodel()
                   .get_outputs()
                   .dc_current_filtered_pu.read();
    }

    speed_pu_t get_speed_error_pu()
    {
        return system.speed_controller.get_signal_flow_datamodel().get_inputs().speed_reference_pu.read() -
               system.speed_controller.get_signal_flow_datamodel().get_inputs().measured_speed_pu.read();
    }

    // System under test
    yeet::application::sfc::integration::sfc_sensorless_speed_controller_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test regulation of machine speed to a reference value
TEST_F(SfcSensorlessSpeedControllerTest, TestSpeedRegulation)
{
    system.plant.get_configuration_datamodel().get_parameters().operation_mode.write(
        sfc_plant_model_datamodel::machine_operation_mode::variable_speed);

    system.plant.get_configuration_datamodel().get_parameters().motor_damping_factor.write(
        damping_coefficient_pu_t{0.01f});
    system.plant.get_configuration_datamodel().get_parameters().motor_inertia_constant.write(inertia_constant_t{0.5});
    // system.plant.get_plant_model().configure_mechanical_load(inertia_constant_t{0.5},
    // damping_coefficient_pu_t{0.001f});
    system.plant.get_plant_model().set_operation_mode(plant_model_machine_operation_mode::variable_speed);
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);
    system.plant.get_plant_model().configure_speed_voltage_relationship(0.9, 0.2f, 0.0f);
    system.plant.get_plant_model().set_initial_mechanical_speed_pu(speed_pu_t{1.0f});
    system.plant.get_plant_model().set_load_torque_pu(torque_pu_t{0.3f});
    system.speed_controller.get_configuration_datamodel().get_parameters().speed_reference_rate_limit.write(2.0);

    const speed_pu_t speed_error_max_value{0.05f};
    const speed_pu_t speed_error_min_value{-0.05f};

    // Check speed error after disturbance
    auto user_activation = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error", [&]() -> speed_pu_t { return get_speed_error_pu(); }, speed_error_min_value,
        speed_error_max_value, yeet::checker_activation_always{}));

    user_activation->deactivate();  // Deactivate initially to avoid false positives

    // ==================== Test Configuration ====================
    // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
    // The pulse generators are now controlled through signal flow, not parameters
    // grid_side_pulse_generator_parameters()->enable = false;
    // machine_side_pulse_generator_parameters()->enable = false;
    system.enable_speed_control = false;

    simulator.simulate_for(1.0);

    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    // Note: 'enable' has been moved from parameters to inputs in the updated datamodel
    // The pulse generators are now controlled through signal flow, not parameters
    // grid_side_pulse_generator_parameters()->enable = true;
    // machine_side_pulse_generator_parameters()->enable = true;

    // Enable speed control mode
    system.enable_speed_control = true;

    // Configure load torque
    system.load_torque_pu = torque_pu_t(0.3);  // No load initially

    // ==================== Test Execution ====================

    // Simulate until system can stabilize
    system.speed_reference_pu = speed_pu_t(1.0);
    bool success = simulator.simulate_for(5.0);

    user_activation->activate();
    success &= simulator.simulate_for(5.0);
    user_activation->deactivate();

    // // Set the speed reference to 0.5 per-unit, and test for 1.0 second
    system.speed_reference_pu = speed_pu_t(0.95);
    success &= simulator.simulate_for(5.0);

    user_activation->activate();
    success &= simulator.simulate_for(5.0);
    user_activation->deactivate();

    EXPECT_TRUE(success) << "Simulation did not complete successfully.";
}
