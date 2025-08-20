#include <gtest/gtest.h>

#include "sfc_integration/plant_complete_sfc_closed_loop.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class CompleteSensorlessSpeedControllerTest : public ::testing::Test {
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

        // ==================== State Machine Signals ====================
        simulator.register_sampler_entry(
            {"auxiliaries_are_on", [&]() {
                 return static_cast<float>(
                     system.state_machine.get_signal_flow_datamodel().get_inputs().auxiliaries_are_on.read());
             }});

        simulator.register_sampler_entry(
            {"breakers_are_on", [&]() {
                 return static_cast<float>(
                     system.state_machine.get_signal_flow_datamodel().get_inputs().breakers_are_on.read());
             }});

        simulator.register_sampler_entry(
            {"start_command", [&]() {
                 return static_cast<float>(
                     system.state_machine.get_signal_flow_datamodel().get_inputs().start_command.read());
             }});

        simulator.register_sampler_entry(
            {"pulse_releases", [&]() {
                 return static_cast<float>(
                     system.state_machine.get_signal_flow_datamodel().get_outputs().pulse_releases.read());
             }});

        simulator.register_sampler_entry(
            {"freeze_integrator", [&]() {
                 return static_cast<float>(
                     system.state_machine.get_signal_flow_datamodel().get_outputs().freeze_integrator.read());
             }});

        // ==================== Current Control Signals ====================
        simulator.register_sampler_entry(
            {"dc_link_current_reference", [&]() {
                 return static_cast<float>(
                     system.current_controller.get_signal_flow_datamodel().get_inputs().current_reference_pu.read());
             }});

        simulator.register_sampler_entry({"dc_link_current_measured", [&]() {
                                              return static_cast<float>(
                                                  system.dc_link_measurements.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .dc_current_filtered_pu.read());
                                          }});

        simulator.register_sampler_entry({"current_controller_output", [&]() {
                                              return static_cast<float>(
                                                  system.current_controller.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .firing_angle_reference.read()
                                                      .get_pu());
                                          }});

        simulator.register_sampler_entry(
            {"current_controller_saturated", [&]() {
                 return static_cast<float>(
                     system.current_controller.get_signal_flow_datamodel().get_outputs().is_saturated.read());
             }});

        // ==================== Observer Signals ====================
        simulator.register_sampler_entry({"grid_voltage_magnitude_estimated", [&]() {
                                              return static_cast<float>(
                                                  system.grid_side_observer.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .estimated_voltage_magnitude.read());
                                          }});

        simulator.register_sampler_entry({"machine_voltage_magnitude_estimated", [&]() {
                                              return static_cast<float>(
                                                  system.machine_side_observer.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .estimated_voltage_magnitude.read());
                                          }});

        simulator.register_sampler_entry(
            {"machine_speed_estimated", [&]() {
                 return static_cast<float>(
                     system.machine_side_observer.get_signal_flow_datamodel().get_outputs().estimated_speed.read());
             }});

        // ==================== Pulse Generator Signals ====================
        simulator.register_sampler_entry({"actual_machine_firing_angle", [&]() {
                                              return static_cast<float>(
                                                  system.machine_side_pulse_generator.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .actual_firing_angle.read()
                                                      .get_pu());
                                          }});

        simulator.register_sampler_entry({"actual_grid_firing_angle", [&]() {
                                              return static_cast<float>(
                                                  system.grid_side_pulse_generator.get_signal_flow_datamodel()
                                                      .get_outputs()
                                                      .actual_firing_angle.read()
                                                      .get_pu());
                                          }});

        // ==================== Plant Model Outputs ====================
        simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>("machine_speed_pu",
            [&]() { return system.plant.get_signal_flow_datamodel().get_outputs().machine_speed.read(); }));

        simulator.register_sampler_entry(
            {"load_torque", [&]() {
                 return static_cast<float>(system.plant.get_signal_flow_datamodel().get_inputs().load_torque_pu.read());
             }});

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

    yeet::parameter_set<sfc_state_machine_datamodel::parameters> state_machine_parameters()
    {
        return parameters<sfc_state_machine_datamodel::parameters>();
    }

    // Helper functions to calculate errors
    speed_pu_t get_speed_error_pu() const
    {
        // Cast away const to access non-const methods - this is safe in test context
        auto& mutable_system = const_cast<sfc_complete_sfc_closed_loop_simulation_builder&>(system);
        return mutable_system.speed_controller.get_signal_flow_datamodel().get_inputs().speed_reference_pu.read() -
               mutable_system.speed_controller.get_signal_flow_datamodel().get_inputs().measured_speed_pu.read();
    }

    // State machine control helpers
    void set_auxiliaries_on(bool on) { system.auxiliaries_are_on = on; }

    void set_breakers_on(bool on) { system.breakers_are_on = on; }

    void set_start_command(bool start) { system.start_command = start; }

    void set_stop_command(bool stop) { system.stop_command = stop; }

    void set_shutdown_request(bool shutdown) { system.shutdown_request = shutdown; }

    void set_current_extinguished(bool extinguished) { system.is_current_extinguished = extinguished; }

    // Complete startup sequence following state machine logic
    void execute_startup_sequence()
    {
        set_auxiliaries_on(true);
        set_breakers_on(true);
        set_start_command(true);
        simulate_for(0.1);
    }

    // Complete shutdown sequence following state machine logic
    void execute_shutdown_sequence()
    {
        // 1. Request shutdown (OPERATION_ON -> SHUTDOWN_REQUESTED)
        set_stop_command(true);
        simulate_for(0.1);

        // 2. Current extinguished (SHUTDOWN_REQUESTED -> OPERATION_OFF)
        set_current_extinguished(true);
        set_stop_command(false);  // Remove stop command
        simulate_for(0.1);

        // 3. Turn off breakers (OPERATION_OFF -> BREAKERS_ON)
        set_breakers_on(false);
        set_start_command(false);  // Remove start command
        simulate_for(0.1);

        // 4. Breakers actually open (BREAKERS_ON -> BREAKERS_OFF)
        simulate_for(0.1);

        // 5. Turn off auxiliaries (BREAKERS_OFF -> IDLE)
        set_auxiliaries_on(false);
        simulate_for(0.1);
    }

    // System under test
    yeet::application::sfc::integration::sfc_complete_sfc_closed_loop_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

// Test speed regulation with state machine startup - based on sensorless_speed_controller_test.cpp
TEST_F(CompleteSensorlessSpeedControllerTest, TestSpeedRegulationWithStateMachine)
{
    // Configure plant for variable speed operation (same as original test)
    system.plant.get_configuration_datamodel().get_parameters().operation_mode.write(
        sfc_plant_model_datamodel::machine_operation_mode::variable_speed);

    system.plant.get_configuration_datamodel().get_parameters().motor_damping_factor.write(
        damping_coefficient_pu_t{0.01f});
    system.plant.get_configuration_datamodel().get_parameters().motor_inertia_constant.write(inertia_constant_t{0.5});

    system.plant.get_plant_model().set_operation_mode(plant_model_machine_operation_mode::variable_speed);
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);
    system.plant.get_plant_model().configure_speed_voltage_relationship(0.8, 0.2f, 0.0f);
    system.plant.get_plant_model().set_initial_mechanical_speed_pu(speed_pu_t{1.0f});
    system.plant.get_plant_model().set_load_torque_pu(torque_pu_t{0.3f});

    const speed_pu_t speed_error_max_value{0.05f};
    const speed_pu_t speed_error_min_value{-0.05f};

    // Check speed error after disturbance
    auto speed_regulation_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed error", [&]() -> speed_pu_t { return get_speed_error_pu(); }, speed_error_min_value,
        speed_error_max_value, yeet::checker_activation_always{}));

    speed_regulation_checker->deactivate();  // Deactivate initially to avoid false positives

    // ==================== Test Configuration ====================
    // Initially disable pulse generators
    grid_side_pulse_generator_parameters()->enable = false;
    machine_side_pulse_generator_parameters()->enable = false;

    // Simulate initial idle state
    // ==================== State Machine Startup Sequence ====================
    // Execute complete startup sequence to reach operational state
    execute_startup_sequence();

    // At this point, state machine should be in OPERATION_ON and pulse_releases should be true
    system.plant.get_plant_model().set_voltage_control_mode(plant_model_voltage_control_mode::speed_dependent);

    // Enable pulse generators (they should now release pulses due to state machine)
    grid_side_pulse_generator_parameters()->enable = true;
    machine_side_pulse_generator_parameters()->enable = true;

    // ==================== Test Execution ====================
    // Simulate until system can stabilize
    system.speed_reference_pu = speed_pu_t(1.0);
    system.load_torque_pu = torque_pu_t(0.2f);
    bool success = simulator.simulate_for(5.0);

    speed_regulation_checker->activate();
    success &= simulator.simulate_for(5.0);
    speed_regulation_checker->deactivate();

    EXPECT_TRUE(success) << "Simulation did not complete successfully with speed regulation and state machine.";

    // Verify that pulse releases are active during operation
    EXPECT_TRUE(system.state_machine.get_signal_flow_datamodel().get_outputs().pulse_releases.read())
        << "Pulse releases should be active during OPERATION_ON state";
}
