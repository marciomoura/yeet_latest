#include <gtest/gtest.h>

#include "sfc_simulators.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

/**
 * @brief Test fixture for validating the sfc_speed_controller in a closed-loop simulation.
 *
 * This test uses the new simulator builder pattern with observer-based pulse generation
 * to create a realistic closed-loop environment for testing the speed controller.
 */
class SfcSpeedControllerSimulationTest : public ::testing::Test {
protected:
    // Simulation components using the new pattern
    sfc_simulation_components components;
    sfc_pulse_generator_conduit pulse_generator_conduit;
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    void SetUp() override {
        // Configure plant model for variable speed operation
        components.plant_model_simulator.set_operation_mode(
            sfc_plant_model_simulator::machine_operation_mode::variable_speed);
        components.plant_model_simulator.configure_mechanical_load(0.5, 0.001);  // 0.5s inertia, 0.001 damping
        components.plant_model_simulator.set_initial_mechanical_speed_pu(speed_pu_t(1.0));
        // components.plant_model_simulator.

        // Configure the machine-side lookup table
        components.machine_side_lookup_table.configure_table(
            20.0, to_pu(components.plant_model_simulator.kMachineSideInductance,
                        components.plant_model_simulator.per_unit_system()
                            .get_machine_bases()
                            .base_inductance));  // gamma=20°, inductance=0.1pu

        // Configure plant model parameters
        components.plant_model_simulator.plant().set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                                                     sfc_plant_model_simulator::kNominalGridVoltageRMS);
        components.plant_model_simulator.plant().set_machine_parameters(
            sfc_plant_model_simulator::kNominalMachineFrequency, sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        // Configure the speed controller (outer loop)
        components.speed_controller.configure_pi_gains(10.0, 1.0);  // Kp, Ti
        components.speed_controller.configure_torque_limits_pu(-1.0, 2.0);
        components.speed_controller.configure_speed_reference_filter(10.0);  // 0.5 Hz filter

        // Configure the current controller (inner loop)
        components.current_controller.configure_pi_gains(1.0, 0.05);  // Kp, Ti
        components.current_controller.configure_current_limits_pu(0.0, 2.0);
        components.current_controller.configure_firing_angle_limits_radians(deg_to_rad(0.0), deg_to_rad(90.0));

        // Start with pulse generation disabled to allow system to stabilize
        pulse_generator_conduit.grid_firing_angle_rad =
            deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg);
        pulse_generator_conduit.machine_firing_angle_rad =
            deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg);

        // Use the builder to configure a simulation with speed controller -> current controller cascade
        sfc_simulator_builder(components)
            .with_measurements_and_filtering()
            .with_observers()
            .with_observer_based_pulse_generation(pulse_generator_conduit)
            .with_speed_controller(pulse_generator_conduit)
            .with_machine_side_lookup_table(pulse_generator_conduit)
            .with_current_controller(pulse_generator_conduit)
            .configure(simulator);

        // Set up the measurement update and enable control once observers are locked
        simulator.register_lambda_runner(components.kControlSamplingTime, [&]() {
            // Get measured speed from the mechanical load
            pulse_generator_conduit.enable = true;  // components.observers.grid_side_observer.is_locked();

            pulse_generator_conduit.measured_speed_pu =
                components.plant_model_simulator.mechanical_load().get_speed_pu();

            pulse_generator_conduit.measured_current_pu =
                components.measurement_and_filtering.get_dc_link_filtering().get_filtered_current_pu();
        });

        setup_logging();
    }

    void setup_logging() {
        auto& measurement_and_filtering = components.measurement_and_filtering;

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "speed_reference_pu", [&]() { return pulse_generator_conduit.speed_ref_pu; }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "speed_measured_pu", [&]() { return pulse_generator_conduit.measured_speed_pu; }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "torque_ref_pu", [&]() { return components.speed_controller.get_controller().get_torque_ref_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "current_ref_pu", [&]() { return pulse_generator_conduit.current_ref_pu.value(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_measured_pu",
            [&]() { return measurement_and_filtering.get_dc_link_filtering().get_filtered_current_pu().value(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_firing_angle_ref_pu", [&]() { return pulse_generator_conduit.grid_firing_angle_rad.get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_firing_angle_ref_pu",
            [&]() { return pulse_generator_conduit.machine_firing_angle_rad.get_pu(); }));
    }
};

TEST_F(SfcSpeedControllerSimulationTest, ShouldRegulateSpeedToReference) {
    // Act: Run the simulation to allow system to stabilize
    pulse_generator_conduit.speed_ref_pu = 1.0;
    simulator.simulate_for(0.225);
    simulator.simulate_for(3.0);
    pulse_generator_conduit.speed_ref_pu = 0.75;
    simulator.simulate_for(3.0);
    // pulse_generator_conduit.speed_ref_pu = 0.8;
    // simulator.simulate_for(2.0);
}