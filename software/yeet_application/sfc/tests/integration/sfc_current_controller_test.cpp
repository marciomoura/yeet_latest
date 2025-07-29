#include <gtest/gtest.h>

#include "sfc_simulators.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

/**
 * @brief Test fixture for validating the sfc_current_controller in a closed-loop simulation.
 *
 * This test uses the new simulator builder pattern with observer-based pulse generation
 * to create a realistic closed-loop environment for testing the current controller.
 */
class SfcCurrentControllerSimulationTest : public ::testing::Test {
protected:
    // Simulation signals
    current_pu_t dc_link_current_ref_pu = current_pu_t{0.8};  // Reference current in per-unit

    // Simulation components using the new pattern
    sfc_simulation_components components;
    sfc_pulse_generator_conduit pulse_generator_conduit;
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();
    void SetUp() override {
        // Configure plant model parameters
        components.plant_model_simulator.plant().set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                                                     sfc_plant_model_simulator::kNominalGridVoltageRMS);
        components.plant_model_simulator.plant().set_machine_parameters(
            sfc_plant_model_simulator::kNominalMachineFrequency, sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        // Configure the current controller
        components.current_controller.configure_pi_gains(0.5, 0.05);  // Kp, Ti
        components.current_controller.configure_current_limits_pu(-10.0, 10.0);
        components.current_controller.configure_firing_angle_limits_radians(deg_to_rad(0.0), deg_to_rad(90.0));

        // Start with pulse generation disabled to allow system to stabilize
        pulse_generator_conduit.grid_firing_angle_rad =
            deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg);
        pulse_generator_conduit.machine_firing_angle_rad =
            deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg);

        // sfc_simulator_builder(components)
        //     .with_measurements_and_filtering()
        //     .with_ideal_pulse_generation(pulse_generator_conduit)
        //     .with_current_controller(pulse_generator_conduit, dc_link_current_ref_pu)
        //     .configure(simulator);

        // Use the builder to configure a simulation with current controller
        sfc_simulator_builder(components)
            .with_measurements_and_filtering()
            .with_observers()
            .with_observer_based_pulse_generation(pulse_generator_conduit)
            .with_current_controller(pulse_generator_conduit, dc_link_current_ref_pu)
            .configure(simulator);

        // Enable pulse generation once the grid observer has locked
        simulator.register_lambda_runner(components.kControlSamplingTime, [&]() {
            // pulse_generator_conduit.enable = components.observers.grid_side_observer.is_locked();
            pulse_generator_conduit.current_ref_pu = dc_link_current_ref_pu;
            pulse_generator_conduit.measured_current_pu =
                components.measurement_and_filtering.get_dc_link_filtering().get_filtered_current_pu();
            pulse_generator_conduit.enable = components.observers.grid_side_observer.is_locked();
        });

        setup_logging();
    }

    void setup_logging() {
        auto& plant_model = components.plant_model_simulator.plant();
        auto& measurement_and_filtering = components.measurement_and_filtering;

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_ref_pu", [&]() { return dc_link_current_ref_pu.value(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_measured_pu",
            [&]() { return measurement_and_filtering.get_dc_link_measurements().get_current_pu().value(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_ref_limited_pu",
            [&]() { return components.current_controller.get_controller().get_current_ref_limited_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_firing_angle_ref_pu", [&]() { return pulse_generator_conduit.grid_firing_angle_rad.get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_firing_angle_ref_pu",
            [&]() { return pulse_generator_conduit.machine_firing_angle_rad.get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>("grid_voltage_a_pu", [&]() {
            return measurement_and_filtering.get_grid_side_measurements().get_voltage_pu().a().value();
        }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>("grid_current_a_pu", [&]() {
            return measurement_and_filtering.get_grid_side_measurements().get_current_pu().a().value();
        }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<std::array<bool, 6>>(
            "grid_pulses",
            [&]() { return components.pulse_generators.grid_side_pulse_generator.get_pulses().to_array(); }));
        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<std::array<bool, 6>>(
            "machine_pulses",
            [&]() { return components.pulse_generators.machine_side_pulse_generator.get_pulses().to_array(); }));

        // auto& observers = components.observers;

        // // Log per-unit measurements
        // simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<voltage_pu_t>>(
        //     "grid_voltage_pu",
        //     [&]() { return measurement_and_filtering.get_grid_side_measurements().get_voltage_pu(); }));
        // simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<current_pu_t>>(
        //     "grid_current_pu",
        //     [&]() { return measurement_and_filtering.get_grid_side_measurements().get_current_pu(); }));
        // simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>("dc_link_current_pu", [&]()
        // {
        //     return measurement_and_filtering.get_dc_link_measurements().get_current_pu();
        // }));

        // // Log observer outputs vs. actual plant values
        // simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
        //     "grid_angle_actual_pu", [&]() { return plant_model.get_grid_voltage_source_angle().get_pu(); }));
        // simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
        //     "grid_angle_observed_pu", [&]() { return observers.grid_side_observer.get_estimated_angle().get_pu();
        //     }));
        // simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
        //     "grid_frequency_observed_pu",
        //     [&]() { return observers.grid_side_observer.get_estimated_frequency_pu().value(); }));

        // simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
        //     "machine_angle_actual_pu", [&]() { return plant_model.get_machine_voltage_source_angle().get_pu(); }));
        // simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>("machine_angle_observed_pu",
        // [&]() {
        //     return observers.machine_side_observer.get_estimated_angle().get_pu();
        // }));
        // simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
        //     "machine_frequency_observed_pu",
        //     [&]() { return observers.machine_side_observer.get_estimated_frequency_pu().value(); }));
    }
};

TEST_F(SfcCurrentControllerSimulationTest, ShouldRegulateDcCurrentToReference) {
    // Act: Run the simulation to allow system to stabilize
    simulator.simulate_for(0.5);

    // Change the reference current
    dc_link_current_ref_pu = current_pu_t{0.5};

    // Run simulation to observe regulation
    simulator.simulate_for(1.0);
    dc_link_current_ref_pu = current_pu_t{1.0};

    simulator.simulate_for(1.0);
}
