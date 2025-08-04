#include <gtest/gtest.h>

#include "sfc_simulators.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

/**
 * @brief Test fixture for validating the sfc_machine_side_firing_angle_lookup_table in a closed-loop simulation.
 *
 * This test uses the new simulator builder pattern with observer-based pulse generation
 * to create a realistic closed-loop environment for testing the machine-side lookup table.
 */
class SfcMachineSideFiringAngleLookupTableTest : public ::testing::Test {
protected:
    // Simulation signals
    current_pu_t dc_link_current_ref_pu = current_pu_t{0.8};  // Reference current in per-unit

    // Simulation components using the new pattern
    sfc_simulation_components components;
    sfc_pulse_generator_conduit pulse_generator_conduit;
    yeet::simulator simulator = yeet::make_gtest_simulator();

    void SetUp() override
    {
        // Configure plant model parameters using the augmented model interface
        components.plant_model_simulator.set_grid_parameters(frequency_t(50.0), voltage_t(2090.0));  // 50 Hz, 2090V RMS
        components.plant_model_simulator.set_machine_parameters(
            frequency_t(50.0), voltage_t(1980.0));  // 50 Hz, 1980V RMS

        // Configure the machine-side lookup table
        components.machine_side_lookup_table.configure_table(20.0, 0.1);  // gamma=20°, inductance=0.1pu

        // Configure the current controller for grid-side firing angle control
        // components.current_controller.configure_pi_gains(0.5, 0.05);  // Kp, Ti
        // components.current_controller.configure_current_limits_pu(-10.0, 10.0);
        // components.current_controller.configure_firing_angle_limits_radians(deg_to_rad(0.0), deg_to_rad(90.0));

        // Start with pulse generation disabled to allow system to stabilize
        pulse_generator_conduit.grid_firing_angle_rad = deg_to_rad(angle_t(13.0));  // 13° nominal grid firing angle
        pulse_generator_conduit.machine_firing_angle_rad =
            deg_to_rad(angle_t(180.0 - 26.84));  // 153.16° nominal machine firing angle

        // Use the builder to configure a simulation with both current controller and machine-side lookup
        sfc_simulator_builder(components)
            .with_measurements_and_filtering()
            .with_observers()
            .with_observer_based_pulse_generation(pulse_generator_conduit)
            .with_current_controller(pulse_generator_conduit)
            .with_machine_side_lookup_table(pulse_generator_conduit)
            .configure(simulator);

        // Enable pulse generation and set references once the grid observer has locked
        simulator.register_lambda_runner(components.kControlSamplingTime, [&]() {
            pulse_generator_conduit.current_ref_pu = dc_link_current_ref_pu;
            pulse_generator_conduit.measured_current_pu =
                components.measurement_and_filtering.get_dc_link_filtering().get_filtered_current_pu();
            pulse_generator_conduit.enable = components.observers.grid_side_observer.is_locked();
        });

        setup_logging();
    }

    void setup_logging()
    {
        auto& plant_model = components.plant_model_simulator;  // Use the augmented model directly
        auto& measurement_and_filtering = components.measurement_and_filtering;

        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "dc_link_current_ref_pu", [&]() { return dc_link_current_ref_pu.value(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("dc_link_current_measured_pu",
            [&]() { return measurement_and_filtering.get_dc_link_measurements().get_current_pu().value(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("dc_link_current_filtered_pu",
            [&]() { return measurement_and_filtering.get_dc_link_filtering().get_filtered_current_pu().value(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid_firing_angle_ref_pu", [&]() { return pulse_generator_conduit.grid_firing_angle_rad.get_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("machine_firing_angle_ref_pu",
            [&]() { return pulse_generator_conduit.machine_firing_angle_rad.get_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid_voltage_a_pu",
            [&]() { return measurement_and_filtering.get_grid_side_measurements().get_voltage_pu().a().value(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>("grid_current_a_pu",
            [&]() { return measurement_and_filtering.get_grid_side_measurements().get_current_pu().a().value(); }));

        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<std::array<bool, 6>>("grid_pulses",
            [&]() { return components.pulse_generators.grid_side_pulse_generator.get_pulses().to_array(); }));
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<std::array<bool, 6>>("machine_pulses",
            [&]() { return components.pulse_generators.machine_side_pulse_generator.get_pulses().to_array(); }));
    }
};

TEST_F(SfcMachineSideFiringAngleLookupTableTest, ShouldControlMachineFiringAngleWithLookupTable)
{
    // Act: Run the simulation to allow system to stabilize
    simulator.simulate_for(0.5);

    // Change the reference current to see current controller response and machine-side lookup behavior
    dc_link_current_ref_pu = current_pu_t{0.5};
    simulator.simulate_for(0.5);

    dc_link_current_ref_pu = current_pu_t{1.0};
    simulator.simulate_for(0.5);
}