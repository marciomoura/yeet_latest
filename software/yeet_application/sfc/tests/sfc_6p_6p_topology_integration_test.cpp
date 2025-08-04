/**
 * @file sfc_6p_6p_topology_integration_test.cpp
 * @brief Integration tests for the SFC 6-pulse to 6-pulse topology.
 *
 * This test file uses a builder pattern to configure and run simulations of the
 * SFC plant model with varying levels of control complexity. It includes:
 *
 * - An "ideal" simulation using the plant's internal reference angles.
 * - A "realistic" simulation using observers to estimate angles from per-unit
 *   measurements, mimicking a closed-loop control system.
 * - Comprehensive logging for visualization and analysis in both scenarios.
 */

#include <gtest/gtest.h>

#include "sfc_simulators.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

namespace {

using namespace yeet::control;
using namespace yeet::application::sfc;

// Test fixture for the "ideal" simulation scenario.
// It uses the plant's internal, perfect reference angles for pulse generation.
class SfcIdealSimulationTest : public ::testing::Test {
protected:
    // Simulation components and control conduits, owned by the test fixture
    sfc_simulation_components components;
    sfc_pulse_generator_conduit pulse_generator_conduit;
    yeet::simulator simulator = yeet::make_gtest_simulator();

    void SetUp() override
    {
        // Configure plant model parameters
        components.plant_model_simulator.plant().set_grid_parameters(
            sfc_plant_model_simulator::kNominalGridFrequency, sfc_plant_model_simulator::kNominalGridVoltageRMS);
        components.plant_model_simulator.plant().set_machine_parameters(
            sfc_plant_model_simulator::kNominalMachineFrequency, sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        sfc_simulator_builder(components).with_ideal_pulse_generation(pulse_generator_conduit).configure(simulator);

        setup_logging();
    }

    void setup_logging()
    {
        auto& plant_model = components.plant_model_simulator.plant();
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_t>>(
            "grid_voltage", [&]() { return plant_model.get_grid_side_voltages_abc(); }));
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_t>>(
            "grid_current", [&]() { return plant_model.get_grid_side_currents_abc(); }));
        simulator.register_sampler_entry(yeet::make_lambda_sampler<current_t>(
            "dc_link_current", [&]() { return plant_model.get_dc_link_current(); }));
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<control::six_pulse_generator::pulses>(
            "grid_side_pulses", [&]() { return components.pulse_generators.grid_side_pulse_generator.get_pulses(); }));
        simulator.register_sampler_entry(
            yeet::make_lambda_array_sampler<control::six_pulse_generator::pulses>("machine_side_pulses",
                [&]() { return components.pulse_generators.machine_side_pulse_generator.get_pulses(); }));
    }
};

TEST_F(SfcIdealSimulationTest, BasicOperation)
{
    // Start with pulse generation disabled to allow initial conditions to stabilize
    pulse_generator_conduit.enable = false;
    EXPECT_TRUE(simulator.simulate_for(0.1));

    // Enable pulse generation
    pulse_generator_conduit.enable = true;
    EXPECT_TRUE(simulator.simulate_for(0.4));
}

// Test fixture for the "realistic" simulation scenario.
// It uses observers to estimate angles from per-unit measurements for closed-loop control.
class SfcRealisticSimulationTest : public ::testing::Test {
protected:
    // Simulation components and control conduits, owned by the test fixture
    sfc_simulation_components components;
    sfc_pulse_generator_conduit pulse_generator_conduit;
    yeet::simulator simulator = yeet::make_gtest_simulator();

    void SetUp() override
    {
        // Configure plant model parameters
        components.plant_model_simulator.plant().set_grid_parameters(
            sfc_plant_model_simulator::kNominalGridFrequency, sfc_plant_model_simulator::kNominalGridVoltageRMS);
        components.plant_model_simulator.plant().set_machine_parameters(
            sfc_plant_model_simulator::kNominalMachineFrequency, sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        pulse_generator_conduit.enable = false;  // Start disabled, enable only when observer is locked

        // Use the builder to configure a realistic, observer-based simulation.
        // All control-related tasks (measurements, observers, pulses) run at 100e-6s.
        sfc_simulator_builder(components)
            .with_measurements_and_filtering()
            .with_observers()
            .with_observer_based_pulse_generation(pulse_generator_conduit)
            .configure(simulator);

        // Add a runner to enable the pulse generators once the grid observer has a stable lock
        simulator.register_lambda_runner(
            100e-6, [&]() { pulse_generator_conduit.enable = components.observers.grid_side_observer.is_locked(); });

        setup_logging();
    }

    void setup_logging()
    {
        auto& plant_model = components.plant_model_simulator.plant();
        auto& measurement_and_filtering = components.measurement_and_filtering;
        auto& observers = components.observers;

        // Log per-unit measurements
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("grid_voltage_pu",
            [&]() { return measurement_and_filtering.get_grid_side_measurements().get_voltage_pu(); }));
        simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<current_pu_t>>("grid_current_pu",
            [&]() { return measurement_and_filtering.get_grid_side_measurements().get_current_pu(); }));
        simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>("dc_link_current_pu",
            [&]() { return measurement_and_filtering.get_dc_link_measurements().get_current_pu(); }));

        // Log observer outputs vs. actual plant values
        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
            "grid_angle_actual_pu", [&]() { return plant_model.get_grid_voltage_source_angle().get_pu(); }));
        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
            "grid_angle_observed_pu", [&]() { return observers.grid_side_observer.get_estimated_angle().get_pu(); }));
        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("grid_frequency_observed_pu",
            [&]() { return observers.grid_side_observer.get_estimated_frequency_pu().value(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>(
            "machine_angle_actual_pu", [&]() { return plant_model.get_machine_voltage_source_angle().get_pu(); }));
        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("machine_angle_observed_pu",
            [&]() { return observers.machine_side_observer.get_estimated_angle().get_pu(); }));
        simulator.register_sampler_entry(yeet::make_lambda_sampler<real_t>("machine_frequency_observed_pu",
            [&]() { return observers.machine_side_observer.get_estimated_frequency_pu().value(); }));
    }
};

TEST_F(SfcRealisticSimulationTest, BasicOperation) { EXPECT_TRUE(simulator.simulate_for(1.0)); }

}  // namespace