#include "sfc/sfc_measurements.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_plant_model_simulator.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

class SfcMeasurementsTest : public ::testing::Test {
protected:
    sfc_plant_model_simulator _plant_sim;
    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();
    control::six_pulse_generator _grid_side_pulse_generator;
    control::six_pulse_generator _machine_side_pulse_generator;

    // // Configurable firing angle for grid side converter
    // double grid_firing_angle_deg_ = sfc_plant_model_simulator::kNominalGridFiringAngleDeg;

    // System definition
    sfc_per_unit_system _pu_system{voltage_t(sfc_plant_model_simulator::kNominalGridVoltageRMS),
                                   frequency_t(sfc_plant_model_simulator::kNominalGridFrequency),
                                   current_t(sfc_plant_model_simulator::kBaseDCLinkCurrent),
                                   power_t(sfc_plant_model_simulator::kMachineRatedPowerVA),
                                   voltage_t(sfc_plant_model_simulator::kNominalMachineVoltageRMS),
                                   frequency_t(sfc_plant_model_simulator::kNominalMachineFrequency),
                                   sfc_plant_model_simulator::kMachineNumPoles};

    // Classes under test
    sfc_grid_side_measurements _grid_measurements;
    sfc_dc_link_measurements _dc_link_measurements;
    sfc_machine_side_measurements _machine_measurements;

    // Data storage for analysis
    std::vector<double> _grid_current_a_pu_history;
    std::vector<double> _machine_current_a_pu_history;
    std::vector<double> _dc_link_current_pu_history;
    std::vector<double> _dc_link_voltage_pu_history;

    void SetUp() override {
        setup_logging();

        auto& plant_model = _plant_sim.plant();

        // Initialize the plant model
        plant_model.initialize();
        plant_model.set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                        sfc_plant_model_simulator::kNominalGridVoltageRMS);
        plant_model.set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency,
                                           sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        // Reference angle is on the voltage measurement
        // _grid_side_pulse_generator.set_firing_offsets(deg_to_rad(30.0f));
        // _machine_side_pulse_generator.set_firing_offsets(deg_to_rad(30.0f));

        // Register the main simulation loop
        _simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            // 1. Update pulse generators to command the plant
            update_pulse_generators();

            // 2. Step the plant
            plant_model.step();

            // 3. Update measurement classes with raw plant outputs
            auto grid_voltages = plant_model.get_grid_side_voltages_abc();
            auto grid_currents = plant_model.get_grid_side_currents_abc();
            auto machine_voltages = plant_model.get_machine_side_voltages_abc();
            auto machine_currents = plant_model.get_machine_side_currents_abc();

            _grid_measurements.update(_pu_system.get_grid_bases(),
                                      abc<voltage_t>{voltage_t(grid_voltages.a()), voltage_t(grid_voltages.b()),
                                                     voltage_t(grid_voltages.c())},
                                      abc<current_t>{current_t(grid_currents.a()), current_t(grid_currents.b()),
                                                     current_t(grid_currents.c())});

            _dc_link_measurements.update(_pu_system.get_dc_link_bases(),
                                         voltage_t(0.0),  // DC voltage not available in plant model
                                         current_t(plant_model.get_dc_link_current()));

            _machine_measurements.update(
                _pu_system.get_machine_bases(),
                abc<voltage_t>{voltage_t(machine_voltages.a()), voltage_t(machine_voltages.b()),
                               voltage_t(machine_voltages.c())},
                abc<current_t>{current_t(machine_currents.a()), current_t(machine_currents.b()),
                               current_t(machine_currents.c())});

            // 4. Store results for later analysis
            _grid_current_a_pu_history.push_back(_grid_measurements.get_current_pu().a());
            _machine_current_a_pu_history.push_back(_machine_measurements.get_current_pu().a());
            _dc_link_current_pu_history.push_back(_dc_link_measurements.get_current_pu());
            _dc_link_voltage_pu_history.push_back(_dc_link_measurements.get_voltage_pu());
        });
    }

    void setup_logging() {
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_a_pu", [&]() { return _grid_measurements.get_voltage_pu().a(); }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_magnitude_pu", [&]() { return _grid_measurements.get_voltage_magnitude_pu(); }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_a_pu", [&]() { return _machine_measurements.get_voltage_pu().a(); }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_voltage_magnitude_pu", [&]() { return _machine_measurements.get_voltage_magnitude_pu(); }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_pu", [&]() { return _dc_link_measurements.get_current_pu(); }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_a_pu", [&]() { return _grid_measurements.get_current_pu().a(); }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "machine_current_a_pu", [&]() { return _machine_measurements.get_current_pu().a(); }));
    }

    void update_pulse_generators() {
        auto& plant_model = _plant_sim.plant();

        // Get reference angles from the plant model
        auto grid_reference_angle = plant_model.get_grid_voltage_source_angle();
        auto machine_reference_angle = plant_model.get_machine_voltage_source_angle();

        // Create firing angle objects
        angle<zero_to_two_pi> grid_firing_angle(deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg));
        angle<zero_to_two_pi> machine_firing_angle(
            deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg));

        // Update pulse generators
        auto grid_pulses = _grid_side_pulse_generator.update(grid_reference_angle, grid_firing_angle, true);
        auto machine_pulses = _machine_side_pulse_generator.update(machine_reference_angle, machine_firing_angle, true);

        // Set the generated pulses as inputs to the plant model
        plant_model.set_grid_side_pulses(grid_pulses.to_array());
        plant_model.set_machine_side_pulses(machine_pulses.to_array());
    }
};

TEST_F(SfcMeasurementsTest, ShouldConvertToCorrectPerUnitValuesAtNominalOperation) { _simulator.simulate_for(1.0f); }