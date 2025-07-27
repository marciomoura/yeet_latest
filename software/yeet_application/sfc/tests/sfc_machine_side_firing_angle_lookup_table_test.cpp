#include <gtest/gtest.h>

#include <vector>

#include "sfc/sfc_commutation_monitor.hpp"
#include "sfc/sfc_machine_side_lookup_table.hpp"
#include "sfc/sfc_measurements.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_plant_model_simulator.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/dual_sogi_srf_pll.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

class SfcMachineSideFiringAngleLookupTableTest : public ::testing::Test {
protected:
    sfc_plant_model_simulator _plant_sim;
    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();
    control::six_pulse_generator _grid_side_pulse_generator;
    control::six_pulse_generator _machine_side_pulse_generator;

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
    sfc_machine_side_firing_angle_lookup_table _machine_side_firing_angle_lookup_table{10.0};

    control::dual_sogi_srf_pll pll{duration_t(sfc_6p_6p_plant_model::kSamplingTime)};
    control::six_pulse_generator::pulses _grid_pulses{};
    control::six_pulse_generator::pulses _machine_pulses{};

    angle<zero_to_two_pi> machine_firing_angle{deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg)};
    angle<zero_to_two_pi> grid_firing_angle{deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg)};

    void SetUp() override {
        setup_logging();

        auto& plant_model = _plant_sim.plant();

        // Initialize the plant model
        plant_model.initialize();
        plant_model.set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                        sfc_plant_model_simulator::kNominalGridVoltageRMS);
        plant_model.set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency,
                                           sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        _machine_side_firing_angle_lookup_table.recalculate_table(
            10.0f,  // Example gamma setpoint
            to_pu(sfc_plant_model_simulator::kMachineSideInductance, _pu_system.get_machine_bases().base_inductance));

        // Configure PLL
        pll.configure_sequence_extractor(1.414, 1.0 / 50.0, true);
        pll.configure_pi_controller(1.0, 0.5);
        pll.configure_frequency_limits(10.0, 100.0);
        pll.configure_nominal_frequency(50.0f);
        pll.reset();

        abc<voltage_t> masked_pll_output_voltage;

        // Register the main simulation loop
        _simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            auto const& plant_model = _plant_sim.plant();
            // Get reference angles from the plant model
            auto const grid_reference_angle = plant_model.get_grid_voltage_source_angle();
            auto const machine_reference_angle = plant_model.get_machine_voltage_source_angle();
            auto const grid_voltages = plant_model.get_grid_side_voltages_abc();
            auto const grid_currents = plant_model.get_grid_side_currents_abc();
            auto const machine_voltages = plant_model.get_machine_side_voltages_abc();
            auto const machine_currents = plant_model.get_machine_side_currents_abc();

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
            // _machine_measurements.
            auto const machine_firing_angle_deg = _machine_side_firing_angle_lookup_table.get_firing_angle_deg(
                1.0f, _dc_link_measurements.get_current_pu());

            machine_firing_angle = deg_to_rad(machine_firing_angle_deg);

            // Update pulse generators
            _grid_pulses = _grid_side_pulse_generator.update(grid_reference_angle, grid_firing_angle, true);
            _machine_pulses = _machine_side_pulse_generator.update(machine_reference_angle, machine_firing_angle, true);

            // Set the generated pulses as inputs to the plant model
            // _commutation_monitor.update(_grid_measurements.get_voltage_pu(), _grid_measurements.get_current_pu(),
            // _dc_link_measurements.get_current_pu(), _grid_pulses);
            // Step the plant
            _plant_sim.plant().set_grid_side_pulses(_grid_pulses.to_array());
            _plant_sim.plant().set_machine_side_pulses(_machine_pulses.to_array());
            _plant_sim.plant().step();
        });
    }

    void setup_logging() {
        _simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<voltage_pu_t>>(
            "grid_voltage_pu", [&]() { return _grid_measurements.get_voltage_pu(); }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<current_pu_t>>(
            "grid_current_pu", [&]() { return _grid_measurements.get_current_pu(); }));

        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc link current", [&]() { return _dc_link_measurements.get_current_pu(); }));

        // Add voltage notch detection logging
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "machine firing angle", [&]() { return machine_firing_angle.get_pu(); }));
    }
};

TEST_F(SfcMachineSideFiringAngleLookupTableTest, Run) {
    _simulator.simulate_for(0.2f);

    grid_firing_angle = deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg * 1.1);

    _simulator.simulate_for(0.2f);

    grid_firing_angle = deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg * 1.2);

    _simulator.simulate_for(0.2f);

    grid_firing_angle = deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg * 1.3);

    _simulator.simulate_for(0.2f);
}