#include <gtest/gtest.h>

#include "sfc/sfc_current_controller.hpp"
#include "sfc/sfc_filtering.hpp"
#include "sfc/sfc_machine_side_lookup_table.hpp"
#include "sfc/sfc_measurements.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_plant_model_simulator.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

class SfcMachineSideFiringAngleLookupTableTest : public ::testing::Test {
protected:
    sfc_plant_model_simulator plant;
    yeet::testing::simulator simulator = yeet::testing::make_gtest_simulator();

    // System definition
    sfc_per_unit_system pu_system{voltage_t(sfc_plant_model_simulator::kNominalGridVoltageRMS),
                                  frequency_t(sfc_plant_model_simulator::kNominalGridFrequency),
                                  current_t(sfc_plant_model_simulator::kBaseDCLinkCurrent),
                                  power_t(sfc_plant_model_simulator::kMachineRatedPowerVA),
                                  voltage_t(sfc_plant_model_simulator::kNominalMachineVoltageRMS),
                                  frequency_t(sfc_plant_model_simulator::kNominalMachineFrequency),
                                  sfc_plant_model_simulator::kMachineNumPoles};

    // Classes under test
    sfc_grid_side_measurements grid_measurements;
    sfc_dc_link_measurements dc_link_measurements;
    sfc_machine_side_measurements machine_measurements;
    sfc_machine_side_firing_angle_lookup_table machine_side_firing_angle_lookup_table{10.0};
    sfc_current_controller current_controller{duration_t(sfc_6p_6p_plant_model::kSamplingTime)};
    sfc_dc_link_filtering dc_link_filtering{duration_t(sfc_6p_6p_plant_model::kSamplingTime)};
    // Simulation signals
    current_pu_t dc_link_current_ref_pu{1.0};  // Reference current in per-unit

    control::six_pulse_generator grid_side_pulse_generator;
    control::six_pulse_generator machine_side_pulse_generator;
    control::six_pulse_generator::pulses grid_pulses{};
    control::six_pulse_generator::pulses machine_pulses{};

    angle<zero_to_two_pi> machine_firing_angle{deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg)};
    angle<zero_to_two_pi> grid_firing_angle{deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg)};

    void SetUp() override {
        setup_logging();

        auto& plant_model = plant.plant();

        // Initialize the plant model
        plant_model.initialize();
        plant_model.set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                        sfc_plant_model_simulator::kNominalGridVoltageRMS);
        plant_model.set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency,
                                           sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        machine_side_firing_angle_lookup_table.recalculate_table(
            20.0f,  // Example gamma setpoint
            to_pu(sfc_plant_model_simulator::kMachineSideInductance, pu_system.get_machine_bases().base_inductance));

        current_controller.set_current_limits_pu(0.0f, 1.2f);
        current_controller.set_firing_angle_limits_radians(0.0, pi / 2);
        current_controller.set_pi_gains(1.0, 0.08);  // Kp, Ti

        // Register the main simulation loop
        simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            auto const& plant_model = plant.plant();
            // Get reference angles from the plant model
            auto const grid_reference_angle = plant_model.get_grid_voltage_source_angle();
            auto const machine_reference_angle = plant_model.get_machine_voltage_source_angle();
            auto const grid_voltages = plant_model.get_grid_side_voltages_abc();
            auto const grid_currents = plant_model.get_grid_side_currents_abc();
            auto const machine_voltages = plant_model.get_machine_side_voltages_abc();
            auto const machine_currents = plant_model.get_machine_side_currents_abc();

            grid_measurements.update(pu_system.get_grid_bases(), grid_voltages, grid_currents);
            machine_measurements.update(pu_system.get_machine_bases(), machine_voltages, machine_currents);

            dc_link_measurements.update(pu_system.get_dc_link_bases(),
                                        voltage_t(0.0),  // DC voltage not available in plant model
                                        plant_model.get_dc_link_current());

            dc_link_filtering.update(voltage_pu_t{}, current_pu_t{dc_link_measurements.get_current_pu()});

            auto const machine_firing_angle_deg = machine_side_firing_angle_lookup_table.get_firing_angle_deg(
                1.0f, dc_link_filtering.get_filtered_current_pu());

            machine_firing_angle = deg_to_rad(machine_firing_angle_deg);

            // Update the current controller
            grid_firing_angle =
                current_controller.update(dc_link_current_ref_pu, dc_link_filtering.get_filtered_current_pu(), 0.5f);
            // current_controller.update(dc_link_current_ref_pu, dc_link_filtering.get_filtered_current_pu(), 0.0f);

            // Update pulse generators
            grid_pulses = grid_side_pulse_generator.update(grid_reference_angle, grid_firing_angle, true);
            machine_pulses = machine_side_pulse_generator.update(machine_reference_angle, machine_firing_angle, true);

            // Set the pulses in the plant model
            plant.plant().set_grid_side_pulses(grid_pulses.to_array());
            plant.plant().set_machine_side_pulses(machine_pulses.to_array());
            plant.plant().step();
        });
    }

    void setup_logging() {
        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<voltage_pu_t>>(
            "grid_voltage_pu", [&]() { return grid_measurements.get_voltage_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_array_sampler<abc<current_pu_t>>(
            "grid_current_pu", [&]() { return grid_measurements.get_current_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc link current reference", [&]() { return dc_link_current_ref_pu; }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc link current filtered", [&]() { return dc_link_filtering.get_filtered_current_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<current_pu_t>(
            "dc link current", [&]() { return dc_link_measurements.get_current_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "current controller cosalpha", [&]() { return current_controller.get_cos_alpha_clamped(); }));

        // Add voltage notch detection logging
        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "grid firing angle pu", [&]() { return grid_firing_angle.get_pu(); }));

        simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<real_t>(
            "machine firing angle pu", [&]() { return machine_firing_angle.get_pu(); }));
    }
};

TEST_F(SfcMachineSideFiringAngleLookupTableTest, Run) {
    dc_link_current_ref_pu = current_pu_t(1.0f);  // Set a reference current
    simulator.simulate_for(0.7f);
    dc_link_current_ref_pu = current_pu_t(0.5f);  // Set a reference current
    simulator.simulate_for(0.5f);

    plant.plant().set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency,
                                         sfc_plant_model_simulator::kNominalMachineVoltageRMS * 0.8f);

    simulator.simulate_for(0.3f);

    plant.plant().set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency * 0.8f,
                                         sfc_plant_model_simulator::kNominalMachineVoltageRMS * 0.8f);

    simulator.simulate_for(0.3f);

    plant.plant().set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency * 0.5f,
                                         sfc_plant_model_simulator::kNominalMachineVoltageRMS * 0.5f);

    simulator.simulate_for(0.3f);
}