#include "sfc/sfc_commutation_monitor.hpp"

#include <gtest/gtest.h>

#include "sfc/sfc_commutation_mask.hpp"
#include "sfc/sfc_plant_model_simulator.hpp"
#include "sfc/sfc_simulators.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/dual_sogi_srf_pll.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/data_sampler_entry.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

class SfcCommutationMonitorTest : public ::testing::Test {
protected:
    sfc_plant_model_simulator _plant_sim;
    yeet::simulator _simulator = yeet::make_gtest_simulator();
    control::six_pulse_generator _grid_side_pulse_generator;
    control::six_pulse_generator _machine_side_pulse_generator;

    // System definition
    sfc_per_unit_system _pu_system{voltage_t(sfc_plant_model_simulator::kNominalGridVoltageRMS),
        frequency_t(sfc_plant_model_simulator::kNominalGridFrequency),
        current_t(sfc_plant_model_simulator::kBaseDCLinkCurrent),
        power_t(sfc_plant_model_simulator::kMachineRatedPowerVA),
        voltage_t(sfc_plant_model_simulator::kNominalMachineVoltageRMS),
        frequency_t(sfc_plant_model_simulator::kNominalMachineFrequency), sfc_plant_model_simulator::kMachineNumPoles};

    // Classes under test
    sfc_grid_side_measurements _grid_measurements;
    sfc_dc_link_measurements _dc_link_measurements;
    sfc_machine_side_measurements _machine_measurements;

    control::six_pulse_generator::pulses _grid_pulses{};
    control::six_pulse_generator::pulses _machine_pulses{};
    control::dual_sogi_srf_pll _grid_side_pll{duration_t(sfc_6p_6p_plant_model::kSamplingTime)};

    abc<voltage_pu_t> masked_voltage_pu{};
    sfc_commutation_mask _commutation_mask;
    sfc_commutation_monitor _commutation_monitor{sfc_commutation_monitor::Config{}};

    void SetUp() override
    {
        setup_logging();

        auto& plant_model = _plant_sim.plant();

        _grid_side_pll.configure_frequency_limits(10.0, 100.0);
        _grid_side_pll.configure_nominal_frequency(50.0f);
        _grid_side_pll.configure_pi_controller(1.0, 0.5);
        _grid_side_pll.configure_sequence_extractor(1.414, 1.0 / 50.0);
        _grid_side_pll.reset();

        // Initialize the plant model
        plant_model.initialize();
        plant_model.set_grid_parameters(
            sfc_plant_model_simulator::kNominalGridFrequency, sfc_plant_model_simulator::kNominalGridVoltageRMS);
        plant_model.set_machine_parameters(
            sfc_plant_model_simulator::kNominalMachineFrequency, sfc_plant_model_simulator::kNominalMachineVoltageRMS);

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
                abc<voltage_t>{
                    voltage_t(grid_voltages.a()), voltage_t(grid_voltages.b()), voltage_t(grid_voltages.c())},
                abc<current_t>{
                    current_t(grid_currents.a()), current_t(grid_currents.b()), current_t(grid_currents.c())});

            _dc_link_measurements.update(_pu_system.get_dc_link_bases(),
                voltage_t(0.0),  // DC voltage not available in plant model
                current_t(plant_model.get_dc_link_current()));

            _machine_measurements.update(_pu_system.get_machine_bases(),
                abc<voltage_t>{
                    voltage_t(machine_voltages.a()), voltage_t(machine_voltages.b()), voltage_t(machine_voltages.c())},
                abc<current_t>{
                    current_t(machine_currents.a()), current_t(machine_currents.b()), current_t(machine_currents.c())});

            // Magnitude...
            // Create a grid-side observer class
            masked_voltage_pu =
                _commutation_mask.update(_grid_measurements.get_voltage_pu(), _grid_side_pll.get_estimated_angle(),
                    frequency_t(_grid_side_pll.get_estimated_frequency()), _commutation_monitor.is_commuting(), 10e-6);

            _grid_side_pll.update(to_dimensionless_alphabeta(masked_voltage_pu.to_alphabeta()));

            // Update pulse generators
            // Create firing angle objects
            angle_wrapped const grid_firing_angle(deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg));
            angle_wrapped const machine_firing_angle(
                deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg));

            _grid_pulses = _grid_side_pulse_generator.update(grid_reference_angle, grid_firing_angle, true);
            _machine_pulses = _machine_side_pulse_generator.update(machine_reference_angle, machine_firing_angle, true);
            _commutation_monitor.update(_grid_measurements.get_voltage_pu(), _grid_pulses);

            // Set the generated pulses as inputs to the plant model
            // _commutation_monitor.update(_grid_measurements.get_voltage_pu(), _grid_measurements.get_current_pu(),
            // _dc_link_measurements.get_current_pu(), _grid_pulses);
            // Step the plant
            _plant_sim.plant().set_grid_side_pulses(_grid_pulses.to_array());
            _plant_sim.plant().set_machine_side_pulses(_machine_pulses.to_array());
            _plant_sim.plant().step();
        });
    }

    void setup_logging()
    {
        _simulator.register_sampler_entry(yeet::make_lambda_array_sampler<alphabeta<voltage_pu_t>>(
            "grid xy voltage", [&]() { return _grid_measurements.get_voltage_pu().to_alphabeta(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_array_sampler<alphabeta<real_t>>(
            "estimated xy voltage", [&]() { return _grid_side_pll.get_positive_sequence(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_array_sampler<abc<voltage_pu_t>>(
            "masked_voltage_pu", [&]() { return masked_voltage_pu; }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "plant angle", [&]() { return _plant_sim.plant().get_grid_voltage_source_angle().get_pu(); }));

        _simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "estimated angle", [&]() { return _grid_side_pll.get_estimated_angle().get_pu(); }));

        _simulator.register_sampler_entry(
            yeet::make_lambda_array_sampler<abc<voltage_pu_t>>("grid_line_voltage_pu", [&]() {
                return abc<voltage_pu_t>{
                    (_grid_measurements.get_voltage_pu().a() - _grid_measurements.get_voltage_pu().b()) /
                        std::sqrt(3.0f),
                    (_grid_measurements.get_voltage_pu().b() - _grid_measurements.get_voltage_pu().c()) /
                        std::sqrt(3.0f),
                    (_grid_measurements.get_voltage_pu().c() - _grid_measurements.get_voltage_pu().a()) /
                        std::sqrt(3.0f)};
            }));

        // Add voltage notch detection logging
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<bool>("is commuting", [&]() { return _commutation_monitor.is_commuting(); }));

        // Add voltage notch detection logging
        _simulator.register_sampler_entry(
            yeet::make_lambda_sampler<bool>("is masking", [&]() { return _commutation_mask.is_masking(); }));
    }
};

TEST_F(SfcCommutationMonitorTest, Run) { _simulator.simulate_for(1.0f); }