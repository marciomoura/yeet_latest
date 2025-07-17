#include "sfc/sfc_current_controller.hpp"

#include <gtest/gtest.h>

#include "sfc/sfc_filtering.hpp"
#include "sfc/sfc_measurements.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_plant_model_simulator.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"


using namespace yeet::application::sfc;
using namespace yeet::control;

/**
 * @brief Test fixture for validating the sfc_current_controller in a closed-loop simulation.
 *
 * This test instantiates the full simulation environment, including the plant,
 * measurements, filtering, and the controller itself. It verifies that the
 * controller can regulate the DC-link current to a given reference.
 */
class SfcCurrentControllerSimulationTest : public ::testing::Test {
protected:
    static constexpr double kMachineFiringAngleDeg = 180.0 - 40.0 + 30.0;  // Inverter operation

    // Simulation signals
    double dc_link_current_ref_pu = 1.0;  // Reference current in per-unit

    // Simulation components
    sfc_plant_model_simulator _plant_sim;
    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();
    six_pulse_generator _grid_side_pulse_generator;
    six_pulse_generator _machine_side_pulse_generator;

    // System definition
    sfc_per_unit_system _pu_system{
        sfc_plant_model_simulator::kGridVoltageRMS,    sfc_plant_model_simulator::kNominalGridFrequency,
        sfc_plant_model_simulator::kBaseDCLinkCurrent, sfc_plant_model_simulator::kMachineRatedPowerVA,
        sfc_plant_model_simulator::kMachineVoltageRMS, sfc_plant_model_simulator::kNominalMachineFrequency,
        sfc_plant_model_simulator::kMachineNumPoles};

    // Measurement, Filtering, and Control classes
    sfc_grid_side_measurements _grid_measurements;
    sfc_dc_link_measurements _dc_link_measurements;
    sfc_machine_side_measurements _machine_measurements;
    sfc_grid_side_filtering _grid_filtering{sfc_6p_6p_plant_model::kSamplingTime};
    sfc_dc_link_filtering _dc_link_filtering{sfc_6p_6p_plant_model::kSamplingTime};
    sfc_machine_side_filtering _machine_filtering{sfc_6p_6p_plant_model::kSamplingTime};
    sfc_current_controller _current_controller{sfc_6p_6p_plant_model::kSamplingTime};

    void SetUp() override {
        auto& plant_model = _plant_sim.plant();

        // 1. Initialize the plant model
        plant_model.initialize();
        plant_model.set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                        sfc_plant_model_simulator::kGridVoltageRMS);
        plant_model.set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency,
                                           sfc_plant_model_simulator::kMachineVoltageRMS);

        // 2. Configure filtering
        _grid_filtering.configure(50.0, 50.0);
        _grid_filtering.configure_voltage_magnitude_filter(20.0);
        _dc_link_filtering.configure(100.0, 6.0 * 50.0);
        _machine_filtering.configure_voltage_filter(100.0);
        _machine_filtering.configure_current_filter(100.0);
        _machine_filtering.configure_voltage_magnitude_filter(20.0);

        // 3. Configure the current controller
        _current_controller.set_pi_gains(0.5, 0.05);  // Kp, Ti
        _current_controller.set_current_limits_pu(0.0, 1.2);
        _current_controller.set_firing_angle_limits_radians(deg_to_rad(10.0), deg_to_rad(90.0));

        // 4. Setup logging and the main simulation loop
        setup_logging();
        setup_simulation_loop();
    }

    void setup_simulation_loop() {
        _simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            auto& plant = _plant_sim.plant();

            // --- Measurement and Filtering ---
            _grid_measurements.update(_pu_system.get_grid_bases(), plant.get_grid_side_voltages_abc(),
                                      plant.get_grid_side_currents_abc());
            _dc_link_measurements.update(_pu_system.get_dc_link_bases(), 0.0f, plant.get_dc_link_current());
            _machine_measurements.update(_pu_system.get_machine_bases(), plant.get_machine_side_voltages_abc(),
                                         plant.get_machine_side_currents_abc());

            _grid_filtering.update(_grid_measurements.get_voltage_pu(), _grid_measurements.get_current_pu());
            _dc_link_filtering.update(_dc_link_measurements.get_voltage_pu(), _dc_link_measurements.get_current_pu());
            _machine_filtering.update(_machine_measurements.get_voltage_pu(), _machine_measurements.get_current_pu(),
                                      sfc_plant_model_simulator::kNominalMachineFrequency);

            // --- Control ---
            // Estimate inverter back-EMF for feedforward term
            const double v_inv_est_pu = {};
            // _machine_filtering.get_filtered_voltage_magnitude_pu() * std::cos(deg_to_rad(kMachineFiringAngleDeg));

            // Update the current controller
            const double grid_firing_angle_ref = _current_controller.update(
                dc_link_current_ref_pu, _dc_link_filtering.get_filtered_current_pu(), v_inv_est_pu + 0.2f);

            // --- Actuation ---
            update_pulse_generators(grid_firing_angle_ref);
            plant.step();
        });
    }

    void setup_logging() {
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_ref_pu", [&]() { return dc_link_current_ref_pu; }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_measured_pu", [&]() { return _dc_link_measurements.get_current_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_ref_limited_pu", [&]() { return _current_controller.get_current_ref_limited_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_firing_angle_ref_deg", [&]() { return _current_controller.get_cos_alpha_clamped(); }));
        // dc-link current filtered
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_filtered_pu", [&]() { return _dc_link_filtering.get_filtered_current_pu(); }));
        // Grid voltage and current measurements
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_voltage_a_pu", [&]() { return _grid_measurements.get_voltage_pu().a(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_current_a_pu", [&]() { return _grid_measurements.get_current_pu().a(); }));
    }

    void update_pulse_generators(double grid_firing_angle_ref_radians) {
        auto& plant_model = _plant_sim.plant();
        auto grid_reference_angle = plant_model.get_grid_voltage_source_angle();
        auto machine_reference_angle = plant_model.get_machine_voltage_source_angle();

        angle<zero_to_two_pi> grid_firing_angle(grid_firing_angle_ref_radians);
        angle<zero_to_two_pi> machine_firing_angle(deg_to_rad(kMachineFiringAngleDeg));

        auto grid_pulses = _grid_side_pulse_generator.update(grid_reference_angle, grid_firing_angle, true);
        auto machine_pulses = _machine_side_pulse_generator.update(machine_reference_angle, machine_firing_angle, true);

        plant_model.set_grid_side_pulses(grid_pulses.to_array());
        plant_model.set_machine_side_pulses(machine_pulses.to_array());
    }
};

TEST_F(SfcCurrentControllerSimulationTest, ShouldRegulateDcCurrentToReference) {
    // Act: Run the simulation.
    _simulator.simulate_for(0.5f);

    dc_link_current_ref_pu = 0.5;

    _simulator.simulate_for(5.0f);
}
