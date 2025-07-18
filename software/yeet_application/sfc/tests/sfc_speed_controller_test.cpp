#include "sfc/sfc_speed_controller.hpp"

#include <gtest/gtest.h>

#include "sfc/sfc_current_controller.hpp"
#include "sfc/sfc_filtering.hpp"
#include "sfc/sfc_measurements.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_plant_model_simulator.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::control;
using yeet::current_t;
using yeet::duration_t;
using yeet::frequency_t;
using yeet::power_t;
using yeet::voltage_t;
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::control;
using yeet::angle;
using yeet::deg_to_rad;
using yeet::rad_to_deg;
using yeet::zero_to_two_pi;

namespace yeet::application::sfc {

/**
 * @brief Test fixture for validating the sfc_speed_controller in a closed-loop simulation.
 *
 * This test instantiates the speed controller and a mechanical load model to
 * verify that the controller can regulate the load's speed to a given reference.
 * It simulates an ideal inner torque loop by directly connecting the controller's
 * output to the mechanical load's input.
 */
class SfcSpeedControllerSimulationTest : public ::testing::Test {
protected:
    // Simulation and Control Constants
    static constexpr double kSamplingTime = 0.001;   // 1 ms
    static constexpr double kSimulationTime = 10.0;  // 5 seconds to allow settling
    static constexpr double kLoadTorquePu = 0.0;

    // Components
    sfc_speed_controller _speed_controller{kSamplingTime};
    mechanical_load _mechanical_load{kSamplingTime};
    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();

    //
    double speed_reference_pu = 0.0;

    void SetUp() override {
        // 1. Configure the mechanical load (plant)
        const double inertia_constant_seconds = 2.0;  // Inertia constant in seconds
        const double damping_factor_pu = 0.01;        // Damping factor in per-unit
        _mechanical_load.configure(inertia_constant_seconds, damping_factor_pu);
        _mechanical_load.reset();

        // 2. Configure the speed controller
        _speed_controller.set_pi_gains(10, 0.5);  // Kp, Ti - tuned for this plant
        _speed_controller.set_torque_limits_pu(-10.0, 10.0);
        _speed_controller.configure_speed_reference_filter(0.2);  // 0.2 Hz filter for smooth ramp
        _speed_controller.reset();

        // 3. Setup logging and the main simulation loop
        setup_logging();
        setup_simulation_loop();
    }

    void setup_simulation_loop() {
        _simulator.register_lambda_runner(kSamplingTime, [&]() {
            // Get measurement from the plant
            const double measured_speed_pu = _mechanical_load.get_speed_pu();

            // Update the controller
            const double torque_ref_pu = _speed_controller.update(speed_reference_pu, measured_speed_pu);

            // Apply controller output to the plant
            _mechanical_load.update(torque_ref_pu, kLoadTorquePu);
        });
    }

    void setup_logging() {
        _simulator.register_sampler_entry(
            yeet::testing::make_lambda_sampler<double>("speed_ref_pu", [&]() { return speed_reference_pu; }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "speed_ref_filtered_pu", [&]() { return _speed_controller.get_filtered_speed_ref_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "speed_measured_pu", [&]() { return _mechanical_load.get_speed_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "torque_ref_pu", [&]() { return _speed_controller.get_torque_ref_pu(); }));
    }
};

TEST_F(SfcSpeedControllerSimulationTest, ShouldRegulateSpeedToReference) {
    EXPECT_NEAR(_mechanical_load.get_speed_pu(), speed_reference_pu, 0.01);

    speed_reference_pu = 1.0;
    _simulator.simulate_for(kSimulationTime);
    EXPECT_NEAR(_mechanical_load.get_speed_pu(), speed_reference_pu, 0.01);

    speed_reference_pu = 0.5;
    _simulator.simulate_for(kSimulationTime);
    EXPECT_NEAR(_mechanical_load.get_speed_pu(), speed_reference_pu, 0.01);
}

/**
 * @brief Test fixture for validating the sfc_speed_controller in a complete LCI drive simulation.
 *
 * This test instantiates the full simulation environment, including the plant with variable speed,
 * measurements, filtering, current controller, and speed controller. It verifies that the
 * speed controller can regulate the motor speed to a given reference through the current loop.
 */
class SfcSpeedControllerPlantModelSimulationTest : public ::testing::Test {
protected:
    static constexpr double kMachineFiringAngleDeg = 180.0 - 40.0;  // Inverter operation
    static constexpr double kSimulationTime = 5.0;                  // 5 seconds simulation
    static constexpr double kLoadTorquePu = 0.1;                    // Small load torque

    // Simulation signals
    double speed_reference_pu = 1.0;

    // Simulation components
    sfc_plant_model_simulator _plant_sim;
    yeet::testing::simulator _simulator = yeet::testing::make_gtest_simulator();
    six_pulse_generator _grid_side_pulse_generator;
    six_pulse_generator _machine_side_pulse_generator;

    // System definition
    sfc_per_unit_system _pu_system{yeet::voltage_t(sfc_plant_model_simulator::kNominalGridVoltageRMS),
                                   yeet::frequency_t(sfc_plant_model_simulator::kNominalGridFrequency),
                                   yeet::current_t(sfc_plant_model_simulator::kBaseDCLinkCurrent),
                                   yeet::power_t(sfc_plant_model_simulator::kMachineRatedPowerVA),
                                   yeet::voltage_t(sfc_plant_model_simulator::kNominalMachineVoltageRMS),
                                   yeet::frequency_t(sfc_plant_model_simulator::kNominalMachineFrequency),
                                   sfc_plant_model_simulator::kMachineNumPoles};

    // Measurement, Filtering, and Control classes
    sfc_grid_side_measurements _grid_measurements;
    sfc_dc_link_measurements _dc_link_measurements;
    sfc_machine_side_measurements _machine_measurements;
    sfc_grid_side_filtering _grid_filtering{yeet::duration_t(sfc_6p_6p_plant_model::kSamplingTime)};
    sfc_dc_link_filtering _dc_link_filtering{yeet::duration_t(sfc_6p_6p_plant_model::kSamplingTime)};
    sfc_machine_side_filtering _machine_filtering{yeet::duration_t(sfc_6p_6p_plant_model::kSamplingTime)};
    sfc_current_controller _current_controller{yeet::duration_t(sfc_6p_6p_plant_model::kSamplingTime)};
    sfc_speed_controller _speed_controller{yeet::duration_t(sfc_6p_6p_plant_model::kSamplingTime)};

    void SetUp() override {
        auto& plant_model = _plant_sim.plant();

        // 1. Configure plant for variable speed operation
        _plant_sim.set_operation_mode(sfc_plant_model_simulator::machine_operation_mode::variable_speed);
        _plant_sim.configure_mechanical_load(0.5, 0.0001);  // 2 s inertia, 0.01 pu damping
        _plant_sim.set_initial_mechanical_speed_pu(speed_pu_t(1.0));

        // 2. Initialize the plant model
        plant_model.initialize();
        plant_model.set_grid_parameters(sfc_plant_model_simulator::kNominalGridFrequency,
                                        sfc_plant_model_simulator::kNominalGridVoltageRMS);
        plant_model.set_machine_parameters(sfc_plant_model_simulator::kNominalMachineFrequency,
                                           sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        // 3. Configure filtering
        _grid_filtering.configure(yeet::frequency_t(50.0), yeet::frequency_t(50.0));
        _grid_filtering.configure_voltage_magnitude_filter(yeet::frequency_t(20.0));
        _dc_link_filtering.configure(yeet::frequency_t(100.0), yeet::frequency_t(2.0 * 6.0 * 50.0));
        _machine_filtering.configure_voltage_filter(yeet::frequency_t(100.0));
        _machine_filtering.configure_current_filter(yeet::frequency_t(100.0));
        _machine_filtering.configure_voltage_magnitude_filter(yeet::frequency_t(20.0));

        // 4. Configure the current controller (inner loop)
        _current_controller.set_pi_gains(1.0, 0.02);  // Kp, Ti
        _current_controller.set_current_limits_pu(0.0, 2.0);
        _current_controller.set_firing_angle_limits_radians(deg_to_rad(10.0), deg_to_rad(90.0));

        // 5. Configure the speed controller (outer loop)
        _speed_controller.set_pi_gains(5.0, 1.0);  // Kp, Ti - tuned for mechanical load
        _speed_controller.set_torque_limits_pu(-2.0, 2.0);
        _speed_controller.configure_speed_reference_filter(0.5);  // 0.5 Hz filter for smooth ramp
        _speed_controller.reset(speed_reference_pu);

        // 6. Setup logging and the main simulation loop
        setup_logging();
        setup_simulation_loop();
    }

    void setup_simulation_loop() {
        _simulator.register_lambda_runner(sfc_6p_6p_plant_model::kSamplingTime, [&]() {
            auto& plant = _plant_sim.plant();

            // --- Measurement and Filtering ---
            _grid_measurements.update(_pu_system.get_grid_bases(),
                                      abc<voltage_t>{yeet::voltage_t(plant.get_grid_side_voltages_abc().a()),
                                                     yeet::voltage_t(plant.get_grid_side_voltages_abc().b()),
                                                     yeet::voltage_t(plant.get_grid_side_voltages_abc().c())},
                                      abc<current_t>{yeet::current_t(plant.get_grid_side_currents_abc().a()),
                                                     yeet::current_t(plant.get_grid_side_currents_abc().b()),
                                                     yeet::current_t(plant.get_grid_side_currents_abc().c())});
            _dc_link_measurements.update(_pu_system.get_dc_link_bases(), yeet::voltage_t(0.0),
                                         yeet::current_t(plant.get_dc_link_current()));
            _machine_measurements.update(_pu_system.get_machine_bases(),
                                         abc<voltage_t>{yeet::voltage_t(plant.get_machine_side_voltages_abc().a()),
                                                        yeet::voltage_t(plant.get_machine_side_voltages_abc().b()),
                                                        yeet::voltage_t(plant.get_machine_side_voltages_abc().c())},
                                         abc<current_t>{yeet::current_t(plant.get_machine_side_currents_abc().a()),
                                                        yeet::current_t(plant.get_machine_side_currents_abc().b()),
                                                        yeet::current_t(plant.get_machine_side_currents_abc().c())});

            _grid_filtering.update(_grid_measurements.get_voltage_pu(), _grid_measurements.get_current_pu());
            _dc_link_filtering.update(voltage_pu_t(_dc_link_measurements.get_voltage_pu()),
                                      current_pu_t(_dc_link_measurements.get_current_pu()));
            _machine_filtering.update(_machine_measurements.get_voltage_pu(), _machine_measurements.get_current_pu(),
                                      yeet::frequency_t(sfc_plant_model_simulator::kNominalMachineFrequency));

            // --- Speed Control (Outer Loop) ---
            const double measured_speed_pu = _plant_sim.mechanical_load().get_speed_pu();
            const double torque_ref_pu = _speed_controller.update(speed_reference_pu, measured_speed_pu);

            // --- Current Control (Inner Loop) ---
            // Convert torque reference to current reference
            // For LCI drives: I_dc = T_e / flux (assuming unit flux in pu)
            const double dc_link_current_ref_pu = torque_ref_pu;

            // Estimate inverter back-EMF for feedforward term
            const double v_inv_est_pu = 0.2;  // Simplified estimate

            // Update the current controller
            const double grid_firing_angle_ref = _current_controller.update(
                dc_link_current_ref_pu, _dc_link_filtering.get_filtered_current_pu(), v_inv_est_pu);

            // --- Actuation ---
            update_pulse_generators(grid_firing_angle_ref);

            // Step the plant with variable speed mode
            _plant_sim.step(0.75);
        });
    }

    void setup_logging() {
        _simulator.register_sampler_entry(
            yeet::testing::make_lambda_sampler<double>("speed_ref_pu", [&]() { return speed_reference_pu; }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "speed_ref_filtered_pu", [&]() { return _speed_controller.get_filtered_speed_ref_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "speed_measured_pu", [&]() { return _plant_sim.mechanical_load().get_speed_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "torque_ref_pu", [&]() { return _speed_controller.get_torque_ref_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_ref_pu", [&]() { return _current_controller.get_current_ref_limited_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_measured_pu", [&]() { return _dc_link_measurements.get_current_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "dc_link_current_filtered_pu", [&]() { return _dc_link_filtering.get_filtered_current_pu(); }));
        _simulator.register_sampler_entry(yeet::testing::make_lambda_sampler<double>(
            "grid_firing_angle_cos_alpha_clamped",
            [&]() { return rad_to_deg(_current_controller.get_cos_alpha_clamped()); }));
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

TEST_F(SfcSpeedControllerPlantModelSimulationTest, ShouldRegulateSpeedToReference) {
    // Initial condition: speed should start near zero
    EXPECT_NEAR(_plant_sim.mechanical_load().get_speed_pu(), speed_reference_pu, 0.05);

    // Set speed reference to 1.0 pu and run simulation
    speed_reference_pu = 1.0;
    _simulator.simulate_for(5.0);
    EXPECT_NEAR(_plant_sim.mechanical_load().get_speed_pu(), speed_reference_pu, 0.05);

    speed_reference_pu = 0.9;
    _simulator.simulate_for(5.0);

    speed_reference_pu = 0.6;
    _simulator.simulate_for(5.0);

    // // Change speed reference to 0.5 pu and run simulation
    // speed_reference_pu = 0.5;
    // _simulator.simulate_for(kSimulationTime);
    // EXPECT_NEAR(_plant_sim.mechanical_load().get_speed_pu(), speed_reference_pu, 0.05);
}

}  // namespace yeet::application::sfc