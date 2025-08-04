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
    // Simulation signals
    speed_pu_t speed_ref_pu = speed_pu_t{1.0};  // Reference speed in per-unit

    // Simulation components using the new pattern
    sfc_simulation_components components;
    sfc_pulse_generator_conduit pulse_generator_conduit;
    yeet::simulator simulator = yeet::make_gtest_simulator(1000e-6);  // 1kHz sampling rate

    const duration_t start_up_time = duration_t(0.5);  // 500ms startup time

    void SetUp() override
    {
        // Configure plant model for variable speed operation
        components.plant_model_simulator.set_operation_mode(
            sfc_plant_model_simulator::machine_operation_mode::variable_speed);
        components.plant_model_simulator.configure_mechanical_load(
            inertia_constant_t{0.5}, damping_coefficient_pu_t{0.001});  // 0.5s inertia, 0.001 damping
        components.plant_model_simulator.set_initial_mechanical_speed_pu(speed_pu_t(1.0));

        // Configure the machine-side lookup table
        components.machine_side_lookup_table.configure_table(
            20.0, to_pu(components.plant_model_simulator.kMachineSideInductance,
                      components.plant_model_simulator.per_unit_system()
                          .get_machine_bases()
                          .base_inductance));  // gamma=20°, inductance=0.1pu

        // Configure plant model parameters
        components.plant_model_simulator.plant().set_grid_parameters(
            sfc_plant_model_simulator::kNominalGridFrequency, sfc_plant_model_simulator::kNominalGridVoltageRMS);
        components.plant_model_simulator.plant().set_machine_parameters(
            sfc_plant_model_simulator::kNominalMachineFrequency, sfc_plant_model_simulator::kNominalMachineVoltageRMS);

        // Configure the speed controller (outer loop)
        components.speed_controller.configure_pi_gains(10.0, duration_t(0.5));  // Kp, Ti
        components.speed_controller.speed_controller.configure_torque_limits_pu(torque_pu_t(-1.5), torque_pu_t(1.5));
        components.speed_controller.speed_controller.configure_speed_reference_limits_pu(
            speed_pu_t(0.0), speed_pu_t(1.5));
        components.speed_controller.speed_controller.configure_speed_reference_rate_limit(0.2);  // 1.0 pu/s rate limit
        components.speed_controller.speed_controller.configure_feedforward_pu(0.0);  // Small feedforward gain
        components.speed_controller.speed_controller.reset(speed_pu_t{1.0});

        // Configure the current controller (inner loop) - enable for cascaded operation
        components.current_controller.configure_pi_gains(0.2, duration_t(ms(100.0)));  // Kp, Ti
        components.current_controller.configure_current_reference_slew_rate(10000.0);
        components.current_controller.configure_current_limits_pu(current_pu_t(-1.5), current_pu_t(1.5));
        components.current_controller.configure_firing_angle_limits_radians(
            angle_t(deg_to_rad(0.0)), angle_t(deg_to_rad(90.0)));

        components.measurement_and_filtering.dc_link_filtering.configure_current_filter(frequency_t{10.0});

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
            // .with_ideal_pulse_generation(pulse_generator_conduit)
            .with_speed_controller(pulse_generator_conduit)
            .with_machine_side_lookup_table(pulse_generator_conduit)
            .with_current_controller(pulse_generator_conduit)
            .configure(simulator);

        // Set up the measurement update and enable control once observers are locked
        simulator.register_lambda_runner(components.kControlSamplingTime, [&]() {
            // Enable control once observers are locked
            pulse_generator_conduit.enable = components.observers.grid_side_observer.is_locked();
            if (!pulse_generator_conduit.enable) {
                // If observers are not locked, skip updates
                return;
            }

            // Update speed reference
            pulse_generator_conduit.speed_ref_pu = speed_ref_pu;

            // Get measured speed from the mechanical load
            pulse_generator_conduit.measured_speed_pu =
                speed_pu_t{components.observers.machine_side_observer.get_estimated_frequency_pu().value()};

            // Get measured current for current controller
            pulse_generator_conduit.measured_current_pu =
                components.measurement_and_filtering.get_dc_link_filtering().get_filtered_current_pu();
        });

        setup_logging();

        // Until pulses are released
        simulator.simulate_for(start_up_time.value());
    }

    void setup_logging()
    {
        auto& measurement_and_filtering = components.measurement_and_filtering;

        simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
            "speed_measured_pu", [&]() { return pulse_generator_conduit.measured_speed_pu; }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>("speed_actual_pu",
            [&]() { return speed_pu_t{components.plant_model_simulator.mechanical_load().get_speed_pu()}; }));

        simulator.register_sampler_entry(
            yeet::make_lambda_sampler<double>("speed_error_pu", [&]() { return get_speed_error_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<speed_pu_t>(
            "speed_ref", [&]() { return components.speed_controller.get_controller().get_processed_speed_ref_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<torque_pu_t>(
            "torque_ref_pu", [&]() { return components.speed_controller.get_controller().get_torque_ref_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<current_pu_t>("dc_link_current_pu",
            [&]() { return measurement_and_filtering.get_dc_link_filtering().get_filtered_current_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "grid_firing_angle_pu", [&]() { return pulse_generator_conduit.grid_firing_angle_rad.get_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<double>(
            "machine_firing_angle_pu", [&]() { return pulse_generator_conduit.machine_firing_angle_rad.get_pu(); }));

        simulator.register_sampler_entry(yeet::make_lambda_sampler<voltage_pu_t>("machine_voltage_magnitude",
            [&]() { return components.observers.machine_side_observer.get_estimated_voltage_magnitude(); }));
    }

    double get_speed_error_pu() const { return (speed_ref_pu - pulse_generator_conduit.measured_speed_pu).value(); }
};

// Test regulation of speed to a reference value
TEST_F(SfcSpeedControllerSimulationTest, TestSpeedRegulation)
{
    const double speed_error_tolerance = 0.05;  // 5% tolerance

    // Register checkers for speed tracking performance
    simulator.register_checker(yeet::make_range_checker<double>(
        "speed error during steady state", [&]() -> double { return std::abs(get_speed_error_pu()); }, 0.0,
        speed_error_tolerance,
        yeet::checker_activation_time_range{2.0 + start_up_time.value(), 3.0 + start_up_time.value()}));

    simulator.register_checker(yeet::make_range_checker<double>(
        "speed error after load step", [&]() -> double { return std::abs(get_speed_error_pu()); }, 0.0,
        speed_error_tolerance,
        yeet::checker_activation_time_range{8.0 + start_up_time.value(), 10.0 + start_up_time.value()}));

    simulator.register_checker(yeet::make_range_checker<double>(
        "speed error after speed reference change", [&]() -> double { return std::abs(get_speed_error_pu()); }, 0.0,
        speed_error_tolerance, yeet::checker_activation_time_range{20.0 + start_up_time.value()}));

    // Phase 1: System stabilization at nominal speed with light load
    speed_ref_pu = speed_pu_t{1.0};
    components.plant_model_simulator.set_load_torque_pu(torque_pu_t{0.4});
    bool success = simulator.simulate_for(3.0);  // Allow observers to lock and system to stabilize

    // Phase 2: Test load torque step response (disturbance rejection)
    components.plant_model_simulator.set_load_torque_pu(torque_pu_t{0.6});  // Moderate load step
    success &= simulator.simulate_for(5.0);                                 // Allow system to respond and settle

    // Phase 3: Test higher load step
    components.plant_model_simulator.set_load_torque_pu(torque_pu_t{1.0});  // High load step
    success &= simulator.simulate_for(5.0);                                 // Test sustained high-load operation

    // Phase 4: Test speed reference change with load
    speed_ref_pu = speed_pu_t{0.8};          // Speed step down
    success &= simulator.simulate_for(5.0);  // Test speed tracking under load

    // Phase 5: Return to nominal conditions
    speed_ref_pu = speed_pu_t{1.0};
    components.plant_model_simulator.set_load_torque_pu(torque_pu_t{0.5});
    success &= simulator.simulate_for(3.0);

    EXPECT_TRUE(success) << "Speed regulation test did not complete successfully.";
}

// Test speed reference clamping limits
TEST_F(SfcSpeedControllerSimulationTest, TestSpeedReferenceLimits)
{
    components.plant_model_simulator.set_load_torque_pu(torque_pu_t{0.5});

    // Configure limits: min=0.2, max=1.1
    const speed_pu_t min_limit = speed_pu_t{0.9};
    const speed_pu_t max_limit = speed_pu_t{1.1};
    components.speed_controller.speed_controller.configure_speed_reference_limits_pu(speed_pu_t(0.9), speed_pu_t(1.1));

    speed_ref_pu = speed_pu_t{1.0};  // Above max limit
    bool success = simulator.simulate_for(1.0);

    // Test upper limit clamping
    speed_ref_pu = speed_pu_t{0.8};  // Below min min

    // Check that processed reference is clamped to minimum
    simulator.register_checker(yeet::make_range_checker<double>(
        "processed speed ref should be clamped to min",
        [&]() -> double { return components.speed_controller.get_controller().get_processed_speed_ref_pu().value(); },
        min_limit - 0.001, min_limit + 0.001,
        yeet::checker_activation_time_range{
            simulator.get_current_simulation_time() + 1.0f, simulator.get_current_simulation_time() + 2.5f}));

    simulator.register_checker(yeet::make_range_checker<double>(
        "actual speed should be clamped to min",
        [&]() -> double { return components.observers.machine_side_observer.get_estimated_frequency_pu().value(); },
        min_limit - 0.05, min_limit + 0.05,
        yeet::checker_activation_time_range{
            simulator.get_current_simulation_time() + 1.0f, simulator.get_current_simulation_time() + 2.5f}));

    success &= simulator.simulate_for(3.0);

    // Test lower limit clamping
    speed_ref_pu = speed_pu_t{1.2};  // Above max

    // Check that processed reference is clamped to minimum
    simulator.register_checker(yeet::make_range_checker<double>(
        "processed speed ref should be clamped to max",
        [&]() -> double { return components.speed_controller.get_controller().get_processed_speed_ref_pu().value(); },
        max_limit - 0.001, max_limit + 0.001,
        yeet::checker_activation_time_range{simulator.get_current_simulation_time() + 1.5f}));

    simulator.register_checker(yeet::make_range_checker<double>(
        "actual speed should be clamped to max",
        [&]() -> double { return components.observers.machine_side_observer.get_estimated_frequency_pu().value(); },
        max_limit - 0.05, max_limit + 0.05,
        yeet::checker_activation_time_range{simulator.get_current_simulation_time() + 1.5f}));

    success &= simulator.simulate_for(3.0);
    EXPECT_TRUE(success) << "Speed reference limits test did not complete successfully.";
}

// TODO: feed-forward, torque limits, rate of change limiter

TEST_F(SfcSpeedControllerSimulationTest, TestSpeedRegulationVariableSpeedModel)
{
    const double speed_error_tolerance = 0.05;  // 5% tolerance

    components.speed_controller.configure_pi_gains(5.0, duration_t(1.0));  // Kp, Ti

    // Configure the plant model for variable speed operation
    components.plant_model_simulator.set_voltage_control_mode(
        sfc_plant_model_simulator::voltage_control_mode::speed_dependent);

    components.plant_model_simulator.configure_speed_voltage_relationship(1.0, 0.0, 1.0f / 50.0f);
    components.plant_model_simulator.set_initial_mechanical_speed_pu(speed_pu_t{1.0});

    // Register checkers for speed tracking performance
    // simulator.register_checker(yeet::make_range_checker<double>(
    //     "speed error during steady state", [&]() -> double { return std::abs(get_speed_error_pu()); }, 0.0,
    //     speed_error_tolerance,
    //     yeet::checker_activation_time_range{2.0 + start_up_time.value(), 3.0 + start_up_time.value()}));

    // simulator.register_checker(yeet::make_range_checker<double>(
    //     "speed error after load step", [&]() -> double { return std::abs(get_speed_error_pu()); }, 0.0,
    //     speed_error_tolerance,
    //     yeet::checker_activation_time_range{8.0 + start_up_time.value(), 10.0 + start_up_time.value()}));

    // simulator.register_checker(yeet::make_range_checker<double>(
    //     "speed error after speed reference change", [&]() -> double { return std::abs(get_speed_error_pu()); }, 0.0,
    //     speed_error_tolerance, yeet::checker_activation_time_range{20.0 + start_up_time.value()}));

    components.plant_model_simulator.set_load_torque_pu(torque_pu_t{0.5});

    // Phase 1: System stabilization at nominal speed with light load
    speed_ref_pu = speed_pu_t{1.0};
    simulator.simulate_for(3.0);  // Allow observers to lock and system to stabilize

    // Phase 1: System stabilization at nominal speed with light load
    speed_ref_pu = speed_pu_t{0.90};
    simulator.simulate_for(15.0);  // Allow observers to lock and system to stabilize
}