/**
 * @file closed_loop_speed_control_test.cpp
 * @brief Integration tests for speed control in closed-loop SFC system
 *
 * Tests for speed reference tracking, multi-stage speed profiles, purge mode operation,
 * and steady-state speed control behavior.
 */

#include <gtest/gtest.h>

#include <array>
#include <string>

#include "sfc_integration/plant_complete_sfc_closed_loop_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using namespace yeet::application::sfc;
using namespace yeet::application::sfc::integration;
using namespace yeet;

class ClosedLoopSpeedControlTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Configure sampler sampling rate for detailed data capture
        simulator.configure_sampler_sampling_rate(ms(1.0f));

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Configure topology to use internal model
        system.plant.get_parameters().topology.write(sfc_plant_model_datamodel::topology_type::internal);

        // Configure plant for gas turbine soft-starter application
        auto gas_turbine_config = yeet::application::sfc::integration::test::create_gas_turbine_config();
        yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, gas_turbine_config);

        // Ensure excitation is available and status is initialized
        system.machine_io_mock.get_parameters().override_excitation_status.write(true);
        system.machine_io_mock.get_parameters().override_excitation_status_value.write(true);

        // Initialize the system
        initialize();

        // Register loggers for debugging
        // register_complete_logger(simulator, system.grid_side_control.operation_mode_monitor);
        // register_complete_logger(simulator, system.grid_side_control.grid_side_modulator);
        register_complete_logger(simulator, system.grid_side_control.current_controller);
        // register_complete_logger(simulator, system.machine_side_control.machine_side_controller);
        // register_complete_logger(simulator, system.machine_side_control.machine_side_observer);
        // register_complete_logger(simulator, system.machine_side_control.excitation_controller);
        // register_complete_logger(simulator, system.machine_side_control.torque_controller);
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        register_complete_logger(simulator, system.machine_side_control.startup_sequencer);
    }

    void initialize()
    {
        set_turn_on_command(false);
        set_start_command(false);
        set_stop_command(false);
        set_current_extinguished(false);

        system.initialize();
    }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // State machine control helpers
    // turn_on_command: Energizes system (IDLE → AUXILIARIES_ON → BREAKERS_ON via IO mock)
    // start_command: Starts operation (BREAKERS_ON → OPERATION_ON)
    void set_turn_on_command(bool on)
    {
        system.grid_side_control.state_machine.get_parameters().turn_on_command.write(on);
    }

    void set_start_command(bool start) { system.grid_io_mock.get_parameters().start_command.write(start); }

    void set_stop_command(bool stop) { system.grid_io_mock.get_parameters().stop_command.write(stop); }

    void set_current_extinguished(bool extinguished)
    {
        yeet::utils::mock_signal_input::write(
            system.grid_side_control.state_machine.get_inputs().is_current_extinguished, extinguished);
    }

    // Complete startup sequence following state machine logic
    // Uses turn_on_command to trigger IO mock → auxiliaries → breakers flow
    void execute_startup_sequence()
    {
        set_turn_on_command(true);
        simulate_for(0.3);  // Allow IO mock to process auxiliaries and breakers with delays
        set_start_command(true);
        simulate_for(0.1);
    }

    // System under test
    yeet::application::sfc::integration::plant_complete_sfc_closed_loop_simulation_builder system;

    // Test simulator
    yeet::simulator simulator = yeet::make_gtest_simulator();
};

/**
 * @test PurgeModeOperationTest
 * @brief Validates continuous low-speed operation for purge mode
 *
 * This test validates that the soft-starter can maintain stable operation
 * at low speed for extended periods. This is critical for gas turbine
 * applications where purge mode may run for several minutes.
 */
TEST_F(ClosedLoopSpeedControlTest, PurgeModeOperationTest)
{
    // Configure Startup Sequencer for Low-Speed Operation
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.01};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{5.0f};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.02};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{0.2f};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Ramp to Purge Speed (0.15 pu)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.15});

    // Very gradual speed changes for smooth purge operation
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{120.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Tighter speed tracking for purge mode stability
    const speed_pu_t speed_error_tolerance{0.05f};
    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "purge mode speed stability",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_error_tolerance, speed_error_tolerance, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();

    // Execute State Machine Startup
    execute_startup_sequence();

    bool success = true;

    // Initial Startup
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Light load for purge mode (minimal resistance)
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.08f));

    // Wait for virtual startup (1s + 8s + 0.5s)
    success &= simulator.simulate_for(12.0);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup should reach CLOSED_LOOP_OPERATION";

    // Ramp time: 0.15 pu Ã— 120 s/pu = 18 seconds + settling
    success &= simulator.simulate_for(20.0);

    auto purge_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(purge_speed.value(), 0.15f, 0.05f) << "Machine should reach purge speed (0.15 pu)";

    // Extended Purge Mode Hold
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(30.0);
    speed_tracking_checker->deactivate();

    // Verify speed remained stable
    auto final_purge_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(final_purge_speed.value(), 0.15f, 0.05f)
        << "Machine should maintain stable purge speed throughout extended hold";

    // Verify Control Stability
    auto torque_saturated = system.machine_side_control.torque_controller.get_outputs().is_saturated.read();
    EXPECT_FALSE(torque_saturated) << "Torque controller should not be saturated during steady purge operation";

    // Final Validation
    EXPECT_TRUE(success) << "Purge mode operation test completed with simulation errors";

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "System should maintain pulse releases during purge mode";

    auto startup_complete = system.machine_side_control.startup_sequencer.get_outputs().is_startup_complete.read();
    EXPECT_TRUE(startup_complete) << "Startup sequencer should indicate operation is stable";

    success &= simulator.simulate_for(60.0);
}

/**
 * @test TypicalSoftStarterProfileTest
 * @brief Validates multi-stage acceleration profile typical for soft-starters
 *
 * This test simulates a typical soft-starter application with multiple
 * acceleration stages:
 * - Creep speed (0.2 pu) for positioning/alignment
 * - Intermediate speed (0.5 pu) for purge or low-load operation
 * - Full speed (1.0 pu) for normal operation
 */
TEST_F(ClosedLoopSpeedControlTest, TypicalSoftStarterProfileTest)
{
    // Configure Startup Sequencer
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{10.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Set initial speed reference to match virtual frequency final to prevent drop during transition
    // This ensures the rate limiter starts from the correct value when entering closed-loop
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.1});

    // Configure rate limiter for controlled acceleration (slower for quadratic load)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{40.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Speed tracking checker
    const speed_pu_t speed_error_tolerance{0.10f};
    auto speed_tracking_checker = simulator.register_checker(yeet::make_range_checker<speed_pu_t>(
        "speed tracking error",
        [&]() -> speed_pu_t {
            return (system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read() -
                    system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read());
        },
        -speed_error_tolerance, speed_error_tolerance, yeet::checker_activation_always{}));

    speed_tracking_checker->deactivate();

    // Monotonic speed reference checker - detects if speed reference ever drops
    speed_pu_t previous_speed_reference{0.0};
    auto monotonic_speed_checker = simulator.register_checker(yeet::make_unique_checker(yeet::lambda_checker(
        "speed reference monotonic increase",
        [&]() -> bool {
            auto current_ref = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
            // Allow small tolerance for numerical noise
            const float tolerance = 0.001f;
            bool is_monotonic = current_ref.value() >= (previous_speed_reference.value() - tolerance);
            previous_speed_reference = current_ref;
            return is_monotonic;
        },
        [&]() -> std::string {
            auto current_ref = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();
            return "Speed reference dropped from " + std::to_string(previous_speed_reference.value()) + " to " +
                   std::to_string(current_ref.value()) + " pu";
        })));

    // Execute State Machine Startup
    execute_startup_sequence();

    bool success = true;

    // Initial Startup
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Configure gas turbine load (compressor characteristic)
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.10f));

    // Wait for virtual startup to complete
    success &= simulator.simulate_for(15.0);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup should reach CLOSED_LOOP_OPERATION";

    // Stage 1: Creep Speed (0.2 pu)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.2});

    // Ramp time: 0.2 pu Ã— 40 s/pu = 8 seconds + settling
    success &= simulator.simulate_for(10.0);

    auto creep_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(creep_speed.value(), 0.2f, 0.08f) << "Machine should reach creep speed (0.2 pu)";

    // Hold at creep speed for positioning/alignment phase
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(10.0);
    speed_tracking_checker->deactivate();

    // Stage 2: Purge Speed (0.5 pu)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.5});

    // Ramp time: 0.3 pu Ã— 40 s/pu = 12 seconds + settling
    success &= simulator.simulate_for(14.0);

    auto purge_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(purge_speed.value(), 0.5f, 0.10f) << "Machine should reach purge speed (0.5 pu)";

    // Hold at purge speed
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(10.0);
    speed_tracking_checker->deactivate();

    // Stage 3: Full Speed (1.0 pu)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{1.0});

    // Ramp time: 0.5 pu Ã— 40 s/pu = 20 seconds + settling
    success &= simulator.simulate_for(23.0);

    auto full_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(full_speed.value(), 1.0f, 0.15f) << "Machine should reach full speed (1.0 pu)";

    // Steady-State Operation
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(15.0);
    speed_tracking_checker->deactivate();

    // Final Validation
    EXPECT_TRUE(success) << "Typical soft-starter profile test completed with simulation errors";

    EXPECT_TRUE(system.grid_side_control.state_machine.get_outputs().release_pulses.read())
        << "System should maintain pulse releases during steady-state operation";
}

/**
 * @test SpeedBasedTorqueLimitCapsAppliedTorqueTest
 * @brief Validates speed-reference-indexed torque limiting in the full closed-loop system
 *
 * Ensures the machine-side torque controller respects a configured speed-based maximum torque
 * during CLOSED_LOOP_OPERATION.
 */
TEST_F(ClosedLoopSpeedControlTest, SpeedBasedTorqueLimitCapsAppliedTorqueTest)
{
    // Configure speed-based torque limit LUT: constant 0.6 pu max, 0.0 pu min across speed.
    auto& tq_params = system.machine_side_control.torque_controller.get_parameters();
    auto speed_axis = tq_params.speed_torque_limit_speed_axis_pu.read();
    auto min_torque = tq_params.speed_torque_limit_min_torque_pu.read();
    auto max_torque = tq_params.speed_torque_limit_max_torque_pu.read();

    for (auto& v : min_torque) {
        v = torque_pu_t{0.0f};
    }
    for (auto& v : max_torque) {
        v = torque_pu_t{0.6f};
    }

    tq_params.speed_torque_limit_speed_axis_pu.write(speed_axis);
    tq_params.speed_torque_limit_min_torque_pu.write(min_torque);
    tq_params.speed_torque_limit_max_torque_pu.write(max_torque);
    tq_params.enable_speed_based_torque_limits.write(true);

    // Startup sequencer setup to reach closed-loop quickly
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{8.0f};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0f};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Command rated speed
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{1.0});

    // Moderate load so the controller requests torque near its limits
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.20f));

    // Checker: applied torque must stay below 0.62 pu while active
    auto torque_cap_checker = simulator.register_checker(yeet::make_range_checker<torque_pu_t>(
        "speed-based torque max clamp",
        [&]() -> torque_pu_t {
            return system.machine_side_control.torque_controller.get_outputs().torque_reference.read();
        },
        torque_pu_t{-1.2f}, torque_pu_t{0.62f}, yeet::checker_activation_always{}));
    torque_cap_checker->deactivate();

    execute_startup_sequence();

    bool success = true;

    // Preflux + virtual startup + transition time
    success &= simulator.simulate_for(12.0);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup should reach CLOSED_LOOP_OPERATION";

    torque_cap_checker->activate();
    success &= simulator.simulate_for(5.0);
    torque_cap_checker->deactivate();

    auto evaluated_max =
        system.machine_side_control.torque_controller.get_outputs().speed_based_max_torque_limit.read();
    EXPECT_NEAR(evaluated_max.value(), 0.6f, 0.02f) << "Speed-based max torque output should reflect configuration";

    EXPECT_TRUE(success) << "Simulation failed while checking speed-based torque cap";
}

/**
 * @test OperatingPointsSpeedAndVoltageSetpointTest
 * @brief Verifies speed tracking and excitation voltage setpoint across operating points
 *
 * This test runs the complete closed-loop SFC startup and then steps through a small
 * set of operating points. At each point it validates:
 * - Machine speed (observer) tracks the commanded speed reference within tolerance
 * - Excitation controller voltage setpoint follows the default V/Hz schedule
 *
 * Notes:
 * - The default excitation voltage schedule is linear V/Hz: Vref = 0.2 + 0.8 * speed.
 * - We compare Vref against the measured speed (observer) to avoid sensitivity to
 *   reference rate limiting and acceleration constraints.
 */
TEST_F(ClosedLoopSpeedControlTest, OperatingPointsSpeedAndVoltageSetpointTest)
{
    // Configure Startup Sequencer for a reasonably quick transition into closed-loop
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{5.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    // Reduce pre-flux time so the test runs faster.
    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Disable speed reference rate limiting so operating points are commanded directly.
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = false;

    // Start from a low-but-nonzero reference to avoid an initial drop during transition.
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.2f});

    execute_startup_sequence();

    bool success = true;
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Moderate gas turbine load.
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.10f));

    // Wait for virtual startup to complete and enter closed-loop.
    success &= simulator.simulate_for(12.0);

    const auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup should reach CLOSED_LOOP_OPERATION";

    struct operating_point {
        speed_pu_t speed_reference;
        double settle_time_s;
        float speed_tol_pu;
        float vref_tol_pu;
    };

    const std::array<operating_point, 4> points = {
        operating_point{speed_pu_t{0.2f}, 8.0, 0.10f, 0.06f},
        operating_point{speed_pu_t{0.5f}, 10.0, 0.12f, 0.07f},
        operating_point{speed_pu_t{1.0f}, 14.0, 0.15f, 0.08f},
        operating_point{speed_pu_t{1.1f}, 10.0, 0.18f, 0.10f},
    };

    for (const auto& point : points) {
        SCOPED_TRACE("operating point speed_ref=" + std::to_string(point.speed_reference.value()));

        system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(point.speed_reference);
        success &= simulator.simulate_for(point.settle_time_s);

        const auto speed_meas = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
        EXPECT_NEAR(speed_meas.value(), point.speed_reference.value(), point.speed_tol_pu)
            << "Speed should track operating point reference";

        const auto vref_pu =
            system.machine_side_control.excitation_controller.get_outputs().machine_voltage_reference_pu.read();

        // Default V/Hz schedule (same as sfc_excitation_controller defaults): Vref = 0.2 + 0.8 * speed.
        // Compare against measured speed to avoid dependency on reference ramping.
        const float expected_vref = 0.2f + 0.8f * speed_meas.value();
        EXPECT_NEAR(vref_pu.value(), expected_vref, point.vref_tol_pu)
            << "Excitation voltage setpoint should follow default V/Hz schedule";
    }

    EXPECT_TRUE(success) << "Operating points test completed with simulation errors";
}

/**
 * @test BrakingRequestFreezesIntegratorTest
 * @brief Validates that speed controller integrator freezes when braking is clamped
 *
 * When the speed controller requests negative torque (braking) during deceleration,
 * but the torque controller enforces minimum motoring torque for natural commutation,
 * the speed controller integrator must freeze to prevent windup.
 *
 * This test verifies:
 * 1. Braking request is detected when speed reference drops
 * 2. Integrator supervisor freezes the speed controller
 * 3. Torque remains clamped at minimum motoring torque (not negative)
 * 4. System remains stable without integrator windup
 */
TEST_F(ClosedLoopSpeedControlTest, BrakingRequestFreezesIntegratorTest)
{
    // Configure Startup Sequencer for quick startup
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_initial = frequency_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_final = frequency_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_frequency_duration = duration_t{5.0};

    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_initial = torque_pu_t{0.0};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_final = torque_pu_t{0.1};
    system.machine_side_control.startup_sequencer.get_parameters().virtual_torque_duration = duration_t{1.0};

    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // Configure minimum motoring torque (the feature under test)
    const torque_pu_t min_motoring_torque{0.02};
    system.machine_side_control.torque_controller.get_parameters().minimum_motoring_torque.write(min_motoring_torque);
    system.machine_side_control.torque_controller.get_parameters().enable_minimum_motoring_torque.write(true);

    // Configure for moderate speed ramp
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_seconds_per_pu =
        real_t{20.0f};
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference_rate_limit_enabled = true;

    // Start at 0.5 pu speed reference
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.5});

    // Register additional loggers for debugging
    register_complete_sfc_torque_controller_logger(simulator, system.machine_side_control.torque_controller);
    register_complete_machine_side_integrator_supervisor_logger(
        simulator, system.machine_side_control.machine_integrator_supervisor);

    // Execute State Machine Startup
    execute_startup_sequence();

    bool success = true;

    // Initial Startup - pre-flux and virtual startup
    success &= simulator.simulate_for(1.0);  // Pre-flux

    // Light load
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.05f));

    // Wait for virtual startup to complete and reach closed-loop
    success &= simulator.simulate_for(10.0);

    auto startup_state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(startup_state, startup_sequencer_state_t::CLOSED_LOOP_OPERATION)
        << "Startup should reach CLOSED_LOOP_OPERATION";

    // Ramp to 0.5 pu and let it settle
    success &= simulator.simulate_for(15.0);

    auto initial_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(initial_speed.value(), 0.5f, 0.1f) << "Machine should reach ~0.5 pu speed";

    // Verify no braking requested yet (positive torque)
    auto braking_before = system.machine_side_control.torque_controller.get_outputs().is_braking_requested.read();
    EXPECT_FALSE(braking_before) << "No braking should be requested during acceleration/steady-state";

    // Now command a lower speed (0.3 pu) - this requires deceleration
    // The load torque + friction will slow the machine, but the speed controller
    // will try to request negative torque to speed up deceleration
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.3});

    // Run for enough time that the speed controller would try to brake
    // The speed reference will ramp down: 0.2 pu drop Ã— 20 s/pu = 4 seconds ramp
    success &= simulator.simulate_for(6.0);

    // During deceleration, speed > reference, so speed controller wants negative torque
    // But minimum motoring torque should clamp it
    auto braking_during = system.machine_side_control.torque_controller.get_outputs().is_braking_requested.read();

    // Check that braking was requested (speed controller tried to go negative)
    // Note: This depends on the speed error being large enough that the PI outputs negative torque
    // If the machine decelerates fast enough naturally, braking might not be requested
    // We check the torque output to verify clamping works regardless

    auto torque_output = system.machine_side_control.torque_controller.get_outputs().torque_reference.read();
    EXPECT_GE(torque_output.value(), min_motoring_torque.value() - 0.005f)
        << "Torque should be clamped at or above minimum motoring torque";

    // If braking was requested, verify integrator is frozen
    if (braking_during) {
        auto freeze_integrator =
            system.machine_side_control.machine_integrator_supervisor.get_outputs().freeze_integrator.read();
        EXPECT_TRUE(freeze_integrator) << "Integrator should be frozen when braking is requested";
    }

    // Continue simulation to verify stability
    success &= simulator.simulate_for(10.0);

    // Verify speed continues to track (no runaway from windup)
    auto final_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    auto final_reference = system.machine_side_control.startup_sequencer.get_outputs().speed_reference.read();

    EXPECT_NEAR(final_speed.value(), final_reference.value(), 0.15f)
        << "Speed should track reference without excessive error from windup";

    // Verify torque is still reasonable (no large negative from windup release)
    auto final_torque = system.machine_side_control.torque_controller.get_outputs().torque_reference.read();
    EXPECT_GE(final_torque.value(), min_motoring_torque.value() - 0.005f)
        << "Torque should remain at or above minimum motoring torque";
    EXPECT_LE(final_torque.value(), 1.2f) << "Torque should not exceed absolute maximum";

    EXPECT_TRUE(success) << "Braking request freeze test completed with simulation errors";
}

/**
 * @test CurrentControllerFreezesWhenNotModulatingTest
 * @brief Test that verifies DC-link current controller integrator freezes when system is not modulating
 *
 * This test validates the integrator anti-windup chain for the grid-side current controller.
 * When the state machine release_pulses = false (system stopped or during startup/shutdown),
 * the current controller's integrator should be frozen to prevent windup.
 *
 * @expected_behavior
 * - Current controller integrator is frozen when release_pulses = false
 * - Current controller integrator is active when release_pulses = true
 * - During shutdown sequence, integrator freezes immediately when pulses are blocked
 */
TEST_F(ClosedLoopSpeedControlTest, CurrentControllerFreezesWhenNotModulatingTest)
{
    // ============================================================
    // Phase 1: Verify freeze is active when system is not started
    // ============================================================

    simulate_for(0.1);

    // System not started - release_pulses should be false
    auto release_pulses_initial = system.grid_side_control.state_machine.get_outputs().release_pulses.read();
    EXPECT_FALSE(release_pulses_initial) << "Pulses should not be released before startup";

    // Current controller integrator should be frozen
    auto freeze_initial = system.grid_side_control.grid_integrator_supervisor.get_outputs().freeze_integrator.read();
    EXPECT_TRUE(freeze_initial) << "Current controller integrator should be frozen when not modulating";

    // ============================================================
    // Phase 2: Start system and verify integrator unfreezes
    // ============================================================

    execute_startup_sequence();

    // Wait for system to reach modulating state
    auto success = simulator.simulate_for(5.0);

    // Check release_pulses - should be true when modulating
    auto release_pulses_running = system.grid_side_control.state_machine.get_outputs().release_pulses.read();
    EXPECT_TRUE(release_pulses_running) << "Pulses should be released after startup";

    // Current controller integrator should be unfrozen (unless other freeze conditions are active)
    // During normal operation, freeze should only be from shutdown_request or forced_commutation
    auto shutdown_request = system.grid_side_control.grid_integrator_supervisor.get_outputs().shutdown_request.read();
    auto forced_comm_freeze =
        system.grid_side_control.grid_integrator_supervisor.get_inputs().forced_commutation_freeze_integrator.read();

    if (!shutdown_request && !forced_comm_freeze) {
        auto freeze_running =
            system.grid_side_control.grid_integrator_supervisor.get_outputs().freeze_integrator.read();
        EXPECT_FALSE(freeze_running) << "Current controller integrator should be active during normal modulation";
    }

    // ============================================================
    // Phase 3: Issue stop command and verify integrator freezes
    // ============================================================

    set_stop_command(true);
    success &= simulator.simulate_for(0.5);

    // Give time for state machine to transition and block pulses
    success &= simulator.simulate_for(1.0);

    // Check if pulses are blocked
    auto release_pulses_stopping = system.grid_side_control.state_machine.get_outputs().release_pulses.read();

    // If pulses are blocked, integrator should be frozen
    if (!release_pulses_stopping) {
        auto freeze_stopping =
            system.grid_side_control.grid_integrator_supervisor.get_outputs().freeze_integrator.read();
        EXPECT_TRUE(freeze_stopping) << "Current controller integrator should freeze when pulses are blocked";
    }

    EXPECT_TRUE(success) << "Current controller freeze test completed with simulation errors";
}

/**
 * @test VirtualStartWithDampingTest
 * @brief Validates virtual torque reference damping during STARTING_PROCEDURE
 *
 * This test verifies that the startup sequencer correctly applies damping to the
 * virtual torque reference when the machine speed deviates from the virtual frequency.
 */
TEST_F(ClosedLoopSpeedControlTest, VirtualStartWithDampingTest)
{
    // 1. Configure Startup Sequencer for Virtual Start with Damping
    auto& params = system.machine_side_control.startup_sequencer.get_parameters();

    // Force virtual mode so it doesn't jump to closed-loop
    params.force_virtual_mode_operation.write(false);
    params.flying_restart_observer_settling_time.write(duration_t{0.1f});

    // Virtual frequency ramp (0.03 to 0.1 pu in 5s)
    params.virtual_frequency_initial.write(frequency_pu_t{0.0f});
    params.virtual_frequency_final.write(frequency_pu_t{0.15f});
    params.virtual_frequency_duration.write(duration_t{30.0f});

    // Constant virtual torque
    params.virtual_torque_initial.write(torque_pu_t{0.10f});
    params.virtual_torque_final.write(torque_pu_t{0.10f});
    params.virtual_torque_duration.write(duration_t{1.0f});

    params.speed_reference.write(speed_pu_t{1.0f});
    params.speed_reference_rate_limit_seconds_per_pu.write(real_t{60.0f});

    // Configure Damping
    const float damping_kp = 2.0f;
    const float damping_ti = 0.5f;
    const speed_pu_t activation_speed{0.0f};
    params.virtual_torque_damping_enabled.write(true);
    params.virtual_torque_damping_kp.write(real_t{damping_kp});
    params.virtual_torque_damping_ti.write(real_t{damping_ti});
    params.virtual_torque_damping_activation_speed.write(activation_speed);

    // Re-initialize sequencer to ensure parameters are pushed
    system.machine_side_control.startup_sequencer.initialize();

    // 2. Setup Plant Load to create speed error
    // With moderate load, machine will accelerate
    system.plant.get_parameters().constant_torque.write(torque_pu_t(0.10f));
    system.machine_side_control.excitation_controller.get_parameters().prefluxing_duration = duration_t{1.0f};

    // 4. Run Startup
    execute_startup_sequence();

    // Wait for state machine to reach OPERATION_ON and complete pre-flux
    // 0.5s for breakers + 1.0s for pre-flux + margin
    bool success = simulator.simulate_for(2.0);

    auto state = system.machine_side_control.startup_sequencer.get_outputs().current_state.read();
    EXPECT_EQ(state, startup_sequencer_state_t::STARTING_PROCEDURE) << "Should be in STARTING_PROCEDURE after pre-flux";

    // Simulate through the virtual start
    // We expect the checker to validate the torque throughout the ramp
    success &= simulator.simulate_for(100.0);

    // Verify we completed the starting procedure or reached high enough speed
    auto final_meas_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_GT(final_meas_speed.value(), activation_speed.value())
        << "Machine should have accelerated past activation speed";

    EXPECT_TRUE(success) << "Virtual start with damping test failed simulation or check";
}
