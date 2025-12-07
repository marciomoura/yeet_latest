/**
 * @file closed_loop_speed_control_test.cpp
 * @brief Integration tests for speed control in closed-loop SFC system
 *
 * Tests for speed reference tracking, multi-stage speed profiles, purge mode operation,
 * and steady-state speed control behavior.
 */

#include <gtest/gtest.h>

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
        simulator.configure_sampler_sampling_rate(ms(10.0f));

        // Register all applications with the simulator
        system.register_applications(simulator);

        // Configure topology to use internal model
        system.plant.get_parameters().topology.write(sfc_plant_model_datamodel::topology_type::internal);

        // Configure plant for gas turbine soft-starter application
        auto gas_turbine_config = yeet::application::sfc::integration::test::create_gas_turbine_config();
        yeet::application::sfc::integration::test::configure_variable_speed_plant(system.plant, gas_turbine_config);

        // Initialize the system
        initialize();

        // Register loggers for debugging
        register_complete_logger(simulator, system.machine_side_control.speed_controller);
        register_complete_logger(simulator, system.machine_side_control.startup_sequencer);
        register_complete_logger(simulator, system.machine_side_control.machine_side_observer);
    }

    void initialize()
    {
        set_auxiliaries_on(false);
        set_breakers_on(false);
        set_start_command(false);
        set_stop_command(false);
        set_shutdown_request(false);
        set_current_extinguished(false);

        system.initialize();
    }

    void simulate_for(double time) { simulator.simulate_for(time); }

    // State machine control helpers using mock inputs
    void set_auxiliaries_on(bool on)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().auxiliaries_are_on, on);
    }

    void set_breakers_on(bool on)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().mcb_is_closed, on);
    }

    void set_start_command(bool start)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().start_command, start);
    }

    void set_stop_command(bool stop)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().stop_command, stop);
    }

    void set_shutdown_request(bool shutdown)
    {
        yeet::mock_signal_input::write(system.grid_side_control.state_machine.get_inputs().shutdown_request, shutdown);
    }

    void set_current_extinguished(bool extinguished)
    {
        yeet::mock_signal_input::write(
            system.grid_side_control.state_machine.get_inputs().is_current_extinguished, extinguished);
    }

    // Complete startup sequence following state machine logic
    void execute_startup_sequence()
    {
        set_auxiliaries_on(true);
        set_breakers_on(true);
        simulate_for(0.2);
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

    // Ramp time: 0.15 pu × 120 s/pu = 18 seconds + settling
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

    // Ramp time: 0.2 pu × 40 s/pu = 8 seconds + settling
    success &= simulator.simulate_for(10.0);

    auto creep_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(creep_speed.value(), 0.2f, 0.08f) << "Machine should reach creep speed (0.2 pu)";

    // Hold at creep speed for positioning/alignment phase
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(10.0);
    speed_tracking_checker->deactivate();

    // Stage 2: Purge Speed (0.5 pu)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{0.5});

    // Ramp time: 0.3 pu × 40 s/pu = 12 seconds + settling
    success &= simulator.simulate_for(14.0);

    auto purge_speed = system.machine_side_control.machine_side_observer.get_outputs().estimated_speed.read();
    EXPECT_NEAR(purge_speed.value(), 0.5f, 0.10f) << "Machine should reach purge speed (0.5 pu)";

    // Hold at purge speed
    speed_tracking_checker->activate();
    success &= simulator.simulate_for(10.0);
    speed_tracking_checker->deactivate();

    // Stage 3: Full Speed (1.0 pu)
    system.machine_side_control.startup_sequencer.get_parameters().speed_reference.write(speed_pu_t{1.0});

    // Ramp time: 0.5 pu × 40 s/pu = 20 seconds + settling
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
